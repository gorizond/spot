#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {

double clampd(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string getenv_or(const std::string &name, const std::string &fallback) {
  const char *v = std::getenv(name.c_str());
  if (!v) return fallback;
  const auto t = trim(v);
  return t.empty() ? fallback : t;
}

double getenv_double(const std::string &name, double fallback) {
  try {
    return std::stod(getenv_or(name, std::to_string(fallback)));
  } catch (...) {
    return fallback;
  }
}

bool getenv_bool(const std::string &name, bool fallback) {
  auto v = getenv_or(name, fallback ? "1" : "0");
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c){ return std::tolower(c); });
  if (v == "1" || v == "true" || v == "yes" || v == "y" || v == "on") return true;
  if (v == "0" || v == "false" || v == "no" || v == "n" || v == "off") return false;
  return fallback;
}

std::string json_escape(const std::string &s) {
  std::ostringstream o;
  for (char c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default: o << c;
    }
  }
  return o.str();
}

bool parse_bool_json_field(const std::string &raw, const std::string &field, bool default_value) {
  const auto key = "\"" + field + "\"";
  auto p = raw.find(key);
  if (p == std::string::npos) return default_value;
  p = raw.find(':', p + key.size());
  if (p == std::string::npos) return default_value;
  auto rest = raw.substr(p + 1);
  rest = trim(rest);
  std::transform(rest.begin(), rest.end(), rest.begin(), [](unsigned char c){ return std::tolower(c); });
  if (rest.rfind("true", 0) == 0) return true;
  if (rest.rfind("false", 0) == 0) return false;
  return default_value;
}

struct MappedJoint {
  std::string out_name;
  std::string kind;
};

std::optional<MappedJoint> map_champ_joint_to_spot(const std::string &name) {
  // format: front_left_shoulder | rear_right_leg | ...
  std::vector<std::string> parts;
  std::stringstream ss(name);
  std::string token;
  while (std::getline(ss, token, '_')) parts.push_back(token);
  if (parts.size() != 3) return std::nullopt;

  std::string leg;
  if (parts[0] == "front" && parts[1] == "left") leg = "lf";
  else if (parts[0] == "front" && parts[1] == "right") leg = "rf";
  else if (parts[0] == "rear" && parts[1] == "left") leg = "lh";
  else if (parts[0] == "rear" && parts[1] == "right") leg = "rh";
  else return std::nullopt;

  if (parts[2] == "shoulder") return MappedJoint{leg + "_hip", "hip"};
  if (parts[2] == "leg") return MappedJoint{leg + "_upper", "upper"};
  if (parts[2] == "foot") return MappedJoint{leg + "_lower", "lower"};
  return std::nullopt;
}

}  // namespace

class ChampBridgeNode : public rclcpp::Node {
 public:
  ChampBridgeNode()
      : Node("spot_champ_bridge"),
        joint_states_topic_(getenv_or("JOINT_STATES_TOPIC", "/joint_states")),
        status_topic_(getenv_or("STATUS_TOPIC", "/spot/state/servo")),
        cmd_topic_(getenv_or("CMD_TOPIC", "/spot/cmd/servo_auto")),
        gain_(clampd(getenv_double("CHAMP_GAIN", 0.25), 0.0, 2.0)),
        publish_hz_(std::max(1.0, getenv_double("CHAMP_BRIDGE_HZ", 50.0))),
        hip_range_(std::max(1e-6, getenv_double("CHAMP_HIP_RANGE_RAD", 0.55))),
        upper_range_(std::max(1e-6, getenv_double("CHAMP_UPPER_RANGE_RAD", 1.0))),
        lower_range_(std::max(1e-6, getenv_double("CHAMP_LOWER_RANGE_RAD", 1.0))),
        hip_sign_(clampd(getenv_double("CHAMP_HIP_SIGN", 1.0), -1.0, 1.0)),
        require_armed_(getenv_bool("CHAMP_REQUIRE_ARMED", true)),
        stand_hip_(clampd(getenv_double("STAND_HIP", 0.02), -1.0, 1.0)),
        stand_upper_(clampd(getenv_double("STAND_UPPER", 0.05), -1.0, 1.0)),
        stand_lower_(clampd(getenv_double("STAND_LOWER", 0.05), -1.0, 1.0)),
        stand_rear_hip_offset_(clampd(getenv_double("STAND_HIP_REAR_OFFSET", 0.0), -1.0, 1.0)),
        stand_rear_upper_offset_(clampd(getenv_double("STAND_REAR_UPPER_OFFSET", 0.0), -1.0, 1.0)),
        stand_rear_lower_offset_(clampd(getenv_double("STAND_REAR_LOWER_OFFSET", 0.0), -1.0, 1.0)) {
    cmd_pub_ = create_publisher<std_msgs::msg::String>(cmd_topic_, 10);

    rclcpp::QoS qos_joint(1);
    qos_joint.best_effort();
    qos_joint.durability_volatile();

    sub_status_ = create_subscription<std_msgs::msg::String>(
        status_topic_, 10, std::bind(&ChampBridgeNode::on_status, this, _1));
    sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, qos_joint, std::bind(&ChampBridgeNode::on_joint_states, this, _1));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
        std::bind(&ChampBridgeNode::tick, this));

    RCLCPP_INFO(get_logger(),
      "starting; joint_states=%s status=%s cmd=%s publish_hz=%.1f gain=%.2f",
      joint_states_topic_.c_str(), status_topic_.c_str(), cmd_topic_.c_str(), publish_hz_, gain_);
  }

 private:
  void on_status(const std_msgs::msg::String::SharedPtr msg) {
    const auto &raw = msg->data;
    bool armed = parse_bool_json_field(raw, "armed", armed_);
    bool estop = parse_bool_json_field(raw, "estop", estop_);
    if (armed != armed_ || estop != estop_) force_publish_ = true;
    armed_ = armed;
    estop_ = estop;
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = msg;
    latest_seq_++;
  }

  double stand_base(const std::string &out_name, const std::string &kind) const {
    double base = (kind == "hip") ? stand_hip_ : (kind == "upper" ? stand_upper_ : stand_lower_);
    if (out_name == "rh_hip" || out_name == "lh_hip") {
      if (kind == "hip") base += stand_rear_hip_offset_;
      if (kind == "upper") base += stand_rear_upper_offset_;
      if (kind == "lower") base += stand_rear_lower_offset_;
    }
    return base;
  }

  double range_for(const std::string &kind) const {
    if (kind == "hip") return hip_range_;
    if (kind == "upper") return upper_range_;
    return lower_range_;
  }

  void tick() {
    if (estop_) return;
    if (require_armed_ && !armed_) return;
    if (!latest_joint_state_) return;
    if (latest_seq_ == processed_seq_ && !force_publish_) return;

    const auto &msg = *latest_joint_state_;
    if (msg.name.empty() || msg.position.empty() || msg.name.size() != msg.position.size()) return;

    if (zero_by_joint_.empty()) {
      for (size_t i = 0; i < msg.name.size(); ++i) zero_by_joint_[msg.name[i]] = msg.position[i];
      RCLCPP_INFO(get_logger(), "captured CHAMP zero angles (stand reference)");
    }

    std::map<std::string, double> targets;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      auto mapped = map_champ_joint_to_spot(msg.name[i]);
      if (!mapped) continue;

      const double zero = zero_by_joint_.count(msg.name[i]) ? zero_by_joint_[msg.name[i]] : 0.0;
      const double delta = msg.position[i] - zero;
      double scaled = clampd(delta / range_for(mapped->kind), -1.0, 1.0);
      if (mapped->kind == "hip") scaled *= hip_sign_;
      double out = clampd(stand_base(mapped->out_name, mapped->kind) + scaled * gain_, -1.0, 1.0);
      targets[mapped->out_name] = out;
    }

    processed_seq_ = latest_seq_;
    force_publish_ = false;
    if (targets.empty()) return;

    std::ostringstream js;
    js << "{\"cmd\":\"set\",\"mode\":\"norm\",\"targets\":{";
    bool first = true;
    for (const auto &kv : targets) {
      if (!first) js << ',';
      first = false;
      js << '"' << json_escape(kv.first) << "\":" << std::setprecision(6) << kv.second;
    }
    js << "}}";

    std_msgs::msg::String out;
    out.data = js.str();
    cmd_pub_->publish(out);

    auto now = std::chrono::steady_clock::now();
    if (!last_log_.has_value() || (now - *last_log_) > 5s) {
      last_log_ = now;
      RCLCPP_INFO(get_logger(), "published %zu joint targets to %s (armed=%s)",
                  targets.size(), cmd_topic_.c_str(), armed_ ? "true" : "false");
    }
  }

 private:
  std::string joint_states_topic_;
  std::string status_topic_;
  std::string cmd_topic_;

  double gain_;
  double publish_hz_;
  double hip_range_;
  double upper_range_;
  double lower_range_;
  double hip_sign_;
  bool require_armed_;

  double stand_hip_;
  double stand_upper_;
  double stand_lower_;
  double stand_rear_hip_offset_;
  double stand_rear_upper_offset_;
  double stand_rear_lower_offset_;

  bool armed_{false};
  bool estop_{false};
  bool force_publish_{false};

  uint64_t latest_seq_{0};
  uint64_t processed_seq_{0};

  std::unordered_map<std::string, double> zero_by_joint_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<std::chrono::steady_clock::time_point> last_log_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChampBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
