#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {
std::string trim(const std::string &s) {
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string getenv_or(const std::string &name, const std::string &fallback) {
  const char *v = std::getenv(name.c_str());
  if (!v) return fallback;
  const auto t = trim(v);
  return t.empty() ? fallback : t;
}

std::string normalize_mode(const std::string &raw) {
  const auto v = lower(trim(raw));
  if (v == "auto" || v == "manual") return v;
  return "";
}

std::string json_escape(const std::string &s) {
  std::ostringstream o;
  for (const auto c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\b': o << "\\b"; break;
      case '\f': o << "\\f"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          o << "\\u" << std::hex << std::uppercase << static_cast<int>(c);
        } else {
          o << c;
        }
    }
  }
  return o.str();
}
}  // namespace

class CmdMuxNode : public rclcpp::Node {
 public:
  CmdMuxNode()
      : Node("spot_cmd_mux"),
        auto_topic_(getenv_or("AUTO_TOPIC", "/spot/cmd/servo_auto")),
        manual_topic_(getenv_or("MANUAL_TOPIC", "/spot/cmd/servo_manual")),
        out_topic_(getenv_or("OUT_TOPIC", "/spot/cmd/servo")),
        mode_topic_(getenv_or("MODE_TOPIC", "/spot/ctrl/mode")),
        status_topic_(getenv_or("STATUS_TOPIC", "/spot/state/mux")),
        mode_(normalize_mode(getenv_or("DEFAULT_MODE", "auto"))),
        manual_timeout_s_(std::max(0.0, std::stod(getenv_or("MANUAL_TIMEOUT_S", "0")))),
        status_publish_s_(std::max(0.1, std::stod(getenv_or("STATUS_PUBLISH_S", "1.0")))) {
    if (mode_.empty()) mode_ = "auto";

    pub_ = this->create_publisher<std_msgs::msg::String>(out_topic_, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

    sub_auto_ = this->create_subscription<std_msgs::msg::String>(
        auto_topic_, 10, std::bind(&CmdMuxNode::on_auto, this, _1));
    sub_manual_ = this->create_subscription<std_msgs::msg::String>(
        manual_topic_, 10, std::bind(&CmdMuxNode::on_manual, this, _1));
    sub_mode_ = this->create_subscription<std_msgs::msg::String>(
        mode_topic_, 10, std::bind(&CmdMuxNode::on_mode, this, _1));

    tick_timer_ = this->create_wall_timer(100ms, std::bind(&CmdMuxNode::tick, this));

    RCLCPP_INFO(this->get_logger(), "starting; mode=%s auto=%s manual=%s out=%s",
                mode_.c_str(), auto_topic_.c_str(), manual_topic_.c_str(), out_topic_.c_str());
  }

 private:
  void publish_payload(const std::string &payload) {
    if (payload.empty()) return;
    std_msgs::msg::String msg;
    msg.data = payload;
    pub_->publish(msg);
  }

  void on_auto(const std_msgs::msg::String::SharedPtr msg) {
    const auto payload = trim(msg->data);
    if (payload.empty()) return;
    last_auto_ = payload;
    if (mode_ == "auto") publish_payload(payload);
  }

  void on_manual(const std_msgs::msg::String::SharedPtr msg) {
    const auto payload = trim(msg->data);
    if (payload.empty()) return;
    last_manual_ = payload;
    last_manual_tp_ = std::chrono::steady_clock::now();
    has_last_manual_ts_ = true;
    if (mode_ == "manual") publish_payload(payload);
  }

  void on_mode(const std_msgs::msg::String::SharedPtr msg) {
    const auto m = normalize_mode(msg->data);
    if (m.empty() || m == mode_) return;
    mode_ = m;
    RCLCPP_INFO(this->get_logger(), "mode=%s", mode_.c_str());

    if (mode_ == "auto" && !last_auto_.empty()) publish_payload(last_auto_);
    if (mode_ == "manual" && !last_manual_.empty()) publish_payload(last_manual_);
  }

  void publish_status() {
    const auto now = std::chrono::steady_clock::now();
    if (has_last_status_ts_) {
      const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_tp_).count() / 1000.0;
      if (dt < status_publish_s_) return;
    }
    last_status_tp_ = now;
    has_last_status_ts_ = true;

    std::ostringstream s;
    s << "{"
      << "\"mode\":\"" << json_escape(mode_) << "\"," 
      << "\"auto_topic\":\"" << json_escape(auto_topic_) << "\"," 
      << "\"manual_topic\":\"" << json_escape(manual_topic_) << "\"," 
      << "\"out_topic\":\"" << json_escape(out_topic_) << "\"," 
      << "\"manual_timeout_s\":" << manual_timeout_s_
      << "}";

    std_msgs::msg::String msg;
    msg.data = s.str();
    status_pub_->publish(msg);
  }

  void tick() {
    if (manual_timeout_s_ > 0.0 && mode_ == "manual" && has_last_manual_ts_) {
      const auto now = std::chrono::steady_clock::now();
      const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_manual_tp_).count() / 1000.0;
      if (dt > manual_timeout_s_) {
        mode_ = "auto";
        if (!last_auto_.empty()) publish_payload(last_auto_);
        RCLCPP_INFO(this->get_logger(), "manual timeout; switching to auto");
      }
    }
    publish_status();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_auto_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_manual_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  std::string auto_topic_;
  std::string manual_topic_;
  std::string out_topic_;
  std::string mode_topic_;
  std::string status_topic_;

  std::string mode_;
  std::string last_auto_;
  std::string last_manual_;

  double manual_timeout_s_{0.0};
  double status_publish_s_{1.0};

  std::chrono::steady_clock::time_point last_manual_tp_{};
  std::chrono::steady_clock::time_point last_status_tp_{};
  bool has_last_manual_ts_{false};
  bool has_last_status_ts_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
