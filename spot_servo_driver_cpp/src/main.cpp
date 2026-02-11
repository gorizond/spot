#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdlib>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace {
std::string getenv_or(const std::string &name, const std::string &fallback) {
  const char *v = std::getenv(name.c_str());
  if (!v) return fallback;
  std::string s(v);
  return s.empty() ? fallback : s;
}
}

class SpotServoDriverCpp : public rclcpp::Node {
 public:
  SpotServoDriverCpp()
      : Node("spot_servo_driver_cpp"),
        servo_map_topic_(getenv_or("SERVO_MAP_TOPIC", "/spot/config/servo_map")),
        cmd_topic_(getenv_or("CMD_TOPIC", "/spot/cmd/servo")),
        status_topic_(getenv_or("STATUS_TOPIC", "/spot/state/servo")) {
    sub_servo_map_ = create_subscription<std_msgs::msg::String>(
        servo_map_topic_, 10, std::bind(&SpotServoDriverCpp::on_servo_map, this, _1));
    sub_cmd_ = create_subscription<std_msgs::msg::String>(
        cmd_topic_, 10, std::bind(&SpotServoDriverCpp::on_cmd, this, _1));
    pub_status_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    timer_ = create_wall_timer(1s, std::bind(&SpotServoDriverCpp::tick, this));

    RCLCPP_INFO(get_logger(),
      "starting P2 scaffold; servo_map=%s cmd=%s status=%s",
      servo_map_topic_.c_str(), cmd_topic_.c_str(), status_topic_.c_str());
  }

 private:
  void on_servo_map(const std_msgs::msg::String::SharedPtr msg) {
    last_servo_map_size_ = msg->data.size();
  }

  void on_cmd(const std_msgs::msg::String::SharedPtr msg) {
    last_cmd_size_ = msg->data.size();
  }

  void tick() {
    std_msgs::msg::String st;
    st.data = "{\"armed\":false,\"estop\":false,\"impl\":\"spot_servo_driver_cpp_scaffold\"}";
    pub_status_->publish(st);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "P2 scaffold alive; last_servo_map_bytes=%zu last_cmd_bytes=%zu",
                         last_servo_map_size_, last_cmd_size_);
  }

  std::string servo_map_topic_;
  std::string cmd_topic_;
  std::string status_topic_;

  size_t last_servo_map_size_{0};
  size_t last_cmd_size_{0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_servo_map_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpotServoDriverCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
