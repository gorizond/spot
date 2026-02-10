#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "spot_node_label_config_cpp/ports/servo_map_publisher.hpp"

namespace spot::adapters {

class RosStringPublisher : public spot::ports::IServoMapPublisher {
 public:
  RosStringPublisher(rclcpp::Node& node, const std::string& topic);

  void PublishJson(const std::string& json) override;

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace spot::adapters
