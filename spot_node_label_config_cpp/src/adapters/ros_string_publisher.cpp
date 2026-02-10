#include "spot_node_label_config_cpp/adapters/ros_string_publisher.hpp"

namespace spot::adapters {

RosStringPublisher::RosStringPublisher(rclcpp::Node& node, const std::string& topic) {
  // Match python QoS: depth=1, reliable, transient_local
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pub_ = node.create_publisher<std_msgs::msg::String>(topic, qos);
}

void RosStringPublisher::PublishJson(const std::string& json) {
  std_msgs::msg::String msg;
  msg.data = json;
  pub_->publish(msg);
}

}  // namespace spot::adapters
