#include "spot_lcd_node.h"

LcdDisplay::LcdDisplay(const LcdConfig& config) : config_(config) {}

LcdDisplay::~LcdDisplay() {}

bool LcdDisplay::initialize() {
#ifndef USE_GPIO
    std::cout << "LCD initialized in simulation mode" << std::endl;
    initialized_ = true;
#else
    // Инициализация GPIO и LCD
    // Это будет работать только на Raspberry Pi
    std::cout << "LCD initialized in GPIO mode" << std::endl;
    initialized_ = true;
#endif
    return initialized_;
}

bool LcdDisplay::clear() {
#ifndef USE_GPIO
    std::cout << "LCD cleared (simulation)" << std::endl;
#else
    // Очистка дисплея через GPIO
    std::cout << "LCD cleared" << std::endl;
#endif
    return true;
}

bool LcdDisplay::writeLine(int line, const std::string& text) {
#ifndef USE_GPIO
    std::cout << "LCD line " << line << ": " << text << std::endl;
#else
    // Запись строки на дисплей через GPIO
    std::cout << "LCD line " << line << ": " << text << std::endl;
#endif
    return true;
}

bool LcdDisplay::writeChar(char c) {
#ifndef USE_GPIO
    std::cout << "LCD char: " << c << std::endl;
#else
    // Запись символа на дисплей через GPIO
    std::cout << c << std::flush;
#endif
    return true;
}

#ifdef USE_ROS2
RosPublisher::RosPublisher(rclcpp::Node::SharedPtr node) : node_(node) {
    temp_publisher_ = node_->create_publisher<std_msgs::msg::String>("lcd_temperature", 10);
    uptime_publisher_ = node_->create_publisher<std_msgs::msg::String>("lcd_uptime", 10);
}

void RosPublisher::publishTemperature(const std::string& temp) {
    auto msg = std_msgs::msg::String();
    msg.data = temp;
    temp_publisher_->publish(msg);
}

void RosPublisher::publishUptime(const std::string& uptime) {
    auto msg = std_msgs::msg::String();
    msg.data = uptime;
    uptime_publisher_->publish(msg);
}
#endif