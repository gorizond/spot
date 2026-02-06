#include "spot_lcd_node.h"
#ifdef USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class UptimeNode : public rclcpp::Node
{
public:
    UptimeNode() : Node("uptime_node")
    {
        config_ = Config::fromEnv();
        
        // Publisher для аптайма
        uptime_publisher_ = this->create_publisher<std_msgs::msg::String>("/system/uptime", 10);
        
        // Запускаем монитор аптайма
        auto uptime_callback = [this](const std::string& uptime) {
            PublishUptime(uptime);
        };
        
        uptime_monitor_ = std::make_unique<UptimeMonitor>(uptime_callback, config_.uptime_monitor);
        uptime_monitor_->startMonitoring();
        
        RCLCPP_INFO(this->get_logger(), "Uptime Node initialized");
    }

private:
    void PublishUptime(const std::string& uptime_str) {
        auto msg = std_msgs::msg::String();
        msg.data = uptime_str;
        uptime_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published uptime: %s", uptime_str.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uptime_publisher_;
    Config config_;
    std::unique_ptr<UptimeMonitor> uptime_monitor_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UptimeNode>());
    rclcpp::shutdown();
    return 0;
}
#else

// Альтернативная реализация без ROS2
#include <iostream>
#include <functional>

int main(int argc, char * argv[])
{
    std::cout << "Uptime Node running without ROS2" << std::endl;
    
    Config config = Config::fromEnv();
    
    // Callback для обработки аптайма
    auto uptime_callback = [](const std::string& uptime) {
        std::cout << "Uptime: " << uptime << std::endl;
    };
    
    auto uptime_monitor = std::make_unique<UptimeMonitor>(uptime_callback, config.uptime_monitor);
    uptime_monitor->startMonitoring();
    
    // Ждем бесконечно
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
#endif