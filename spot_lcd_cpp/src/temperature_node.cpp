#include "spot_lcd_node.h"

#if defined(USE_ROS2) && USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>

namespace {
std::string escapeJson(const std::string& input) {
    std::string out;
    out.reserve(input.size());
    for (char c : input) {
        if (c == '\\' || c == '"') {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    return out;
}
}

class TemperatureNode : public rclcpp::Node
{
public:
    TemperatureNode() : Node("temperature_node")
    {
        config_ = Config::fromEnv();
        
        // Publisher для температуры
        temp_publisher_ = this->create_publisher<std_msgs::msg::String>("/system/temperature", 10);
        
        // Запускаем монитор температуры
        auto temp_callback = [this](const std::string& temp) {
            PublishTemperature(temp);
        };
        
        temp_monitor_ = std::make_unique<TempMonitor>(temp_callback, config_.temp_monitor);
        temp_monitor_->startMonitoring();
        
        RCLCPP_INFO(this->get_logger(), "Temperature Node initialized");
    }

private:
    void PublishTemperature(const std::string& temp_str) {
        auto msg = std_msgs::msg::String();
        std::string payload =
            "{\"line\":" + std::to_string(config_.temp_publish.line) +
            ",\"col\":" + std::to_string(config_.temp_publish.col) +
            ",\"width\":" + std::to_string(config_.temp_publish.width) +
            ",\"text\":\"" + escapeJson(temp_str) + "\"}";
        msg.data = payload;
        temp_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published temperature: %s", temp_str.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr temp_publisher_;
    Config config_;
    std::unique_ptr<TempMonitor> temp_monitor_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureNode>());
    rclcpp::shutdown();
    return 0;
}
#else

// Альтернативная реализация без ROS2
#include <iostream>
#include <functional>

int main(int argc, char * argv[])
{
    std::cout << "Temperature Node running without ROS2" << std::endl;
    
    Config config = Config::fromEnv();
    
    // Callback для обработки температуры
    auto temp_callback = [](const std::string& temp) {
        std::cout << "Temperature: " << temp << std::endl;
    };
    
    auto temp_monitor = std::make_unique<TempMonitor>(temp_callback, config.temp_monitor);
    temp_monitor->startMonitoring();
    
    // Ждем бесконечно
    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
#endif
