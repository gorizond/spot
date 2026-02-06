#include "spot_lcd_node.h"
#ifdef USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class LcdNode : public rclcpp::Node
{
public:
    LcdNode() : Node("lcd_node")
    {
        // Инициализация дисплея
        config_ = Config::fromEnv();
        display_service_ = std::make_unique<DisplayService>(config_.lcd);
        
        // Подписка на темпы температуры
        temp_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/temperature", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ProcessTemperatureData(msg->data);
            });

        // Подписка на темпы аптайма
        uptime_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/uptime", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ProcessUptimeData(msg->data);
            });

        // Таймер для обновления дисплея
        display_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                // Обновляем дисплей с текущими данными
                std::lock_guard<std::mutex> lock(*(display_service_->getLcdReference()));
                
                auto temp_it = data_map_.find("temperature");
                auto uptime_it = data_map_.find("uptime");
                
                std::string temp = temp_it != data_map_.end() ? temp_it->second : "N/A";
                std::string uptime = uptime_it != data_map_.end() ? uptime_it->second : "N/A";
                
                if (display_service_) {
                    display_service_->setData("temperature", temp);
                    display_service_->setData("uptime", uptime);
                }
            });

        RCLCPP_INFO(this->get_logger(), "LCD Node initialized");
    }

private:
    void ProcessTemperatureData(const std::string& temp_str) {
        data_map_["temperature"] = temp_str;
        RCLCPP_DEBUG(this->get_logger(), "Received temperature: %s", temp_str.c_str());
    }

    void ProcessUptimeData(const std::string& uptime_str) {
        data_map_["uptime"] = uptime_str;
        RCLCPP_DEBUG(this->get_logger(), "Received uptime: %s", uptime_str.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr temp_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uptime_subscription_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    Config config_;
    std::unique_ptr<DisplayService> display_service_;
    std::map<std::string, std::string> data_map_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LcdNode>());
    rclcpp::shutdown();
    return 0;
}
#else

// Альтернативная реализация без ROS2
#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <memory>

int main(int argc, char * argv[])
{
    std::cout << "LCD Node running without ROS2" << std::endl;
    
    // Инициализация дисплея
    Config config = Config::fromEnv();
    auto display_service = std::make_unique<DisplayService>(config.lcd);
    
    std::map<std::string, std::string> data_map;
    
    // Запускаем цикл обновления дисплея
    while(true) {
        // Здесь должна быть логика получения данных температуры и аптайма
        // Для упрощенной реализации используем фиктивные данные
        std::string temp = "N/A";
        std::string uptime = "N/A";
        
        if (display_service) {
            display_service->setData("temperature", temp);
            display_service->setData("uptime", uptime);
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}
#endif