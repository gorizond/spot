#include "spot_lcd_node.h"
#include <getopt.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <sstream>

#if defined(USE_ROS2) && USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SpotLcdNode : public rclcpp::Node {
public:
    SpotLcdNode() : Node("spot_lcd_node") {
        // Инициализация дисплея
        config_ = Config::fromEnv();
        display_service_ = std::make_unique<DisplayService>(config_.lcd);
        
        // Запускаем обычные мониторы как fallback
        auto temp_callback = [this](const std::string& temp) {
            ProcessTemperatureData(temp);
        };
        
        auto uptime_callback = [this](const std::string& uptime) {
            ProcessUptimeData(uptime);
        };
        
        temp_monitor_ = std::make_unique<TempMonitor>(temp_callback, config_.temp_monitor);
        uptime_monitor_ = std::make_unique<UptimeMonitor>(uptime_callback, config_.uptime_monitor);
        
        // Подписка на темпы температуры (если доступны)
        temp_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/temperature", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ProcessTemperatureData(msg->data);
            });

        // Подписка на темпы аптайма (если доступны)
        uptime_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/uptime", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                ProcessUptimeData(msg->data);
            });

        // Запускаем мониторы
        temp_monitor_->startMonitoring();
        uptime_monitor_->startMonitoring();
        
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

        RCLCPP_INFO(this->get_logger(), "Spot LCD Node initialized");
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
    std::unique_ptr<TempMonitor> temp_monitor_;
    std::unique_ptr<UptimeMonitor> uptime_monitor_;
    std::map<std::string, std::string> data_map_;
};
#endif

int main(int argc, char *argv[]) {
#if defined(USE_ROS2) && USE_ROS2
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SpotLcdNode>();
    
    std::cout << "Starting LCD service with ROS2 integration" << std::endl;
    
    rclcpp::spin(node);
    rclcpp::shutdown();
#else
    std::string modules = "all";
    
    int opt;
    while ((opt = getopt(argc, argv, "m:")) != -1) {
        switch (opt) {
            case 'm':
                modules = optarg;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-m modules]" << std::endl;
                std::cerr << "Modules: all, lcd, temp, uptime (comma separated)" << std::endl;
                return 1;
        }
    }
    
    std::cout << "Starting LCD service modules: " << modules << std::endl;
    
    std::vector<std::string> module_list;
    std::stringstream ss(modules);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // Убираем пробелы
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);
        module_list.push_back(item);
    }
    
    bool run_all = std::find(module_list.begin(), module_list.end(), "all") != module_list.end();
    bool run_lcd = run_all || std::find(module_list.begin(), module_list.end(), "lcd") != module_list.end();
    bool run_temp = run_all || std::find(module_list.begin(), module_list.end(), "temp") != module_list.end();
    bool run_uptime = run_all || std::find(module_list.begin(), module_list.end(), "uptime") != module_list.end();
    
    Config config = Config::fromEnv();
    
    std::unique_ptr<DisplayService> display_service;
    
    if (run_lcd) {
        display_service = std::make_unique<DisplayService>(config.lcd);
        display_service->startDisplayLoop();
    }
    
    // Создаем мониторы
    std::unique_ptr<TempMonitor> temp_monitor;
    std::unique_ptr<UptimeMonitor> uptime_monitor;
    
    if (run_temp) {
        auto temp_callback = [&display_service](const std::string& temp) {
            if (display_service) {
                display_service->setData("temperature", temp);
                std::cout << "Temperature: " << temp << std::endl;
            } else {
                std::cout << "Temperature: " << temp << std::endl;
            }
        };
        
        temp_monitor = std::make_unique<TempMonitor>(temp_callback, config.temp_monitor);
        temp_monitor->startMonitoring();
    }
    
    if (run_uptime) {
        auto uptime_callback = [&display_service](const std::string& uptime) {
            if (display_service) {
                display_service->setData("uptime", uptime);
                std::cout << "Uptime: " << uptime << std::endl;
            } else {
                std::cout << "Uptime: " << uptime << std::endl;
            }
        };
        
        uptime_monitor = std::make_unique<UptimeMonitor>(uptime_callback, config.uptime_monitor);
        uptime_monitor->startMonitoring();
    }
    
    // Ждем бесконечно
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
#endif

    return 0;
}