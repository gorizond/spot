#ifndef SPOT_LCD_NODE_H
#define SPOT_LCD_NODE_H

#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

// Configuration structures
struct LcdConfig {
    int pin_rs = 25;
    int pin_en = 24;
    std::vector<int> pins_data = {23, 17, 18, 22};
    int cols = 16;
    int rows = 2;
};

struct TempMonitorConfig {
    int interval = 5; // seconds
};

struct UptimeMonitorConfig {
    int interval = 10; // seconds
};

struct RosPublisherConfig {
    std::string temp_topic = "lcd_temperature";
    std::string uptime_topic = "lcd_uptime";
};

struct Config {
    LcdConfig lcd;
    TempMonitorConfig temp_monitor;
    UptimeMonitorConfig uptime_monitor;
    RosPublisherConfig ros_publisher;
    
    static Config fromEnv();
};

#ifdef USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RosPublisher {
public:
    RosPublisher(rclcpp::Node::SharedPtr node);
    void publishTemperature(const std::string& temp);
    void publishUptime(const std::string& uptime);
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr temp_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uptime_publisher_;
};
#endif

class LcdDisplay {
public:
    LcdDisplay(const LcdConfig& config);
    ~LcdDisplay();
    
    bool initialize();
    bool clear();
    bool writeLine(int line, const std::string& text);
    bool writeChar(char c);

private:
    LcdConfig config_;
    bool initialized_ = false;
    
#ifdef USE_GPIO
    void pulseEnable();
    void sendNibble(uint8_t data);
    void sendByte(uint8_t data, bool rs);
#endif
};

class DisplayService {
public:
    DisplayService(const LcdConfig& config);
    ~DisplayService();
    
    void startDisplayLoop();
    std::shared_ptr<std::mutex> getLcdReference();
    
private:
    std::shared_ptr<std::mutex> data_mutex_;
    std::map<std::string, std::string> data_;
    LcdConfig config_;
    std::unique_ptr<LcdDisplay> lcd_display_;
    std::thread display_thread_;
    bool running_ = false;
};

class TempMonitor {
public:
    using Callback = std::function<void(const std::string&)>;
    
    TempMonitor(Callback callback, const TempMonitorConfig& config);
    void startMonitoring();
    
private:
    Callback callback_;
    TempMonitorConfig config_;
    std::thread monitor_thread_;
    bool running_ = false;
    
    std::string getCurrentTemperature();
};

class UptimeMonitor {
public:
    using Callback = std::function<void(const std::string&)>;
    
    UptimeMonitor(Callback callback, const UptimeMonitorConfig& config);
    void startMonitoring();
    
private:
    Callback callback_;
    UptimeMonitorConfig config_;
    std::thread monitor_thread_;
    bool running_ = false;
    
    std::string getSystemUptime();
};

#endif // SPOT_LCD_NODE_H