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
#include <vector>

// Configuration structures
struct LcdConfig {
    int pin_rs = 25;
    int pin_en = 24;
    std::vector<int> pins_data = {23, 17, 18, 22};
    int cols = 16;
    int rows = 2;
    int temp_line = 0;   // 0-based line index for temperature
    int uptime_line = 1; // 0-based line index for uptime
};

struct TempMonitorConfig {
    int interval = 10; // seconds
};

struct UptimeMonitorConfig {
    int interval = 10; // seconds
};

struct LcdPublishConfig {
    int line = 1;  // 1-based
    int col = 0;   // 0-based
    int width = 8; // characters
};

struct RosPublisherConfig {
    std::string temp_topic = "lcd_temperature";
    std::string uptime_topic = "lcd_uptime";
};

struct Config {
    LcdConfig lcd;
    TempMonitorConfig temp_monitor;
    UptimeMonitorConfig uptime_monitor;
    LcdPublishConfig temp_publish;
    LcdPublishConfig uptime_publish;
    RosPublisherConfig ros_publisher;
    
    static Config fromEnv();
};

#if defined(USE_ROS2) && USE_ROS2
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
#else
// Forward declaration or empty class when ROS2 is disabled
class RosPublisher {};
#endif

// GPIO definitions for libgpiod
#ifdef USE_GPIO
#include <gpiod.h>
extern "C" {
    #include <gpiod.h>
}
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
    bool setCursorPosition(uint8_t row, uint8_t col);
    
#ifdef USE_GPIO
    // GPIO resources for libgpiod
    struct gpiod_chip *chip = nullptr;
    struct gpiod_line *gpio_lines[6]; // RS, EN, D4, D5, D6, D7
    
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
    void setLineText(int line, const std::string& text);
    void setSegment(int line, int col, int width, const std::string& text);
    void setData(const std::string& key, const std::string& value);
    
private:
    std::shared_ptr<std::mutex> data_mutex_;
    std::vector<std::string> lines_;
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