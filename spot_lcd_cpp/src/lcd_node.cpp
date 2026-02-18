#include "spot_lcd_node.h"

#if defined(USE_ROS2) && USE_ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <string>
#include <cctype>

namespace {
bool extractInt(const std::string& json, const std::string& key, int& out) {
    auto key_pos = json.find("\"" + key + "\"");
    if (key_pos == std::string::npos) {
        return false;
    }
    auto colon = json.find(':', key_pos);
    if (colon == std::string::npos) {
        return false;
    }
    size_t pos = colon + 1;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    bool neg = false;
    if (pos < json.size() && (json[pos] == '-' || json[pos] == '+')) {
        neg = (json[pos] == '-');
        ++pos;
    }
    size_t start = pos;
    while (pos < json.size() && std::isdigit(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    if (start == pos) {
        return false;
    }
    int value = std::stoi(json.substr(start, pos - start));
    out = neg ? -value : value;
    return true;
}

bool extractString(const std::string& json, const std::string& key, std::string& out) {
    auto key_pos = json.find("\"" + key + "\"");
    if (key_pos == std::string::npos) {
        return false;
    }
    auto colon = json.find(':', key_pos);
    if (colon == std::string::npos) {
        return false;
    }
    auto first_quote = json.find('"', colon + 1);
    if (first_quote == std::string::npos) {
        return false;
    }
    std::string value;
    bool escape = false;
    for (size_t i = first_quote + 1; i < json.size(); ++i) {
        char c = json[i];
        if (escape) {
            value.push_back(c);
            escape = false;
            continue;
        }
        if (c == '\\') {
            escape = true;
            continue;
        }
        if (c == '"') {
            out = value;
            return true;
        }
        value.push_back(c);
    }
    return false;
}
}

class LcdNode : public rclcpp::Node
{
public:
    LcdNode() : Node("lcd_node")
    {
        // Инициализация дисплея
        config_ = Config::fromEnv();
        display_service_ = std::make_unique<DisplayService>(config_.lcd);
        display_service_->startDisplayLoop();
        
        // Подписка на темпы температуры
        temp_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/temperature", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                HandlePayload(msg->data, config_.lcd.temp_line, 0, 8);
            });

        // Подписка на темпы аптайма
        uptime_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/system/uptime", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                HandlePayload(msg->data, config_.lcd.uptime_line, 0, 8);
            });

        RCLCPP_INFO(this->get_logger(), "LCD Node initialized (refresh_interval=%ds)", config_.lcd.refresh_interval);
    }

private:
    void HandlePayload(const std::string& payload, int fallback_line, int fallback_col, int fallback_width) {
        int line = fallback_line + 1; // allow fallback to be 0-based internally
        int col = fallback_col;
        int width = fallback_width;
        std::string text;

        bool has_text = extractString(payload, "text", text);
        extractInt(payload, "line", line);
        extractInt(payload, "col", col);
        extractInt(payload, "width", width);

        if (!has_text) {
            text = payload;
        }

        if (line >= 1) {
            line -= 1; // convert to 0-based
        }

        RCLCPP_DEBUG(this->get_logger(), "LCD payload: text='%s' line=%d col=%d width=%d raw='%s'",
                     text.c_str(), line, col, width, payload.c_str());

        std::lock_guard<std::mutex> lock(*(display_service_->getLcdReference()));
        display_service_->setSegment(line, col, width, text);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr temp_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uptime_subscription_;
    Config config_;
    std::unique_ptr<DisplayService> display_service_;
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
    display_service->startDisplayLoop();
    
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
