#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>

#include <gpiod.h>
#include <linux/gpio.h>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <sstream>

using namespace std::chrono_literals;

struct HCSensor {
    std::string name;
    int trig_pin;
    int echo_pin;
    struct gpiod_line* trig_line = nullptr;
    struct gpiod_line* echo_line = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;
    sensor_msgs::msg::Range msg;
};

class HCSr04Node : public rclcpp::Node {
public:
    HCSr04Node() : Node("hcsr04_node") {
        // Config from env
        auto sensor_config = declare_parameter<std::vector<std::string>>(
            "sensors", {"front_left:5:12", "front_right:6:13"});
        
        // Open GPIO chip
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        if (!chip_) {
            RCLCRITICAL("Failed to open gpiochip0");
            throw std::runtime_error("GPIO init failed");
        }
        
        // Initialize sensors
        for (const auto& cfg : sensor_config) {
            HCSensor sensor;
            std::stringstream ss(cfg);
            std::string name, trig_s, echo_s;
            std::getline(ss, name, ':');
            std::getline(ss, trig_s, ':');
            std::getline(ss, echo_s, ':');
            
            sensor.name = name;
            sensor.trig_pin = std::stoi(trig_s);
            sensor.echo_pin = std::stoi(echo_s);
            
            // Request GPIO lines
            sensor.trig_line = gpiod_chip_get_line(chip_, sensor.trig_pin);
            sensor.echo_line = gpiod_chip_get_line(chip_, sensor.echo_pin);
            
            if (!sensor.trig_line || !sensor.echo_line) {
                RCLCRITICAL("Failed to get GPIO lines for %s", name.c_str());
                continue;
            }
            
            gpiod_line_request_output(sensor.trig_line, "hcsr04_trig", 0);
            gpiod_line_request_input(sensor.echo_line, "hcsr04_echo");
            
            // Create publisher
            auto topic_name = "/spot/sensor/ultrasonic/" + name + "/range";
            sensor.pub = create_publisher<sensor_msgs::msg::Range>(topic_name, 10);
            
            // Setup Range message
            sensor.msg.header.frame_id = "ultrasonic_" + name;
            sensor.msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            sensor.msg.field_of_view = 0.26;  // ~15 degrees
            sensor.msg.min_range = 0.02;
            sensor.msg.max_range = 4.0;
            
            sensors_.push_back(sensor);
            RCLCPP_INFO(get_logger(), "Initialized sensor: %s (TRIG=%d, ECHO=%d)", 
                       name.c_str(), sensor.trig_pin, sensor.echo_pin);
        }
        
        // Timer for measurements (10Hz per sensor, staggered)
        timer_ = create_wall_timer(100ms, std::bind(&HCSr04Node::measure_callback, this));
    }
    
    ~HCSr04Node() {
        for (auto& s : sensors_) {
            if (s.trig_line) gpiod_line_release(s.trig_line);
            if (s.echo_line) gpiod_line_release(s.echo_line);
        }
        if (chip_) gpiod_chip_close(chip_);
    }

private:
    void measure_callback() {
        static size_t sensor_idx = 0;
        if (sensors_.empty()) return;
        
        auto& sensor = sensors_[sensor_idx];
        float distance_m = measure_distance(sensor);
        
        auto now = now();
        sensor.msg.header.stamp = now;
        sensor.msg.range = distance_m;
        sensor.pub->publish(sensor.msg);
        
        RCLCPP_DEBUG(get_logger(), "%s: %.2fm", sensor.name.c_str(), distance_m);
        
        sensor_idx = (sensor_idx + 1) % sensors_.size();
    }
    
    float measure_distance(HCSensor& sensor) {
        // Send 10µs trigger pulse
        gpiod_line_set_value(sensor.trig_line, 1);
        usleep(10);
        gpiod_line_set_value(sensor.trig_line, 0);
        
        // Wait for echo rising edge (timeout 25ms = ~4m)
        const int timeout_us = 25000;
        int start_us = 0, end_us = 0;
        
        // Poll for rising edge
        for (int i = 0; i < timeout_us; i++) {
            if (gpiod_line_get_value(sensor.echo_line) == 1) {
                start_us = i;
                break;
            }
            usleep(1);
        }
        
        if (start_us == 0) return -1.0f;  // Timeout
        
        // Poll for falling edge
        for (int i = start_us; i < timeout_us; i++) {
            if (gpiod_line_get_value(sensor.echo_line) == 0) {
                end_us = i;
                break;
            }
            usleep(1);
        }
        
        if (end_us == 0) return -1.0f;  // Timeout
        
        // Calculate distance: speed of sound = 343 m/s
        // distance = (duration_us / 2) * 343 / 1000000
        float duration_s = (end_us - start_us) / 1000000.0f;
        float distance_m = (duration_s * 343.0f) / 2.0f;
        
        // Clamp to valid range
        if (distance_m < sensor.msg.min_range) return sensor.msg.min_range;
        if (distance_m > sensor.msg.max_range) return sensor.msg.max_range;
        
        return distance_m;
    }
    
    struct gpiod_chip* chip_ = nullptr;
    std::vector<HCSensor> sensors_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HCSr04Node>());
    rclcpp::shutdown();
    return 0;
}
