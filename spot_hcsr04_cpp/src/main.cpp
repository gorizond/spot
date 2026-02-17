#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <gpiod.h>

#include <chrono>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

using namespace std::chrono_literals;

namespace {

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    item = trim(item);
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

std::vector<std::string> sensors_from_env_or_default() {
  const char *raw = std::getenv("SENSORS");
  if (!raw) {
    return {"front_left:5:12", "front_right:6:13"};
  }
  const std::string s = trim(raw);
  if (s.empty()) {
    return {"front_left:5:12", "front_right:6:13"};
  }
  return split(s, ',');
}

}  // namespace

struct HCSensor {
  std::string name;
  int trig_pin;
  int echo_pin;

  struct gpiod_line *trig_line = nullptr;
  struct gpiod_line *echo_line = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub;
  sensor_msgs::msg::Range msg;

  uint64_t timeout_count = 0;
};

class HCSr04Node final : public rclcpp::Node {
 public:
  HCSr04Node() : Node("hcsr04_node") {
    const auto default_sensors = sensors_from_env_or_default();
    const auto sensor_config = declare_parameter<std::vector<std::string>>("sensors", default_sensors);

    // Open GPIO chip
    chip_ = gpiod_chip_open_by_name("gpiochip0");
    if (!chip_) {
      RCLCPP_CRITICAL(get_logger(), "Failed to open gpiochip0");
      throw std::runtime_error("GPIO init failed");
    }

    // Initialize sensors
    for (const auto &cfg : sensor_config) {
      auto parts = split(cfg, ':');
      if (parts.size() != 3) {
        RCLCPP_ERROR(get_logger(), "Invalid sensor spec '%s' (expected name:TRIG:echo)", cfg.c_str());
        continue;
      }

      HCSensor sensor;
      sensor.name = parts[0];
      sensor.trig_pin = std::stoi(parts[1]);
      sensor.echo_pin = std::stoi(parts[2]);

      sensor.trig_line = gpiod_chip_get_line(chip_, sensor.trig_pin);
      sensor.echo_line = gpiod_chip_get_line(chip_, sensor.echo_pin);
      if (!sensor.trig_line || !sensor.echo_line) {
        RCLCPP_ERROR(get_logger(), "Failed to get GPIO lines for %s (TRIG=%d, ECHO=%d)", sensor.name.c_str(),
                     sensor.trig_pin, sensor.echo_pin);
        continue;
      }

      if (gpiod_line_request_output(sensor.trig_line, "hcsr04_trig", 0) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to request output access for TRIG (%s)", sensor.name.c_str());
        gpiod_line_release(sensor.trig_line);
        gpiod_line_release(sensor.echo_line);
        continue;
      }

      if (gpiod_line_request_input(sensor.echo_line, "hcsr04_echo") < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to request input access for ECHO (%s)", sensor.name.c_str());
        gpiod_line_release(sensor.trig_line);
        gpiod_line_release(sensor.echo_line);
        continue;
      }

      // Create publisher
      const auto topic_name = "/spot/sensor/ultrasonic/" + sensor.name + "/range";
      sensor.pub = create_publisher<sensor_msgs::msg::Range>(topic_name, 10);

      // Setup Range message
      sensor.msg.header.frame_id = "ultrasonic_" + sensor.name;
      sensor.msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      sensor.msg.field_of_view = 0.26f;  // ~15 degrees
      sensor.msg.min_range = 0.02f;
      sensor.msg.max_range = 4.0f;

      sensors_.push_back(std::move(sensor));
      RCLCPP_INFO(get_logger(), "Initialized sensor: %s (TRIG=%d, ECHO=%d)", sensors_.back().name.c_str(),
                  sensors_.back().trig_pin, sensors_.back().echo_pin);
    }

    // Timer for measurements (staggered across sensors)
    timer_ = create_wall_timer(100ms, std::bind(&HCSr04Node::measure_callback, this));
  }

  ~HCSr04Node() override {
    for (auto &s : sensors_) {
      if (s.trig_line) gpiod_line_release(s.trig_line);
      if (s.echo_line) gpiod_line_release(s.echo_line);
    }
    if (chip_) gpiod_chip_close(chip_);
  }

 private:
  void measure_callback() {
    static size_t sensor_idx = 0;
    if (sensors_.empty()) return;

    auto &sensor = sensors_[sensor_idx];
    const float distance_m = measure_distance(sensor);

    sensor.msg.header.stamp = this->now();
    sensor.msg.range = distance_m;
    sensor.pub->publish(sensor.msg);

    sensor_idx = (sensor_idx + 1) % sensors_.size();
  }

  float measure_distance(HCSensor &sensor) {
    // Trigger pulse (10us)
    (void)gpiod_line_set_value(sensor.trig_line, 0);
    usleep(2);
    (void)gpiod_line_set_value(sensor.trig_line, 1);
    usleep(10);
    (void)gpiod_line_set_value(sensor.trig_line, 0);

    // Wait for echo HIGH then LOW. Max echo time ~23ms for ~4m.
    constexpr auto timeout = 25ms;

    const auto t0 = std::chrono::steady_clock::now();
    while (true) {
      const int v = gpiod_line_get_value(sensor.echo_line);
      if (v < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: ECHO read error", sensor.name.c_str());
        return sensor.msg.max_range;
      }
      if (v == 1) break;
      if (std::chrono::steady_clock::now() - t0 > timeout) {
        sensor.timeout_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: rise timeout (count=%lu)",
                             sensor.name.c_str(), static_cast<unsigned long>(sensor.timeout_count));
        return sensor.msg.max_range;
      }
      usleep(1);
    }

    const auto t_rise = std::chrono::steady_clock::now();
    while (true) {
      const int v = gpiod_line_get_value(sensor.echo_line);
      if (v < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: ECHO read error", sensor.name.c_str());
        return sensor.msg.max_range;
      }
      if (v == 0) break;
      if (std::chrono::steady_clock::now() - t_rise > timeout) {
        sensor.timeout_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: fall timeout (count=%lu)",
                             sensor.name.c_str(), static_cast<unsigned long>(sensor.timeout_count));
        return sensor.msg.max_range;
      }
      usleep(1);
    }

    const auto t_fall = std::chrono::steady_clock::now();
    const auto pulse_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t_fall - t_rise).count();

    // Speed of sound approx 343 m/s at 20°C.
    const double distance_m = (static_cast<double>(pulse_us) * 1e-6 * 343.0) / 2.0;

    if (distance_m < sensor.msg.min_range) return sensor.msg.min_range;
    if (distance_m > sensor.msg.max_range) return sensor.msg.max_range;
    return static_cast<float>(distance_m);
  }

  struct gpiod_chip *chip_ = nullptr;
  std::vector<HCSensor> sensors_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HCSr04Node>());
  rclcpp::shutdown();
  return 0;
}
