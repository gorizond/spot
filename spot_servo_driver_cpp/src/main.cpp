#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using std::placeholders::_1;

namespace {

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

template <typename T>
T clampv(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

std::string getenv_or(const std::string &name, const std::string &fallback) {
  const char *v = std::getenv(name.c_str());
  if (!v) return fallback;
  const auto s = trim(v);
  return s.empty() ? fallback : s;
}

bool parse_bool(const std::string &raw, bool def = false) {
  auto s = raw;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  if (s == "1" || s == "true" || s == "yes" || s == "y" || s == "on") return true;
  if (s == "0" || s == "false" || s == "no" || s == "n" || s == "off") return false;
  return def;
}

int parse_int(const std::string &raw, int def) {
  try {
    size_t idx = 0;
    int v = std::stoi(trim(raw), &idx, 0);
    return v;
  } catch (...) {
    return def;
  }
}

double parse_double(const std::string &raw, double def) {
  try {
    size_t idx = 0;
    double v = std::stod(trim(raw), &idx);
    return v;
  } catch (...) {
    return def;
  }
}

std::string json_escape(const std::string &s) {
  std::ostringstream o;
  for (char c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default: o << c;
    }
  }
  return o.str();
}

bool extract_string_field(const std::string &obj, const std::string &field, std::string &out) {
  std::regex re("\\\"" + field + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch m;
  if (std::regex_search(obj, m, re) && m.size() >= 2) {
    out = m[1].str();
    return true;
  }
  return false;
}

bool extract_number_field(const std::string &obj, const std::string &field, std::string &out) {
  std::regex re("\\\"" + field + "\\\"\\s*:\\s*([-+]?0x[0-9a-fA-F]+|[-+]?[0-9]*\\.?[0-9]+)");
  std::smatch m;
  if (std::regex_search(obj, m, re) && m.size() >= 2) {
    out = m[1].str();
    return true;
  }
  return false;
}

bool extract_bool_field(const std::string &obj, const std::string &field, bool &out) {
  std::regex re("\\\"" + field + "\\\"\\s*:\\s*(true|false|1|0)", std::regex_constants::icase);
  std::smatch m;
  if (std::regex_search(obj, m, re) && m.size() >= 2) {
    out = parse_bool(m[1].str(), out);
    return true;
  }
  return false;
}

std::vector<std::string> extract_flat_objects(const std::string &s) {
  std::vector<std::string> out;
  int depth = 0;
  std::string cur;
  bool in_str = false;
  for (size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '"' && (i == 0 || s[i - 1] != '\\')) in_str = !in_str;
    if (in_str) {
      if (depth > 0) cur.push_back(c);
      continue;
    }
    if (c == '{') {
      depth++;
      if (depth == 1) {
        cur = "{";
      } else {
        cur.push_back(c);
      }
    } else if (c == '}') {
      if (depth > 0) {
        cur.push_back(c);
        depth--;
        if (depth == 0) out.push_back(cur);
      }
    } else if (depth > 0) {
      cur.push_back(c);
    }
  }
  return out;
}

class I2CDev {
 public:
  I2CDev(int bus, int address, const std::string &tmpl) {
    path_ = tmpl;
    const auto p = path_.find("{bus}");
    if (p != std::string::npos) path_.replace(p, 5, std::to_string(bus));

    fd_ = ::open(path_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("open i2c failed: " + path_);
    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("ioctl I2C_SLAVE failed");
    }
  }

  ~I2CDev() { close(); }

  const std::string &path() const { return path_; }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void write_reg(uint8_t reg, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> buf;
    buf.reserve(data.size() + 1);
    buf.push_back(reg);
    buf.insert(buf.end(), data.begin(), data.end());
    if (::write(fd_, buf.data(), buf.size()) < 0) throw std::runtime_error("i2c write failed");
  }

  uint8_t read_u8(uint8_t reg) {
    if (::write(fd_, &reg, 1) < 0) throw std::runtime_error("i2c addr write failed");
    uint8_t b = 0;
    if (::read(fd_, &b, 1) != 1) throw std::runtime_error("i2c read failed");
    return b;
  }

 private:
  int fd_{-1};
  std::string path_;
};

class PCA9685 {
 public:
  PCA9685(int bus, int address, const std::string &tmpl) : bus_(bus), address_(address), dev_(bus, address, tmpl) {
    init_chip();
  }

  void set_pwm_freq(double freq_hz) {
    freq_hz = clampv(freq_hz, 24.0, 1526.0);
    const double osc_hz = 25'000'000.0;
    int prescale = static_cast<int>(std::round(osc_hz / (4096.0 * freq_hz) - 1.0));
    prescale = clampv(prescale, 3, 255);

    const uint8_t old_mode = read_u8(MODE1);
    const uint8_t sleep_mode = (old_mode & 0x7F) | SLEEP;
    write_u8(MODE1, sleep_mode);
    write_u8(PRESCALE, static_cast<uint8_t>(prescale));
    write_u8(MODE1, old_mode);
    usleep(5000);
    write_u8(MODE1, old_mode | RESTART | AI);
    freq_hz_ = freq_hz;
  }

  void set_pwm(int ch, int on, int off) {
    if (ch < 0 || ch > 15) return;
    on = clampv(on, 0, 4095);
    off = clampv(off, 0, 4095);
    const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * ch);
    dev_.write_reg(reg, {
      static_cast<uint8_t>(on & 0xFF), static_cast<uint8_t>((on >> 8) & 0x0F),
      static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>((off >> 8) & 0x0F)
    });
  }

  void set_full_off(int ch) {
    if (ch < 0 || ch > 15) return;
    const uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * ch);
    dev_.write_reg(reg, {0x00, 0x00, 0x00, 0x10});
  }

  void set_pulse_us(int ch, int pulse_us) {
    if (freq_hz_ <= 0.0) throw std::runtime_error("pca9685 freq not set");
    int steps = static_cast<int>(std::round(pulse_us * freq_hz_ * 4096.0 / 1'000'000.0));
    steps = clampv(steps, 0, 4095);
    set_pwm(ch, 0, steps);
  }

  int bus() const { return bus_; }
  int address() const { return address_; }
  double freq_hz() const { return freq_hz_; }
  const std::string &i2c_path() const { return dev_.path(); }

 private:
  void init_chip() {
    write_u8(MODE1, AI);
    write_u8(MODE2, OUTDRV);
  }

  uint8_t read_u8(uint8_t reg) { return dev_.read_u8(reg); }
  void write_u8(uint8_t reg, uint8_t v) { dev_.write_reg(reg, {v}); }

 private:
  static constexpr uint8_t MODE1 = 0x00;
  static constexpr uint8_t MODE2 = 0x01;
  static constexpr uint8_t PRESCALE = 0xFE;
  static constexpr uint8_t LED0_ON_L = 0x06;
  static constexpr uint8_t AI = 0x20;
  static constexpr uint8_t SLEEP = 0x10;
  static constexpr uint8_t RESTART = 0x80;
  static constexpr uint8_t OUTDRV = 0x04;

  int bus_;
  int address_;
  I2CDev dev_;
  double freq_hz_{0.0};
};

struct JointCfg {
  int ch{0};
  int min_us{1000};
  int center_us{1500};
  int max_us{2000};
  bool invert{false};
};

class SpotServoDriverCpp : public rclcpp::Node {
 public:
  SpotServoDriverCpp()
      : Node("spot_servo_driver"),
        servo_map_topic_(getenv_or("SERVO_MAP_TOPIC", "/spot/config/servo_map")),
        cmd_topic_(getenv_or("CMD_TOPIC", "/spot/cmd/servo")),
        status_topic_(getenv_or("STATUS_TOPIC", "/spot/state/servo")),
        pwm_hz_(parse_double(getenv_or("PWM_HZ", "50"), 50.0)),
        update_hz_(parse_double(getenv_or("UPDATE_HZ", "50"), 50.0)),
        max_slew_us_per_s_(parse_double(getenv_or("MAX_SLEW_US_PER_S", "1200"), 1200.0)),
        start_armed_(parse_bool(getenv_or("START_ARMED", "0"), false)),
        disarm_full_off_(parse_bool(getenv_or("DISARM_FULL_OFF", "1"), true)),
        i2c_template_(getenv_or("I2C_DEVICE_TEMPLATE", "/dev/i2c-{bus}")),
        status_publish_seconds_(parse_double(getenv_or("STATUS_PUBLISH_SECONDS", "1.0"), 1.0)) {
    safe_min_us_ = clampv(parse_int(getenv_or("SAFE_MIN_US", "500"), 500), 0, 5000);
    safe_max_us_ = clampv(parse_int(getenv_or("SAFE_MAX_US", "2500"), 2500), 0, 5000);
    if (safe_min_us_ > safe_max_us_) std::swap(safe_min_us_, safe_max_us_);

    armed_ = start_armed_;

    sub_servo_map_ = create_subscription<std_msgs::msg::String>(servo_map_topic_, 10,
      std::bind(&SpotServoDriverCpp::on_servo_map, this, _1));
    sub_cmd_ = create_subscription<std_msgs::msg::String>(cmd_topic_, 10,
      std::bind(&SpotServoDriverCpp::on_cmd, this, _1));
    pub_status_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SpotServoDriverCpp::tick, this));

    last_tick_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(),
      "starting; map=%s cmd=%s pwm_hz=%.1f update_hz=%.1f slew=%.0fus/s armed=%s",
      servo_map_topic_.c_str(), cmd_topic_.c_str(), pwm_hz_, update_hz_, max_slew_us_per_s_, armed_ ? "true" : "false");
  }

  ~SpotServoDriverCpp() override {
    if (disarm_full_off_) set_all_off();
  }

 private:
  void sanitize_triplet(int &min_us, int &center_us, int &max_us) {
    min_us = clampv(min_us, safe_min_us_, safe_max_us_);
    center_us = clampv(center_us, safe_min_us_, safe_max_us_);
    max_us = clampv(max_us, safe_min_us_, safe_max_us_);
    if (min_us > max_us) std::swap(min_us, max_us);
    center_us = clampv(center_us, min_us, max_us);
    if (min_us == max_us) {
      min_us = std::max(safe_min_us_, min_us - 1);
      max_us = std::min(safe_max_us_, max_us + 1);
      center_us = (min_us + max_us) / 2;
    }
    if (center_us == min_us) center_us = min_us + 1;
    if (center_us == max_us) center_us = max_us - 1;
  }

  int norm_to_us(double norm, const JointCfg &cfg) const {
    if (cfg.invert) norm = -norm;
    norm = clampv(norm, -1.0, 1.0);
    double us = cfg.center_us;
    if (norm >= 0.0) us = cfg.center_us + norm * (cfg.max_us - cfg.center_us);
    else us = cfg.center_us + norm * (cfg.center_us - cfg.min_us);
    return static_cast<int>(std::round(clampv(us, static_cast<double>(cfg.min_us), static_cast<double>(cfg.max_us))));
  }

  double us_to_norm(int us, const JointCfg &cfg) const {
    us = clampv(us, cfg.min_us, cfg.max_us);
    double norm = 0.0;
    if (us > cfg.center_us) norm = static_cast<double>(us - cfg.center_us) / std::max(1, cfg.max_us - cfg.center_us);
    else if (us < cfg.center_us) norm = static_cast<double>(us - cfg.center_us) / std::max(1, cfg.center_us - cfg.min_us);
    norm = clampv(norm, -1.0, 1.0);
    return cfg.invert ? -norm : norm;
  }

  void publish_status() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_status_).count();
    if (dt < std::max(0.1, status_publish_seconds_)) return;
    last_status_ = now;

    std::ostringstream js;
    js << "{"
       << "\"armed\":" << (armed_ ? "true" : "false") << ","
       << "\"estop\":" << (estop_ ? "true" : "false") << ","
       << "\"enabled_joints\":[";

    bool first = true;
    for (const auto &j : enabled_joints_) {
      if (!first) js << ',';
      first = false;
      js << '"' << json_escape(j) << '"';
    }

    js << "],\"pca9685\":{";
    js << "\"bus\":" << pca_bus_ << ",\"address\":" << pca_address_ << ",\"freq_hz\":";
    if (pca_) js << pca_->freq_hz(); else js << "null";
    js << "},\"joints\":{";

    first = true;
    for (const auto &kv : joints_) {
      if (!first) js << ',';
      first = false;
      const auto &name = kv.first;
      const auto &cfg = kv.second;
      const double cur = current_us_.count(cfg.ch) ? current_us_[cfg.ch] : cfg.center_us;
      js << '"' << json_escape(name) << "\":{"
         << "\"ch\":" << cfg.ch << ","
         << "\"target_norm\":" << targets_norm_[name] << ","
         << "\"current_us\":" << static_cast<int>(std::round(cur))
         << "}";
    }
    js << "}}";

    std_msgs::msg::String out;
    out.data = js.str();
    pub_status_->publish(out);
  }

  void set_all_off() {
    if (!pca_) return;
    for (int ch = 0; ch < 16; ++ch) {
      try { pca_->set_full_off(ch); } catch (...) {}
    }
  }

  void on_servo_map(const std_msgs::msg::String::SharedPtr msg) {
    const auto raw = msg->data;

    int bus = 1;
    int addr = 0x40;

    std::string n;
    if (extract_number_field(raw, "i2c_bus", n)) bus = parse_int(n, 1);
    if (extract_number_field(raw, "address", n)) addr = parse_int(n, 0x40);

    std::unordered_map<std::string, JointCfg> new_joints;
    // Parse only one-level objects and keep those that explicitly contain joint+ch fields.
    std::regex obj_re("\\{[^\\{\\}]*\\}");
    auto ob = std::sregex_iterator(raw.begin(), raw.end(), obj_re);
    auto oe = std::sregex_iterator();
    for (auto it = ob; it != oe; ++it) {
      const std::string obj = (*it).str();
      std::string joint;
      std::string ch_s;
      if (!extract_string_field(obj, "joint", joint)) continue;
      if (!extract_number_field(obj, "ch", ch_s)) continue;
      int ch = parse_int(ch_s, -1);
      if (ch < 0 || ch > 15 || trim(joint).empty()) continue;

      JointCfg cfg;
      cfg.ch = ch;

      std::string s;
      if (extract_number_field(obj, "min_us", s)) cfg.min_us = parse_int(s, 1000);
      if (extract_number_field(obj, "center_us", s)) cfg.center_us = parse_int(s, 1500);
      if (extract_number_field(obj, "max_us", s)) cfg.max_us = parse_int(s, 2000);
      extract_bool_field(obj, "invert", cfg.invert);
      sanitize_triplet(cfg.min_us, cfg.center_us, cfg.max_us);

      new_joints[trim(joint)] = cfg;
    }

    const bool need_reconnect = (!pca_ || bus != pca_bus_ || addr != pca_address_);
    if (need_reconnect) {
      try {
        pca_ = std::make_unique<PCA9685>(bus, addr, i2c_template_);
        pca_->set_pwm_freq(pwm_hz_);
        pca_bus_ = bus;
        pca_address_ = addr;
        RCLCPP_INFO(get_logger(), "connected to PCA9685 bus=%d address=0x%x (i2c=%s) pwm_hz=%.1f",
                    bus, addr, pca_->i2c_path().c_str(), pwm_hz_);
      } catch (const std::exception &e) {
        pca_.reset();
        pca_bus_ = bus;
        pca_address_ = addr;
        RCLCPP_ERROR(get_logger(), "failed to init PCA9685 (bus=%d addr=0x%x): %s", bus, addr, e.what());
      }
    }

    joints_ = std::move(new_joints);

    std::set<std::string> filtered;
    for (const auto &j : enabled_joints_) if (joints_.count(j)) filtered.insert(j);
    enabled_joints_ = std::move(filtered);

    for (auto it = targets_norm_.begin(); it != targets_norm_.end();) {
      if (!joints_.count(it->first)) it = targets_norm_.erase(it);
      else ++it;
    }
    for (const auto &kv : joints_) {
      if (!targets_norm_.count(kv.first)) targets_norm_[kv.first] = 0.0;
    }

    if (disarm_full_off_ && !armed_) set_all_off();

    std::ostringstream summary;
    bool first = true;
    for (const auto &kv : joints_) {
      if (!first) summary << ", ";
      first = false;
      summary << kv.second.ch << ":" << kv.first;
    }
    RCLCPP_INFO(get_logger(), "servo_map updated; joints=%zu channels=%s", joints_.size(),
                summary.str().empty() ? "(none)" : summary.str().c_str());
  }

  void on_cmd(const std_msgs::msg::String::SharedPtr msg) {
    const auto raw = trim(msg->data);
    if (raw.empty()) return;

    std::string ctype;
    if (!extract_string_field(raw, "cmd", ctype)) {
      RCLCPP_WARN(get_logger(), "unknown cmd: (empty)");
      return;
    }
    std::transform(ctype.begin(), ctype.end(), ctype.begin(), [](unsigned char c) { return std::tolower(c); });

    if (ctype == "arm") {
      bool v = false;
      extract_bool_field(raw, "value", v);
      if (v && estop_) {
        RCLCPP_ERROR(get_logger(), "arm rejected: estop=true (clear estop first)");
        return;
      }
      armed_ = v;
      if (!armed_) enabled_joints_.clear();
      if (disarm_full_off_ && !armed_) set_all_off();
      if (armed_ && enabled_joints_.empty()) RCLCPP_INFO(get_logger(), "armed=true (no enabled joints yet; send cmd=set)");
      else RCLCPP_INFO(get_logger(), "armed=%s", armed_ ? "true" : "false");
      return;
    }

    if (ctype == "estop") {
      bool v = false;
      extract_bool_field(raw, "value", v);
      estop_ = v;
      if (estop_) {
        armed_ = false;
        enabled_joints_.clear();
        if (disarm_full_off_) set_all_off();
        RCLCPP_WARN(get_logger(), "ESTOP engaged; outputs disabled");
      } else {
        enabled_joints_.clear();
        RCLCPP_INFO(get_logger(), "estop cleared; still disarmed");
      }
      return;
    }

    if (ctype == "home") {
      for (auto &kv : targets_norm_) kv.second = 0.0;
      RCLCPP_INFO(get_logger(), "targets set to home (0.0)");
      return;
    }

    if (ctype == "set") {
      std::string mode = "norm";
      extract_string_field(raw, "mode", mode);
      std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return std::tolower(c); });

      std::regex targets_re("\\\"targets\\\"\\s*:\\s*\\{([^}]*)\\}");
      std::smatch tm;
      if (!std::regex_search(raw, tm, targets_re) || tm.size() < 2) {
        RCLCPP_ERROR(get_logger(), "cmd.set: targets must be object");
        return;
      }
      const std::string body = tm[1].str();
      std::regex pair_re("\\\"([^\\\"]+)\\\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");
      auto begin = std::sregex_iterator(body.begin(), body.end(), pair_re);
      auto end = std::sregex_iterator();

      std::vector<std::string> unknown;
      for (auto it = begin; it != end; ++it) {
        const std::string joint = trim((*it)[1].str());
        const double val = parse_double((*it)[2].str(), 0.0);
        if (!joints_.count(joint)) {
          unknown.push_back(joint);
          continue;
        }
        if (mode == "us") {
          const auto &cfg = joints_[joint];
          const int us = parse_int((*it)[2].str(), cfg.center_us);
          targets_norm_[joint] = us_to_norm(us, cfg);
        } else {
          targets_norm_[joint] = clampv(val, -1.0, 1.0);
        }
        enabled_joints_.insert(joint);
      }
      if (!unknown.empty()) {
        std::ostringstream o;
        for (size_t i = 0; i < unknown.size(); ++i) {
          if (i) o << ", ";
          o << unknown[i];
        }
        RCLCPP_WARN(get_logger(), "cmd.set: unknown joints: %s", o.str().c_str());
      }
      return;
    }

    RCLCPP_WARN(get_logger(), "unknown cmd: %s", ctype.c_str());
  }

  void tick() {
    if (!pca_ || joints_.empty() || estop_ || !armed_ || enabled_joints_.empty()) {
      publish_status();
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::max(0.001, std::chrono::duration<double>(now - last_tick_).count());
    last_tick_ = now;

    double max_delta = max_slew_us_per_s_ * dt;
    max_delta = clampv(max_delta, 1.0, 10000.0);

    try {
      for (const auto &joint : enabled_joints_) {
        if (!joints_.count(joint)) continue;
        const auto &cfg = joints_[joint];
        const int target_us = norm_to_us(targets_norm_[joint], cfg);
        const double current = current_us_.count(cfg.ch) ? current_us_[cfg.ch] : static_cast<double>(target_us);
        const double delta = target_us - current;
        double next = current;
        if (std::abs(delta) <= max_delta) next = static_cast<double>(target_us);
        else next = current + std::copysign(max_delta, delta);
        next = clampv(next, static_cast<double>(cfg.min_us), static_cast<double>(cfg.max_us));
        current_us_[cfg.ch] = next;
        pca_->set_pulse_us(cfg.ch, static_cast<int>(std::round(next)));
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "tick failed: %s", e.what());
    }

    publish_status();
  }

 private:
  std::string servo_map_topic_;
  std::string cmd_topic_;
  std::string status_topic_;

  double pwm_hz_{50.0};
  double update_hz_{50.0};
  double max_slew_us_per_s_{1200.0};
  bool start_armed_{false};
  bool disarm_full_off_{true};
  std::string i2c_template_;
  double status_publish_seconds_{1.0};

  int safe_min_us_{500};
  int safe_max_us_{2500};

  bool armed_{false};
  bool estop_{false};

  int pca_bus_{1};
  int pca_address_{0x40};
  std::unique_ptr<PCA9685> pca_;

  std::unordered_map<std::string, JointCfg> joints_;
  std::unordered_map<std::string, double> targets_norm_;
  std::set<std::string> enabled_joints_;
  std::unordered_map<int, double> current_us_;

  std::chrono::steady_clock::time_point last_tick_{};
  std::chrono::steady_clock::time_point last_status_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_servo_map_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpotServoDriverCpp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
