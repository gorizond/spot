#include "spot_node_label_config_cpp/domain/label_parser.hpp"

#include <algorithm>
#include <cctype>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace spot::domain {

static int ParseInt(const std::string& raw, int default_value) {
  try {
    size_t idx = 0;
    int base = 10;
    std::string s = raw;
    // trim
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isspace(c); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c) { return !std::isspace(c); }).base(), s.end());
    if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
      base = 16;
    }
    int v = std::stoi(s, &idx, base);
    (void)idx;
    return v;
  } catch (...) {
    return default_value;
  }
}

static std::optional<std::tuple<int, int, int>> ParseUsTriplet(const std::string& raw) {
  // Match python: re.findall(r"\d+", raw)
  std::regex re("(\\d+)");
  std::sregex_iterator it(raw.begin(), raw.end(), re);
  std::sregex_iterator end;
  std::vector<int> nums;
  for (; it != end; ++it) {
    try {
      nums.push_back(std::stoi((*it)[1].str()));
    } catch (...) {
    }
  }
  if (nums.size() < 3) return std::nullopt;
  return std::make_tuple(nums[0], nums[1], nums[2]);
}

static bool ParseBool(const std::string& raw) {
  std::string s;
  s.reserve(raw.size());
  for (char c : raw) s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  // trim spaces
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isspace(c); }));
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c) { return !std::isspace(c); }).base(), s.end());
  return (s == "1" || s == "true" || s == "yes" || s == "y" || s == "on");
}

static std::string GetLabel(const Labels& labels, const std::string& key) {
  auto it = labels.find(key);
  return it == labels.end() ? std::string() : it->second;
}

ServoMap ParseServoMapFromLabels(const Labels& labels, const ParserConfig& cfg) {
  ServoMap out;
  out.pca9685.i2c_bus = ParseInt(GetLabel(labels, cfg.label_prefix + "i2c-bus"), 1);
  out.pca9685.address = ParseInt(GetLabel(labels, cfg.label_prefix + "address"), 0x40);

  out.channels.reserve(16);
  for (int ch = 0; ch < 16; ch++) {
    ChannelConfig c;
    c.ch = ch;

    {
      std::ostringstream k;
      k << cfg.label_prefix << "ch" << ch << "-joint";
      c.joint = GetLabel(labels, k.str());
      // trim
      c.joint.erase(c.joint.begin(), std::find_if(c.joint.begin(), c.joint.end(), [](unsigned char x) { return !std::isspace(x); }));
      c.joint.erase(std::find_if(c.joint.rbegin(), c.joint.rend(), [](unsigned char x) { return !std::isspace(x); }).base(), c.joint.end());
    }

    const int def_min = 1000, def_center = 1500, def_max = 2000;

    std::string raw_triplet;
    {
      std::ostringstream k;
      k << cfg.label_prefix << "ch" << ch << "-us";
      raw_triplet = GetLabel(labels, k.str());
    }

    auto triplet = raw_triplet.empty() ? std::nullopt : ParseUsTriplet(raw_triplet);
    if (triplet) {
      c.min_us = std::get<0>(*triplet);
      c.center_us = std::get<1>(*triplet);
      c.max_us = std::get<2>(*triplet);
    } else {
      {
        std::ostringstream k;
        k << cfg.label_prefix << "ch" << ch << "-min-us";
        c.min_us = ParseInt(GetLabel(labels, k.str()), def_min);
      }
      {
        std::ostringstream k;
        k << cfg.label_prefix << "ch" << ch << "-center-us";
        c.center_us = ParseInt(GetLabel(labels, k.str()), def_center);
      }
      {
        std::ostringstream k;
        k << cfg.label_prefix << "ch" << ch << "-max-us";
        c.max_us = ParseInt(GetLabel(labels, k.str()), def_max);
      }
    }

    {
      std::ostringstream k;
      k << cfg.label_prefix << "ch" << ch << "-invert";
      std::string raw_inv = GetLabel(labels, k.str());
      c.invert = ParseBool(raw_inv.empty() ? "0" : raw_inv);
    }

    out.channels.push_back(std::move(c));
  }

  return out;
}

static std::string JsonEscape(const std::string& s) {
  std::ostringstream o;
  for (char c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\b': o << "\\b"; break;
      case '\f': o << "\\f"; break;
      case '\n': o << "\\n"; break;
      case '\r': o << "\\r"; break;
      case '\t': o << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          o << "\\u";
          o << std::hex << std::uppercase;
          o.width(4);
          o.fill('0');
          o << (int)(unsigned char)c;
          o << std::dec;
        } else {
          o << c;
        }
    }
  }
  return o.str();
}

std::string ToStableJson(const ServoMap& map) {
  // Stable key order by construction.
  std::ostringstream o;
  o << "{\"channels\":[";
  for (size_t i = 0; i < map.channels.size(); i++) {
    const auto& c = map.channels[i];
    if (i) o << ',';
    o << "{\"center_us\":" << c.center_us
      << ",\"ch\":" << c.ch
      << ",\"invert\":" << (c.invert ? "true" : "false")
      << ",\"joint\":\"" << JsonEscape(c.joint) << "\""
      << ",\"max_us\":" << c.max_us
      << ",\"min_us\":" << c.min_us
      << '}';
  }
  o << "],\"pca9685\":{\"address\":" << map.pca9685.address
    << ",\"i2c_bus\":" << map.pca9685.i2c_bus
    << "}}";
  return o.str();
}

}  // namespace spot::domain
