#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace spot::domain {

struct Pca9685Config {
  int i2c_bus = 1;
  int address = 0x40;
};

struct ChannelConfig {
  int ch = 0;
  std::string joint;
  int min_us = 1000;
  int center_us = 1500;
  int max_us = 2000;
  bool invert = false;
};

struct ServoMap {
  Pca9685Config pca9685;
  std::vector<ChannelConfig> channels;
};

}  // namespace spot::domain
