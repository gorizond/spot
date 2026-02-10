#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "spot_node_label_config_cpp/domain/servo_map.hpp"

namespace spot::domain {

using Labels = std::unordered_map<std::string, std::string>;

struct ParserConfig {
  std::string label_prefix = "gorizond.io/spot-pca9685-";
};

ServoMap ParseServoMapFromLabels(const Labels& labels, const ParserConfig& cfg);

// Stable JSON for downstream python components: sort_keys + compact separators.
std::string ToStableJson(const ServoMap& map);

}  // namespace spot::domain
