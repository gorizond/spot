#include "spot_node_label_config_cpp/application/publish_servo_map_uc.hpp"

#include <sstream>

namespace spot::application {

std::string PublishServoMapUseCase::PublishIfChanged(
    const spot::ports::Labels& labels,
    const std::string& last_payload) {
  spot::domain::Labels dlabels;
  dlabels.reserve(labels.size());
  for (const auto& kv : labels) dlabels.emplace(kv.first, kv.second);

  auto servo_map = spot::domain::ParseServoMapFromLabels(dlabels, parser_cfg_);
  auto payload = spot::domain::ToStableJson(servo_map);

  if (payload == last_payload) {
    return last_payload;
  }

  publisher_.PublishJson(payload);
  return payload;
}

}  // namespace spot::application
