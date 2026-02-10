#pragma once

#include <optional>
#include <string>

#include "spot_node_label_config_cpp/domain/label_parser.hpp"
#include "spot_node_label_config_cpp/ports/labels_source.hpp"
#include "spot_node_label_config_cpp/ports/servo_map_publisher.hpp"

namespace spot::application {

class PublishServoMapUseCase {
 public:
  PublishServoMapUseCase(const spot::domain::ParserConfig& parser_cfg,
                        spot::ports::IServoMapPublisher& publisher)
      : parser_cfg_(parser_cfg), publisher_(publisher) {}

  // Returns updated last_payload
  std::string PublishIfChanged(const spot::ports::Labels& labels,
                              const std::string& last_payload);

 private:
  spot::domain::ParserConfig parser_cfg_;
  spot::ports::IServoMapPublisher& publisher_;
};

}  // namespace spot::application
