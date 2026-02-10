#pragma once

#include <string>

namespace spot::ports {

class IServoMapPublisher {
 public:
  virtual ~IServoMapPublisher() = default;
  virtual void PublishJson(const std::string& json) = 0;
};

}  // namespace spot::ports
