#pragma once

#include <functional>
#include <string>
#include <unordered_map>

namespace spot::ports {

using Labels = std::unordered_map<std::string, std::string>;

struct NodeSnapshot {
  std::string resource_version;
  Labels labels;
};

enum class WatchEventType {
  ADDED,
  MODIFIED,
  ERROR,
  UNKNOWN,
};

struct WatchEvent {
  WatchEventType type = WatchEventType::UNKNOWN;
  Labels labels;
  std::string raw;  // for logging/debug
};

class INodeLabelsSource {
 public:
  virtual ~INodeLabelsSource() = default;

  virtual NodeSnapshot GetNodeSnapshot() = 0;

  // Blocks and calls on_event for each watch event.
  virtual void WatchNodeEvents(
      const std::string& resource_version,
      const std::function<void(const WatchEvent&)>& on_event) = 0;
};

}  // namespace spot::ports
