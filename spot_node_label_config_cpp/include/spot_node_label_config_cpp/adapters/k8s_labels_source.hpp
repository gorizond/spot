#pragma once

#include <atomic>
#include <string>

#include "spot_node_label_config_cpp/ports/labels_source.hpp"

namespace spot::adapters {

struct K8sConfig {
  std::string base = "https://kubernetes.default.svc";
  std::string sa_token_path = "/var/run/secrets/kubernetes.io/serviceaccount/token";
  std::string sa_ca_path = "/var/run/secrets/kubernetes.io/serviceaccount/ca.crt";
  int request_timeout_seconds = 35;
  int watch_timeout_seconds = 30;
};

class K8sNodeLabelsSource : public spot::ports::INodeLabelsSource {
 public:
  K8sNodeLabelsSource(const K8sConfig& cfg, const std::string& node_name);

  spot::ports::NodeSnapshot GetNodeSnapshot() override;

  void WatchNodeEvents(
      const std::string& resource_version,
      const std::function<void(const spot::ports::WatchEvent&)>& on_event) override;

  void RequestStop();

 private:
  K8sConfig cfg_;
  std::string node_name_;
  std::atomic<bool> stop_{false};
};

}  // namespace spot::adapters
