#include <csignal>
#include <chrono>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "spot_node_label_config_cpp/adapters/k8s_labels_source.hpp"
#include "spot_node_label_config_cpp/adapters/ros_string_publisher.hpp"
#include "spot_node_label_config_cpp/application/publish_servo_map_uc.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_stop{false};

static void HandleStop(int) { g_stop.store(true); }

static std::string GetEnv(const char* k, const std::string& def = "") {
  const char* v = std::getenv(k);
  if (!v) return def;
  std::string s(v);
  // trim
  while (!s.empty() && (s.back() == ' ' || s.back() == '\n' || s.back() == '\r' || s.back() == '\t')) s.pop_back();
  size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++;
  s.erase(0, i);
  return s.empty() ? def : s;
}

int main(int argc, char** argv) {
  std::signal(SIGTERM, HandleStop);
  std::signal(SIGINT, HandleStop);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spot_node_label_config_cpp");

  const auto node_name = GetEnv("NODE_NAME");
  const auto label_prefix = GetEnv("LABEL_PREFIX", "gorizond.io/spot-pca9685-");
  const auto topic = GetEnv("TOPIC", "/spot/config/servo_map");

  RCLCPP_INFO(node->get_logger(), "starting; node=%s topic=%s label_prefix=%s",
              node_name.c_str(), topic.c_str(), label_prefix.c_str());

  spot::domain::ParserConfig parser_cfg;
  parser_cfg.label_prefix = label_prefix;

  spot::adapters::RosStringPublisher ros_pub(*node, topic);
  spot::application::PublishServoMapUseCase uc(parser_cfg, ros_pub);

  spot::adapters::K8sConfig kcfg;
  kcfg.base = GetEnv("K8S_BASE", "https://kubernetes.default.svc");
  kcfg.request_timeout_seconds = std::stoi(GetEnv("REQUEST_TIMEOUT_SECONDS", "35"));
  kcfg.watch_timeout_seconds = std::stoi(GetEnv("WATCH_TIMEOUT_SECONDS", "30"));

  spot::adapters::K8sNodeLabelsSource labels_source(kcfg, node_name);

  std::string last_payload;
  int backoff = 1;
  const int backoff_max = std::stoi(GetEnv("BACKOFF_MAX_SECONDS", "30"));

  while (rclcpp::ok() && !g_stop.load()) {
    try {
      auto snap = labels_source.GetNodeSnapshot();
      last_payload = uc.PublishIfChanged(snap.labels, last_payload);
      backoff = 1;

      labels_source.WatchNodeEvents(
          snap.resource_version,
          [&](const spot::ports::WatchEvent& ev) {
            if (g_stop.load() || !rclcpp::ok()) return;
            if (ev.type == spot::ports::WatchEventType::ADDED ||
                ev.type == spot::ports::WatchEventType::MODIFIED) {
              last_payload = uc.PublishIfChanged(ev.labels, last_payload);
            } else if (ev.type == spot::ports::WatchEventType::ERROR) {
              RCLCPP_ERROR(node->get_logger(), "watch error event: %s", ev.raw.c_str());
              throw std::runtime_error("watch error");
            }
          });

    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "k8s watch failed: %s; retry in %ds", e.what(), backoff);
      std::this_thread::sleep_for(std::chrono::seconds(backoff));
      backoff = std::min(backoff * 2, backoff_max);
    }
  }

  rclcpp::shutdown();
  return 0;
}
