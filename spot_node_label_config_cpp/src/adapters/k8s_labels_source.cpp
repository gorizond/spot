#include "spot_node_label_config_cpp/adapters/k8s_labels_source.hpp"

#include <curl/curl.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace spot::adapters {

static std::string ReadFileTrim(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read file: " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  // trim
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\n' || s.front() == '\r')) s.erase(s.begin());
  return s;
}

static size_t WriteToString(void* contents, size_t size, size_t nmemb, void* userp) {
  auto* s = static_cast<std::string*>(userp);
  size_t total = size * nmemb;
  s->append(static_cast<char*>(contents), total);
  return total;
}

struct WatchCtx {
  std::string buf;
  std::function<void(const spot::ports::WatchEvent&)> on_event;
  std::atomic<bool>* stop;
};

static std::string ExtractStringField(const std::string& json, const std::string& key) {
  // Minimal JSON extractor for known fields: "key":"value".
  // Good enough for: metadata.resourceVersion and type.
  std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch m;
  if (std::regex_search(json, m, re) && m.size() >= 2) {
    return m[1].str();
  }
  return {};
}

static spot::ports::Labels ExtractLabelsObject(const std::string& json) {
  // Extremely small parser for labels object: "labels":{ "k":"v", ... }
  // This assumes no nested objects inside labels.
  spot::ports::Labels out;

  auto pos = json.find("\"labels\"");
  if (pos == std::string::npos) return out;
  pos = json.find('{', pos);
  if (pos == std::string::npos) return out;

  int depth = 0;
  size_t start = pos;
  size_t end = std::string::npos;
  for (size_t i = pos; i < json.size(); i++) {
    if (json[i] == '{') depth++;
    else if (json[i] == '}') {
      depth--;
      if (depth == 0) {
        end = i;
        break;
      }
    }
  }
  if (end == std::string::npos || end <= start) return out;

  std::string obj = json.substr(start + 1, end - start - 1);

  // parse "k":"v" pairs (skip non-string values)
  std::regex pair_re("\\\"([^\\\"]+)\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  auto it = std::sregex_iterator(obj.begin(), obj.end(), pair_re);
  auto ed = std::sregex_iterator();
  for (; it != ed; ++it) {
    out[(*it)[1].str()] = (*it)[2].str();
  }
  return out;
}

static spot::ports::NodeSnapshot ParseNodeSnapshot(const std::string& body) {
  spot::ports::NodeSnapshot snap;
  snap.resource_version = ExtractStringField(body, "resourceVersion");
  snap.labels = ExtractLabelsObject(body);
  return snap;
}

static spot::ports::WatchEventType ParseEventType(const std::string& t) {
  if (t == "ADDED") return spot::ports::WatchEventType::ADDED;
  if (t == "MODIFIED") return spot::ports::WatchEventType::MODIFIED;
  if (t == "ERROR") return spot::ports::WatchEventType::ERROR;
  return spot::ports::WatchEventType::UNKNOWN;
}

static size_t WatchWriteCb(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* ctx = static_cast<WatchCtx*>(userdata);
  if (ctx->stop && ctx->stop->load()) {
    return 0;  // abort transfer
  }

  size_t total = size * nmemb;
  ctx->buf.append(ptr, total);

  // Split by lines; k8s watch is line-delimited JSON
  for (;;) {
    auto nl = ctx->buf.find('\n');
    if (nl == std::string::npos) break;
    std::string line = ctx->buf.substr(0, nl);
    ctx->buf.erase(0, nl + 1);

    // trim
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i) line.erase(0, i);
    if (line.empty()) continue;

    spot::ports::WatchEvent ev;
    ev.raw = line;
    auto t = ExtractStringField(line, "type");
    ev.type = ParseEventType(t);

    // object.metadata.labels are in the same line; we reuse the labels extractor
    ev.labels = ExtractLabelsObject(line);

    if (ctx->on_event) ctx->on_event(ev);
  }

  return total;
}

K8sNodeLabelsSource::K8sNodeLabelsSource(const K8sConfig& cfg, const std::string& node_name)
    : cfg_(cfg), node_name_(node_name) {
  if (node_name_.empty()) throw std::runtime_error("NODE_NAME is required");
  curl_global_init(CURL_GLOBAL_DEFAULT);
}

spot::ports::NodeSnapshot K8sNodeLabelsSource::GetNodeSnapshot() {
  const auto token = ReadFileTrim(cfg_.sa_token_path);

  std::string url = cfg_.base + "/api/v1/nodes/" + node_name_;

  CURL* curl = curl_easy_init();
  if (!curl) throw std::runtime_error("curl_easy_init failed");

  std::string body;
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, ("Authorization: Bearer " + token).c_str());
  headers = curl_slist_append(headers, "Accept: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, cfg_.request_timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_CAINFO, cfg_.sa_ca_path.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

  CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (rc != CURLE_OK) {
    throw std::runtime_error(std::string("k8s GET failed: ") + curl_easy_strerror(rc));
  }
  if (http_code < 200 || http_code >= 300) {
    throw std::runtime_error("k8s GET http " + std::to_string(http_code) + ": " + body);
  }

  auto snap = ParseNodeSnapshot(body);
  if (snap.resource_version.empty()) {
    throw std::runtime_error("node resourceVersion is empty");
  }
  return snap;
}

void K8sNodeLabelsSource::WatchNodeEvents(
    const std::string& resource_version,
    const std::function<void(const spot::ports::WatchEvent&)>& on_event) {
  const auto token = ReadFileTrim(cfg_.sa_token_path);

  // watch=true&fieldSelector=metadata.name=...&resourceVersion=...&timeoutSeconds=...
  auto esc = [](const std::string& s) {
    std::ostringstream o;
    for (unsigned char c : s) {
      if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' ) {
        o << c;
      } else {
        o << '%' << std::hex << std::uppercase << (int)c << std::nouppercase << std::dec;
      }
    }
    return o.str();
  };

  std::string url = cfg_.base + "/api/v1/nodes?watch=true";
  url += "&fieldSelector=" + esc("metadata.name=" + node_name_);
  url += "&resourceVersion=" + esc(resource_version);
  url += "&timeoutSeconds=" + std::to_string(cfg_.watch_timeout_seconds);

  CURL* curl = curl_easy_init();
  if (!curl) throw std::runtime_error("curl_easy_init failed");

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, ("Authorization: Bearer " + token).c_str());
  headers = curl_slist_append(headers, "Accept: application/json");

  WatchCtx ctx;
  ctx.on_event = on_event;
  ctx.stop = &stop_;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WatchWriteCb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ctx);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, cfg_.request_timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_CAINFO, cfg_.sa_ca_path.c_str());
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
  curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);

  CURLcode rc = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (stop_.load()) {
    return;
  }
  if (rc != CURLE_OK) {
    throw std::runtime_error(std::string("k8s watch failed: ") + curl_easy_strerror(rc));
  }
  if (http_code < 200 || http_code >= 300) {
    throw std::runtime_error("k8s watch http " + std::to_string(http_code));
  }
}

void K8sNodeLabelsSource::RequestStop() { stop_.store(true); }

}  // namespace spot::adapters
