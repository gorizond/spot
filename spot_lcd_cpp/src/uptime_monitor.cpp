#include "spot_lcd_node.h"
#include <fstream>
#include <sstream>
#include <cmath>

UptimeMonitor::UptimeMonitor(Callback callback, const UptimeMonitorConfig& config) 
    : callback_(callback), config_(config) {}

void UptimeMonitor::startMonitoring() {
    running_ = true;
    monitor_thread_ = std::thread([this]() {
        while (running_) {
            std::string uptime = getSystemUptime();
            callback_(uptime);
            std::this_thread::sleep_for(std::chrono::seconds(config_.interval));
        }
    });
}

std::string UptimeMonitor::getSystemUptime() {
    std::ifstream uptimeFile("/proc/uptime");
    if (uptimeFile.is_open()) {
        std::string uptimeStr;
        std::getline(uptimeFile, uptimeStr);
        
        std::istringstream iss(uptimeStr);
        double uptimeSeconds;
        iss >> uptimeSeconds;
        
        int totalSeconds = static_cast<int>(uptimeSeconds);
        int days = totalSeconds / 86400;
        int hours = (totalSeconds % 86400) / 3600;
        int minutes = (totalSeconds % 3600) / 60;
        
        std::ostringstream oss;
        if (days > 0) {
            oss << days << "d " << hours << "h " << minutes << "m";
        } else if (hours > 0) {
            oss << hours << "h " << minutes << "m";
        } else {
            oss << minutes << "m";
        }
        
        return oss.str();
    }
    return "0m"; // значение по умолчанию
}