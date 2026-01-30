#include "spot_lcd_node.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

#ifdef __APPLE__
#include <sys/sysctl.h>
#include <time.h>
#endif

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
    // Попробуем получить аптайм для Linux
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
#ifdef __APPLE__
    // Для macOS используем sysctl
    else {
        struct timeval boottime;
        size_t len = sizeof(boottime);
        int mib[2] = {CTL_KERN, KERN_BOOTTIME};
        
        if (sysctl(mib, 2, &boottime, &len, NULL, 0) == 0) {
            time_t bsec = boottime.tv_sec;
            time_t current = time(NULL);
            int totalSeconds = static_cast<int>(current - bsec);
            
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
    }
#endif
    
    // Значение по умолчанию
    return "0m";
}