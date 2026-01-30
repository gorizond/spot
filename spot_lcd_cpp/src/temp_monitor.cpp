#include "spot_lcd_node.h"
#include <fstream>
#include <sstream>

TempMonitor::TempMonitor(Callback callback, const TempMonitorConfig& config) 
    : callback_(callback), config_(config) {}

void TempMonitor::startMonitoring() {
    running_ = true;
    monitor_thread_ = std::thread([this]() {
        while (running_) {
            std::string temp = getCurrentTemperature();
            callback_(temp);
            std::this_thread::sleep_for(std::chrono::seconds(config_.interval));
        }
    });
}

std::string TempMonitor::getCurrentTemperature() {
    // В реальности здесь будет чтение температуры системы
    // Попробуем прочитать из /sys/class/thermal/thermal_zone0/temp на Raspberry Pi
    std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
    if (tempFile.is_open()) {
        std::string tempStr;
        std::getline(tempFile, tempStr);
        try {
            int tempInt = std::stoi(tempStr);
            double tempCelsius = tempInt / 1000.0;
            return std::to_string(tempCelsius) + "°C";
        } catch (...) {
            return "36.5°C"; // значение по умолчанию
        }
    }
    return "36.5°C"; // значение по умолчанию
}