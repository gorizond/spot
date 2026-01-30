#include "spot_lcd_node.h"

DisplayService::DisplayService(const LcdConfig& config) : config_(config) {
    data_mutex_ = std::make_shared<std::mutex>();
    
#ifndef USE_GPIO
    lcd_display_ = std::make_unique<LcdDisplay>(config_);
    lcd_display_->initialize();
#endif
}

DisplayService::~DisplayService() {
    if (running_) {
        running_ = false;
        if (display_thread_.joinable()) {
            display_thread_.join();
        }
    }
}

void DisplayService::startDisplayLoop() {
    running_ = true;
    display_thread_ = std::thread([this]() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::lock_guard<std::mutex> lock(*data_mutex_);
            auto temp_it = data_.find("temperature");
            auto uptime_it = data_.find("uptime");
            
            std::string temp = temp_it != data_.end() ? temp_it->second : "N/A";
            std::string uptime = uptime_it != data_.end() ? uptime_it->second : "N/A";
            
            std::cout << "LCD Display - Temp: " << temp << ", Uptime: " << uptime << std::endl;
        }
    });
}

std::shared_ptr<std::mutex> DisplayService::getLcdReference() {
    return data_mutex_;
}