#include "spot_lcd_node.h"
#include <getopt.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>

Config Config::fromEnv() {
    Config config;
    // В реальности здесь будет чтение из env переменных
    return config;
}

LcdDisplay::LcdDisplay(const LcdConfig& config) : config_(config) {}

LcdDisplay::~LcdDisplay() {}

bool LcdDisplay::initialize() {
#ifndef USE_GPIO
    std::cout << "LCD initialized in simulation mode" << std::endl;
    initialized_ = true;
#else
    // Инициализация GPIO и LCD
    // Это будет работать только на Raspberry Pi
    std::cout << "LCD initialized in GPIO mode" << std::endl;
    initialized_ = true;
#endif
    return initialized_;
}

bool LcdDisplay::clear() {
#ifndef USE_GPIO
    std::cout << "LCD cleared (simulation)" << std::endl;
#else
    // Очистка дисплея через GPIO
    std::cout << "LCD cleared" << std::endl;
#endif
    return true;
}

bool LcdDisplay::writeLine(int line, const std::string& text) {
#ifndef USE_GPIO
    std::cout << "LCD line " << line << ": " << text << std::endl;
#else
    // Запись строки на дисплей через GPIO
    std::cout << "LCD line " << line << ": " << text << std::endl;
#endif
    return true;
}

bool LcdDisplay::writeChar(char c) {
#ifndef USE_GPIO
    std::cout << "LCD char: " << c << std::endl;
#else
    // Запись символа на дисплей через GPIO
    std::cout << c << std::flush;
#endif
    return true;
}

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
    return "36.5°C";
}

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
    // В реальности здесь будет чтение аптайма системы
    return "2d 5h 32m";
}

int main(int argc, char *argv[]) {
    std::string modules = "all";
    
    int opt;
    while ((opt = getopt(argc, argv, "m:")) != -1) {
        switch (opt) {
            case 'm':
                modules = optarg;
                break;
            default:
                std::cerr << "Usage: " << argv[0] << " [-m modules]" << std::endl;
                std::cerr << "Modules: all, lcd, temp, uptime (comma separated)" << std::endl;
                return 1;
        }
    }
    
    std::cout << "Starting LCD service modules: " << modules << std::endl;
    
    std::vector<std::string> module_list;
    std::stringstream ss(modules);
    std::string item;
    while (std::getline(ss, item, ',')) {
        // Убираем пробелы
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);
        module_list.push_back(item);
    }
    
    bool run_all = std::find(module_list.begin(), module_list.end(), "all") != module_list.end();
    bool run_lcd = run_all || std::find(module_list.begin(), module_list.end(), "lcd") != module_list.end();
    bool run_temp = run_all || std::find(module_list.begin(), module_list.end(), "temp") != module_list.end();
    bool run_uptime = run_all || std::find(module_list.begin(), module_list.end(), "uptime") != module_list.end();
    
    Config config = Config::fromEnv();
    
    std::unique_ptr<DisplayService> display_service;
    
    if (run_lcd) {
        display_service = std::make_unique<DisplayService>(config.lcd);
        display_service->startDisplayLoop();
    }
    
    if (run_temp) {
        auto temp_callback = [display_service](const std::string& temp) {
            if (display_service) {
                display_service->setData("temperature", temp);
                std::cout << "Temperature: " << temp << std::endl;
            } else {
                std::cout << "Temperature: " << temp << std::endl;
            }
        };
        
        TempMonitor temp_monitor(temp_callback, config.temp_monitor);
        temp_monitor.startMonitoring();
    }
    
    if (run_uptime) {
        auto uptime_callback = [display_service](const std::string& uptime) {
            if (display_service) {
                display_service->setData("uptime", uptime);
                std::cout << "Uptime: " << uptime << std::endl;
            } else {
                std::cout << "Uptime: " << uptime << std::endl;
            }
        };
        
        UptimeMonitor uptime_monitor(uptime_callback, config.uptime_monitor);
        uptime_monitor.startMonitoring();
    }
    
    // Ждем бесконечно
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    return 0;
}