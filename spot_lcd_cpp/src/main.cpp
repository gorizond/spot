#include "spot_lcd_node.h"
#include <getopt.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <sstream>

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
        auto temp_callback = [&display_service](const std::string& temp) {
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
        auto uptime_callback = [&display_service](const std::string& uptime) {
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