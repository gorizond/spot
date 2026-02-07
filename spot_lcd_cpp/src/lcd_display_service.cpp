#include "spot_lcd_node.h"
#include <algorithm>

DisplayService::DisplayService(const LcdConfig& config) : config_(config) {
    data_mutex_ = std::make_shared<std::mutex>();

    lines_.assign(config_.rows, std::string(config_.cols, ' '));
    
    lcd_display_ = std::make_unique<LcdDisplay>(config_);
    lcd_display_->initialize();
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

            // Обновляем LCD дисплей каждую секунду
            if (lcd_display_) {
                lcd_display_->clear();
                for (int i = 0; i < config_.rows; ++i) {
                    lcd_display_->writeLine(i, lines_[i]);
                }
            }
        }
    });
}

std::shared_ptr<std::mutex> DisplayService::getLcdReference() {
    return data_mutex_;
}

namespace {
std::string normalizeLine(const std::string& text, int width) {
    if (width <= 0) {
        return "";
    }
    if ((int)text.size() >= width) {
        return text.substr(0, width);
    }
    return text + std::string(width - text.size(), ' ');
}
}

void DisplayService::setLineText(int line, const std::string& text) {
    if (line < 0 || line >= config_.rows) {
        return;
    }
    lines_[line] = normalizeLine(text, config_.cols);
}

void DisplayService::setSegment(int line, int col, int width, const std::string& text) {
    if (line < 0 || line >= config_.rows || col < 0 || col >= config_.cols) {
        return;
    }
    int max_width = std::min(width, config_.cols - col);
    if (max_width <= 0) {
        return;
    }
    auto segment = normalizeLine(text, max_width);
    lines_[line].replace(col, max_width, segment);
}

void DisplayService::setData(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lock(*data_mutex_);
    if (key == "temperature") {
        setLineText(config_.temp_line, "Temp: " + value);
    } else if (key == "uptime") {
        setLineText(config_.uptime_line, "Up: " + value);
    }
}
