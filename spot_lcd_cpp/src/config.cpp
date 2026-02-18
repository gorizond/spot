#include "spot_lcd_node.h"
#include <cstdlib>

namespace {
int readEnvInt(const char* name, int fallback) {
    const char* val = std::getenv(name);
    if (!val || !*val) {
        return fallback;
    }
    try {
        return std::stoi(val);
    } catch (...) {
        return fallback;
    }
}
}

Config Config::fromEnv() {
    Config config;

    // LCD pins and geometry (optional overrides)
    config.lcd.pin_rs = readEnvInt("LCD_PIN_RS", config.lcd.pin_rs);
    config.lcd.pin_en = readEnvInt("LCD_PIN_EN", config.lcd.pin_en);
    config.lcd.cols = readEnvInt("LCD_COLS", config.lcd.cols);
    config.lcd.rows = readEnvInt("LCD_ROWS", config.lcd.rows);
    config.lcd.refresh_interval = readEnvInt("LCD_REFRESH_INTERVAL", config.lcd.refresh_interval);
    if (config.lcd.refresh_interval <= 0) {
        config.lcd.refresh_interval = 10;
    }

    // Display line mapping (accepts 0-based or 1-based)
    config.lcd.temp_line = readEnvInt("LCD_TEMP_LINE", config.lcd.temp_line);
    config.lcd.uptime_line = readEnvInt("LCD_UPTIME_LINE", config.lcd.uptime_line);
    if (config.lcd.temp_line >= 1) {
        config.lcd.temp_line -= 1;
    }
    if (config.lcd.uptime_line >= 1) {
        config.lcd.uptime_line -= 1;
    }

    // Monitor intervals
    config.temp_monitor.interval = readEnvInt("TEMP_INTERVAL", config.temp_monitor.interval);
    config.uptime_monitor.interval = readEnvInt("UPTIME_INTERVAL", config.uptime_monitor.interval);

    // Publish layout for temp/uptime (1-based line)
    config.temp_publish.line = readEnvInt("TEMP_PUB_LINE", config.temp_publish.line);
    config.temp_publish.col = readEnvInt("TEMP_PUB_COL", config.temp_publish.col);
    config.temp_publish.width = readEnvInt("TEMP_PUB_WIDTH", config.temp_publish.width);
    config.uptime_publish.line = readEnvInt("UPTIME_PUB_LINE", config.uptime_publish.line);
    config.uptime_publish.col = readEnvInt("UPTIME_PUB_COL", config.uptime_publish.col);
    config.uptime_publish.width = readEnvInt("UPTIME_PUB_WIDTH", config.uptime_publish.width);

    return config;
}