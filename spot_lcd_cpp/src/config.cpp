#include "spot_lcd_node.h"
#include <cstdlib>

Config Config::fromEnv() {
    Config config;
    
    // В реальности здесь будет чтение из env переменных
    const char* pin_rs_env = std::getenv("LCD_PIN_RS");
    if (pin_rs_env) config.lcd.pin_rs = std::stoi(pin_rs_env);
    
    const char* pin_en_env = std::getenv("LCD_PIN_EN");
    if (pin_en_env) config.lcd.pin_en = std::stoi(pin_en_env);
    
    // Аналогично для других параметров...
    
    return config;
}