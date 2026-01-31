#include <iostream>
#include <string>
#include "spot_lcd_cpp/include/spot_lcd_node.h"

int main() {
    std::cout << "Тестирование обновленного LCD C++ приложения с GPIO поддержкой" << std::endl;
    
    // Создаем конфигурацию с правильными пинами для LCD
    LcdConfig config;
    config.pin_rs = 25;
    config.pin_en = 24;
    config.pins_data = {23, 17, 18, 22};
    config.cols = 16;
    config.rows = 2;
    
    // Создаем экземпляр дисплея
    LcdDisplay display(config);
    
    std::cout << "Инициализация дисплея..." << std::endl;
    
    // Пытаемся инициализировать дисплей
    if (display.initialize()) {
        std::cout << "Дисплей успешно инициализирован!" << std::endl;
        
        // Тестируем вывод на дисплей
        std::cout << "Тестирование вывода на дисплей..." << std::endl;
        display.writeLine(0, "Test: GPIO OK");
        display.writeLine(1, "Temp: 36.6C");
        
        std::cout << "Тест завершен успешно!" << std::endl;
    } else {
        std::cout << "Ошибка инициализации дисплея!" << std::endl;
        return 1;
    }
    
    return 0;
}