#include "spot_lcd_node.h"

#ifdef USE_GPIO
#include <gpiod.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#else
#include <iostream>
#include <unistd.h>
#endif

LcdDisplay::LcdDisplay(const LcdConfig& config) : config_(config) {}

LcdDisplay::~LcdDisplay() {
#ifndef USE_GPIO
    std::cout << "LCD destructor called in simulation mode" << std::endl;
#else
    // Release GPIO lines if they were acquired
    if (chip) {
        for (auto& line : gpio_lines) {
            if (line) {
                gpiod_line_release(line);
            }
        }
        gpiod_chip_close(chip);
    }
#endif
}

void LcdDisplay::pulseEnable() {
#ifdef USE_GPIO
    // Set EN pin HIGH
    if (gpiod_line_set_value(gpio_lines[1], 1) < 0) {
        std::cerr << "Failed to set EN HIGH" << std::endl;
    }
    usleep(1000); // Enable pulse > 450ns
    // Set EN pin LOW
    if (gpiod_line_set_value(gpio_lines[1], 0) < 0) {
        std::cerr << "Failed to set EN LOW" << std::endl;
    }
    usleep(1000); // > 450ns
#endif
}

void LcdDisplay::sendNibble(uint8_t data) {
#ifdef USE_GPIO
    // Устанавливаем значения на пинах данных
    for (int i = 0; i < 4; i++) {
        int value = (data >> i) & 0x01;
        if (gpiod_line_set_value(gpio_lines[2+i], value) < 0) {
            std::cerr << "Failed to set data pin " << i << " to " << value << std::endl;
        }
    }
    
    pulseEnable();
#endif
}

void LcdDisplay::sendByte(uint8_t data, bool rs) {
#ifdef USE_GPIO
    // Устанавливаем RS
    if (gpiod_line_set_value(gpio_lines[0], rs ? 1 : 0) < 0) {
        std::cerr << "Failed to set RS to " << (rs ? "HIGH" : "LOW") << std::endl;
    }
    
    // Отправляем старшие 4 бита
    sendNibble((data >> 4) & 0x0F);
    
    // Отправляем младшие 4 бита
    sendNibble(data & 0x0F);
    
    usleep(100); // Small delay between commands/data
#endif
}

bool LcdDisplay::setCursorPosition(uint8_t row, uint8_t col) {
#ifdef USE_GPIO
    uint8_t pos = 0;
    switch(row) {
        case 0: pos = 0x00; break;
        case 1: pos = 0x40; break;
        case 2: pos = 0x14; break;
        case 3: pos = 0x54; break;
    }
    sendByte(0x80 | (pos + col), false); // Set DDRAM address
#endif
    return true;
}

bool LcdDisplay::initialize() {
#ifndef USE_GPIO
    std::cout << "LCD initialized in simulation mode" << std::endl;
    initialized_ = true;
#else
    // Используем libgpiod для доступа к GPIO
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        
        // Пробуем другие возможные имена чипа
        chip = gpiod_chip_open_by_name("pinctrl-bcm2835");  // Для Raspberry Pi
        if (!chip) {
            std::cerr << "Failed to open GPIO chip (both gpiochip0 and pinctrl-bcm2835)" << std::endl;
            std::cerr << "Continuing in simulation mode..." << std::endl;
            initialized_ = true;
            return true;
        }
    }

    // Получаем линии для всех GPIO пинов
    // gpio_lines[0] = RS, gpio_lines[1] = EN, gpio_lines[2..5] = data pins
    gpio_lines[0] = gpiod_chip_get_line(chip, config_.pin_rs);  // RS
    gpio_lines[1] = gpiod_chip_get_line(chip, config_.pin_en);  // EN
    gpio_lines[2] = gpiod_chip_get_line(chip, config_.pins_data[0]); // D4
    gpio_lines[3] = gpiod_chip_get_line(chip, config_.pins_data[1]); // D5
    gpio_lines[4] = gpiod_chip_get_line(chip, config_.pins_data[2]); // D6
    gpio_lines[5] = gpiod_chip_get_line(chip, config_.pins_data[3]); // D7

    // Проверяем, что все линии успешно получены
    for (int i = 0; i < 6; i++) {
        if (!gpio_lines[i]) {
            std::cerr << "Failed to get GPIO line for pin " << i << std::endl;
            // Закрываем уже открытые линии
            for (int j = 0; j < i; j++) {
                if (gpio_lines[j]) {
                    gpiod_line_release(gpio_lines[j]);
                }
            }
            gpiod_chip_close(chip);
            std::cerr << "Continuing in simulation mode..." << std::endl;
            initialized_ = true;
            return true;
        }
    }

    // Запрашиваем права на управление линиями (OUTPUT)
    for (int i = 0; i < 6; i++) {
        if (gpiod_line_request_output(gpio_lines[i], "spot-lcd", 0) < 0) {
            std::cerr << "Failed to request output access for GPIO line " << i << std::endl;
            // Закрываем уже открытые линии
            for (int j = 0; j <= i; j++) {
                if (gpio_lines[j]) {
                    gpiod_line_release(gpio_lines[j]);
                }
            }
            gpiod_chip_close(chip);
            std::cerr << "Continuing in simulation mode..." << std::endl;
            initialized_ = true;
            return true;
        }
    }

    std::cout << "LCD initialized with libgpiod" << std::endl;

    // Устанавливаем начальное состояние
    if (gpiod_line_set_value(gpio_lines[0], 0) < 0) {  // RS = LOW
        std::cerr << "Failed to set initial RS state" << std::endl;
    }
    if (gpiod_line_set_value(gpio_lines[1], 0) < 0) {  // EN = LOW
        std::cerr << "Failed to set initial EN state" << std::endl;
    }
    
    for (int i = 2; i < 6; i++) {
        if (gpiod_line_set_value(gpio_lines[i], 0) < 0) {  // Data pins = LOW
            std::cerr << "Failed to set initial data pin state" << std::endl;
        }
    }

    // Задержка для стабилизации
    usleep(50000); // 50ms delay according to HD44780 spec

    // Инициализационная последовательность для 4-битного режима
    // Step 1: Send 0x30 three times with delays
    sendNibble(0x3);
    usleep(5000); // 4.1ms minimum delay
    
    sendNibble(0x3);
    usleep(5000); // 4.1ms minimum delay
    
    sendNibble(0x3);
    usleep(1500); // 100us minimum delay
    
    // Step 2: Set to 4-bit mode
    sendNibble(0x2);
    usleep(1500);

    // Now we can send 8-bit commands using sendByte
    sendByte(0x28, false); // Function set: 4-bit, 2 lines, 5x8 dots
    usleep(1500);
    
    sendByte(0x0C, false); // Display on, cursor off, blink off
    usleep(1500);
    
    sendByte(0x06, false); // Entry mode set: increment, no shift
    usleep(1500);
    
    sendByte(0x01, false); // Clear display
    usleep(2000); // Clear command takes longer

    initialized_ = true;
#endif
    return initialized_;
}

bool LcdDisplay::clear() {
#ifndef USE_GPIO
    std::cout << "LCD cleared (simulation)" << std::endl;
#else
    std::cout << "LCD cleared" << std::endl;
#endif
    sendByte(0x01, false); // Clear display command
    usleep(2000); // Clear command takes longer
    return true;
}

bool LcdDisplay::writeLine(int line, const std::string& text) {
#ifndef USE_GPIO
    std::cout << "LCD line " << line << ": " << text << std::endl;
#else
    // Устанавливаем курсор в начало строки
    setCursorPosition(line, 0);
    
    // Отправляем каждый символ
    for (char c : text) {
        sendByte(c, true); // true означает, что это данные, а не команда
    }
    
    // Если текст короче, чем ширина экрана, заполняем пробелами
    int remaining = config_.cols - text.length();
    if (remaining > 0) {
        for (int i = 0; i < remaining; i++) {
            sendByte(' ', true);
        }
    }
#endif
    return true;
}

bool LcdDisplay::writeChar(char c) {
#ifndef USE_GPIO
    std::cout << c << std::flush;
#else
    sendByte(c, true);
#endif
    return true;
}

#ifdef USE_ROS2
RosPublisher::RosPublisher(rclcpp::Node::SharedPtr node) : node_(node) {
    temp_publisher_ = node_->create_publisher<std_msgs::msg::String>("lcd_temperature", 10);
    uptime_publisher_ = node_->create_publisher<std_msgs::msg::String>("lcd_uptime", 10);
}

void RosPublisher::publishTemperature(const std::string& temp) {
    auto msg = std_msgs::msg::String();
    msg.data = temp;
    temp_publisher_->publish(msg);
}

void RosPublisher::publishUptime(const std::string& uptime) {
    auto msg = std_msgs::msg::String();
    msg.data = uptime;
    uptime_publisher_->publish(msg);
}
#endif