#include "spot_lcd_node.h"

#ifdef USE_GPIO
#include <wiringPi.h>
#include <unistd.h>
#else
#include <iostream>
#include <unistd.h>
#endif

LcdDisplay::LcdDisplay(const LcdConfig& config) : config_(config) {}

LcdDisplay::~LcdDisplay() {}

void LcdDisplay::pulseEnable() {
#ifdef USE_GPIO
    digitalWrite(config_.pin_en, HIGH);
    usleep(1); // Enable pulse > 450ns
    digitalWrite(config_.pin_en, LOW);
    usleep(1); // > 450ns
#endif
}

void LcdDisplay::sendNibble(uint8_t data) {
#ifdef USE_GPIO
    // Устанавливаем значения на пинах данных
    for (int i = 0; i < 4; i++) {
        int pin = config_.pins_data[i];
        int value = (data >> i) & 0x01;
        digitalWrite(pin, value);
    }
    
    pulseEnable();
#endif
}

void LcdDisplay::sendByte(uint8_t data, bool rs) {
#ifdef USE_GPIO
    // Устанавливаем RS
    digitalWrite(config_.pin_rs, rs ? HIGH : LOW);
    
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
    // Используем системный режим wiringPi, который работает в контейнере
    // Этот режим позволяет обращаться к GPIO без определения версии платы
    if (wiringPiSetupSys() == -1) {
        std::cerr << "Failed to initialize WiringPi in sys mode" << std::endl;
        // Пробуем включить эмуляцию, если невозможно получить доступ к GPIO
        std::cerr << "Trying to continue in simulation mode..." << std::endl;
        initialized_ = true;
        return true; // Продолжаем работу в эмуляционном режиме
    }

    std::cout << "LCD initialized in GPIO sys mode" << std::endl;

    // Устанавливаем GPIO пины как выходы
    pinMode(config_.pin_rs, OUTPUT);
    pinMode(config_.pin_en, OUTPUT);
    
    for (int pin : config_.pins_data) {
        pinMode(pin, OUTPUT);
    }

    // Устанавливаем начальное состояние
    digitalWrite(config_.pin_rs, LOW);
    digitalWrite(config_.pin_en, LOW);
    
    for (int pin : config_.pins_data) {
        digitalWrite(pin, LOW);
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