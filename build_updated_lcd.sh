#!/bin/bash
# Скрипт для сборки обновленного LCD C++ приложения с GPIO поддержкой

echo "=== Сборка обновленного LCD C++ приложения ==="

# Определяем операционную систему
OS="$(uname)"
echo "Операционная система: $OS"

# Переходим в директорию проекта
cd /Users/negash/Yandex.Disk.localized/Projects/gorizond/spot

# Создаем директорию для сборки
BUILD_DIR="/Users/negash/Yandex.Disk.localized/Projects/gorizond/spot/spot_lcd_cpp/build"
if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Определяем количество ядер для параллельной сборки
if [ "$OS" = "Linux" ]; then
    CORES=$(nproc)
    # Проверяем, установлен ли wiringPi
    if ! command -v gpio &> /dev/null; then
        echo "Устанавливаем wiringPi..."
        sudo apt-get update
        sudo apt-get install -y wiringpi
    fi
    # На Linux включаем GPIO поддержку
    echo "Конфигурируем проект с GPIO поддержкой для Linux..."
    cmake .. -DENABLE_GPIO=ON -DENABLE_ROS2=OFF
elif [ "$OS" = "Darwin" ]; then
    # На macOS отключаем GPIO поддержку (она не работает на macOS)
    echo "Конфигурируем проект без GPIO поддержки для macOS..."
    cmake .. -DENABLE_GPIO=OFF -DENABLE_ROS2=OFF
else
    # На других системах отключаем GPIO поддержку
    echo "Конфигурируем проект без GPIO поддержки..."
    cmake .. -DENABLE_GPIO=OFF -DENABLE_ROS2=OFF
fi

if [ $? -ne 0 ]; then
    echo "Ошибка при конфигурации проекта!"
    exit 1
fi

# Устанавливаем количество ядер для сборки
if [ "$OS" = "Linux" ]; then
    CORES=$(nproc)
else
    CORES=4  # По умолчанию 4 ядра для других систем
fi

echo "Собираем проект с использованием $CORES ядер..."
make -j$CORES

if [ $? -ne 0 ]; then
    echo "Ошибка при сборке проекта!"
    exit 1
fi

echo "Сборка завершена успешно!"
echo "Проверяем результат:"
ls -la

echo "=== Обновленное приложение готово ==="