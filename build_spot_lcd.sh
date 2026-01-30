#!/bin/bash
# build_spot_lcd.sh

echo "Проверяем структуру проекта..."

# Проверяем наличие всех необходимых файлов
if [ ! -f "spot_lcd_cpp/CMakeLists.txt" ]; then
    echo "ERROR: CMakeLists.txt не найден"
    exit 1
fi

if [ ! -f "spot_lcd_cpp/package.xml" ]; then
    echo "ERROR: package.xml не найден"
    exit 1
fi

if [ ! -d "spot_lcd_cpp/src" ]; then
    echo "ERROR: src директория не найдена"
    exit 1
fi

if [ ! -d "spot_lcd_cpp/include" ]; then
    echo "ERROR: include директория не найдена"
    exit 1
fi

echo "Все файлы на месте. Создаем временный workspace для сборки..."

# Создаем временную директорию для сборки
TEMP_WS="/tmp/spot_lcd_ws"
rm -rf $TEMP_WS
mkdir -p $TEMP_WS/src
cp -r spot_lcd_cpp $TEMP_WS/src/

echo "Содержимое временного workspace:"
ls -la $TEMP_WS/src/

echo "Проверка файлов в пакете:"
ls -la $TEMP_WS/src/spot_lcd_cpp/

# Проверяем, установлен ли ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "ROS2 Humble найден, производим сборку..."
    cd $TEMP_WS
    source /opt/ros/humble/setup.bash
    colcon build --packages-select spot_lcd_cpp
    BUILD_RESULT=$?
    
    if [ $BUILD_RESULT -eq 0 ]; then
        echo "Сборка прошла успешно!"
        ls -la build/spot_lcd_cpp/
    else
        echo "Ошибка при сборке!"
        exit 1
    fi
else
    echo "ROS2 Humble не установлен. Проверяем с помощью cmake без ROS2..."
    cd $TEMP_WS/src/spot_lcd_cpp
    
    mkdir -p build
    cd build
    cmake .. -DENABLE_ROS2=OFF -DENABLE_GPIO=ON
    BUILD_RESULT=$?
    
    if [ $BUILD_RESULT -eq 0 ]; then
        make -j$(nproc)
        BUILD_RESULT=$?
        
        if [ $BUILD_RESULT -eq 0 ]; then
            echo "Сборка прошла успешно (без ROS2)!"
            ls -la
        else
            echo "Ошибка при сборке (без ROS2)!"
            exit 1
        fi
    else
        echo "Ошибка при конфигурации cmake!"
        exit 1
    fi
fi

echo "Локальная проверка завершена успешно!"