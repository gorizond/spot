# Stage 1: Build stage (ROS2-enabled)
FROM ros:kilted-ros-base AS builder

# Устанавливаем разработчикские библиотеки ROS2 и другие зависимости для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgpiod-dev \
    ros-kilted-rclcpp \
    ros-kilted-std-msgs \
    ros-kilted-builtin-interfaces \
    ros-kilted-rcutils \
    ros-kilted-rosidl-default-runtime \
    ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копируем исходный код
COPY ../../../spot_lcd_cpp /app/src

# Создаем директорию для сборки
WORKDIR /app/build

# Конфигурируем и собираем проект (GPIO + ROS2 включены)
RUN bash -lc "source /opt/ros/kilted/setup.bash && cmake /app/src -DENABLE_GPIO=ON -DENABLE_ROS2=ON -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)"

# Stage 2: Runtime stage with ROS2 Kilted
FROM ros:kilted-ros-base AS runtime

# Устанавливаем зависимости для выполнения
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libgcc-s1 \
    libgpiod2 \
    ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Копируем исполняемые файлы из стадии сборки
COPY --from=builder /app/build/spot_lcd_cpp_lcd_node /usr/local/bin/
COPY --from=builder /app/build/spot_lcd_cpp_temp_node /usr/local/bin/
COPY --from=builder /app/build/spot_lcd_cpp_uptime_node /usr/local/bin/
COPY --from=builder /app/build/spot_lcd_cpp_node /usr/local/bin/

# Копируем launch файлы и скрипты
COPY --from=builder /app/src/launch /app/launch
COPY --from=builder /app/src/scripts/run_node.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/run_node.sh

# Устанавливаем рабочую директорию
WORKDIR /app

# Устанавливаем переменные окружения ROS2
ENV ROS_DISTRO=kilted
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Точка входа - по умолчанию запускаем все модули
ENTRYPOINT ["/usr/local/bin/run_node.sh"]
CMD ["all"]