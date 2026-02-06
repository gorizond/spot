# Stage 1: Build stage
FROM ubuntu:22.04 AS builder

# Устанавливаем зависимости для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgpiod-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Установим ROS2 Kilted для сборки (для доступа к заголовочным файлам)
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    wget \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Добавим ключ ROS
RUN wget -q -O /tmp/ros_repo_key.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && \
    gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros_repo_key.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

# Установим ROS2 development libraries
RUN apt-get update && apt-get install -y \
    ros-kilted-rclcpp \
    ros-kilted-std-msgs \
    ros-kilted-builtin-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/kilted/setup.bash' >> /etc/bash.bashrc

WORKDIR /app

# Копируем исходный код
COPY ../../../spot_lcd_cpp /app/src

# Создаем директорию для сборки
WORKDIR /app/build

# Конфигурируем и собираем проект (GPIO включен)
RUN cmake /app/src -DENABLE_GPIO=ON -DENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Stage 2: Runtime stage with ROS2 Kilted
FROM ros:kilted-ros-base AS runtime

# Устанавливаем зависимости для выполнения
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libgcc-s1 \
    libgpiod2 \
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