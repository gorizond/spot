# Stage 1: Build stage
FROM ubuntu:22.04 AS builder

# Устанавливаем зависимости для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    wiringpi \
    libwiringpi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копируем исходный код
COPY ../../../spot_lcd_cpp /app/src

# Создаем директорию для сборки
WORKDIR /app/build

# Конфигурируем и собираем проект (GPIO включен)
RUN cmake /app/src -DENABLE_GPIO=ON -DENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Stage 2: Runtime stage
FROM ubuntu:22.04 AS runtime

# Устанавливаем минимальные зависимости для выполнения
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libgcc-s1 \
    wiringpi \
    libwiringpi-dev \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем переменные окружения для корректной работы wiringPi в контейнере
ENV WIRINGPI_GPIOMEM=1
ENV WIRINGPI_CODES=1
ENV BCM2835_NO_ERROR=1

# Копируем исполняемый файл из стадии сборки
COPY --from=builder /app/build/spot_lcd_cpp_node /usr/local/bin/
COPY --from=builder /app/src/config /app/config

# Устанавливаем рабочую директорию
WORKDIR /app

# Точка входа
ENTRYPOINT ["/usr/local/bin/spot_lcd_cpp_node", "-m", "all"]