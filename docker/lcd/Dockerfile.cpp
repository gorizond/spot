# Stage 1: Build stage
FROM ubuntu:22.04 AS builder

# Устанавливаем зависимости для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libwiringpi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копируем исходный код
COPY ../../../spot_lcd_cpp /app/src

# Создаем директорию для сборки
WORKDIR /app/build

# Конфигурируем и собираем проект (с GPIO поддержкой)
RUN cmake /app/src -DENABLE_GPIO=ON -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Stage 2: Runtime stage
FROM ubuntu:22.04 AS runtime

# Устанавливаем минимальные зависимости для выполнения
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libgcc-s1 \
    wiringpi \
    && rm -rf /var/lib/apt/lists/*

# Копируем исполняемый файл из стадии сборки
COPY --from=builder /app/build/spot_lcd_node /usr/local/bin/
COPY --from=builder /app/src/config /app/config

# Устанавливаем рабочую директорию
WORKDIR /app

# Точка входа
ENTRYPOINT ["/usr/local/bin/spot_lcd_node", "-m", "all"]