# Stage 1: Build stage
FROM ubuntu:24.04 AS builder

# Устанавливаем зависимости для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копируем исходный код
COPY ../../../spot_lcd_cpp /app/src

# Создаем директорию для сборки
WORKDIR /app/build

# Конфигурируем и собираем проект (в режиме симуляции)
RUN cmake /app/src -DENABLE_GPIO=OFF -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Stage 2: Runtime stage
FROM ubuntu:24.04

# Устанавливаем минимальные зависимости для работы
RUN apt-get update && apt-get install -y \
    libstdc++6 \
    libc6 \
    libgcc-s1 \
    && rm -rf /var/lib/apt/lists/*

# Копируем исполняемый файл из стадии сборки
COPY --from=builder /usr/local/bin/spot_lcd_node /usr/local/bin/
COPY --from=builder /app/src/config /app/config

# Устанавливаем рабочую директорию
WORKDIR /app

# Точка входа
ENTRYPOINT ["/usr/local/bin/spot_lcd_node", "-m", "all"]