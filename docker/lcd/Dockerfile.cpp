# Stage 1: Build stage
FROM ubuntu:22.04 AS builder

# Устанавливаем необходимые пакеты для сборки
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libwiringpi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Копируем исходный код
COPY ./spot_lcd_cpp/ ./

# Создаем build директорию и собираем проект
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Stage 2: Runtime stage
FROM ubuntu:22.04

# Устанавливаем необходимые runtime зависимости
RUN apt-get update && apt-get install -y \
    libwiringpi3 \
    && rm -rf /var/lib/apt/lists/*

# Копируем исполняемый файл из стадии сборки
COPY --from=builder /app/build/spot_lcd_node /usr/local/bin/

# Создаем пользователя
RUN useradd --create-home --shell /bin/bash app

# Устанавливаем рабочую директорию
WORKDIR /app

# Копируем конфигурацию
COPY ./spot_lcd_node/config /app/config

# Меняем владельца файлов
RUN chown -R app:app /app
USER app

# Точка входа
ENTRYPOINT ["/usr/local/bin/spot_lcd_node"]