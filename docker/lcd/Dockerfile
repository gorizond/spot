# Stage 1: Build stage
FROM rust:1.70-bookworm AS builder

SHELL ["/bin/bash", "-c"]

# Устанавливаем Rust (уже установлен в базовом образе, просто устанавливаем нужную версию)
RUN set -eux; \
    rustup default stable

# Устанавливаем зависимости для rppal и ROS2
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      libudev-dev \
      libsystemd-dev \
      libclang-dev \
      pkg-config \
      cmake \
      git \
    ; \
    rm -rf /var/lib/apt/lists/*

ENV CARGO_TARGET_DIR=/build/target

WORKDIR /build

# Копируем зависимости
COPY spot_lcd_node/Cargo.toml spot_lcd_node/Cargo.lock ./
RUN mkdir src && echo "fn main() {}" > src/main.rs
RUN cargo build --release --features ros2
RUN rm src/main.rs

# Копируем исходники и собираем
COPY spot_lcd_node/src ./src
RUN touch src/main.rs
RUN cargo build --release --features ros2

# Stage 2: Runtime stage
FROM debian:bookworm-slim

SHELL ["/bin/bash", "-c"]

# Устанавливаем зависимости для rppal
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      libudev1 \
      ca-certificates \
      curl \
    ; \
    rm -rf /var/lib/apt/lists/*

# Создаем пользователя
RUN useradd --create-home --shell /bin/bash app

# Копируем бинарник
COPY --from=builder /build/target/release/spot_lcd_node /usr/local/bin/
RUN chmod +x /usr/local/bin/spot_lcd_node

# Устанавливаем рабочую директорию
WORKDIR /app

# Копируем конфигурацию
COPY spot_lcd_node/config /app/config

# Меняем владельца файлов
RUN chown -R app:app /app
USER app

# Точка входа
ENTRYPOINT ["/usr/local/bin/spot_lcd_node"]