# Stage 1: Build stage
FROM rust:1.70-bookworm AS builder

SHELL ["/bin/bash", "-c"]

# Устанавливаем Rust (уже установлен в базовом образе, просто устанавливаем нужную версию)
RUN set -eux; \
    rustup default stable

# Устанавливаем зависимости для rppal
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      libudev-dev \
      pkg-config \
      git \
    ; \
    rm -rf /var/lib/apt/lists/*

ENV CARGO_TARGET_DIR=/build/target

WORKDIR /build

# Копируем зависимости
COPY spot_lcd_node/Cargo.toml spot_lcd_node/Cargo.lock ./
# Удаляем ROS2 зависимости из Cargo.toml перед сборкой
RUN sed -i '/rclrs/d; /rosidl/d; /std-msgs/d' Cargo.toml

RUN mkdir src && echo "fn main() {}" > src/main.rs
RUN cargo build --release --locked
RUN rm src/main.rs

# Копируем исходники и собираем
COPY spot_lcd_node/src ./src
# Удаляем или комментируем импорты ROS2 из main.rs
RUN sed -i 's/^mod ros_publisher;/\/\/mod ros_publisher;/g' src/main.rs
RUN sed -i 's/^use ros_publisher::RosPublisher;/\/\/use ros_publisher::RosPublisher;/g' src/main.rs
RUN sed -i 's/ros_publisher::RosPublisher/DummyPublisher/g' src/main.rs
RUN sed -i 's/let ros_publisher = Arc::new(RosPublisher::new()?)?;/let ros_publisher = Arc::new(DummyPublisher::new()?);/g' src/main.rs || true
RUN sed -i 's/let ros_publisher = Some(RosPublisher::new()?)?;/let ros_publisher = Some(DummyPublisher::new()?);/g' src/main.rs || true
RUN sed -i 's/ros_publisher.as_ref().unwrap().start_ros_bridge();//g' src/main.rs || true
RUN sed -i 's/ros_publisher_temp.publish_temperature(temp);//g' src/main.rs || true
RUN sed -i 's/ros_publisher_uptime.publish_uptime(uptime);//g' src/main.rs || true

# Создаем заглушку для RosPublisher
RUN mkdir -p src/ros_publisher
RUN echo '#[derive(Debug)]\n pub struct DummyPublisher;\n\n impl DummyPublisher {\n     pub fn new() -> Result<Self, anyhow::Error> {\n         Ok(DummyPublisher)\n     }\n\n     pub fn publish_temperature(&self, temp: &str) {\n         println!("Temperature: {}", temp);\n     }\n\n     pub fn publish_uptime(&self, uptime: &str) {\n         println!("Uptime: {}", uptime);\n     }\n\n     pub fn start_ros_bridge(&self) {\n         println!("Dummy publisher started");\n     }\n }' > src/ros_publisher/mod.rs

RUN cargo build --release --locked

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