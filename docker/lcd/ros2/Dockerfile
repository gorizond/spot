# Stage 1: Build stage
FROM ros:humble AS builder

SHELL ["/bin/bash", "-c"]

# Устанавливаем Rust
RUN set -eux; \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y; \
    source ~/.cargo/env; \
    rustup default stable

ENV CARGO_TARGET_DIR=/build/target
ENV RUST_BACKTRACE=1

WORKDIR /build

# Устанавливаем ROS2 Rust экосистему
RUN source /opt/ros/humble/setup.bash && \
    # Создаем временный ROS2 workspace для сборки rust экосистемы \
    mkdir -p /tmp/ros2_rust_ws/src && \
    cd /tmp/ros2_rust_ws && \
    # Используем vcs для получения ROS2 Rust пакетов \
    apt-get update && \
    apt-get install -y python3-vcstool python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Копируем зависимости
COPY spot_lcd_node/Cargo.toml.ros2 ./Cargo.toml
COPY spot_lcd_node/Cargo.lock ./

# Создаем пустой src/main.rs для первой сборки зависимостей
RUN mkdir -p src && echo "fn main() {}" > src/main.rs
RUN source ~/.cargo/env && \
    source /opt/ros/humble/setup.bash && \
    # Устанавливаем rustc-rosdeps \
    # cargo build --release --locked --features ros2 || \
    # Если прямая сборка не работает, используем standalone для установки базовых зависимостей \
    cargo build --release --locked --features standalone
RUN rm -rf src

# Копируем исходники
COPY spot_lcd_node/src ./src

# Собираем с ROS2 фичей
RUN source ~/.cargo/env && \
    source /opt/ros/humble/setup.bash && \
    # Для ROS2 Rust пакетов используем особую стратегию \
    # Устанавливаем зависимости через rosdep \
    apt-get update && \
    apt-get install -y \
      python3-rosdep \
      python3-colcon-common-extensions \
      && \
    rosdep update && \
    # Создаем символические ссылки на ROS2 Rust экосистему \
    # Временно устанавливаем зависимости из crates.io, если возможно \
    # Для rclrs используем версию из git репозитория \
    sed -i 's/rclrs = { version = "0.5", optional = true }/rclrs = { git = "https:\/\/github.com\/ros2-rust\/rclrs.git", optional = true }/g' Cargo.toml && \
    sed -i 's/rosidl-typesupport-common = { version = "0.5", optional = true }/rosidl-typesupport-common = { git = "https:\/\/github.com\/ros2-rust\/rclrs.git", optional = true }/g' Cargo.toml && \
    sed -i 's/std-msgs = { version = "0.5", optional = true }/std-msgs = { git = "https:\/\/github.com\/ros2-rust\/rclrs.git", optional = true }/g' Cargo.toml && \
    # Пытаемся собрать с ROS2 фичей \
    cargo build --release --locked --features ros2

# Stage 2: Runtime stage
FROM ros:humble

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