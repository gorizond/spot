# Stage 1: Build stage (ROS2-enabled)
FROM ros:kilted-ros-base AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libcurl4-openssl-dev \
    ros-kilted-rclcpp \
    ros-kilted-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Copy only this package (monorepo-style)
COPY spot_node_label_config_cpp /ws/src/spot_node_label_config_cpp

WORKDIR /ws

RUN bash -lc "source /opt/ros/kilted/setup.bash && colcon build --packages-select spot_node_label_config_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Stage 2: Runtime stage
FROM ros:kilted-ros-base AS runtime

RUN apt-get update && apt-get install -y \
    libcurl4t64 \
    ca-certificates \
    ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install/spot_node_label_config_cpp /opt/spot_node_label_config_cpp

ENV ROS_DISTRO=kilted
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["bash", "-lc"]
CMD ["source /opt/ros/kilted/setup.bash && source /opt/spot_node_label_config_cpp/share/spot_node_label_config_cpp/local_setup.bash && exec ros2 run spot_node_label_config_cpp spot_node_label_config_cpp_node"]
