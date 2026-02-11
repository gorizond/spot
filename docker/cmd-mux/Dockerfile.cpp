# Stage 1: Build stage
FROM ros:kilted-ros-base AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    ros-kilted-rclcpp \
    ros-kilted-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY spot_cmd_mux_cpp /ws/src/spot_cmd_mux_cpp

RUN bash -lc "source /opt/ros/kilted/setup.bash && colcon build --packages-select spot_cmd_mux_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Stage 2: Runtime stage
FROM ros:kilted-ros-base AS runtime

ENV ROS_DISTRO=kilted

COPY --from=builder /ws/install/spot_cmd_mux_cpp /opt/spot_cmd_mux_cpp

ENTRYPOINT ["bash", "-lc"]
CMD ["source /opt/ros/kilted/setup.bash && source /opt/spot_cmd_mux_cpp/share/spot_cmd_mux_cpp/local_setup.bash && exec ros2 run spot_cmd_mux_cpp spot_cmd_mux_cpp_node"]
