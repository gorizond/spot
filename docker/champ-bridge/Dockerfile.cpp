FROM ros:kilted-ros-base AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    ros-kilted-rclcpp \
    ros-kilted-std-msgs \
    ros-kilted-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY spot_champ_bridge_cpp /ws/src/spot_champ_bridge_cpp

RUN bash -lc "source /opt/ros/kilted/setup.bash && colcon build --packages-select spot_champ_bridge_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

FROM ros:kilted-ros-base AS runtime
ENV ROS_DISTRO=kilted
COPY --from=builder /ws/install/spot_champ_bridge_cpp /opt/spot_champ_bridge_cpp

ENTRYPOINT ["bash", "-lc"]
CMD ["source /opt/ros/kilted/setup.bash && source /opt/spot_champ_bridge_cpp/share/spot_champ_bridge_cpp/local_setup.bash && exec ros2 run spot_champ_bridge_cpp spot_champ_bridge_cpp_node"]
