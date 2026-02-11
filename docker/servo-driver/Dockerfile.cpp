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
COPY spot_servo_driver_cpp /ws/src/spot_servo_driver_cpp
RUN bash -lc "source /opt/ros/kilted/setup.bash && colcon build --packages-select spot_servo_driver_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

FROM ros:kilted-ros-base AS runtime
ENV ROS_DISTRO=kilted
COPY --from=builder /ws/install/spot_servo_driver_cpp /opt/spot_servo_driver_cpp

ENTRYPOINT ["bash", "-lc"]
CMD ["source /opt/ros/kilted/setup.bash && source /opt/spot_servo_driver_cpp/share/spot_servo_driver_cpp/local_setup.bash && exec ros2 run spot_servo_driver_cpp spot_servo_driver_cpp_node"]
