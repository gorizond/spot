# Stage 1: Build
FROM ros:kilted-ros-base AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgpiod-dev \
    ros-kilted-rclcpp \
    ros-kilted-sensor-msgs \
    ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY spot_hcsr04_cpp /ws/src/spot_hcsr04_cpp

RUN bash -lc "source /opt/ros/kilted/setup.bash && colcon build --packages-select spot_hcsr04_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Stage 2: Runtime
FROM ros:kilted-ros-base AS runtime

ENV ROS_DISTRO=kilted
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y \
    libgpiod2 \
    ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install/spot_hcsr04_cpp /opt/spot_hcsr04_cpp

# Create run script
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/kilted/setup.bash\nsource /opt/spot_hcsr04_cpp/share/spot_hcsr04_cpp/local_setup.bash\nexec ros2 run spot_hcsr04_cpp spot_hcsr04_cpp_node\n' > /usr/local/bin/run_hcsr04_node.sh && chmod +x /usr/local/bin/run_hcsr04_node.sh

ENTRYPOINT ["/usr/local/bin/run_hcsr04_node.sh"]
