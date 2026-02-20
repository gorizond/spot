FROM ros:kilted-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kilted-usb-cam \
    ros-kilted-image-proc \
    ros-kilted-stereo-image-proc \
    ros-kilted-rmw-cyclonedds-cpp \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=kilted
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["bash", "-lc"]
CMD ["source /opt/ros/kilted/setup.bash && echo 'spot-stereo-c270 image ready' && sleep infinity"]
