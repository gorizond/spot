FROM ros:kilted-ros-base

SHELL ["/bin/bash", "-lc"]

ARG ROS_DISTRO=kilted

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-xacro \
    ; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    rosdep init || true; \
    rosdep update

ENV COLCON_WS=/ws
WORKDIR ${COLCON_WS}/src

RUN set -eux; \
    git clone --branch ros2 --depth 1 --recurse-submodules https://github.com/chvmp/champ.git

WORKDIR ${COLCON_WS}

RUN set -ex; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    rosdep install --from-paths \
      src/champ/champ \
      src/champ/champ_base \
      src/champ/champ_bringup \
      src/champ/champ_config \
      src/champ/champ_description \
      src/champ/champ_msgs \
      --ignore-src -r -y; \
    colcon build --symlink-install --packages-select \
      champ champ_config champ_description champ_msgs champ_base champ_bringup

COPY docker/champ/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
