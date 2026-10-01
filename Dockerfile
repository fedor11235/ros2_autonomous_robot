# ROS 2 Humble + Gazebo Classic 11 image with the workspace pre-built.
# Build:  docker build -t autobot .
# Run  :  see docker-compose.yml / run.sh (needs X11 for the GUI).
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

# Runtime dependencies (desktop-full already ships Gazebo Classic + RViz).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-plugins \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src ./src

# Resolve any remaining package deps, then build.
RUN source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y || true \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --symlink-install

# Source ROS + workspace automatically in every shell.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_bringup", "bringup.launch.py"]
