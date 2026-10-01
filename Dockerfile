# ROS 2 Humble + Ignition/Gazebo Fortress (gz-sim) image with the workspace built.
# Multi-arch (amd64/arm64).
# Build:  docker build -t autobot .
# Run  :  see docker-compose.yml / run.sh (needs X11 for the GUI).
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-image \
      ros-humble-xacro \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-rviz2 \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src ./src
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_bringup", "bringup.launch.py"]
