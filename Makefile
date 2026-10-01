# Developer shortcuts. Assumes ROS 2 Humble is sourced (source /opt/ros/humble/setup.bash).
.PHONY: build clean test run run-headless gazebo rviz docker docker-run

build:
	colcon build --symlink-install

clean:
	rm -rf build install log

test:
	colcon test --packages-select robot_navigation
	colcon test-result --verbose

# Full simulation: Gazebo + RViz + autonomy (sources the workspace first).
run:
	./run.sh

run-headless:
	./run.sh gui:=false rviz:=false

# Just Gazebo + the robot, no autonomy (drive it manually with teleop).
gazebo:
	. install/setup.bash && ros2 launch robot_gazebo gazebo.launch.py

docker:
	docker build -t autobot .

docker-run:
	docker compose up --build
