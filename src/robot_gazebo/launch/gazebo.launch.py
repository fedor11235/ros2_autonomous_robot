"""Bring up Ignition/Gazebo Fortress with the course world and spawn the robot.

    ros2 launch robot_gazebo gazebo.launch.py gui:=true

Starts gz-sim (GUI or headless), robot_state_publisher, the ros_gz parameter
bridge (gz <-> ROS topics), and spawns the URDF from the robot_description topic.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_description = get_package_share_directory("robot_description")

    default_world = os.path.join(pkg_gazebo, "worlds", "course.sdf")
    bridge_config = os.path.join(pkg_gazebo, "config", "bridge.yaml")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, "launch", "description.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Fortress simulator. GUI when gui:=true, otherwise server-only with
    # offscreen (headless) rendering so sensors still produce data.
    gz_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "4", world],
        output="screen",
        condition=IfCondition(gui),
    )
    gz_headless = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "-v", "4", "--headless-rendering", world],
        output="screen",
        condition=UnlessCondition(gui),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Spawn the robot from the /robot_description topic once the server is up.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "autobot",
            "-x", x_pose, "-y", y_pose, "-z", "0.1", "-Y", yaw,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        description,
        gz_gui,
        gz_headless,
        bridge,
        # Give gz-sim a moment to start before spawning / the bridge connects.
        TimerAction(period=4.0, actions=[spawn]),
    ])
