"""Bring up Gazebo Classic with the course world and spawn the robot.

    ros2 launch robot_gazebo gazebo.launch.py gui:=true

Starts gzserver+gzclient, robot_state_publisher (via description.launch.py),
and spawns the URDF into the running simulation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_description = get_package_share_directory("robot_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")

    default_world = os.path.join(pkg_gazebo, "worlds", "course.world")

    # Let Gazebo find bundled models (start/goal pads reuse built-ins; this also
    # covers any custom models dropped into robot_gazebo/models).
    models_path = os.path.join(pkg_gazebo, "models")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world, "verbose": "true"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
        condition=IfCondition(gui),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, "launch", "description.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "autobot",
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.1",
            "-Y", yaw,
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH",
                               models_path + os.pathsep +
                               os.environ.get("GAZEBO_MODEL_PATH", "")),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.785"),  # face the goal corner
        gzserver,
        gzclient,
        description,
        spawn_robot,
    ])
