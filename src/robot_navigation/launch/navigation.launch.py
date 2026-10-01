"""Launch the autonomous navigation stack (waypoint navigator + GPS localizer).

    ros2 launch robot_navigation navigation.launch.py

Both nodes read their parameters from config/nav_params.yaml; override the
params file with params_file:=/path/to/your.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_navigation")
    default_params = os.path.join(pkg, "config", "nav_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="robot_navigation",
            executable="waypoint_navigator",
            name="waypoint_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="robot_navigation",
            executable="gps_localizer",
            name="gps_localizer",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
