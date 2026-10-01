"""Publish the robot description (robot_state_publisher) and optionally RViz.

Standalone-usable for inspecting the URDF without Gazebo:
    ros2 launch robot_description description.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("robot_description")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    rviz_config = os.path.join(pkg, "rviz", "robot.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("rviz")

    # xacro is expanded at launch time; `Command` keeps it a string substitution.
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file]),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Run joint_state_publisher_gui (standalone URDF inspection only)."),
        DeclareLaunchArgument("rviz", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        # Only needed when NOT running Gazebo (Gazebo publishes joint states itself).
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
