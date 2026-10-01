"""Publish the robot description (robot_state_publisher) and optionally RViz.

Standalone-usable for inspecting the URDF without Gazebo:
    ros2 launch robot_description description.launch.py standalone_rviz:=true jsp_gui:=true

NOTE: the arguments are deliberately named `jsp_gui` / `standalone_rviz` (not
`gui` / `rviz`) so they do NOT collide with the Gazebo `gui` / top-level `rviz`
configurations, which otherwise leak into this included launch file and would
spuriously start joint_state_publisher_gui / a second RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("robot_description")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    rviz_config = os.path.join(pkg, "rviz", "robot.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("jsp_gui")
    use_rviz = LaunchConfiguration("standalone_rviz")

    # xacro is expanded at launch time; wrap in ParameterValue(value_type=str)
    # so Humble does not try to parse the URDF string as YAML.
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_file]),
                                            value_type=str),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("jsp_gui", default_value="false",
                              description="Run joint_state_publisher_gui (standalone URDF inspection only)."),
        DeclareLaunchArgument("standalone_rviz", default_value="false"),

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
