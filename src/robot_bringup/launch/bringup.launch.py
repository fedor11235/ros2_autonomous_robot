"""Single-command bring-up: Gazebo + robot + autonomous navigation + RViz.

    ros2 launch robot_bringup bringup.launch.py

Arguments:
    gui:=true|false        show the Gazebo client (default true)
    rviz:=true|false       open RViz (default true)
    autonomy:=true|false   start the navigator automatically (default true)
    world:=<path>          override the world file
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("robot_gazebo")
    pkg_nav = get_package_share_directory("robot_navigation")
    pkg_description = get_package_share_directory("robot_description")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    autonomy = LaunchConfiguration("autonomy")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_world = os.path.join(pkg_gazebo, "worlds", "course.sdf")
    rviz_config = os.path.join(pkg_description, "rviz", "robot.rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")),
        launch_arguments={
            "gui": gui,
            "world": world,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "navigation.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(autonomy),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autonomy", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),

        gazebo,
        rviz_node,
        # Give Gazebo a few seconds to spawn the robot and start /odom + /scan
        # before the reactive controller starts issuing velocity commands.
        TimerAction(period=6.0, actions=[navigation]),
    ])
