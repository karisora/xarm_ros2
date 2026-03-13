#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("api_signal_topic", default_value="/xarm/api_requests"),
        DeclareLaunchArgument("unit_id_filter", default_value=""),
        DeclareLaunchArgument("hw_ns", default_value="xarm"),
        DeclareLaunchArgument("status_topic", default_value=""),
        DeclareLaunchArgument("active_topic", default_value=""),
        DeclareLaunchArgument("enable_robot_on_start", default_value="false"),
        DeclareLaunchArgument("set_auto_mode_on_start", default_value="true"),
        DeclareLaunchArgument("set_ready_state_on_start", default_value="true"),
        DeclareLaunchArgument("move_home_on_start", default_value="false"),
        DeclareLaunchArgument("stop_state_value", default_value="4"),
        DeclareLaunchArgument("pause_state_value", default_value="3"),
        DeclareLaunchArgument("ready_state_value", default_value="0"),
    ]

    node = Node(
        package="autonomous_controller",
        executable="autonomous_controller_node",
        name="autonomous_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "api_signal_topic": LaunchConfiguration("api_signal_topic"),
                "unit_id_filter": LaunchConfiguration("unit_id_filter"),
                "hw_ns": LaunchConfiguration("hw_ns"),
                "status_topic": LaunchConfiguration("status_topic"),
                "active_topic": LaunchConfiguration("active_topic"),
                "enable_robot_on_start": LaunchConfiguration("enable_robot_on_start"),
                "set_auto_mode_on_start": LaunchConfiguration("set_auto_mode_on_start"),
                "set_ready_state_on_start": LaunchConfiguration("set_ready_state_on_start"),
                "move_home_on_start": LaunchConfiguration("move_home_on_start"),
                "stop_state_value": LaunchConfiguration("stop_state_value"),
                "pause_state_value": LaunchConfiguration("pause_state_value"),
                "ready_state_value": LaunchConfiguration("ready_state_value"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [node])
