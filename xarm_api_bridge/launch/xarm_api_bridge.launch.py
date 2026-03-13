#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("api_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("api_port", default_value="8000"),
        DeclareLaunchArgument("api_key", default_value=""),
        DeclareLaunchArgument("unit_id", default_value="unit-xarm01"),
        DeclareLaunchArgument("hw_ns", default_value="xarm"),
        DeclareLaunchArgument("api_signal_topic", default_value=""),
        DeclareLaunchArgument("task_duration_sec", default_value="90.0"),
        DeclareLaunchArgument("manual_gripper_via_topic", default_value="false"),
    ]

    bridge_node = Node(
        package="xarm_api_bridge",
        executable="xarm_api_bridge_server",
        name="xarm_api_bridge_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "api_host": LaunchConfiguration("api_host"),
                "api_port": LaunchConfiguration("api_port"),
                "api_key": LaunchConfiguration("api_key"),
                "unit_id": LaunchConfiguration("unit_id"),
                "hw_ns": LaunchConfiguration("hw_ns"),
                "api_signal_topic": LaunchConfiguration("api_signal_topic"),
                "task_duration_sec": LaunchConfiguration("task_duration_sec"),
                "manual_gripper_via_topic": LaunchConfiguration("manual_gripper_via_topic"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [bridge_node])
