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
        DeclareLaunchArgument("emulate_mode_services", default_value="true"),
        DeclareLaunchArgument("manual_mode_topic", default_value=""),
    ]

    node = Node(
        package="master_controller",
        executable="master_controller_node",
        name="master_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "api_signal_topic": LaunchConfiguration("api_signal_topic"),
                "unit_id_filter": LaunchConfiguration("unit_id_filter"),
                "hw_ns": LaunchConfiguration("hw_ns"),
                "emulate_mode_services": LaunchConfiguration("emulate_mode_services"),
                "manual_mode_topic": LaunchConfiguration("manual_mode_topic"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [node])
