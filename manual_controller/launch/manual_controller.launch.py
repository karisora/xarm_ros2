#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("api_signal_topic", default_value="/xarm/api_requests"),
        DeclareLaunchArgument("unit_id_filter", default_value=""),
        DeclareLaunchArgument("gripper_controller_name", default_value="xarm_gripper_traj_controller"),
        DeclareLaunchArgument("trajectory_topic", default_value=""),
        DeclareLaunchArgument("gripper_joint_name", default_value="drive_joint"),
        DeclareLaunchArgument("open_position", default_value="0.0"),
        DeclareLaunchArgument("close_position", default_value="0.85"),
        DeclareLaunchArgument("command_duration_sec", default_value="1.0"),
    ]

    node = Node(
        package="manual_controller",
        executable="manual_controller_node",
        name="manual_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "api_signal_topic": LaunchConfiguration("api_signal_topic"),
                "unit_id_filter": LaunchConfiguration("unit_id_filter"),
                "gripper_controller_name": LaunchConfiguration("gripper_controller_name"),
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "gripper_joint_name": LaunchConfiguration("gripper_joint_name"),
                "open_position": LaunchConfiguration("open_position"),
                "close_position": LaunchConfiguration("close_position"),
                "command_duration_sec": LaunchConfiguration("command_duration_sec"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [node])
