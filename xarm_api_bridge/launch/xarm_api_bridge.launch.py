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
        DeclareLaunchArgument("initial_pose_deg", default_value="[0.0, -56.0, -37.0, 0.0, 93.0, 0.0]"),
        DeclareLaunchArgument("initial_pose_controller_name", default_value="xarm6_traj_controller"),
        DeclareLaunchArgument(
            "initial_pose_joint_names",
            default_value="['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']",
        ),
        DeclareLaunchArgument("initial_pose_goal_time_sec", default_value="3.0"),
        DeclareLaunchArgument("initial_pose_wait_timeout_sec", default_value="15.0"),
        DeclareLaunchArgument("initial_pose_use_trajectory_action", default_value="true"),
        DeclareLaunchArgument("auto_ready_on_startup", default_value="true"),
        DeclareLaunchArgument("auto_ready_delay_sec", default_value="1.0"),
        DeclareLaunchArgument("auto_ready_max_attempts", default_value="10"),
        DeclareLaunchArgument("auto_ready_retry_interval_sec", default_value="2.0"),
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
                "initial_pose_deg": LaunchConfiguration("initial_pose_deg"),
                "initial_pose_controller_name": LaunchConfiguration("initial_pose_controller_name"),
                "initial_pose_joint_names": LaunchConfiguration("initial_pose_joint_names"),
                "initial_pose_goal_time_sec": LaunchConfiguration("initial_pose_goal_time_sec"),
                "initial_pose_wait_timeout_sec": LaunchConfiguration("initial_pose_wait_timeout_sec"),
                "initial_pose_use_trajectory_action": LaunchConfiguration("initial_pose_use_trajectory_action"),
                "auto_ready_on_startup": LaunchConfiguration("auto_ready_on_startup"),
                "auto_ready_delay_sec": LaunchConfiguration("auto_ready_delay_sec"),
                "auto_ready_max_attempts": LaunchConfiguration("auto_ready_max_attempts"),
                "auto_ready_retry_interval_sec": LaunchConfiguration("auto_ready_retry_interval_sec"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [bridge_node])
