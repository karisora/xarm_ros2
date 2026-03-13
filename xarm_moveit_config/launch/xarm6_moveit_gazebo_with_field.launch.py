#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_scene_file = PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "config", "xarm6", "field.scene"])

    hw_ns = LaunchConfiguration("hw_ns")
    add_gripper = LaunchConfiguration("add_gripper")
    scene_file = LaunchConfiguration("scene_file")
    scene_apply_delay = LaunchConfiguration("scene_apply_delay")
    scene_frame_id = LaunchConfiguration("scene_frame_id")
    scene_topic = LaunchConfiguration("scene_topic")
    scene_service = LaunchConfiguration("scene_service")
    scene_wait_timeout = LaunchConfiguration("scene_wait_timeout")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_delay = LaunchConfiguration("initial_pose_delay")
    initial_joint_values_deg = LaunchConfiguration("initial_joint_values_deg")
    initial_pose_goal_time = LaunchConfiguration("initial_pose_goal_time")
    initial_pose_wait_timeout = LaunchConfiguration("initial_pose_wait_timeout")
    launch_manual_controller = LaunchConfiguration("launch_manual_controller")
    api_signal_topic = LaunchConfiguration("api_signal_topic")
    gripper_controller_name = LaunchConfiguration("gripper_controller_name")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
    manual_mode_topic = LaunchConfiguration("manual_mode_topic")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "launch", "xarm6_moveit_gazebo.launch.py"])
        ),
        launch_arguments={
            "hw_ns": hw_ns,
            "add_gripper": add_gripper,
        }.items(),
    )

    manual_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("manual_controller"), "launch", "manual_controller.launch.py"])
        ),
        launch_arguments={
            "api_signal_topic": api_signal_topic,
            "gripper_controller_name": gripper_controller_name,
            "gripper_joint_name": gripper_joint_name,
            "manual_mode_topic": manual_mode_topic,
        }.items(),
        condition=IfCondition(launch_manual_controller),
    )

    scene_loader = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "launch", "lib", "apply_field_scene.py"]),
            "--scene",
            scene_file,
            "--frame-id",
            scene_frame_id,
            "--topic",
            scene_topic,
            "--service",
            scene_service,
            "--wait-timeout",
            scene_wait_timeout,
        ],
        output="screen",
    )

    initial_pose_loader = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "launch", "lib", "set_initial_joint_positions.py"]),
            "--controller",
            "xarm6_traj_controller",
            "--joint-names",
            "joint1 joint2 joint3 joint4 joint5 joint6",
            "--joint-values-deg",
            initial_joint_values_deg,
            "--wait-timeout",
            initial_pose_wait_timeout,
            "--goal-time",
            initial_pose_goal_time,
        ],
        output="screen",
        condition=IfCondition(set_initial_pose),
    )

    delayed_initial_pose_loader = TimerAction(period=initial_pose_delay, actions=[initial_pose_loader])
    delayed_scene_loader = TimerAction(period=scene_apply_delay, actions=[scene_loader])

    return LaunchDescription(
        [
            DeclareLaunchArgument("hw_ns", default_value="xarm"),
            DeclareLaunchArgument("add_gripper", default_value="true"),
            DeclareLaunchArgument("scene_file", default_value=default_scene_file),
            DeclareLaunchArgument("scene_apply_delay", default_value="5.0"),
            DeclareLaunchArgument("scene_frame_id", default_value="world"),
            DeclareLaunchArgument("scene_topic", default_value="/collision_object"),
            DeclareLaunchArgument("scene_service", default_value="/apply_planning_scene"),
            DeclareLaunchArgument("scene_wait_timeout", default_value="15.0"),
            DeclareLaunchArgument("set_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_delay", default_value="3.0"),
            DeclareLaunchArgument("initial_joint_values_deg", default_value="0 -56 -37 0 93 0"),
            DeclareLaunchArgument("initial_pose_goal_time", default_value="3.0"),
            DeclareLaunchArgument("initial_pose_wait_timeout", default_value="15.0"),
            DeclareLaunchArgument("launch_manual_controller", default_value="false"),
            DeclareLaunchArgument("api_signal_topic", default_value="/xarm/api_requests"),
            DeclareLaunchArgument("gripper_controller_name", default_value="xarm_gripper_traj_controller"),
            DeclareLaunchArgument("gripper_joint_name", default_value="drive_joint"),
            DeclareLaunchArgument("manual_mode_topic", default_value="/xarm/manual_mode_active"),
            base_launch,
            manual_controller_launch,
            delayed_initial_pose_loader,
            delayed_scene_loader,
        ]
    )
