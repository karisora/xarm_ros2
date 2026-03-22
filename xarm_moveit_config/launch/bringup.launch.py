#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration("hw_ns")
    add_gripper = LaunchConfiguration("add_gripper")
    api_signal_topic = LaunchConfiguration("api_signal_topic")
    manual_mode_topic = LaunchConfiguration("manual_mode_topic")
    gripper_controller_name = LaunchConfiguration("gripper_controller_name")
    gripper_joint_name = LaunchConfiguration("gripper_joint_name")
    launch_manual_controller = LaunchConfiguration("launch_manual_controller")
    launch_autonomous_controller = LaunchConfiguration("launch_autonomous_controller")
    launch_planner_node = LaunchConfiguration("launch_planner_node")

    launch_master_controller = LaunchConfiguration("launch_master_controller")
    emulate_mode_services = LaunchConfiguration("emulate_mode_services")

    launch_api_bridge = LaunchConfiguration("launch_api_bridge")
    host = LaunchConfiguration("host")
    api_port = LaunchConfiguration("api_port")
    api_key = LaunchConfiguration("api_key")
    unit_id = LaunchConfiguration("unit_id")
    manual_gripper_via_topic = LaunchConfiguration("manual_gripper_via_topic")
    bridge_start_delay = LaunchConfiguration("bridge_start_delay")
    waypoints_file = LaunchConfiguration("waypoints_file")
    path_mode = LaunchConfiguration("path_mode")
    saved_path_file = LaunchConfiguration("saved_path_file")

    moveit_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("xarm_moveit_config"), "launch", "xarm6_moveit_gazebo_with_field.launch.py"])
        ),
        launch_arguments={
            "hw_ns": hw_ns,
            "add_gripper": add_gripper,
            "api_signal_topic": api_signal_topic,
            "manual_mode_topic": manual_mode_topic,
            "gripper_controller_name": gripper_controller_name,
            "gripper_joint_name": gripper_joint_name,
            "launch_manual_controller": launch_manual_controller,
            "launch_autonomous_controller": launch_autonomous_controller,
            "launch_planner_node": launch_planner_node,
            "waypoints_file": waypoints_file,
            "path_mode": path_mode,
            "saved_path_file": saved_path_file,
        }.items(),
    )

    master_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("master_controller"), "launch", "master_controller.launch.py"])
        ),
        launch_arguments={
            "api_signal_topic": api_signal_topic,
            "hw_ns": hw_ns,
            "emulate_mode_services": emulate_mode_services,
            "manual_mode_topic": manual_mode_topic,
        }.items(),
        condition=IfCondition(launch_master_controller),
    )

    api_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("xarm_api_bridge"), "launch", "xarm_api_bridge.launch.py"])
        ),
        launch_arguments={
            "api_host": host,
            "api_port": api_port,
            "api_key": api_key,
            "unit_id": unit_id,
            "hw_ns": hw_ns,
            "api_signal_topic": api_signal_topic,
            "manual_gripper_via_topic": manual_gripper_via_topic,
        }.items(),
        condition=IfCondition(launch_api_bridge),
    )

    delayed_api_bridge_launch = TimerAction(
        period=bridge_start_delay,
        actions=[api_bridge_launch],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hw_ns", default_value="xarm"),
            DeclareLaunchArgument("add_gripper", default_value="true"),
            DeclareLaunchArgument("api_signal_topic", default_value="/xarm/api_requests"),
            DeclareLaunchArgument("manual_mode_topic", default_value="/xarm/manual_mode_active"),
            DeclareLaunchArgument("gripper_controller_name", default_value="xarm_gripper_traj_controller"),
            DeclareLaunchArgument("gripper_joint_name", default_value="drive_joint"),
            DeclareLaunchArgument("launch_manual_controller", default_value="true"),
            DeclareLaunchArgument("launch_autonomous_controller", default_value="true"),
            DeclareLaunchArgument("launch_planner_node", default_value="true"),
            DeclareLaunchArgument("launch_master_controller", default_value="true"),
            DeclareLaunchArgument("emulate_mode_services", default_value="true"),
            DeclareLaunchArgument("launch_api_bridge", default_value="true"),
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("api_port", default_value="8000"),
            DeclareLaunchArgument("api_key", default_value=""),
            DeclareLaunchArgument("unit_id", default_value="unit-mys01"),
            DeclareLaunchArgument("manual_gripper_via_topic", default_value="true"),
            DeclareLaunchArgument("bridge_start_delay", default_value="2.0"),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("xarm_planner"), "config", "n1.yaml"]
                ),
            ),
            DeclareLaunchArgument("path_mode", default_value="waypoint"),
            DeclareLaunchArgument("saved_path_file", default_value=""),
            moveit_stack_launch,
            master_controller_launch,
            delayed_api_bridge_launch,
        ]
    )
