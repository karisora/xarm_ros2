#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default='true')
    gripper_controller_name = LaunchConfiguration('gripper_controller_name', default='xarm_gripper_traj_controller')
    gripper_trajectory_topic = LaunchConfiguration('gripper_trajectory_topic', default='')
    gripper_joint_name = LaunchConfiguration('gripper_joint_name', default='drive_joint')
    gripper_command_duration_sec = LaunchConfiguration('gripper_command_duration_sec', default='1.0')
    waypoints_file = LaunchConfiguration(
        'waypoints_file',
        default=PathJoinSubstitution(
            [FindPackageShare('xarm_planner'), 'config', 'eef_waypoints_example.yaml']
        ),
    )

    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'])
        ),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'add_gripper': add_gripper,
            'no_gui_ctrl': 'true',
        }.items(),
    )

    waypoint_commander = Node(
        package='xarm_planner',
        executable='eef_waypoint_commander_node',
        output='screen',
        parameters=[
            waypoints_file,
            {
                'gripper_controller_name': gripper_controller_name,
                'gripper_trajectory_topic': gripper_trajectory_topic,
                'gripper_joint_name': gripper_joint_name,
                'gripper_command_duration_sec': gripper_command_duration_sec,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('hw_ns', default_value='xarm'),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument('gripper_controller_name', default_value='xarm_gripper_traj_controller'),
        DeclareLaunchArgument('gripper_trajectory_topic', default_value=''),
        DeclareLaunchArgument('gripper_joint_name', default_value='drive_joint'),
        DeclareLaunchArgument('gripper_command_duration_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('xarm_planner'), 'config', 'eef_waypoints_example.yaml']
            ),
            description='YAML file containing end-effector pose waypoints',
        ),
        robot_moveit_gazebo_launch,
        # _robot_moveit_gazebo.launch.py starts xarm_planner_node when no_gui_ctrl=true.
        # Wait for move_group/planner startup before sending waypoints.
        TimerAction(period=4.0, actions=[waypoint_commander]),
    ])
