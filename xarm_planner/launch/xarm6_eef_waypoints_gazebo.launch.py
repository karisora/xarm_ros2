#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
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
            'no_gui_ctrl': 'true',
        }.items(),
    )

    waypoint_commander = Node(
        package='xarm_planner',
        executable='eef_waypoint_commander_node',
        output='screen',
        parameters=[waypoints_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument('hw_ns', default_value='xarm'),
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
