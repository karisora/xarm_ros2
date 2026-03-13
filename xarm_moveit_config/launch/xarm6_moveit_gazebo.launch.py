#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    
    # robot moveit gazebo launch
    # xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'add_gripper': add_gripper,
            'no_gui_ctrl': 'false',
        }.items(),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('add_gripper', default_value='false'),
        robot_moveit_gazebo_launch
    ])
