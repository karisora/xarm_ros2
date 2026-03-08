#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default='6')
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    target_x = LaunchConfiguration('target_x', default='0.30')
    target_y = LaunchConfiguration('target_y', default='0.00')
    target_z = LaunchConfiguration('target_z', default='0.30')
    target_qx = LaunchConfiguration('target_qx', default='1.0')
    target_qy = LaunchConfiguration('target_qy', default='0.0')
    target_qz = LaunchConfiguration('target_qz', default='0.0')
    target_qw = LaunchConfiguration('target_qw', default='0.0')
    keep_gripper_level = LaunchConfiguration('keep_gripper_level', default='false')
    use_position_target_only_with_level_constraint = LaunchConfiguration(
        'use_position_target_only_with_level_constraint',
        default='true'
    )
    fallback_to_pose_target_if_position_plan_failed = LaunchConfiguration(
        'fallback_to_pose_target_if_position_plan_failed',
        default='true'
    )
    level_qx = LaunchConfiguration('level_qx', default=target_qx)
    level_qy = LaunchConfiguration('level_qy', default=target_qy)
    level_qz = LaunchConfiguration('level_qz', default=target_qz)
    level_qw = LaunchConfiguration('level_qw', default=target_qw)
    level_x_axis_tolerance = LaunchConfiguration('level_x_axis_tolerance', default='0.03')
    level_y_axis_tolerance = LaunchConfiguration('level_y_axis_tolerance', default='0.03')
    level_z_axis_tolerance = LaunchConfiguration('level_z_axis_tolerance', default='3.14')
    level_weight = LaunchConfiguration('level_weight', default='1.0')
    constraint_frame = LaunchConfiguration('constraint_frame', default='')
    constraint_link = LaunchConfiguration('constraint_link', default='')

    node_executable = 'goal_pose_commander_node'
    node_parameters = {
        'target_x': float(target_x.perform(context)),
        'target_y': float(target_y.perform(context)),
        'target_z': float(target_z.perform(context)),
        'target_qx': float(target_qx.perform(context)),
        'target_qy': float(target_qy.perform(context)),
        'target_qz': float(target_qz.perform(context)),
        'target_qw': float(target_qw.perform(context)),
        'keep_gripper_level': keep_gripper_level.perform(context).lower() in ('true', '1', 'yes'),
        'use_position_target_only_with_level_constraint':
            use_position_target_only_with_level_constraint.perform(context).lower() in ('true', '1', 'yes'),
        'fallback_to_pose_target_if_position_plan_failed':
            fallback_to_pose_target_if_position_plan_failed.perform(context).lower() in ('true', '1', 'yes'),
        'level_qx': float(level_qx.perform(context)),
        'level_qy': float(level_qy.perform(context)),
        'level_qz': float(level_qz.perform(context)),
        'level_qw': float(level_qw.perform(context)),
        'level_x_axis_tolerance': float(level_x_axis_tolerance.perform(context)),
        'level_y_axis_tolerance': float(level_y_axis_tolerance.perform(context)),
        'level_z_axis_tolerance': float(level_z_axis_tolerance.perform(context)),
        'level_weight': float(level_weight.perform(context)),
        'constraint_frame': constraint_frame.perform(context),
        'constraint_link': constraint_link.perform(context),
        'wait': True,
    }

    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])
        ),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'node_executable': node_executable,
            'node_parameters': json.dumps(node_parameters),
        }.items(),
    )

    return [robot_planner_node_launch]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
