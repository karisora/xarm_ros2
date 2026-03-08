/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 * Modified by Codex for single-goal pose execution
 ============================================================================*/

#include <signal.h>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    (void)signum;
    fprintf(stderr, "[goal_pose_commander_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("goal_pose_commander_node", node_options);
    signal(SIGINT, exit_sig_handler);

    int dof = 7;
    node->get_parameter_or("dof", dof, 7);

    std::string robot_type = "xarm";
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));

    std::string prefix = "";
    node->get_parameter_or("prefix", prefix, std::string(""));

    std::string group_name = robot_type;
    if (robot_type == "xarm" || robot_type == "lite")
    {
        group_name = robot_type + std::to_string(dof);
    }
    if (!prefix.empty())
    {
        group_name = prefix + group_name;
    }

    double target_x = 0.30;
    double target_y = 0.00;
    double target_z = 0.30;
    double target_qx = 1.0;
    double target_qy = 0.0;
    double target_qz = 0.0;
    double target_qw = 0.0;
    bool wait = true;
    bool keep_gripper_level = false;
    bool use_position_target_only_with_level_constraint = true;
    bool fallback_to_pose_target_if_position_plan_failed = true;

    node->get_parameter_or("target_x", target_x, target_x);
    node->get_parameter_or("target_y", target_y, target_y);
    node->get_parameter_or("target_z", target_z, target_z);
    node->get_parameter_or("target_qx", target_qx, target_qx);
    node->get_parameter_or("target_qy", target_qy, target_qy);
    node->get_parameter_or("target_qz", target_qz, target_qz);
    node->get_parameter_or("target_qw", target_qw, target_qw);
    node->get_parameter_or("wait", wait, wait);
    node->get_parameter_or("keep_gripper_level", keep_gripper_level, keep_gripper_level);
    node->get_parameter_or(
        "use_position_target_only_with_level_constraint",
        use_position_target_only_with_level_constraint,
        use_position_target_only_with_level_constraint);
    node->get_parameter_or(
        "fallback_to_pose_target_if_position_plan_failed",
        fallback_to_pose_target_if_position_plan_failed,
        fallback_to_pose_target_if_position_plan_failed);

    double level_qx = target_qx;
    double level_qy = target_qy;
    double level_qz = target_qz;
    double level_qw = target_qw;
    double level_x_axis_tolerance = 0.03;
    double level_y_axis_tolerance = 0.03;
    double level_z_axis_tolerance = 3.14;
    double level_weight = 1.0;
    std::string constraint_frame = "";
    std::string constraint_link = "";

    node->get_parameter_or("level_qx", level_qx, level_qx);
    node->get_parameter_or("level_qy", level_qy, level_qy);
    node->get_parameter_or("level_qz", level_qz, level_qz);
    node->get_parameter_or("level_qw", level_qw, level_qw);
    node->get_parameter_or("level_x_axis_tolerance", level_x_axis_tolerance, level_x_axis_tolerance);
    node->get_parameter_or("level_y_axis_tolerance", level_y_axis_tolerance, level_y_axis_tolerance);
    node->get_parameter_or("level_z_axis_tolerance", level_z_axis_tolerance, level_z_axis_tolerance);
    node->get_parameter_or("level_weight", level_weight, level_weight);
    node->get_parameter_or("constraint_frame", constraint_frame, constraint_frame);
    node->get_parameter_or("constraint_link", constraint_link, constraint_link);

    RCLCPP_INFO(node->get_logger(), "namespace=%s, group_name=%s", node->get_namespace(), group_name.c_str());
    RCLCPP_INFO(
        node->get_logger(),
        "goal pose: pos(%.3f, %.3f, %.3f), quat(%.3f, %.3f, %.3f, %.3f)",
        target_x,
        target_y,
        target_z,
        target_qx,
        target_qy,
        target_qz,
        target_qw);

    xarm_planner::XArmPlanner planner(node, group_name);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z;
    target_pose.orientation.x = target_qx;
    target_pose.orientation.y = target_qy;
    target_pose.orientation.z = target_qz;
    target_pose.orientation.w = target_qw;

    if (keep_gripper_level)
    {
        if (constraint_frame.empty())
        {
            constraint_frame = planner.getPlanningFrame();
        }
        geometry_msgs::msg::Quaternion level_quat;
        level_quat.x = level_qx;
        level_quat.y = level_qy;
        level_quat.z = level_qz;
        level_quat.w = level_qw;

        const bool constraint_set = planner.setPathOrientationConstraint(
            level_quat,
            constraint_frame,
            constraint_link,
            level_x_axis_tolerance,
            level_y_axis_tolerance,
            level_z_axis_tolerance,
            level_weight);
        if (!constraint_set)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set orientation path constraint");
            rclcpp::shutdown();
            return 3;
        }

        RCLCPP_INFO(
            node->get_logger(),
            "orientation constraint enabled: frame=%s link=%s tol=(%.3f, %.3f, %.3f)",
            constraint_frame.c_str(),
            constraint_link.empty() ? planner.getEndEffectorLink().c_str() : constraint_link.c_str(),
            level_x_axis_tolerance,
            level_y_axis_tolerance,
            level_z_axis_tolerance);
    }

    bool planned = false;
    if (keep_gripper_level && use_position_target_only_with_level_constraint)
    {
        planned = planner.planPositionTarget(target_x, target_y, target_z);
        if (!planned && fallback_to_pose_target_if_position_plan_failed)
        {
            RCLCPP_WARN(
                node->get_logger(),
                "planPositionTarget with level constraint failed, retrying with planPoseTarget");
            planned = planner.planPoseTarget(target_pose);
        }
    }
    else
    {
        planned = planner.planPoseTarget(target_pose);
    }
    if (keep_gripper_level)
    {
        planner.clearPathConstraints();
    }
    if (!planned)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to requested goal pose");
        rclcpp::shutdown();
        return 1;
    }

    const bool executed = planner.executePath(wait);
    if (!executed)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute planned trajectory");
        rclcpp::shutdown();
        return 2;
    }

    RCLCPP_INFO(node->get_logger(), "Goal pose execution completed successfully");
    rclcpp::shutdown();
    return 0;
}
