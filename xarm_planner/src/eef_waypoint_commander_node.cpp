/* Copyright 2026
 *
 * Software License Agreement (BSD License)
 *
 * Author: Codex
 ============================================================================*/

#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include "xarm_planner/xarm_planner.h"

namespace
{
void exit_sig_handler(int signum)
{
  (void)signum;
  fprintf(stderr, "[eef_waypoint_commander_node] Ctrl-C caught, exit process...\n");
  std::exit(-1);
}

std::string resolve_group_name(
  const std::string &robot_type,
  int dof,
  const std::string &prefix)
{
  std::string group_name = robot_type;
  if (robot_type == "xarm" || robot_type == "lite") {
    group_name = robot_type + std::to_string(dof);
  }
  if (!prefix.empty()) {
    group_name = prefix + group_name;
  }
  return group_name;
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("eef_waypoint_commander_node", node_options);
  signal(SIGINT, exit_sig_handler);

  int dof = 6;
  node->get_parameter_or("dof", dof, 6);

  std::string robot_type = "xarm";
  node->get_parameter_or("robot_type", robot_type, std::string("xarm"));

  std::string prefix = "";
  node->get_parameter_or("prefix", prefix, std::string(""));

  const std::string group_name = resolve_group_name(robot_type, dof, prefix);

  bool wait_each = true;
  node->get_parameter_or("wait_each", wait_each, true);

  bool use_cartesian = false;
  node->get_parameter_or("use_cartesian", use_cartesian, false);

  bool position_only = false;
  node->get_parameter_or("position_only", position_only, false);

  int waypoint_count = 0;
  node->get_parameter_or("waypoint_count", waypoint_count, 0);
  if (waypoint_count <= 0) {
    RCLCPP_ERROR(node->get_logger(), "waypoint_count must be >= 1");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(), "namespace=%s, group_name=%s, waypoint_count=%d, use_cartesian=%d, position_only=%d",
    node->get_namespace(), group_name.c_str(), waypoint_count, use_cartesian, position_only);

  if (use_cartesian && position_only) {
    RCLCPP_WARN(
      node->get_logger(),
      "position_only is ignored when use_cartesian=true because Cartesian interpolation still uses waypoint orientation");
  }

  xarm_planner::XArmPlanner planner(node, group_name);

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(static_cast<size_t>(waypoint_count));

  for (int i = 0; i < waypoint_count; ++i) {
    const std::string pos_key = "waypoints." + std::to_string(i) + ".position";
    const std::string ori_key = "waypoints." + std::to_string(i) + ".orientation";
    std::vector<double> pos;
    std::vector<double> ori;
    if (!node->get_parameter(pos_key, pos) || pos.size() != 3) {
      RCLCPP_ERROR(
        node->get_logger(), "missing/invalid parameter %s (expected 3 values)",
        pos_key.c_str());
      rclcpp::shutdown();
      return 2;
    }
    if (!node->get_parameter(ori_key, ori) || ori.size() != 4) {
      RCLCPP_ERROR(
        node->get_logger(), "missing/invalid parameter %s (expected 4 values)",
        ori_key.c_str());
      rclcpp::shutdown();
      return 3;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    pose.orientation.x = ori[0];
    pose.orientation.y = ori[1];
    pose.orientation.z = ori[2];
    pose.orientation.w = ori[3];
    poses.push_back(pose);
  }

  if (use_cartesian && poses.size() >= 2) {
    RCLCPP_INFO(node->get_logger(), "planning cartesian path with %zu waypoints", poses.size());
    if (!planner.planCartesianPath(poses)) {
      RCLCPP_ERROR(node->get_logger(), "planCartesianPath failed");
      rclcpp::shutdown();
      return 4;
    }
    if (!planner.executePath(wait_each)) {
      RCLCPP_ERROR(node->get_logger(), "executePath failed");
      rclcpp::shutdown();
      return 5;
    }
    RCLCPP_INFO(node->get_logger(), "cartesian path execution completed");
    rclcpp::shutdown();
    return 0;
  }

  for (size_t i = 0; i < poses.size(); ++i) {
    const auto &pose = poses[i];
    RCLCPP_INFO(
      node->get_logger(),
      "waypoint[%zu]: pos(%.3f, %.3f, %.3f) quat(%.3f, %.3f, %.3f, %.3f)",
      i,
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    const bool planned = position_only ?
      planner.planPositionTarget(pose.position.x, pose.position.y, pose.position.z) :
      planner.planPoseTarget(pose);
    if (!planned) {
      RCLCPP_ERROR(
        node->get_logger(),
        "%s failed at waypoint[%zu]",
        position_only ? "planPositionTarget" : "planPoseTarget",
        i);
      rclcpp::shutdown();
      return 6;
    }
    if (!planner.executePath(wait_each)) {
      RCLCPP_ERROR(node->get_logger(), "executePath failed at waypoint[%zu]", i);
      rclcpp::shutdown();
      return 7;
    }
  }

  RCLCPP_INFO(node->get_logger(), "pose waypoint execution completed");
  rclcpp::shutdown();
  return 0;
}
