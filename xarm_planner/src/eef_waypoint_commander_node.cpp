/* Copyright 2026
 *
 * Software License Agreement (BSD License)
 *
 * Author: Codex
 ============================================================================*/

#include <signal.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "xarm_planner/xarm_planner.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct WaypointCommand
{
  geometry_msgs::msg::Pose pose;
  bool has_gripper = false;
  double gripper_target_rad = 0.0;
};

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

double deg_to_rad(double value_deg)
{
  return value_deg * kPi / 180.0;
}

bool load_gripper_target(
  const rclcpp::Node::SharedPtr &node,
  int waypoint_index,
  WaypointCommand &command)
{
  const std::string gripper_key = "waypoints." + std::to_string(waypoint_index) + ".gripper";
  const std::string gripper_deg_key = "waypoints." + std::to_string(waypoint_index) + ".gripper_deg";

  double gripper_target = 0.0;
  if (node->get_parameter(gripper_key, gripper_target)) {
    command.has_gripper = true;
    command.gripper_target_rad = gripper_target;
    return true;
  }

  if (node->get_parameter(gripper_deg_key, gripper_target)) {
    command.has_gripper = true;
    command.gripper_target_rad = deg_to_rad(gripper_target);
    return true;
  }

  return false;
}

std::string normalize_topic(std::string topic)
{
  if (topic.empty()) {
    return topic;
  }
  if (topic.front() != '/') {
    topic.insert(topic.begin(), '/');
  }
  return topic;
}

void publish_gripper_command(
  const rclcpp::Node::SharedPtr &node,
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &publisher,
  const std::string &joint_name,
  double target_rad,
  double command_duration_sec,
  size_t waypoint_index)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names.push_back(joint_name);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(target_rad);
  point.time_from_start = rclcpp::Duration::from_seconds(command_duration_sec);
  trajectory.points.push_back(point);

  publisher->publish(trajectory);
  RCLCPP_INFO(
    node->get_logger(),
    "published gripper command at waypoint[%zu]: joint=%s, target=%.3f rad, duration=%.3f sec",
    waypoint_index,
    joint_name.c_str(),
    target_rad,
    command_duration_sec);
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

  std::string gripper_controller_name = "xarm_gripper_traj_controller";
  node->get_parameter_or(
    "gripper_controller_name",
    gripper_controller_name,
    std::string("xarm_gripper_traj_controller"));

  std::string gripper_trajectory_topic = "";
  node->get_parameter_or(
    "gripper_trajectory_topic",
    gripper_trajectory_topic,
    std::string(""));

  std::string gripper_joint_name = "drive_joint";
  node->get_parameter_or(
    "gripper_joint_name",
    gripper_joint_name,
    std::string("drive_joint"));

  double gripper_command_duration_sec = 1.0;
  node->get_parameter_or(
    "gripper_command_duration_sec",
    gripper_command_duration_sec,
    1.0);
  gripper_command_duration_sec = std::max(0.01, gripper_command_duration_sec);

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

  std::vector<WaypointCommand> commands;
  commands.reserve(static_cast<size_t>(waypoint_count));
  bool has_any_gripper_waypoint = false;

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

    WaypointCommand command;
    command.pose = pose;
    load_gripper_target(node, i, command);
    has_any_gripper_waypoint = has_any_gripper_waypoint || command.has_gripper;
    commands.push_back(command);
  }

  if (gripper_trajectory_topic.empty()) {
    gripper_trajectory_topic = "/" + gripper_controller_name + "/joint_trajectory";
  }
  gripper_trajectory_topic = normalize_topic(gripper_trajectory_topic);
  auto gripper_trajectory_publisher =
    node->create_publisher<trajectory_msgs::msg::JointTrajectory>(gripper_trajectory_topic, 10);

  if (use_cartesian && has_any_gripper_waypoint) {
    RCLCPP_WARN(
      node->get_logger(),
      "gripper targets were provided, so use_cartesian=true will fall back to sequential waypoint execution");
    use_cartesian = false;
  }

  if (use_cartesian && commands.size() >= 2) {
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(commands.size());
    for (const auto &command : commands) {
      poses.push_back(command.pose);
    }

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

  for (size_t i = 0; i < commands.size(); ++i) {
    const auto &command = commands[i];
    const auto &pose = command.pose;
    RCLCPP_INFO(
      node->get_logger(),
      "waypoint[%zu]: pos(%.3f, %.3f, %.3f) quat(%.3f, %.3f, %.3f, %.3f)%s",
      i,
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
      command.has_gripper ? " with gripper target" : "");

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

    if (command.has_gripper) {
      publish_gripper_command(
        node,
        gripper_trajectory_publisher,
        gripper_joint_name,
        command.gripper_target_rad,
        gripper_command_duration_sec,
        i);
      if (wait_each) {
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(gripper_command_duration_sec)));
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "pose waypoint execution completed");
  rclcpp::shutdown();
  return 0;
}
