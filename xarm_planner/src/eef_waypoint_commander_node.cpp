/* Copyright 2026
 *
 * Software License Agreement (BSD License)
 *
 * Author: Codex
 ============================================================================*/

#include <signal.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <yaml-cpp/yaml.h>

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

struct SavedSegment
{
  std::string kind;
  std::string name;
  trajectory_msgs::msg::JointTrajectory trajectory;
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

double duration_to_sec(const builtin_interfaces::msg::Duration &duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) / 1000000000.0;
}

void set_duration_from_sec(double sec_value, builtin_interfaces::msg::Duration &duration)
{
  const double clamped = std::max(0.0, sec_value);
  duration.sec = static_cast<int32_t>(clamped);
  duration.nanosec = static_cast<uint32_t>((clamped - static_cast<double>(duration.sec)) * 1000000000.0);
}

double yaml_double_or(const YAML::Node &node, const char *key, double fallback)
{
  const auto child = node[key];
  return child ? child.as<double>() : fallback;
}

std::string expand_user_path(const std::string &path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }

  const char *home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }

  if (path.size() == 1) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::vector<std::string> load_saved_path_files(const rclcpp::Node::SharedPtr &node)
{
  std::vector<std::string> files;
  node->get_parameter_or("saved_path_files", files, std::vector<std::string>{});

  std::string single_file;
  node->get_parameter_or("saved_path_file", single_file, std::string(""));
  if (!single_file.empty()) {
    files.insert(files.begin(), single_file);
  }

  std::vector<std::string> normalized;
  for (const auto &file : files) {
    const std::string expanded = expand_user_path(file);
    if (!expanded.empty()) {
      normalized.push_back(expanded);
    }
  }
  return normalized;
}

bool file_exists(const std::string &path)
{
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

void ensure_parent_directory(const std::string &path)
{
  const std::string::size_type pos = path.find_last_of('/');
  if (pos == std::string::npos) {
    return;
  }

  const std::string directory = path.substr(0, pos);
  if (directory.empty()) {
    return;
  }

  std::string current;
  if (directory.front() == '/') {
    current = "/";
  }

  size_t start = (directory.front() == '/') ? 1 : 0;
  while (start <= directory.size()) {
    const size_t end = directory.find('/', start);
    const std::string part = directory.substr(start, end - start);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') {
        current += "/";
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error("failed to create directory: " + current);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
}

trajectory_msgs::msg::JointTrajectory build_gripper_trajectory(
  const std::string &joint_name,
  double target_rad,
  double command_duration_sec)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names.push_back(joint_name);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(target_rad);
  set_duration_from_sec(command_duration_sec, point.time_from_start);
  trajectory.points.push_back(point);
  return trajectory;
}

YAML::Node trajectory_to_yaml(const trajectory_msgs::msg::JointTrajectory &trajectory)
{
  YAML::Node node;
  for (const auto &joint_name : trajectory.joint_names) {
    node["joint_names"].push_back(joint_name);
  }

  for (const auto &point : trajectory.points) {
    YAML::Node point_node;
    for (double value : point.positions) {
      point_node["positions"].push_back(value);
    }
    for (double value : point.velocities) {
      point_node["velocities"].push_back(value);
    }
    for (double value : point.accelerations) {
      point_node["accelerations"].push_back(value);
    }
    for (double value : point.effort) {
      point_node["effort"].push_back(value);
    }
    point_node["time_from_start_sec"] = duration_to_sec(point.time_from_start);
    node["points"].push_back(point_node);
  }

  return node;
}

trajectory_msgs::msg::JointTrajectory trajectory_from_yaml(const YAML::Node &node)
{
  trajectory_msgs::msg::JointTrajectory trajectory;

  const auto joint_names = node["joint_names"];
  if (joint_names && joint_names.IsSequence()) {
    for (const auto &joint_name : joint_names) {
      trajectory.joint_names.push_back(joint_name.as<std::string>());
    }
  }

  const auto points = node["points"];
  if (points && points.IsSequence()) {
    for (const auto &point_node : points) {
      trajectory_msgs::msg::JointTrajectoryPoint point;

      const auto positions = point_node["positions"];
      if (positions && positions.IsSequence()) {
        for (const auto &value : positions) {
          point.positions.push_back(value.as<double>());
        }
      }

      const auto velocities = point_node["velocities"];
      if (velocities && velocities.IsSequence()) {
        for (const auto &value : velocities) {
          point.velocities.push_back(value.as<double>());
        }
      }

      const auto accelerations = point_node["accelerations"];
      if (accelerations && accelerations.IsSequence()) {
        for (const auto &value : accelerations) {
          point.accelerations.push_back(value.as<double>());
        }
      }

      const auto effort = point_node["effort"];
      if (effort && effort.IsSequence()) {
        for (const auto &value : effort) {
          point.effort.push_back(value.as<double>());
        }
      }

      set_duration_from_sec(yaml_double_or(point_node, "time_from_start_sec", 0.0), point.time_from_start);
      trajectory.points.push_back(point);
    }
  }

  return trajectory;
}

std::vector<SavedSegment> load_saved_segments(const std::string &path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node segments_node = root["segments"];
  if (!segments_node || !segments_node.IsSequence() || segments_node.size() == 0) {
    throw std::runtime_error("saved path file has no segments: " + path);
  }

  std::vector<SavedSegment> segments;
  for (size_t i = 0; i < segments_node.size(); ++i) {
    const auto &segment_node = segments_node[i];
    SavedSegment segment;
    segment.kind = segment_node["kind"].as<std::string>("");
    segment.name = segment_node["name"].as<std::string>("segment_" + std::to_string(i));
    segment.trajectory = trajectory_from_yaml(segment_node["trajectory"]);
    if (segment.kind.empty()) {
      throw std::runtime_error("saved segment kind is missing in: " + path);
    }
    if (segment.trajectory.joint_names.empty() || segment.trajectory.points.empty()) {
      throw std::runtime_error("saved segment trajectory is missing points in: " + path);
    }
    segments.push_back(segment);
  }

  return segments;
}

void save_recorded_segments(
  const std::string &path,
  const std::vector<SavedSegment> &segments)
{
  if (segments.empty()) {
    throw std::runtime_error("no trajectory segments were recorded");
  }

  YAML::Node root;
  root["version"] = 1;
  for (const auto &segment : segments) {
    YAML::Node segment_node;
    segment_node["kind"] = segment.kind;
    segment_node["name"] = segment.name;
    segment_node["trajectory"] = trajectory_to_yaml(segment.trajectory);
    root["segments"].push_back(segment_node);
  }

  ensure_parent_directory(path);
  std::ofstream stream(path, std::ios::out | std::ios::trunc);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open saved path file for writing: " + path);
  }
  stream << root;
}

void replay_saved_paths(
  const rclcpp::Node::SharedPtr &node,
  const std::vector<std::string> &saved_path_files,
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &arm_publisher,
  const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &gripper_publisher,
  bool wait_each)
{
  if (saved_path_files.empty()) {
    throw std::runtime_error("saved_path_file or saved_path_files is required for replay_path mode");
  }

  size_t total_segments = 0;
  std::vector<std::pair<std::string, std::vector<SavedSegment>>> loaded_files;
  for (const auto &saved_path_file : saved_path_files) {
    auto segments = load_saved_segments(saved_path_file);
    total_segments += segments.size();
    loaded_files.emplace_back(saved_path_file, std::move(segments));
  }

  size_t current_segment_index = 0;
  for (const auto &loaded_file : loaded_files) {
    const auto &saved_path_file = loaded_file.first;
    const auto &segments = loaded_file.second;
    RCLCPP_INFO(
      node->get_logger(),
      "replaying %zu saved segments from %s",
      segments.size(),
      saved_path_file.c_str());

    for (const auto &segment : segments) {
      ++current_segment_index;
      RCLCPP_INFO(
        node->get_logger(),
        "replaying segment %zu/%zu: %s (%s)",
        current_segment_index,
        total_segments,
        segment.name.c_str(),
        segment.kind.c_str());

      if (segment.kind == "arm") {
        arm_publisher->publish(segment.trajectory);
      } else if (segment.kind == "gripper") {
        gripper_publisher->publish(segment.trajectory);
      } else {
        throw std::runtime_error("unsupported saved segment kind: " + segment.kind);
      }

      if (wait_each) {
        const double duration_sec = duration_to_sec(segment.trajectory.points.back().time_from_start);
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.01, duration_sec))));
      }
    }
  }
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

  std::string path_mode = "waypoint";
  node->get_parameter_or("path_mode", path_mode, std::string("waypoint"));

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

  std::string arm_controller_name = "xarm6_traj_controller";
  node->get_parameter_or(
    "arm_controller_name",
    arm_controller_name,
    std::string("xarm6_traj_controller"));

  std::string arm_trajectory_topic = "";
  node->get_parameter_or(
    "arm_trajectory_topic",
    arm_trajectory_topic,
    std::string(""));

  const std::vector<std::string> saved_path_files = load_saved_path_files(node);
  const bool has_replay_files =
    !saved_path_files.empty() &&
    std::all_of(saved_path_files.begin(), saved_path_files.end(), file_exists);
  if (path_mode == "auto") {
    path_mode = has_replay_files ? "replay_path" : "record_path";
  }
  if (
    path_mode != "waypoint" &&
    path_mode != "record_path" &&
    path_mode != "replay_path")
  {
    RCLCPP_ERROR(node->get_logger(), "unsupported path_mode: %s", path_mode.c_str());
    rclcpp::shutdown();
    return 1;
  }

  int waypoint_count = 0;
  node->get_parameter_or("waypoint_count", waypoint_count, 0);
  if (path_mode != "replay_path" && waypoint_count <= 0) {
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
  if (arm_trajectory_topic.empty()) {
    arm_trajectory_topic = "/" + arm_controller_name + "/joint_trajectory";
  }
  arm_trajectory_topic = normalize_topic(arm_trajectory_topic);
  gripper_trajectory_topic = normalize_topic(gripper_trajectory_topic);
  auto arm_trajectory_publisher =
    node->create_publisher<trajectory_msgs::msg::JointTrajectory>(arm_trajectory_topic, 10);
  auto gripper_trajectory_publisher =
    node->create_publisher<trajectory_msgs::msg::JointTrajectory>(gripper_trajectory_topic, 10);

  if (path_mode == "replay_path") {
    try {
      replay_saved_paths(
        node,
        saved_path_files,
        arm_trajectory_publisher,
        gripper_trajectory_publisher,
        wait_each);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node->get_logger(), "replay_path failed: %s", error.what());
      rclcpp::shutdown();
      return 8;
    }

    RCLCPP_INFO(node->get_logger(), "saved path replay completed");
    rclcpp::shutdown();
    return 0;
  }

  if (use_cartesian && has_any_gripper_waypoint) {
    RCLCPP_WARN(
      node->get_logger(),
      "gripper targets were provided, so use_cartesian=true will fall back to sequential waypoint execution");
    use_cartesian = false;
  }

  std::vector<SavedSegment> recorded_segments;

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
    if (path_mode == "record_path" || path_mode == "auto") {
      recorded_segments.push_back({"arm", "cartesian_path", planner.getLastJointTrajectory()});
    }
    RCLCPP_INFO(node->get_logger(), "cartesian path execution completed");
  } else {
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
      if (path_mode == "record_path" || path_mode == "auto") {
        recorded_segments.push_back(
          {"arm", "waypoint_" + std::to_string(i), planner.getLastJointTrajectory()});
      }
      if (!planner.executePath(wait_each)) {
        RCLCPP_ERROR(node->get_logger(), "executePath failed at waypoint[%zu]", i);
        rclcpp::shutdown();
        return 7;
      }

      if (command.has_gripper) {
        const auto gripper_trajectory = build_gripper_trajectory(
          gripper_joint_name,
          command.gripper_target_rad,
          gripper_command_duration_sec);
        publish_gripper_command(
          node,
          gripper_trajectory_publisher,
          gripper_joint_name,
          command.gripper_target_rad,
          gripper_command_duration_sec,
          i);
        if (path_mode == "record_path" || path_mode == "auto") {
          recorded_segments.push_back(
            {"gripper", "gripper_" + std::to_string(i), gripper_trajectory});
        }
        if (wait_each) {
          rclcpp::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(gripper_command_duration_sec)));
        }
      }
    }

    RCLCPP_INFO(node->get_logger(), "pose waypoint execution completed");
  }

  if (path_mode == "record_path" || path_mode == "auto") {
    if (saved_path_files.empty()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "saved_path_file is required when path_mode is record_path or auto");
      rclcpp::shutdown();
      return 9;
    }
    try {
      save_recorded_segments(saved_path_files.front(), recorded_segments);
      RCLCPP_INFO(
        node->get_logger(),
        "saved recorded path to %s",
        saved_path_files.front().c_str());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node->get_logger(), "failed to save recorded path: %s", error.what());
      rclcpp::shutdown();
      return 10;
    }
  }
  rclcpp::shutdown();
  return 0;
}
