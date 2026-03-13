#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <regex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "xarm_msgs/msg/api_request.hpp"

namespace
{
std::string trim(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base(), value.end());
  return value;
}

std::string normalize_topic(std::string topic)
{
  topic = trim(std::move(topic));
  if (topic.empty()) {
    return topic;
  }
  if (topic.front() != '/') {
    topic.insert(topic.begin(), '/');
  }
  return topic;
}

std::string extract_json_string(const std::string & json_text, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]+)\"");
  std::smatch match;
  if (std::regex_search(json_text, match, pattern) && match.size() > 1) {
    return match[1].str();
  }
  return "";
}
}  // namespace

class ManualControllerNode : public rclcpp::Node
{
public:
  ManualControllerNode()
  : Node("manual_controller")
  {
    api_signal_topic_ = this->declare_parameter<std::string>("api_signal_topic", "/xarm/api_requests");
    unit_id_filter_ = this->declare_parameter<std::string>("unit_id_filter", "");
    gripper_controller_name_ =
      this->declare_parameter<std::string>("gripper_controller_name", "xarm_gripper_traj_controller");
    trajectory_topic_ = this->declare_parameter<std::string>("trajectory_topic", "");
    gripper_joint_name_ = this->declare_parameter<std::string>("gripper_joint_name", "drive_joint");
    open_position_ = this->declare_parameter<double>("open_position", 0.0);
    close_position_ = this->declare_parameter<double>("close_position", 0.85);
    command_duration_sec_ = this->declare_parameter<double>("command_duration_sec", 1.0);

    if (trajectory_topic_.empty()) {
      trajectory_topic_ = "/" + gripper_controller_name_ + "/joint_trajectory";
    }
    trajectory_topic_ = normalize_topic(trajectory_topic_);

    api_signal_topic_ = normalize_topic(api_signal_topic_);

    trajectory_publisher_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(trajectory_topic_, 10);
    api_request_subscription_ = this->create_subscription<xarm_msgs::msg::ApiRequest>(
      api_signal_topic_, 10,
      std::bind(&ManualControllerNode::handle_api_request, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "manual_controller started: api_signal_topic=%s, trajectory_topic=%s, gripper_joint_name=%s, open_position=%.3f, close_position=%.3f",
      api_signal_topic_.c_str(), trajectory_topic_.c_str(), gripper_joint_name_.c_str(), open_position_,
      close_position_);
    if (open_position_ > close_position_) {
      RCLCPP_WARN(
        this->get_logger(),
        "open_position (%.3f) is greater than close_position (%.3f); verify gripper direction for this robot",
        open_position_, close_position_);
    }
  }

private:
  void handle_api_request(const xarm_msgs::msg::ApiRequest::SharedPtr msg)
  {
    if (!unit_id_filter_.empty() && msg->unit_id != unit_id_filter_) {
      return;
    }

    if (msg->event_type != "manual_command" || msg->command_name != "gripper") {
      return;
    }

    const std::string action = extract_json_string(msg->payload_json, "action");
    if (!action.empty() && action != "gripper") {
      return;
    }

    std::string state = extract_json_string(msg->payload_json, "state");
    if (state.empty()) {
      state = extract_json_string(msg->payload_json, "mode");
    }

    if (state != "open" && state != "close") {
      RCLCPP_WARN(
        this->get_logger(),
        "received gripper command with unsupported state: payload_json=%s",
        msg->payload_json.c_str());
      return;
    }

    publish_gripper_command(state, state == "open" ? open_position_ : close_position_);
  }

  void publish_gripper_command(const std::string & state, double position)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    // Leave the stamp at zero so the controller starts the trajectory immediately
    // in both simulated time and wall-clock configurations.
    trajectory.joint_names.push_back(gripper_joint_name_);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(position);
    point.time_from_start = rclcpp::Duration::from_seconds(command_duration_sec_);
    trajectory.points.push_back(point);

    trajectory_publisher_->publish(trajectory);
    RCLCPP_INFO(
      this->get_logger(),
      "published gripper %s command: joint=%s, position=%.3f, topic=%s",
      state.c_str(), gripper_joint_name_.c_str(), position, trajectory_topic_.c_str());
  }

  std::string api_signal_topic_;
  std::string unit_id_filter_;
  std::string gripper_controller_name_;
  std::string trajectory_topic_;
  std::string gripper_joint_name_;
  double open_position_;
  double close_position_;
  double command_duration_sec_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<xarm_msgs::msg::ApiRequest>::SharedPtr api_request_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
