#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


def parse_joint_values(values_text, dof):
    values = [float(x) for x in values_text.strip().split()]
    if len(values) != dof:
        raise ValueError(f"Expected {dof} joint values, got {len(values)}")
    return [math.radians(v) for v in values]


def send_initial_pose(controller_name, joint_names, joint_values_deg, wait_timeout, goal_time):
    rclpy.init()
    node = rclpy.create_node("set_initial_joint_positions")
    action_name = f"/{controller_name}/follow_joint_trajectory"
    client = ActionClient(node, FollowJointTrajectory, action_name)

    if not client.wait_for_server(timeout_sec=wait_timeout):
        print(f"[set_initial_joint_positions] Action server not available: {action_name}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    positions = parse_joint_values(joint_values_deg, len(joint_names))

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0] * len(joint_names)
    sec = int(goal_time)
    nanosec = int((goal_time - sec) * 1e9)
    point.time_from_start.sec = sec
    point.time_from_start.nanosec = nanosec
    goal.trajectory.points = [point]

    send_future = client.send_goal_async(goal)
    while rclpy.ok() and not send_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)

    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print(f"[set_initial_joint_positions] Goal rejected: {action_name}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result_future = goal_handle.get_result_async()
    end_time = time.time() + wait_timeout + goal_time
    while rclpy.ok() and not result_future.done() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not result_future.done():
        print(f"[set_initial_joint_positions] Goal timed out: {action_name}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result = result_future.result()
    if result is None:
        print(f"[set_initial_joint_positions] Empty result: {action_name}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(
        "[set_initial_joint_positions] Applied initial pose (deg): "
        + " ".join(str(v) for v in joint_values_deg.strip().split())
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description="Send initial joint positions to trajectory controller.")
    parser.add_argument("--controller", default="xarm6_traj_controller")
    parser.add_argument("--joint-names", default="joint1 joint2 joint3 joint4 joint5 joint6")
    parser.add_argument("--joint-values-deg", required=True)
    parser.add_argument("--wait-timeout", type=float, default=15.0)
    parser.add_argument("--goal-time", type=float, default=3.0)
    args = parser.parse_args()

    joint_names = args.joint_names.strip().split()
    rc = send_initial_pose(
        controller_name=args.controller,
        joint_names=joint_names,
        joint_values_deg=args.joint_values_deg,
        wait_timeout=args.wait_timeout,
        goal_time=args.goal_time,
    )
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
