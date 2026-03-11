#!/usr/bin/env python3

import argparse
import os
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def _parse_floats(line, expected):
    vals = [float(x) for x in line.strip().split()]
    if len(vals) != expected:
        raise ValueError(f"Expected {expected} floats, got {len(vals)} in line: {line!r}")
    return vals


def parse_scene_file(scene_path):
    with open(scene_path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines()]

    objects = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if not line.startswith("* "):
            i += 1
            continue

        obj_id = line[2:].strip()
        if i + 6 >= len(lines):
            break

        pos = _parse_floats(lines[i + 1], 3)
        quat = _parse_floats(lines[i + 2], 4)
        enabled = lines[i + 3]
        shape = lines[i + 4].lower()
        dims = _parse_floats(lines[i + 5], 3)

        if enabled == "1" and shape == "box":
            objects.append(
                {
                    "id": obj_id,
                    "pos": pos,
                    "quat": quat,
                    "dims": dims,
                }
            )
        i += 6

    return objects


def build_collision_object(obj, frame_id):
    msg = CollisionObject()
    msg.header.frame_id = frame_id
    msg.id = obj["id"]
    msg.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = obj["dims"]
    msg.primitives.append(primitive)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = obj["pos"]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = obj["quat"]
    msg.primitive_poses.append(pose)
    return msg


def apply_planning_scene(node, collision_objects, service_name, wait_timeout):
    client = node.create_client(ApplyPlanningScene, service_name)
    if not client.wait_for_service(timeout_sec=wait_timeout):
        print(f"[apply_field_scene] Warning: service {service_name} not available after {wait_timeout}s")
        return False

    request = ApplyPlanningScene.Request()
    request.scene = PlanningScene()
    request.scene.is_diff = True
    request.scene.robot_state.is_diff = True
    request.scene.world.collision_objects = collision_objects

    future = client.call_async(request)
    end_time = time.time() + wait_timeout
    while rclpy.ok() and not future.done() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not future.done():
        print(f"[apply_field_scene] Warning: service {service_name} call timed out")
        return False

    response = future.result()
    if response is None or not response.success:
        print(f"[apply_field_scene] Warning: service {service_name} returned failure")
        return False

    return True


def publish_collision_objects(node, collision_objects, topic, wait_timeout):
    pub = node.create_publisher(CollisionObject, topic, 10)

    start = time.time()
    while pub.get_subscription_count() == 0 and (time.time() - start) < wait_timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    if pub.get_subscription_count() == 0:
        print(f"[apply_field_scene] Warning: no subscribers on {topic} after {wait_timeout}s")
        return False

    for msg in collision_objects:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    return True


def load_scene(scene_path, frame_id, topic, service_name, wait_timeout):
    objects = parse_scene_file(scene_path)
    if not objects:
        print(f"[apply_field_scene] No enabled box objects found in: {scene_path}")
        return 1

    rclpy.init()
    node = rclpy.create_node("apply_field_scene_loader")
    collision_objects = [build_collision_object(obj, frame_id) for obj in objects]

    applied = apply_planning_scene(node, collision_objects, service_name, wait_timeout)
    if not applied:
        publish_collision_objects(node, collision_objects, topic, wait_timeout)

    for obj in objects:
        print(f"[apply_field_scene] Added: {obj['id']} dims={obj['dims']} pos={obj['pos']} frame={frame_id}")

    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main():
    parser = argparse.ArgumentParser(description="Load xarm field.scene and publish MoveIt collision objects.")
    parser.add_argument("--scene", required=True, help="Path to field.scene")
    parser.add_argument("--frame-id", default="world", help="Collision object frame_id")
    parser.add_argument("--topic", default="/collision_object", help="CollisionObject topic")
    parser.add_argument("--service", default="/apply_planning_scene", help="ApplyPlanningScene service")
    parser.add_argument("--wait-timeout", type=float, default=15.0, help="Max seconds to wait for subscriber")
    args = parser.parse_args()

    scene_path = os.path.abspath(args.scene)
    if not os.path.exists(scene_path):
        print(f"[apply_field_scene] Scene file not found: {scene_path}")
        raise SystemExit(2)

    rc = load_scene(
        scene_path=scene_path,
        frame_id=args.frame_id,
        topic=args.topic,
        service_name=args.service,
        wait_timeout=args.wait_timeout,
    )
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
