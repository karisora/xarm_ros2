from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from xarm_msgs.msg import ApiRequest
from xarm_msgs.srv import (
    GetPlannedJointTrajectory,
    MoveHome,
    PlanExec,
    PlanJoint,
    PlanPose,
    SetInt16,
    SetInt16ById,
)


@dataclass
class WaypointSpec:
    name: str
    pose: Pose | None = None
    joints: list[float] | None = None
    gripper: float | None = None


@dataclass
class TaskContext:
    unit_id: str
    request_id: str
    job_id: str
    unit_task_id: str
    payload: dict[str, Any]


@dataclass
class PathExecutionConfig:
    mode: str
    saved_path_file: str


class AutonomousControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("autonomous_controller")

        default_waypoints_file = self._default_waypoints_file()
        self.declare_parameter("api_signal_topic", "/xarm/api_requests")
        self.declare_parameter("unit_id_filter", "")
        self.declare_parameter("hw_ns", "xarm")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("active_topic", "")
        self.declare_parameter("enable_robot_on_start", False)
        self.declare_parameter("set_auto_mode_on_start", True)
        self.declare_parameter("set_ready_state_on_start", True)
        self.declare_parameter("move_home_on_start", False)
        self.declare_parameter("stop_state_value", 4)
        self.declare_parameter("pause_state_value", 3)
        self.declare_parameter("ready_state_value", 0)
        self.declare_parameter("waypoints_file", default_waypoints_file)
        self.declare_parameter("planner_pose_service", "xarm_pose_plan")
        self.declare_parameter("planner_joint_service", "xarm_joint_plan")
        self.declare_parameter("planner_exec_service", "xarm_exec_plan")
        self.declare_parameter("planner_get_trajectory_service", "xarm_get_planned_joint_trajectory")
        self.declare_parameter("planner_service_timeout_sec", 10.0)
        self.declare_parameter("planner_exec_timeout_sec", 180.0)
        self.declare_parameter("arm_controller_name", "xarm6_traj_controller")
        self.declare_parameter("arm_trajectory_topic", "")
        self.declare_parameter("gripper_controller_name", "xarm_gripper_traj_controller")
        self.declare_parameter("gripper_trajectory_topic", "")
        self.declare_parameter("gripper_joint_name", "drive_joint")
        self.declare_parameter("gripper_command_duration_sec", 1.0)
        self.declare_parameter("path_mode", "waypoint")
        self.declare_parameter("saved_path_file", "")
        self.declare_parameter("wait_each", True)

        self.api_signal_topic = str(self.get_parameter("api_signal_topic").value)
        self.unit_id_filter = str(self.get_parameter("unit_id_filter").value).strip()
        self.hw_ns = str(self.get_parameter("hw_ns").value).strip("/")
        self.enable_robot_on_start = bool(self.get_parameter("enable_robot_on_start").value)
        self.set_auto_mode_on_start = bool(self.get_parameter("set_auto_mode_on_start").value)
        self.set_ready_state_on_start = bool(self.get_parameter("set_ready_state_on_start").value)
        self.move_home_on_start = bool(self.get_parameter("move_home_on_start").value)
        self.stop_state_value = int(self.get_parameter("stop_state_value").value)
        self.pause_state_value = int(self.get_parameter("pause_state_value").value)
        self.ready_state_value = int(self.get_parameter("ready_state_value").value)
        self.waypoints_file = str(self.get_parameter("waypoints_file").value).strip()
        self.planner_pose_service = str(self.get_parameter("planner_pose_service").value).strip()
        self.planner_joint_service = str(self.get_parameter("planner_joint_service").value).strip()
        self.planner_exec_service = str(self.get_parameter("planner_exec_service").value).strip()
        self.planner_get_trajectory_service = str(
            self.get_parameter("planner_get_trajectory_service").value
        ).strip()
        self.planner_service_timeout_sec = max(1.0, float(self.get_parameter("planner_service_timeout_sec").value))
        self.planner_exec_timeout_sec = max(1.0, float(self.get_parameter("planner_exec_timeout_sec").value))
        self.arm_controller_name = str(self.get_parameter("arm_controller_name").value).strip()
        self.arm_trajectory_topic = str(self.get_parameter("arm_trajectory_topic").value).strip()
        self.gripper_controller_name = str(self.get_parameter("gripper_controller_name").value).strip()
        self.gripper_trajectory_topic = str(self.get_parameter("gripper_trajectory_topic").value).strip()
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value).strip()
        self.gripper_command_duration_sec = max(
            0.01, float(self.get_parameter("gripper_command_duration_sec").value)
        )
        self.path_mode = str(self.get_parameter("path_mode").value).strip().lower()
        self.saved_path_file = str(self.get_parameter("saved_path_file").value).strip()
        self.wait_each = bool(self.get_parameter("wait_each").value)

        ns_prefix = f"/{self.hw_ns}" if self.hw_ns else ""
        default_status_topic = f"{ns_prefix}/autonomous_controller/status" if ns_prefix else "/autonomous_controller/status"
        default_active_topic = f"{ns_prefix}/autonomous_controller/active" if ns_prefix else "/autonomous_controller/active"
        self.status_topic = str(self.get_parameter("status_topic").value).strip() or default_status_topic
        self.active_topic = str(self.get_parameter("active_topic").value).strip() or default_active_topic

        self._status_publisher = self.create_publisher(String, self.status_topic, 10)
        self._active_publisher = self.create_publisher(Bool, self.active_topic, 10)
        self.create_subscription(ApiRequest, self.api_signal_topic, self._on_api_request, 10)

        if not self.arm_trajectory_topic:
            self.arm_trajectory_topic = f"/{self.arm_controller_name}/joint_trajectory"
        self.arm_trajectory_topic = self._normalize_topic(self.arm_trajectory_topic)
        self._arm_trajectory_publisher = self.create_publisher(
            JointTrajectory, self.arm_trajectory_topic, 10
        )

        if not self.gripper_trajectory_topic:
            self.gripper_trajectory_topic = f"/{self.gripper_controller_name}/joint_trajectory"
        self.gripper_trajectory_topic = self._normalize_topic(self.gripper_trajectory_topic)
        self._gripper_trajectory_publisher = self.create_publisher(
            JointTrajectory, self.gripper_trajectory_topic, 10
        )

        self._service_clients = {
            "motion_enable": self.create_client(SetInt16ById, f"{ns_prefix}/motion_enable"),
            "set_mode": self.create_client(SetInt16, f"{ns_prefix}/set_mode"),
            "set_state": self.create_client(SetInt16, f"{ns_prefix}/set_state"),
            "move_gohome": self.create_client(MoveHome, f"{ns_prefix}/move_gohome"),
            "planner_pose": self.create_client(PlanPose, self.planner_pose_service),
            "planner_joint": self.create_client(PlanJoint, self.planner_joint_service),
            "planner_exec": self.create_client(PlanExec, self.planner_exec_service),
            "planner_get_trajectory": self.create_client(
                GetPlannedJointTrajectory, self.planner_get_trajectory_service
            ),
        }

        self._lock = threading.Lock()
        self._pause_condition = threading.Condition(self._lock)
        self._current_task: TaskContext | None = None
        self._worker_thread: threading.Thread | None = None
        self._stop_requested = False
        self._pause_requested = False
        self._current_phase = "idle"

        self._publish_active(False)
        self._publish_status("idle", message="autonomous controller is ready")
        self.get_logger().info(
            "autonomous_controller started: "
            f"api_signal_topic={self.api_signal_topic}, unit_id_filter={self.unit_id_filter or '*'}, "
            f"hw_ns={self.hw_ns or '/'}, status_topic={self.status_topic}, active_topic={self.active_topic}, "
            f"waypoints_file={self.waypoints_file or '(none)'}, path_mode={self.path_mode or 'waypoint'}"
        )

    def _on_api_request(self, msg: ApiRequest) -> None:
        if self.unit_id_filter and msg.unit_id != self.unit_id_filter:
            return

        payload = self._decode_payload(msg.payload_json)

        if msg.event_type == "unit_task_start":
            self._handle_start_request(msg, payload)
            return

        if msg.event_type == "command":
            self._handle_command_request(msg, payload)

    def _handle_start_request(self, msg: ApiRequest, payload: dict[str, Any]) -> None:
        with self._lock:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                self.get_logger().warning(
                    "ignoring unit_task_start because another sequence is already active: "
                    f"current_unit_task_id={self._current_task.unit_task_id if self._current_task else 'unknown'}"
                )
                return

            context = TaskContext(
                unit_id=msg.unit_id,
                request_id=msg.request_id,
                job_id=msg.job_id,
                unit_task_id=msg.unit_task_id,
                payload=payload,
            )
            self._current_task = context
            self._stop_requested = False
            self._pause_requested = False
            self._current_phase = "starting"
            worker = threading.Thread(
                target=self._run_sequence,
                args=(context,),
                name=f"autonomous-sequence-{msg.unit_task_id or msg.request_id}",
                daemon=True,
            )
            self._worker_thread = worker

        self._publish_active(True)
        self._publish_status(
            "accepted",
            task=context,
            message="accepted unit_task_start request",
        )
        worker.start()

    def _handle_command_request(self, msg: ApiRequest, payload: dict[str, Any]) -> None:
        command = str(payload.get("command") or msg.command_name or "").strip().lower()
        if command not in {"pause", "resume", "stop", "emergency_stop"}:
            return

        with self._lock:
            task = self._current_task
            if task is None:
                self.get_logger().debug(f"ignoring '{command}' because no autonomous sequence is active")
                return

            if msg.unit_task_id and task.unit_task_id and msg.unit_task_id != task.unit_task_id:
                return

            if command == "pause":
                self._pause_requested = True
                self._current_phase = "paused"
                self._publish_status("paused", task=task, message="pause command received")
                self._set_state_if_configured(self.pause_state_value)
                return

            if command == "resume":
                self._pause_requested = False
                self._current_phase = "running"
                self._pause_condition.notify_all()
                self._publish_status("running", task=task, message="resume command received")
                self._set_state_if_configured(self.ready_state_value)
                return

            self._stop_requested = True
            self._pause_requested = False
            self._current_phase = "stopping"
            self._pause_condition.notify_all()
            status = "emergency_stop" if command == "emergency_stop" else "stopping"
            self._publish_status(status, task=task, message=f"{command} command received")
            self._set_state_if_configured(self.stop_state_value)

    def _run_sequence(self, task: TaskContext) -> None:
        try:
            self._prepare_robot_for_sequence(task)
            parameters = self._load_waypoint_parameters(self.waypoints_file)
            path_config = self._load_path_execution_config(parameters)
            saved_path = self._resolve_saved_path(path_config.saved_path_file, self.waypoints_file)
            if path_config.mode == "replay_path" and not saved_path.is_file():
                raise RuntimeError(f"saved path file not found: {saved_path}")

            if path_config.mode in {"replay_path", "auto"} and saved_path.is_file():
                self.get_logger().info(
                    f"starting autonomous sequence from saved path: unit_task_id={task.unit_task_id}, "
                    f"saved_path={saved_path}"
                )
                self._replay_saved_path(task, saved_path)
            else:
                waypoints = self._load_waypoints_from_parameters(parameters)
                self.get_logger().info(
                    f"starting autonomous sequence: unit_task_id={task.unit_task_id}, "
                    f"waypoints={len(waypoints)}, mode={path_config.mode}"
                )
                if not waypoints:
                    raise RuntimeError(f"no waypoints found in '{self.waypoints_file}'")

                should_record = path_config.mode in {"record_path", "auto"}
                recorded_segments = self._run_waypoint_sequence(task, waypoints, capture_segments=should_record)
                if should_record:
                    if not saved_path:
                        raise RuntimeError("saved_path_file is required when path_mode is record_path or auto")
                    self._save_recorded_path(saved_path, recorded_segments)

            if self._stop_requested:
                self._publish_status("stopped", task=task, message="sequence stopped")
                return

            self._current_phase = "completed"
            self._publish_status("completed", task=task, message="autonomous sequence completed")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"autonomous sequence failed: {error}")
            self._publish_status("error", task=task, message=str(error))
        finally:
            with self._lock:
                self._current_task = None
                self._worker_thread = None
                self._stop_requested = False
                self._pause_requested = False
                self._current_phase = "idle"
            self._publish_active(False)

    def _prepare_robot_for_sequence(self, task: TaskContext) -> None:
        self._publish_status("preparing", task=task, message="preparing robot for autonomous sequence")
        if self.enable_robot_on_start:
            request = SetInt16ById.Request()
            request.id = 8
            request.data = 1
            self._call_service("motion_enable", request, timeout_sec=5.0)

        if self.set_auto_mode_on_start:
            request = SetInt16.Request()
            request.data = 0
            self._call_service("set_mode", request, timeout_sec=5.0)

        if self.set_ready_state_on_start:
            request = SetInt16.Request()
            request.data = self.ready_state_value
            self._call_service("set_state", request, timeout_sec=5.0)

        if self.move_home_on_start:
            request = MoveHome.Request()
            request.speed = 0.0
            request.acc = 0.0
            request.mvtime = 0.0
            request.wait = False
            request.timeout = -1.0
            self._call_service("move_gohome", request, timeout_sec=10.0)

    def _run_waypoint_sequence(
        self,
        task: TaskContext,
        waypoints: list[WaypointSpec],
        *,
        capture_segments: bool,
    ) -> list[dict[str, Any]]:
        recorded_segments: list[dict[str, Any]] = []
        self._publish_status(
            "running",
            task=task,
            phase="waypoint_sequence",
            message=f"loaded {len(waypoints)} waypoints from {self.waypoints_file}",
        )
        for index, waypoint in enumerate(waypoints):
            self._wait_until_resumed_or_stopped()
            if self._stop_requested:
                return recorded_segments

            with self._lock:
                self._current_phase = waypoint.name
            self._publish_status(
                "planning",
                task=task,
                phase=waypoint.name,
                message=f"planning waypoint {index + 1}/{len(waypoints)}: {waypoint.name}",
            )

            if waypoint.pose is not None:
                request = PlanPose.Request()
                request.target = waypoint.pose
                self._call_service("planner_pose", request, timeout_sec=self.planner_service_timeout_sec)
            elif waypoint.joints is not None:
                request = PlanJoint.Request()
                request.target = waypoint.joints
                self._call_service("planner_joint", request, timeout_sec=self.planner_service_timeout_sec)
            elif waypoint.gripper is None:
                raise RuntimeError(f"waypoint '{waypoint.name}' has no pose or joints")
            else:
                self.get_logger().info(f"waypoint '{waypoint.name}' has gripper-only command")

            if waypoint.pose is not None or waypoint.joints is not None:
                if capture_segments:
                    recorded_segments.append(self._capture_planned_arm_segment(waypoint.name))
                self._wait_until_resumed_or_stopped()
                if self._stop_requested:
                    return recorded_segments

                self._publish_status(
                    "executing",
                    task=task,
                    phase=waypoint.name,
                    message=f"executing waypoint {index + 1}/{len(waypoints)}: {waypoint.name}",
                )
                exec_request = PlanExec.Request()
                exec_request.wait = self.wait_each
                self._call_service("planner_exec", exec_request, timeout_sec=self.planner_exec_timeout_sec)

            if waypoint.gripper is not None:
                gripper_trajectory = self._build_gripper_trajectory(waypoint.gripper)
                if capture_segments:
                    recorded_segments.append(
                        self._make_saved_segment("gripper", f"{waypoint.name}_gripper", gripper_trajectory)
                    )
                self._wait_until_resumed_or_stopped()
                if self._stop_requested:
                    return recorded_segments

                self._publish_status(
                    "executing",
                    task=task,
                    phase=waypoint.name,
                    message=(
                        f"executing gripper for waypoint {index + 1}/{len(waypoints)}: "
                        f"{waypoint.name} ({waypoint.gripper:.3f} rad)"
                    ),
                )
                self._gripper_trajectory_publisher.publish(gripper_trajectory)
                if self.wait_each:
                    time.sleep(self.gripper_command_duration_sec)

        self._publish_status("running", task=task, phase="waypoint_sequence", message="all waypoints executed")
        return recorded_segments

    def _wait_until_resumed_or_stopped(self) -> None:
        with self._pause_condition:
            while self._pause_requested and not self._stop_requested and rclpy.ok():
                self._pause_condition.wait(timeout=0.2)

    def _set_state_if_configured(self, state_value: int) -> None:
        request = SetInt16.Request()
        request.data = int(state_value)
        try:
            self._call_service("set_state", request, timeout_sec=5.0)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"failed to set state={state_value}: {error}")

    def _call_service(self, name: str, request: Any, timeout_sec: float) -> Any:
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"service '{client.srv_name}' is unavailable")

        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                raise RuntimeError(f"service '{client.srv_name}' timed out")
            time.sleep(0.05)

        result = future.result()
        if result is None:
            raise RuntimeError(f"service '{client.srv_name}' returned no response")

        ret_value = getattr(result, "ret", 0)
        if hasattr(result, "ret") and int(ret_value) != 0:
            message = getattr(result, "message", "")
            raise RuntimeError(f"service '{client.srv_name}' failed: ret={int(ret_value)} message={message}")
        if hasattr(result, "success") and not bool(result.success):
            raise RuntimeError(f"service '{client.srv_name}' reported success=false")
        return result

    def _publish_status(
        self,
        status: str,
        *,
        task: TaskContext | None = None,
        phase: str | None = None,
        message: str = "",
    ) -> None:
        payload: dict[str, Any] = {
            "status": status,
            "phase": phase or self._current_phase,
            "message": message,
            "timestamp": self.get_clock().now().to_msg().sec,
        }
        if task is not None:
            payload.update(
                {
                    "unit_id": task.unit_id,
                    "request_id": task.request_id,
                    "job_id": task.job_id,
                    "unit_task_id": task.unit_task_id,
                }
            )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._status_publisher.publish(msg)
        if message:
            self.get_logger().info(message)

    def _publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._active_publisher.publish(msg)

    def _decode_payload(self, payload_json: str) -> dict[str, Any]:
        if not payload_json:
            return {}
        try:
            value = json.loads(payload_json)
        except json.JSONDecodeError:
            self.get_logger().warning("failed to decode payload_json from ApiRequest")
            return {}
        return value if isinstance(value, dict) else {}

    def _publish_gripper_trajectory(self, target_rad: float) -> None:
        trajectory = self._build_gripper_trajectory(target_rad)
        self._gripper_trajectory_publisher.publish(trajectory)
        self.get_logger().info(
            f"published gripper trajectory: joint={self.gripper_joint_name}, "
            f"target={target_rad:.3f} rad, topic={self.gripper_trajectory_topic}"
        )

    def _load_waypoint_parameters(self, file_path: str) -> dict[str, Any]:
        if not file_path:
            raise RuntimeError("waypoints_file is empty")

        path = Path(file_path)
        if not path.exists():
            raise RuntimeError(f"waypoints file not found: {file_path}")

        with path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}

        parameters = self._extract_ros_parameters(loaded)
        if bool(parameters.get("use_cartesian", False)):
            self.get_logger().warning(
                "use_cartesian=true is not supported by autonomous_controller; sequential waypoint execution will be used"
            )
        return parameters

    def _load_waypoints_from_parameters(self, parameters: dict[str, Any]) -> list[WaypointSpec]:
        raw_waypoints = parameters.get("waypoints", {})
        waypoint_count = self._coerce_non_negative_int(parameters.get("waypoint_count"))

        entries: list[tuple[str, dict[str, Any]]] = []
        if isinstance(raw_waypoints, list):
            for index, item in enumerate(raw_waypoints):
                if isinstance(item, dict):
                    entries.append((str(index), item))
        elif isinstance(raw_waypoints, dict):
            for key, item in sorted(raw_waypoints.items(), key=lambda item: self._sort_waypoint_key(item[0])):
                if isinstance(item, dict):
                    entries.append((str(key), item))

        if waypoint_count > 0:
            entries = entries[:waypoint_count]

        waypoints: list[WaypointSpec] = []
        for key, item in entries:
            name = str(item.get("name") or f"waypoint_{key}")
            gripper = self._extract_gripper_value(item)
            if "position" in item and "orientation" in item:
                waypoints.append(WaypointSpec(name=name, pose=self._build_pose(item), gripper=gripper))
                continue
            joints = self._extract_joint_values(item)
            if joints is not None:
                waypoints.append(WaypointSpec(name=name, joints=joints, gripper=gripper))
                continue
            if gripper is not None:
                waypoints.append(WaypointSpec(name=name, gripper=gripper))
                continue
            raise RuntimeError(f"waypoint '{name}' must define pose, joints, or gripper")

        return waypoints

    def _load_path_execution_config(self, parameters: dict[str, Any]) -> PathExecutionConfig:
        mode = str(parameters.get("path_mode", self.path_mode) or self.path_mode or "waypoint").strip().lower()
        if mode not in {"waypoint", "record_path", "replay_path", "auto"}:
            raise RuntimeError(f"unsupported path_mode: {mode}")

        saved_path_file = str(parameters.get("saved_path_file", self.saved_path_file) or self.saved_path_file).strip()
        return PathExecutionConfig(mode=mode, saved_path_file=saved_path_file)

    def _resolve_saved_path(self, saved_path_file: str, base_file: str) -> Path:
        if not saved_path_file:
            return Path("")
        path = Path(saved_path_file).expanduser()
        if path.is_absolute():
            return path
        return Path(base_file).resolve().parent / path

    def _capture_planned_arm_segment(self, name: str) -> dict[str, Any]:
        request = GetPlannedJointTrajectory.Request()
        result = self._call_service("planner_get_trajectory", request, timeout_sec=self.planner_service_timeout_sec)
        return self._make_saved_segment("arm", name, result.trajectory)

    def _build_gripper_trajectory(self, target_rad: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.gripper_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(target_rad)]
        duration_sec = self.gripper_command_duration_sec
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1_000_000_000)
        trajectory.points = [point]
        return trajectory

    def _make_saved_segment(self, kind: str, name: str, trajectory: JointTrajectory) -> dict[str, Any]:
        return {
            "kind": kind,
            "name": name,
            "trajectory": self._trajectory_to_dict(trajectory),
        }

    def _trajectory_to_dict(self, trajectory: JointTrajectory) -> dict[str, Any]:
        return {
            "joint_names": list(trajectory.joint_names),
            "points": [
                {
                    "positions": [float(value) for value in point.positions],
                    "velocities": [float(value) for value in point.velocities],
                    "accelerations": [float(value) for value in point.accelerations],
                    "effort": [float(value) for value in point.effort],
                    "time_from_start_sec": self._duration_to_sec(point.time_from_start.sec, point.time_from_start.nanosec),
                }
                for point in trajectory.points
            ],
        }

    def _trajectory_from_dict(self, data: dict[str, Any]) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = [str(name) for name in data.get("joint_names", [])]
        points: list[JointTrajectoryPoint] = []
        for raw_point in data.get("points", []):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in raw_point.get("positions", [])]
            point.velocities = [float(value) for value in raw_point.get("velocities", [])]
            point.accelerations = [float(value) for value in raw_point.get("accelerations", [])]
            point.effort = [float(value) for value in raw_point.get("effort", [])]
            sec_value = float(raw_point.get("time_from_start_sec", 0.0))
            point.time_from_start.sec = int(sec_value)
            point.time_from_start.nanosec = int((sec_value - int(sec_value)) * 1_000_000_000)
            points.append(point)
        trajectory.points = points
        return trajectory

    def _save_recorded_path(self, path: Path, segments: list[dict[str, Any]]) -> None:
        if not segments:
            raise RuntimeError("no trajectory segments were recorded")
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "source_waypoints_file": self.waypoints_file,
            "segments": segments,
        }
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        self.get_logger().info(f"saved recorded path to {path}")

    def _load_saved_path(self, path: Path) -> list[dict[str, Any]]:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        segments = loaded.get("segments")
        if not isinstance(segments, list) or not segments:
            raise RuntimeError(f"saved path file has no segments: {path}")
        return segments

    def _replay_saved_path(self, task: TaskContext, path: Path) -> None:
        segments = self._load_saved_path(path)
        self._publish_status(
            "running",
            task=task,
            phase="saved_path",
            message=f"loaded {len(segments)} saved path segments from {path}",
        )
        for index, segment in enumerate(segments):
            self._wait_until_resumed_or_stopped()
            if self._stop_requested:
                return

            kind = str(segment.get("kind", "")).strip().lower()
            name = str(segment.get("name", f"segment_{index}"))
            trajectory = self._trajectory_from_dict(segment.get("trajectory", {}))
            if not trajectory.joint_names or not trajectory.points:
                raise RuntimeError(f"saved segment '{name}' is missing trajectory points")

            self._publish_status(
                "executing",
                task=task,
                phase=name,
                message=f"replaying saved path segment {index + 1}/{len(segments)}: {name}",
            )
            if kind == "arm":
                self._arm_trajectory_publisher.publish(trajectory)
            elif kind == "gripper":
                self._gripper_trajectory_publisher.publish(trajectory)
            else:
                raise RuntimeError(f"unsupported saved path segment kind: {kind}")

            if self.wait_each:
                time.sleep(max(0.01, self._trajectory_duration_sec(trajectory)))

        self._publish_status("running", task=task, phase="saved_path", message="all saved path segments executed")

    def _extract_ros_parameters(self, loaded: Any) -> dict[str, Any]:
        if not isinstance(loaded, dict):
            raise RuntimeError("waypoints YAML must be a mapping")
        if "ros__parameters" in loaded and isinstance(loaded["ros__parameters"], dict):
            return loaded["ros__parameters"]
        for candidate in ("autonomous_controller", "eef_waypoint_commander_node"):
            node_block = loaded.get(candidate)
            if isinstance(node_block, dict) and isinstance(node_block.get("ros__parameters"), dict):
                return node_block["ros__parameters"]
        for value in loaded.values():
            if isinstance(value, dict) and isinstance(value.get("ros__parameters"), dict):
                return value["ros__parameters"]
        return loaded

    def _build_pose(self, waypoint: dict[str, Any]) -> Pose:
        position = waypoint.get("position")
        orientation = waypoint.get("orientation")
        if not isinstance(position, list) or len(position) != 3:
            raise RuntimeError("waypoint position must be a list of 3 values")
        if not isinstance(orientation, list) or len(orientation) != 4:
            raise RuntimeError("waypoint orientation must be a list of 4 values")
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])
        return pose

    def _extract_joint_values(self, waypoint: dict[str, Any]) -> list[float] | None:
        if isinstance(waypoint.get("joints"), list):
            return [float(value) for value in waypoint["joints"]]
        if isinstance(waypoint.get("joints_rad"), list):
            return [float(value) for value in waypoint["joints_rad"]]
        if isinstance(waypoint.get("joints_deg"), list):
            return [self._deg_to_rad(float(value)) for value in waypoint["joints_deg"]]
        return None

    def _extract_gripper_value(self, waypoint: dict[str, Any]) -> float | None:
        if "gripper" in waypoint:
            return float(waypoint["gripper"])
        if "gripper_deg" in waypoint:
            return self._deg_to_rad(float(waypoint["gripper_deg"]))
        return None

    def _coerce_non_negative_int(self, value: Any) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return 0
        return max(0, parsed)

    def _sort_waypoint_key(self, value: Any) -> tuple[int, str]:
        try:
            return (0, f"{int(value):09d}")
        except (TypeError, ValueError):
            return (1, str(value))

    def _duration_to_sec(self, sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) / 1_000_000_000.0

    def _trajectory_duration_sec(self, trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        point = trajectory.points[-1]
        return self._duration_to_sec(point.time_from_start.sec, point.time_from_start.nanosec)

    def _deg_to_rad(self, value_deg: float) -> float:
        return value_deg * 3.141592653589793 / 180.0

    def _normalize_topic(self, topic: str) -> str:
        topic = topic.strip()
        if not topic:
            return topic
        return topic if topic.startswith("/") else f"/{topic}"

    def _default_waypoints_file(self) -> str:
        try:
            package_share = Path(get_package_share_directory("xarm_planner"))
        except PackageNotFoundError:
            return ""
        return str(package_share / "config" / "eef_waypoints_example.yaml")


def main() -> None:
    rclpy.init(args=None)
    node = AutonomousControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
