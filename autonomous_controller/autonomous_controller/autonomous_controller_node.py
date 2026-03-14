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
from xarm_msgs.msg import ApiRequest
from xarm_msgs.srv import MoveHome, PlanExec, PlanJoint, PlanPose, SetInt16, SetInt16ById


@dataclass
class WaypointSpec:
    name: str
    pose: Pose | None = None
    joints: list[float] | None = None


@dataclass
class TaskContext:
    unit_id: str
    request_id: str
    job_id: str
    unit_task_id: str
    payload: dict[str, Any]


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
        self.declare_parameter("planner_service_timeout_sec", 10.0)
        self.declare_parameter("planner_exec_timeout_sec", 180.0)
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
        self.planner_service_timeout_sec = max(1.0, float(self.get_parameter("planner_service_timeout_sec").value))
        self.planner_exec_timeout_sec = max(1.0, float(self.get_parameter("planner_exec_timeout_sec").value))
        self.wait_each = bool(self.get_parameter("wait_each").value)

        ns_prefix = f"/{self.hw_ns}" if self.hw_ns else ""
        default_status_topic = f"{ns_prefix}/autonomous_controller/status" if ns_prefix else "/autonomous_controller/status"
        default_active_topic = f"{ns_prefix}/autonomous_controller/active" if ns_prefix else "/autonomous_controller/active"
        self.status_topic = str(self.get_parameter("status_topic").value).strip() or default_status_topic
        self.active_topic = str(self.get_parameter("active_topic").value).strip() or default_active_topic

        self._status_publisher = self.create_publisher(String, self.status_topic, 10)
        self._active_publisher = self.create_publisher(Bool, self.active_topic, 10)
        self.create_subscription(ApiRequest, self.api_signal_topic, self._on_api_request, 10)

        self._service_clients = {
            "motion_enable": self.create_client(SetInt16ById, f"{ns_prefix}/motion_enable"),
            "set_mode": self.create_client(SetInt16, f"{ns_prefix}/set_mode"),
            "set_state": self.create_client(SetInt16, f"{ns_prefix}/set_state"),
            "move_gohome": self.create_client(MoveHome, f"{ns_prefix}/move_gohome"),
            "planner_pose": self.create_client(PlanPose, self.planner_pose_service),
            "planner_joint": self.create_client(PlanJoint, self.planner_joint_service),
            "planner_exec": self.create_client(PlanExec, self.planner_exec_service),
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
            f"waypoints_file={self.waypoints_file or '(none)'}"
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
            waypoints = self._load_waypoints(self.waypoints_file)
            self.get_logger().info(
                f"starting autonomous sequence: unit_task_id={task.unit_task_id}, waypoints={len(waypoints)}"
            )
            if not waypoints:
                raise RuntimeError(f"no waypoints found in '{self.waypoints_file}'")

            self._run_waypoint_sequence(task, waypoints)

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

    def _run_waypoint_sequence(self, task: TaskContext, waypoints: list[WaypointSpec]) -> None:
        self._publish_status(
            "running",
            task=task,
            phase="waypoint_sequence",
            message=f"loaded {len(waypoints)} waypoints from {self.waypoints_file}",
        )
        for index, waypoint in enumerate(waypoints):
            self._wait_until_resumed_or_stopped()
            if self._stop_requested:
                return

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
            else:
                raise RuntimeError(f"waypoint '{waypoint.name}' has no pose or joints")

            self._wait_until_resumed_or_stopped()
            if self._stop_requested:
                return

            self._publish_status(
                "executing",
                task=task,
                phase=waypoint.name,
                message=f"executing waypoint {index + 1}/{len(waypoints)}: {waypoint.name}",
            )
            exec_request = PlanExec.Request()
            exec_request.wait = self.wait_each
            self._call_service("planner_exec", exec_request, timeout_sec=self.planner_exec_timeout_sec)

        self._publish_status("running", task=task, phase="waypoint_sequence", message="all waypoints executed")

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

    def _load_waypoints(self, file_path: str) -> list[WaypointSpec]:
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
            if "position" in item and "orientation" in item:
                waypoints.append(WaypointSpec(name=name, pose=self._build_pose(item)))
                continue
            joints = self._extract_joint_values(item)
            if joints is not None:
                waypoints.append(WaypointSpec(name=name, joints=joints))
                continue
            raise RuntimeError(f"waypoint '{name}' must define pose or joints")

        return waypoints

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

    def _deg_to_rad(self, value_deg: float) -> float:
        return value_deg * 3.141592653589793 / 180.0

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
