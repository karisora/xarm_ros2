from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from xarm_msgs.msg import ApiRequest
from xarm_msgs.srv import MoveHome, SetInt16, SetInt16ById


@dataclass
class SequenceStep:
    name: str
    duration_sec: float


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
            f"hw_ns={self.hw_ns or '/'}, status_topic={self.status_topic}, active_topic={self.active_topic}"
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
            steps = self._build_steps(task.payload)
            self.get_logger().info(
                f"starting autonomous sequence: unit_task_id={task.unit_task_id}, steps={len(steps)}"
            )
            if not steps:
                self._publish_status("running", task=task, message="no timed steps defined; sequence completed immediately")
                return

            for step in steps:
                if self._stop_requested:
                    self._publish_status("stopped", task=task, message=f"sequence stopped before '{step.name}'")
                    return
                self._run_step(task, step)

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

    def _build_steps(self, payload: dict[str, Any]) -> list[SequenceStep]:
        params = payload.get("params", {})
        if not isinstance(params, dict):
            return []
        common_params = params.get("common_params", {})
        if not isinstance(common_params, dict):
            return []

        preparation = common_params.get("preparation", {})
        step1 = common_params.get("step1_slurry_injection", {})
        step2 = common_params.get("step2_filtration_wash", {})
        if not isinstance(preparation, dict):
            preparation = {}
        if not isinstance(step1, dict):
            step1 = {}
        if not isinstance(step2, dict):
            step2 = {}

        steps: list[SequenceStep] = []
        steps.append(SequenceStep("preparation_feed", self._as_duration(preparation.get("feed_time_sec"))))
        steps.append(
            SequenceStep(
                "preparation_depressurization",
                self._as_duration(preparation.get("depressurization_time_sec")),
            )
        )
        steps.append(SequenceStep("step1_agitation", self._as_duration(step1.get("agitation_time_sec"))))

        rewash_count = self._as_count(step1.get("rewash_count"))
        rewash_feed = self._as_duration(step1.get("rewash_feed_time_sec"))
        rewash_agitation = self._as_duration(step1.get("agitation_time_sec"))
        for index in range(rewash_count):
            cycle_no = index + 1
            steps.append(SequenceStep(f"step1_rewash_{cycle_no}_feed", rewash_feed))
            steps.append(SequenceStep(f"step1_rewash_{cycle_no}_agitation", rewash_agitation))

        steps.append(
            SequenceStep(
                "step1_additional_depressurization",
                self._as_duration(step1.get("additional_depressurization_before_end_sec")),
            )
        )

        wash_count = self._as_count(step2.get("wash_count"))
        wash_feed = self._as_duration(step2.get("feed_time_sec"))
        wash_dep = self._as_duration(step2.get("depressurization_time_sec"))
        for index in range(wash_count):
            cycle_no = index + 1
            steps.append(SequenceStep(f"step2_wash_{cycle_no}_feed", wash_feed))
            steps.append(SequenceStep(f"step2_wash_{cycle_no}_depressurization", wash_dep))

        return [step for step in steps if step.duration_sec > 0.0]

    def _run_step(self, task: TaskContext, step: SequenceStep) -> None:
        with self._lock:
            self._current_phase = step.name
        self._publish_status(
            "running",
            task=task,
            phase=step.name,
            message=f"running phase '{step.name}' for {step.duration_sec:.1f}s",
        )

        deadline = time.monotonic() + step.duration_sec
        while rclpy.ok():
            with self._pause_condition:
                while self._pause_requested and not self._stop_requested and rclpy.ok():
                    self._pause_condition.wait(timeout=0.2)
            if self._stop_requested:
                return
            if time.monotonic() >= deadline:
                return
            time.sleep(0.1)

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
        if int(ret_value) != 0:
            message = getattr(result, "message", "")
            raise RuntimeError(f"service '{client.srv_name}' failed: ret={int(ret_value)} message={message}")
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

    def _as_duration(self, value: Any) -> float:
        try:
            duration = float(value)
        except (TypeError, ValueError):
            return 0.0
        return max(0.0, duration)

    def _as_count(self, value: Any) -> int:
        try:
            count = int(value)
        except (TypeError, ValueError):
            return 0
        return max(0, count)


def main() -> None:
    rclpy.init(args=None)
    node = AutonomousControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
