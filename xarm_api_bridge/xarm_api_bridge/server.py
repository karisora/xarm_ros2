from __future__ import annotations

import math
import os
import threading
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Optional

import rclpy
import uvicorn
from fastapi import APIRouter, Depends, FastAPI, Header, HTTPException, Query, Request, Response, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, GetFloat32List, GripperMove, MoveHome, MoveJoint, SetDigitalIO, SetInt16, SetInt16ById


def _now_local() -> datetime:
    return datetime.now(timezone.utc).astimezone()


def _iso(dt: datetime) -> str:
    return dt.isoformat(timespec="seconds")


def _to_deg(rad_value: float) -> float:
    return math.degrees(rad_value)


def _to_rad(deg_value: float) -> float:
    return math.radians(deg_value)


def _source_to_suffix(command_source: str) -> str:
    return "remote" if command_source == "upstream" else "local"


def _api_error(status_code: int, code: str, message: str, detail: Optional[dict[str, Any]] = None) -> HTTPException:
    payload: dict[str, Any] = {"code": code, "message": message}
    if detail:
        payload["detail"] = detail
    return HTTPException(status_code=status_code, detail=payload)


class StartUnitTaskPayload(BaseModel):
    job_id: str
    unit_task_id: str
    params: dict[str, Any]
    command_source: str


class CommandRequest(BaseModel):
    command: str
    target_unit: Optional[str] = None
    command_source: str


class RobotEnableRequest(BaseModel):
    command_source: str


class ControlAuthorityRequest(BaseModel):
    control_authority: str


class ArmPoseCheckRequest(BaseModel):
    reference: str = "sequence_start"
    joint_tolerance_deg: float = Field(default=3.0, gt=0.0, le=10.0)


class ArmMoveInitialPoseRequest(BaseModel):
    target: str = "sequence_start"


@dataclass
class TaskRuntime:
    job_id: str
    unit_task_id: str
    params: dict[str, Any]
    command_source: str
    start_time: datetime
    duration_sec: float
    status: str = "running"
    paused_at: Optional[datetime] = None
    paused_total_sec: float = 0.0
    progress_percent: int = 0

    def elapsed_sec(self, now: datetime) -> int:
        paused_current = 0.0
        if self.paused_at is not None:
            paused_current = max(0.0, (now - self.paused_at).total_seconds())
        value = (now - self.start_time).total_seconds() - self.paused_total_sec - paused_current
        return max(0, int(value))

    def estimated_end_time(self) -> datetime:
        return self.start_time + timedelta_seconds(self.duration_sec)


def timedelta_seconds(seconds: float):
    from datetime import timedelta

    return timedelta(seconds=float(seconds))


class BridgeOperationError(Exception):
    def __init__(self, code: str, message: str, *, status_code: int = 503, detail: Optional[dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.status_code = status_code
        self.detail = detail or {}


class XArmApiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("xarm_api_bridge")

        self.declare_parameter("api_host", "127.0.0.1")
        self.declare_parameter("api_port", 8000)
        self.declare_parameter("api_key", os.getenv("FILTRATION_API_KEY", ""))
        self.declare_parameter("unit_id", "unit-xarm01")
        self.declare_parameter("hw_ns", "xarm")
        self.declare_parameter("task_duration_sec", 90.0)
        self.declare_parameter("auto_complete_task", True)
        self.declare_parameter("default_control_authority", "remote")
        self.declare_parameter("initial_pose_deg", [0.0, -30.0, 0.0, 0.0, 30.0, 0.0])
        self.declare_parameter("move_joint_speed", 0.5)
        self.declare_parameter("move_joint_acc", 5.0)
        self.declare_parameter("move_joint_timeout_sec", 120.0)
        self.declare_parameter("object_centrifuge_tube_rack_installed", True)
        self.declare_parameter("object_funnel_rack_installed", True)
        self.declare_parameter("gripper_open_pos", 850.0)
        self.declare_parameter("gripper_close_pos", 0.0)
        self.declare_parameter("manual_pump_io", -1)
        self.declare_parameter("manual_waste_io", -1)
        self.declare_parameter("manual_servo_io", -1)
        self.declare_parameter("auto_set_mode_state_on_enable", True)

        self.api_host = str(self.get_parameter("api_host").value)
        self.api_port = int(self.get_parameter("api_port").value)
        self.api_key = str(self.get_parameter("api_key").value)
        self.unit_id = str(self.get_parameter("unit_id").value)
        self.hw_ns = str(self.get_parameter("hw_ns").value).strip("/")
        self.task_duration_sec = float(self.get_parameter("task_duration_sec").value)
        self.auto_complete_task = bool(self.get_parameter("auto_complete_task").value)
        self.initial_pose_deg = [float(v) for v in self.get_parameter("initial_pose_deg").value]
        self.move_joint_speed = float(self.get_parameter("move_joint_speed").value)
        self.move_joint_acc = float(self.get_parameter("move_joint_acc").value)
        self.move_joint_timeout_sec = float(self.get_parameter("move_joint_timeout_sec").value)
        self.object_centrifuge_tube_rack_installed = bool(
            self.get_parameter("object_centrifuge_tube_rack_installed").value
        )
        self.object_funnel_rack_installed = bool(self.get_parameter("object_funnel_rack_installed").value)
        self.gripper_open_pos = float(self.get_parameter("gripper_open_pos").value)
        self.gripper_close_pos = float(self.get_parameter("gripper_close_pos").value)
        self.manual_pump_io = int(self.get_parameter("manual_pump_io").value)
        self.manual_waste_io = int(self.get_parameter("manual_waste_io").value)
        self.manual_servo_io = int(self.get_parameter("manual_servo_io").value)
        self.auto_set_mode_state_on_enable = bool(self.get_parameter("auto_set_mode_state_on_enable").value)

        default_authority = str(self.get_parameter("default_control_authority").value)
        if default_authority not in ("local", "remote"):
            default_authority = "remote"

        self._lock = threading.RLock()
        self._operation_mode = "auto"
        self._control_authority = default_authority
        self._robot_mode_enabled = False
        self._current_task: Optional[TaskRuntime] = None
        self._task_results: dict[str, dict[str, Any]] = {}
        self._task_history: list[dict[str, Any]] = []
        self._terminal_status: Optional[str] = None
        self._terminal_message: Optional[str] = None
        self._last_robot_msg: Optional[RobotMsg] = None
        self._last_command_source = "gui"
        self._moves: dict[str, dict[str, Any]] = {}

        ns_prefix = f"/{self.hw_ns}" if self.hw_ns else ""
        self._service_names = {
            "motion_enable": f"{ns_prefix}/motion_enable",
            "set_mode": f"{ns_prefix}/set_mode",
            "set_state": f"{ns_prefix}/set_state",
            "clean_error": f"{ns_prefix}/clean_error",
            "clean_warn": f"{ns_prefix}/clean_warn",
            "move_gohome": f"{ns_prefix}/move_gohome",
            "get_servo_angle": f"{ns_prefix}/get_servo_angle",
            "set_servo_angle": f"{ns_prefix}/set_servo_angle",
            "set_gripper_position": f"{ns_prefix}/set_gripper_position",
            "set_tgpio_digital": f"{ns_prefix}/set_tgpio_digital",
        }

        self._clients = {
            "motion_enable": self.create_client(SetInt16ById, self._service_names["motion_enable"]),
            "set_mode": self.create_client(SetInt16, self._service_names["set_mode"]),
            "set_state": self.create_client(SetInt16, self._service_names["set_state"]),
            "clean_error": self.create_client(Call, self._service_names["clean_error"]),
            "clean_warn": self.create_client(Call, self._service_names["clean_warn"]),
            "move_gohome": self.create_client(MoveHome, self._service_names["move_gohome"]),
            "get_servo_angle": self.create_client(GetFloat32List, self._service_names["get_servo_angle"]),
            "set_servo_angle": self.create_client(MoveJoint, self._service_names["set_servo_angle"]),
            "set_gripper_position": self.create_client(GripperMove, self._service_names["set_gripper_position"]),
            "set_tgpio_digital": self.create_client(SetDigitalIO, self._service_names["set_tgpio_digital"]),
        }

        self.create_subscription(RobotMsg, f"{ns_prefix}/robot_states", self._on_robot_state, 10)
        self.get_logger().info(
            f"xarm_api_bridge started: ns=/{self.hw_ns}, api={self.api_host}:{self.api_port}, unit_id={self.unit_id}"
        )

    def _on_robot_state(self, msg: RobotMsg) -> None:
        with self._lock:
            self._last_robot_msg = msg
            # mt_able bitmask > 0 means at least one joint is enabled
            self._robot_mode_enabled = bool(msg.mt_able)

    def _require_command_source(self, command_source: str) -> None:
        if command_source not in ("gui", "upstream"):
            raise BridgeOperationError(
                "INVALID_COMMAND_SOURCE",
                "command_source must be one of: gui, upstream",
                status_code=400,
            )
        if command_source == "upstream" and self._control_authority != "remote":
            raise BridgeOperationError(
                "CONTROL_AUTHORITY_REJECTED",
                "upstream command is rejected while control_authority is local",
                status_code=403,
            )

    def _call_service(self, key: str, request: Any, timeout_sec: float = 3.0) -> Any:
        client = self._clients[key]
        service_name = self._service_names[key]
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise BridgeOperationError(
                "ROS_SERVICE_UNAVAILABLE",
                f"ROS service is not available: {service_name}",
                status_code=503,
            )
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            future.cancel()
            raise BridgeOperationError(
                "ROS_SERVICE_TIMEOUT",
                f"ROS service timeout: {service_name}",
                status_code=504,
            )
        exc = future.exception()
        if exc is not None:
            raise BridgeOperationError(
                "ROS_SERVICE_ERROR",
                f"ROS service call failed: {service_name}",
                detail={"exception": str(exc)},
            )
        return future.result()

    def _set_mode(self, mode: int) -> None:
        req = SetInt16.Request()
        req.data = int(mode)
        res = self._call_service("set_mode", req)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_SET_MODE_FAILED",
                "set_mode failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _set_state(self, state_value: int) -> None:
        req = SetInt16.Request()
        req.data = int(state_value)
        res = self._call_service("set_state", req)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_SET_STATE_FAILED",
                "set_state failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _clean_error(self) -> None:
        res = self._call_service("clean_error", Call.Request())
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_CLEAN_ERROR_FAILED",
                "clean_error failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _clean_warn(self) -> None:
        res = self._call_service("clean_warn", Call.Request())
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_CLEAN_WARN_FAILED",
                "clean_warn failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _move_home(self) -> None:
        req = MoveHome.Request()
        req.speed = 0.0
        req.acc = 0.0
        req.mvtime = 0.0
        req.wait = False
        req.timeout = -1.0
        res = self._call_service("move_gohome", req, timeout_sec=5.0)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_MOVE_HOME_FAILED",
                "move_gohome failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _enable_robot(self) -> None:
        req = SetInt16ById.Request()
        req.id = 8
        req.data = 1
        res = self._call_service("motion_enable", req, timeout_sec=5.0)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_MOTION_ENABLE_FAILED",
                "motion_enable failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )
        self._robot_mode_enabled = True

    def _set_gripper(self, state: str) -> None:
        req = GripperMove.Request()
        req.pos = float(self.gripper_open_pos if state == "open" else self.gripper_close_pos)
        req.wait = False
        req.timeout = 10.0
        res = self._call_service("set_gripper_position", req, timeout_sec=5.0)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_GRIPPER_FAILED",
                "set_gripper_position failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def _set_tgpio_digital(self, ionum: int, value: int) -> None:
        req = SetDigitalIO.Request()
        req.ionum = int(ionum)
        req.value = int(value)
        req.delay_sec = 0.0
        req.xyz = []
        req.tol_r = 0.0
        res = self._call_service("set_tgpio_digital", req, timeout_sec=5.0)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_SET_DIGITAL_IO_FAILED",
                "set_tgpio_digital failed",
                detail={"ret": int(res.ret), "message": str(res.message), "ionum": ionum, "value": value},
            )

    def _get_joint_angles_deg(self) -> list[float]:
        res = self._call_service("get_servo_angle", GetFloat32List.Request(), timeout_sec=5.0)
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_GET_SERVO_ANGLE_FAILED",
                "get_servo_angle failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )
        values = [float(v) for v in res.datas]
        if len(values) < len(self.initial_pose_deg):
            raise BridgeOperationError(
                "ROS_GET_SERVO_ANGLE_INVALID",
                "get_servo_angle response does not contain enough joints",
                detail={"joint_count": len(values), "required": len(self.initial_pose_deg)},
            )
        return [_to_deg(v) for v in values[: len(self.initial_pose_deg)]]

    def _execute_initial_pose_move(self) -> None:
        req = MoveJoint.Request()
        req.angles = [_to_rad(v) for v in self.initial_pose_deg]
        req.speed = float(self.move_joint_speed)
        req.acc = float(self.move_joint_acc)
        req.mvtime = 0.0
        if hasattr(req, "wait"):
            req.wait = True
        if hasattr(req, "timeout"):
            req.timeout = float(self.move_joint_timeout_sec)
        if hasattr(req, "radius"):
            req.radius = -1.0
        if hasattr(req, "relative"):
            req.relative = False
        res = self._call_service("set_servo_angle", req, timeout_sec=max(5.0, self.move_joint_timeout_sec))
        if int(res.ret) != 0:
            raise BridgeOperationError(
                "ROS_SET_SERVO_ANGLE_FAILED",
                "set_servo_angle failed",
                detail={"ret": int(res.ret), "message": str(res.message)},
            )

    def start_move_to_initial_pose(self) -> str:
        move_id = f"move_{uuid.uuid4().hex[:8]}"
        with self._lock:
            self._moves[move_id] = {"status": "queued", "message": "queued"}

        def worker() -> None:
            with self._lock:
                if move_id in self._moves:
                    self._moves[move_id]["status"] = "moving"
                    self._moves[move_id]["message"] = "moving"
            try:
                self._execute_initial_pose_move()
                with self._lock:
                    if move_id in self._moves:
                        self._moves[move_id]["status"] = "completed"
                        self._moves[move_id]["message"] = "completed"
            except BridgeOperationError as error:
                with self._lock:
                    if move_id in self._moves:
                        self._moves[move_id]["status"] = "error"
                        self._moves[move_id]["message"] = error.message
            except Exception as error:  # noqa: BLE001
                with self._lock:
                    if move_id in self._moves:
                        self._moves[move_id]["status"] = "error"
                        self._moves[move_id]["message"] = str(error)

        threading.Thread(target=worker, daemon=True).start()
        return move_id

    def get_move_status(self, move_id: str) -> dict[str, Any]:
        with self._lock:
            status_obj = self._moves.get(move_id)
            if status_obj is None:
                raise BridgeOperationError("MOVE_NOT_FOUND", f"move_id not found: {move_id}", status_code=404)
            return {"move_id": move_id, "status": status_obj["status"], "message": status_obj.get("message")}

    def _tick_task_locked(self, now: datetime) -> None:
        task = self._current_task
        if task is None or task.status != "running":
            return
        elapsed = task.elapsed_sec(now)
        if task.duration_sec > 0:
            task.progress_percent = max(0, min(99, int((elapsed / task.duration_sec) * 100)))
        if self.auto_complete_task and elapsed >= int(task.duration_sec):
            self._finalize_task_locked(
                result_kind="success",
                result_message="Task completed",
                command_source=task.command_source,
            )

    def _finalize_task_locked(self, result_kind: str, result_message: str, command_source: str) -> None:
        task = self._current_task
        if task is None:
            return
        now = _now_local()
        end_time = _iso(now)
        start_time = _iso(task.start_time)
        elapsed = max(0, task.elapsed_sec(now))

        tray = task.params.get("tray", {}) if isinstance(task.params, dict) else {}
        positions = tray.get("positions", []) if isinstance(tray, dict) else []
        if not isinstance(positions, list):
            positions = []
        enabled_units = task.params.get("enabled_units", ["A", "B", "C", "D"])
        if not isinstance(enabled_units, list) or len(enabled_units) == 0:
            enabled_units = ["A", "B", "C", "D"]

        position_results: list[dict[str, Any]] = []
        for index, position in enumerate(positions):
            pos_no = 0
            lot_no = ""
            if isinstance(position, dict):
                pos_no = int(position.get("position_no", 0))
                sample = position.get("sample", {})
                if isinstance(sample, dict):
                    lot_no = str(sample.get("lot_no", ""))
            unit = str(enabled_units[index % len(enabled_units)])
            if result_kind == "success":
                pos_result = "success"
            elif result_kind in ("stop_local", "stop_remote", "emergency_stop_local", "emergency_stop_remote"):
                pos_result = "not_executed"
            else:
                pos_result = "error"
            position_results.append(
                {
                    "position_no": pos_no,
                    "lot_no": lot_no,
                    "processed_unit": unit,
                    "result": pos_result,
                    "error_code": None,
                }
            )

        task_result = {
            "unit_id": self.unit_id,
            "job_id": task.job_id,
            "unit_task_id": task.unit_task_id,
            "task_id_source": "remote",
            "start_time": start_time,
            "end_time": end_time,
            "duration_sec": elapsed,
            "result": result_kind,
            "result_message": result_message,
            "error_code": None,
            "alarm_level": None,
            "details": {
                "requested_params": task.params,
                "tray_result": {
                    "tray_id": str(tray.get("tray_id", "")) if isinstance(tray, dict) else "",
                    "position_results": position_results,
                },
            },
        }

        self._task_results[task.unit_task_id] = task_result
        self._task_history.append(task_result)
        self._current_task = None

    def start_unit_task(self, payload: StartUnitTaskPayload) -> dict[str, Any]:
        self._require_command_source(payload.command_source)

        with self._lock:
            now = _now_local()
            self._tick_task_locked(now)
            if self._terminal_status is not None:
                raise BridgeOperationError(
                    "STATE_CONFLICT",
                    "reset_state is required before starting a new task",
                    status_code=409,
                    detail={"status": self._terminal_status},
                )
            if self._current_task is not None:
                raise BridgeOperationError(
                    "TASK_ALREADY_RUNNING",
                    "a task is already running",
                    status_code=409,
                    detail={"unit_task_id": self._current_task.unit_task_id},
                )
            if self._operation_mode != "auto":
                raise BridgeOperationError("STATE_CONFLICT", "operation_mode must be auto", status_code=409)
            if not self._robot_mode_enabled:
                raise BridgeOperationError("STATE_CONFLICT", "robot mode is disabled", status_code=409)
            if not (self.object_centrifuge_tube_rack_installed and self.object_funnel_rack_installed):
                raise BridgeOperationError(
                    "STATE_CONFLICT",
                    "required objects are not installed",
                    status_code=409,
                )

            duration = self._estimate_task_duration(payload.params)
            self._current_task = TaskRuntime(
                job_id=payload.job_id,
                unit_task_id=payload.unit_task_id,
                params=payload.params,
                command_source=payload.command_source,
                start_time=now,
                duration_sec=duration,
            )
            self._last_command_source = payload.command_source

        return {"unit_task_id": payload.unit_task_id, "status": "running"}

    def _estimate_task_duration(self, params: dict[str, Any]) -> float:
        try:
            common_params = params.get("common_params", {})
            preparation = common_params.get("preparation", {})
            step1 = common_params.get("step1_slurry_injection", {})
            step2 = common_params.get("step2_filtration_wash", {})

            prep_sec = float(preparation.get("feed_time_sec", 0)) + float(preparation.get("depressurization_time_sec", 0))
            agitation = float(step1.get("agitation_time_sec", 0))
            rewash_count = int(step1.get("rewash_count", 0))
            rewash_feed = float(step1.get("rewash_feed_time_sec", 0))
            step1_post = float(step1.get("additional_depressurization_before_end_sec", 0))
            step2_count = int(step2.get("wash_count", 0))
            step2_feed = float(step2.get("feed_time_sec", 0))
            step2_dep = float(step2.get("depressurization_time_sec", 0))

            step1_sec = agitation + rewash_count * (rewash_feed + 5.0 + agitation + 5.0) + step1_post
            step2_sec = step2_count * (step2_feed + step2_dep)
            total = prep_sec + step1_sec + step2_sec
            return max(1.0, total) if total > 0 else self.task_duration_sec
        except Exception:  # noqa: BLE001
            return self.task_duration_sec

    def send_command(self, payload: CommandRequest) -> None:
        self._require_command_source(payload.command_source)
        command = payload.command
        with self._lock:
            now = _now_local()
            self._tick_task_locked(now)
            task = self._current_task
            self._last_command_source = payload.command_source

        if command == "pause":
            if task is None or task.status != "running":
                raise BridgeOperationError("STATE_CONFLICT", "pause is only valid while running", status_code=409)
            self._set_state(3)
            with self._lock:
                if self._current_task is not None and self._current_task.status == "running":
                    self._current_task.status = "paused"
                    self._current_task.paused_at = _now_local()
            return

        if command == "resume":
            if task is None or task.status != "paused":
                raise BridgeOperationError("STATE_CONFLICT", "resume is only valid while paused", status_code=409)
            self._set_state(0)
            with self._lock:
                if self._current_task is not None and self._current_task.status == "paused":
                    pause_end = _now_local()
                    if self._current_task.paused_at is not None:
                        self._current_task.paused_total_sec += (pause_end - self._current_task.paused_at).total_seconds()
                    self._current_task.paused_at = None
                    self._current_task.status = "running"
            return

        if command == "stop":
            if task is None:
                raise BridgeOperationError("STATE_CONFLICT", "stop is only valid while running/paused", status_code=409)
            self._set_state(4)
            with self._lock:
                suffix = _source_to_suffix(payload.command_source)
                self._finalize_task_locked(
                    result_kind=f"stop_{suffix}",
                    result_message="Stopped by command",
                    command_source=payload.command_source,
                )
                self._terminal_status = f"stop_{suffix}"
                self._terminal_message = "Stopped by command"
            return

        if command == "emergency_stop":
            self._set_state(4)
            with self._lock:
                suffix = _source_to_suffix(payload.command_source)
                self._finalize_task_locked(
                    result_kind=f"emergency_stop_{suffix}",
                    result_message="Emergency stop command accepted",
                    command_source=payload.command_source,
                )
                self._terminal_status = f"emergency_stop_{suffix}"
                self._terminal_message = "Emergency stop command accepted"
            return

        if command == "reset_state":
            with self._lock:
                if self._terminal_status is None and self._is_alarm_locked() is False:
                    raise BridgeOperationError(
                        "STATE_CONFLICT",
                        "reset_state is only valid in stop/alarm/emergency states",
                        status_code=409,
                    )
            self._clean_error()
            self._clean_warn()
            self._set_state(0)
            with self._lock:
                self._terminal_status = None
                self._terminal_message = None
            return

        raise BridgeOperationError("INVALID_COMMAND", f"unsupported command: {command}", status_code=400)

    def _is_alarm_locked(self) -> bool:
        msg = self._last_robot_msg
        if msg is None:
            return False
        return bool(msg.err or msg.warn)

    def set_control_authority(self, authority: str) -> None:
        if authority not in ("local", "remote"):
            raise BridgeOperationError("INVALID_CONTROL_AUTHORITY", "control_authority must be local or remote", status_code=400)
        with self._lock:
            self._control_authority = authority

    def enable_robot(self, payload: RobotEnableRequest) -> dict[str, Any]:
        self._require_command_source(payload.command_source)
        self._enable_robot()
        if self.auto_set_mode_state_on_enable:
            self._set_mode(0)
            self._set_state(0)
        with self._lock:
            self._operation_mode = "auto"
        return {"accepted": True, "message": "Robot enabled"}

    def send_manual_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action", ""))
        command_source = str(payload.get("command_source", ""))
        self._require_command_source(command_source)

        if action == "arm_mode":
            mode = str(payload.get("mode", ""))
            if mode in ("auto", "remote"):
                self._set_mode(0)
                self._set_state(0)
                with self._lock:
                    self._operation_mode = "auto"
                return {"accepted": True, "message": "arm_mode set to auto"}
            if mode in ("manual", "local"):
                self._set_mode(2)
                self._set_state(0)
                with self._lock:
                    self._operation_mode = "manual"
                return {"accepted": True, "message": "arm_mode set to manual"}
            return {"accepted": False, "message": "invalid mode"}

        if action == "gripper":
            state = str(payload.get("state", ""))
            if state not in ("open", "close"):
                return {"accepted": False, "message": "gripper state must be open/close"}
            self._set_gripper(state)
            return {"accepted": True, "message": f"gripper {state}"}

        if action in ("pump", "waste", "servo"):
            io_map = {"pump": self.manual_pump_io, "waste": self.manual_waste_io, "servo": self.manual_servo_io}
            ionum = io_map[action]
            if ionum < 0:
                return {"accepted": False, "message": f"{action} is not mapped to a digital IO channel"}
            mode = str(payload.get("mode", ""))
            if mode not in ("run", "stop"):
                return {"accepted": False, "message": "mode must be run/stop"}
            value = 1 if mode == "run" else 0
            self._set_tgpio_digital(ionum, value)
            duration = payload.get("duration_sec")
            if mode == "run" and duration is not None:
                try:
                    duration_sec = max(0.0, float(duration))
                except Exception:  # noqa: BLE001
                    duration_sec = 0.0
                if duration_sec > 0.0:
                    timer = threading.Timer(duration_sec, lambda: self._set_tgpio_digital(ionum, 0))
                    timer.daemon = True
                    timer.start()
            return {"accepted": True, "message": f"{action} {mode}"}

        if action == "stage":
            return {"accepted": False, "message": "stage command is not mapped in this bridge"}

        return {"accepted": False, "message": f"unsupported manual action: {action}"}

    def check_pose(self, payload: ArmPoseCheckRequest) -> dict[str, Any]:
        if payload.reference != "sequence_start":
            raise BridgeOperationError("INVALID_REFERENCE", "reference must be sequence_start", status_code=400)
        current = self._get_joint_angles_deg()
        reference = self.initial_pose_deg[: len(current)]
        tolerance = float(payload.joint_tolerance_deg)
        deltas = [current[i] - reference[i] for i in range(len(reference))]
        within = all(abs(v) <= tolerance for v in deltas)
        return {
            "reference": "sequence_start",
            "joint_count": len(reference),
            "joint_tolerance_deg": tolerance,
            "all_within_tolerance": within,
            "current_joints_deg": current,
            "reference_joints_deg": reference,
            "delta_joints_deg": deltas,
        }

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            now = _now_local()
            self._tick_task_locked(now)
            is_alarm = self._is_alarm_locked()
            status_value = "idle"
            if is_alarm:
                status_value = "alarm"
            elif self._terminal_status is not None:
                status_value = self._terminal_status
            elif self._current_task is not None:
                suffix = _source_to_suffix(self._current_task.command_source)
                if self._current_task.status == "running":
                    status_value = f"running_{suffix}"
                elif self._current_task.status == "paused":
                    status_value = f"pause_{suffix}"
            elif self._robot_mode_enabled and self._operation_mode == "auto":
                status_value = "standby"

            alarm_obj = None
            if is_alarm and self._last_robot_msg is not None:
                alarm_level = "error"
                code = self._last_robot_msg.err if self._last_robot_msg.err else self._last_robot_msg.warn
                alarm_obj = {
                    "code": f"XARM_{int(code)}",
                    "alarm_level": alarm_level,
                    "message": f"xArm reported error/warn code: {int(code)}",
                    "recovery_method": "Run reset_state after clearing robot errors",
                    "occurred_at": _iso(now),
                }

            unit_task_obj = None
            if self._current_task is not None:
                task = self._current_task
                unit_task_obj = {
                    "job_id": task.job_id,
                    "unit_task_id": task.unit_task_id,
                    "task_id_source": "remote",
                    "status": "paused" if task.status == "paused" else "running",
                    "start_time": _iso(task.start_time),
                    "estimated_end_time": _iso(task.estimated_end_time()),
                    "elapsed_time_sec": task.elapsed_sec(now),
                    "progress_percent": task.progress_percent,
                    "phase": "execution" if task.status == "running" else "paused",
                    "message": "Task running" if task.status == "running" else "Task paused",
                    "end_time": None,
                    "result": None,
                }

            filtration_units = {
                "A": {"status": "idle"},
                "B": {"status": "idle"},
                "C": {"status": "idle"},
                "D": {"status": "idle"},
            }
            if self._current_task is not None:
                sample_lot_no = ""
                tray = self._current_task.params.get("tray", {})
                positions = tray.get("positions", []) if isinstance(tray, dict) else []
                if isinstance(positions, list) and len(positions) > 0 and isinstance(positions[0], dict):
                    sample = positions[0].get("sample", {})
                    if isinstance(sample, dict):
                        sample_lot_no = str(sample.get("lot_no", ""))
                unit_status = "paused" if self._current_task.status == "paused" else "running"
                filtration_units["A"] = {
                    "status": unit_status,
                    "phase": "step1_slurry_injection" if unit_status == "running" else "paused",
                    "progress_percent": self._current_task.progress_percent,
                    "sample_lot_no": sample_lot_no or None,
                }

            state = {
                "unit_id": self.unit_id,
                "status": status_value,
                "operation_mode": self._operation_mode,
                "control_authority": self._control_authority,
                "details": {
                    "robot_mode_enabled": self._robot_mode_enabled,
                    "tray": {
                        "input": {"exists": False, "last_changed_at": _iso(now)},
                        "output": {"exists": False, "last_changed_at": _iso(now)},
                    },
                    "object_status": {
                        "centrifuge_tube_rack": {
                            "io_on": self.object_centrifuge_tube_rack_installed,
                            "last_changed_at": _iso(now),
                        },
                        "funnel_rack": {
                            "io_on": self.object_funnel_rack_installed,
                            "last_changed_at": _iso(now),
                        },
                    },
                    "filtration_units": filtration_units,
                },
                "timestamp": _iso(now),
                "unit_task": unit_task_obj,
                "alarm": alarm_obj,
            }
            return state

    def get_task_result(self, unit_task_id: str) -> dict[str, Any]:
        with self._lock:
            if self._current_task is not None and self._current_task.unit_task_id == unit_task_id:
                raise BridgeOperationError("TASK_NOT_FINISHED", "task is still running", status_code=409)
            result = self._task_results.get(unit_task_id)
            if result is None:
                raise BridgeOperationError("TASK_NOT_FOUND", f"task not found: {unit_task_id}", status_code=404)
            return result

    def search_tasks(self, from_iso: str, to_iso: str) -> dict[str, Any]:
        try:
            from_dt = datetime.fromisoformat(from_iso)
            to_dt = datetime.fromisoformat(to_iso)
        except Exception as error:  # noqa: BLE001
            raise BridgeOperationError(
                "INVALID_SEARCH_RANGE",
                "from/to must be ISO8601 datetime strings",
                status_code=400,
                detail={"exception": str(error)},
            ) from error

        with self._lock:
            items: list[dict[str, Any]] = []
            for result in self._task_history:
                try:
                    start_time = datetime.fromisoformat(str(result["start_time"]))
                except Exception:  # noqa: BLE001
                    continue
                if from_dt <= start_time <= to_dt:
                    status_value = "completed" if result["result"] == "success" else "error"
                    if str(result["result"]).startswith("stop_"):
                        status_value = "canceled"
                    items.append(
                        {
                            "unit_task_id": result["unit_task_id"],
                            "task_id_source": result.get("task_id_source", "remote"),
                            "status": status_value,
                            "start_time": result["start_time"],
                            "end_time": result["end_time"],
                        }
                    )
            return {"unit_tasks": items}


def create_app(bridge: XArmApiBridgeNode) -> FastAPI:
    app = FastAPI(title="xarm_api_bridge", version="0.1.0")

    def require_authorization(authorization: Optional[str] = Header(default=None, alias="Authorization")) -> None:
        expected = bridge.api_key
        if not expected:
            raise _api_error(
                status.HTTP_401_UNAUTHORIZED,
                "UNAUTHORIZED",
                "Server API key is not configured.",
                {"hint": "Set FILTRATION_API_KEY or api_key parameter"},
            )
        if authorization != expected:
            raise _api_error(status.HTTP_401_UNAUTHORIZED, "UNAUTHORIZED", "Authentication failed. Check API key.")

    @app.exception_handler(HTTPException)
    async def _http_exception_handler(_: Request, error: HTTPException) -> JSONResponse:
        if isinstance(error.detail, dict) and "code" in error.detail and "message" in error.detail:
            body = {"error": error.detail}
        else:
            body = {"error": {"code": "HTTP_ERROR", "message": str(error.detail)}}
        return JSONResponse(status_code=error.status_code, content=body)

    @app.exception_handler(BridgeOperationError)
    async def _bridge_exception_handler(_: Request, error: BridgeOperationError) -> JSONResponse:
        body = {"error": {"code": error.code, "message": error.message}}
        if error.detail:
            body["error"]["detail"] = error.detail
        return JSONResponse(status_code=error.status_code, content=body)

    @app.exception_handler(Exception)
    async def _unhandled_exception_handler(_: Request, error: Exception) -> JSONResponse:
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"error": {"code": "INTERNAL_SERVER_ERROR", "message": str(error)}},
        )

    @app.get("/status")
    def get_status() -> dict[str, str]:
        return {"status": "ok"}

    api_v1 = APIRouter(prefix="/api/v1", dependencies=[Depends(require_authorization)])

    @api_v1.get("/state")
    def get_state(unit_id: Optional[str] = Query(default=None)) -> dict[str, Any]:
        state_obj = bridge.get_state()
        if unit_id is not None and unit_id != state_obj["unit_id"]:
            raise _api_error(status.HTTP_404_NOT_FOUND, "UNIT_NOT_FOUND", f"unit_id not found: {unit_id}")
        return state_obj

    @api_v1.get("/units/state")
    def get_units_state() -> dict[str, Any]:
        state_obj = bridge.get_state()
        return {state_obj["unit_id"]: state_obj}

    @api_v1.post("/unit-tasks/start")
    @api_v1.post("/unit-task/start", include_in_schema=False)
    def start_task(payload: StartUnitTaskPayload) -> dict[str, Any]:
        return bridge.start_unit_task(payload)

    @api_v1.post("/commands", status_code=status.HTTP_204_NO_CONTENT)
    def post_commands(payload: CommandRequest) -> Response:
        bridge.send_command(payload)
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    @api_v1.post("/manual-commands")
    def manual_commands(payload: dict[str, Any]) -> dict[str, Any]:
        return bridge.send_manual_command(payload)

    @api_v1.post("/robot/enable")
    def robot_enable(payload: RobotEnableRequest) -> dict[str, Any]:
        return bridge.enable_robot(payload)

    @api_v1.post("/control-authority", status_code=status.HTTP_204_NO_CONTENT)
    def control_authority(payload: ControlAuthorityRequest) -> Response:
        bridge.set_control_authority(payload.control_authority)
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    @api_v1.post("/arm/pose-check")
    def arm_pose_check(payload: ArmPoseCheckRequest) -> dict[str, Any]:
        return bridge.check_pose(payload)

    @api_v1.post("/arm/move-initial-pose")
    def arm_move_initial_pose(payload: ArmMoveInitialPoseRequest) -> dict[str, Any]:
        if payload.target != "sequence_start":
            raise _api_error(status.HTTP_400_BAD_REQUEST, "INVALID_REFERENCE", "target must be sequence_start")
        move_id = bridge.start_move_to_initial_pose()
        return {"accepted": True, "move_id": move_id, "message": "move accepted"}

    @api_v1.get("/arm/move-status/{move_id}")
    def arm_move_status(move_id: str) -> dict[str, Any]:
        return bridge.get_move_status(move_id)

    @api_v1.get("/unit-tasks/{unit_task_id}")
    def get_task_result(unit_task_id: str) -> dict[str, Any]:
        return bridge.get_task_result(unit_task_id)

    @api_v1.post("/unit-tasks/search")
    def search_tasks(payload: dict[str, Any]) -> dict[str, Any]:
        from_iso = payload.get("from")
        to_iso = payload.get("to")
        if not isinstance(from_iso, str) or not isinstance(to_iso, str):
            raise _api_error(status.HTTP_400_BAD_REQUEST, "INVALID_SEARCH_RANGE", "from/to are required")
        return bridge.search_tasks(from_iso, to_iso)

    @api_v1.get("/unit-tasks/{unit_task_id}/timeline")
    def get_timeline(unit_task_id: str) -> dict[str, Any]:
        _ = unit_task_id
        raise _api_error(status.HTTP_501_NOT_IMPLEMENTED, "NOT_IMPLEMENTED", "timeline API is not implemented")

    @api_v1.get("/events")
    def event_stream() -> dict[str, Any]:
        raise _api_error(status.HTTP_501_NOT_IMPLEMENTED, "NOT_IMPLEMENTED", "event stream API is not implemented")

    app.include_router(api_v1)
    return app


def main() -> None:
    rclpy.init(args=None)
    bridge = XArmApiBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(bridge)
    uvicorn.run(app, host=bridge.api_host, port=bridge.api_port, log_level="info")

    executor.shutdown()
    bridge.destroy_node()
    rclpy.shutdown()
