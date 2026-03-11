from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from xarm_msgs.msg import ApiRequest
from xarm_msgs.srv import SetInt16, SetInt16ById


class MasterControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("master_controller")

        self.declare_parameter("api_signal_topic", "/xarm/api_requests")
        self.declare_parameter("unit_id_filter", "")
        self.declare_parameter("hw_ns", "xarm")
        self.declare_parameter("emulate_mode_services", True)

        self.api_signal_topic = str(self.get_parameter("api_signal_topic").value)
        self.unit_id_filter = str(self.get_parameter("unit_id_filter").value).strip()
        self.hw_ns = str(self.get_parameter("hw_ns").value).strip("/")
        self.emulate_mode_services = bool(self.get_parameter("emulate_mode_services").value)
        self.last_mode = "unknown"
        self.last_state = -1
        self.robot_enabled = False

        self.create_subscription(ApiRequest, self.api_signal_topic, self._on_api_request, 10)
        if self.emulate_mode_services:
            ns_prefix = f"/{self.hw_ns}" if self.hw_ns else ""
            self._motion_enable_service = self.create_service(
                SetInt16ById, f"{ns_prefix}/motion_enable", self._handle_motion_enable
            )
            self._set_mode_service = self.create_service(SetInt16, f"{ns_prefix}/set_mode", self._handle_set_mode)
            self._set_state_service = self.create_service(SetInt16, f"{ns_prefix}/set_state", self._handle_set_state)
        self.get_logger().info(
            "master_controller started: "
            f"api_signal_topic={self.api_signal_topic}, unit_id_filter={self.unit_id_filter or '*'}, "
            f"hw_ns={self.hw_ns or '/'}, emulate_mode_services={self.emulate_mode_services}"
        )

    def _on_api_request(self, msg: ApiRequest) -> None:
        if self.unit_id_filter and msg.unit_id != self.unit_id_filter:
            return

        if msg.event_type != "manual_command" or msg.command_name != "arm_mode":
            return

        payload = self._decode_payload(msg.payload_json)
        requested_mode = str(payload.get("mode") or "").strip().lower()

        if requested_mode in ("manual", "local"):
            self.last_mode = "manual"
            self.get_logger().info("manual")
            return

        if requested_mode in ("auto", "remote"):
            self.last_mode = "auto"
            self.get_logger().info("auto")
            return

        if requested_mode:
            self.last_mode = requested_mode
            self.get_logger().info(f"mode: {requested_mode}")
            return

        self.last_mode = "unknown"
        self.get_logger().info("not manual: unknown")

    def _decode_payload(self, payload_json: str) -> dict[str, Any]:
        if not payload_json:
            return {}
        try:
            value = json.loads(payload_json)
        except json.JSONDecodeError:
            self.get_logger().warning("failed to decode payload_json from ApiRequest")
            return {}
        if isinstance(value, dict):
            return value
        return {}

    def _handle_set_mode(self, request: SetInt16.Request, response: SetInt16.Response) -> SetInt16.Response:
        mode_value = int(request.data)
        self.last_mode = self._mode_name(mode_value)
        response.ret = 0
        response.message = f"master_controller accepted set_mode={mode_value} ({self.last_mode})"
        self.get_logger().info(response.message)
        return response

    def _handle_set_state(self, request: SetInt16.Request, response: SetInt16.Response) -> SetInt16.Response:
        self.last_state = int(request.data)
        response.ret = 0
        response.message = f"master_controller accepted set_state={self.last_state}"
        self.get_logger().info(response.message)
        return response

    def _handle_motion_enable(
        self, request: SetInt16ById.Request, response: SetInt16ById.Response
    ) -> SetInt16ById.Response:
        self.robot_enabled = bool(request.data)
        response.ret = 0
        state_name = "enabled" if self.robot_enabled else "disabled"
        response.message = f"master_controller accepted motion_enable id={int(request.id)} state={state_name}"
        self.get_logger().info(response.message)
        return response

    def _mode_name(self, mode_value: int) -> str:
        if mode_value == 0:
            return "auto"
        if mode_value == 2:
            return "manual"
        return f"mode_{mode_value}"


def main() -> None:
    rclpy.init(args=None)
    node = MasterControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
