# master_controller

`master_controller` subscribes to `xarm_msgs/msg/ApiRequest` and logs whether the requested arm mode is manual.

## Behavior

- When `POST /api/v1/manual-commands` contains `{"action":"arm_mode","mode":"manual"}` or `mode:"local"`, it logs `manual`
- For other `arm_mode` values, it logs the requested non-manual mode
- It can also emulate `/xarm/set_mode` and `/xarm/set_state` so `xarm_api_bridge` manual mode switching succeeds even when those services are unavailable in the current graph

## Run

```bash
ros2 launch master_controller master_controller.launch.py
```
