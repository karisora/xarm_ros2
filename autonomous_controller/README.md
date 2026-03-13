# autonomous_controller

`autonomous_controller` subscribes to `xarm_msgs/msg/ApiRequest` and executes a basic timed autonomous sequence when the GUI sends `unit_task_start`.

## Behavior

- Consumes `unit_task_start` requests from `/xarm/api_requests`
- Builds sequence phases from the GUI payload `common_params`
- Handles `pause`, `resume`, `stop`, and `emergency_stop`
- Optionally calls `/xarm/set_mode`, `/xarm/set_state`, `/xarm/motion_enable`, and `/xarm/move_gohome`
- Publishes JSON status text to `/xarm/autonomous_controller/status`
- Publishes activity state to `/xarm/autonomous_controller/active`

## Run

```bash
ros2 launch autonomous_controller autonomous_controller.launch.py
```

## Important note

This package is a minimal autonomous execution scaffold. It uses the task timing information from the GUI payload and is intended as the starting point for wiring real arm, pump, and IO actions into each phase.
