# autonomous_controller

`autonomous_controller` subscribes to `xarm_msgs/msg/ApiRequest` and executes a waypoint sequence when the GUI sends `unit_task_start`.

## Behavior

- Consumes `unit_task_start` requests from `/xarm/api_requests`
- Loads waypoint definitions from YAML
- Calls `xarm_planner` services to plan and execute pose/joint waypoints
- Handles `pause`, `resume`, `stop`, and `emergency_stop`
- Optionally calls `/xarm/set_mode`, `/xarm/set_state`, `/xarm/motion_enable`, and `/xarm/move_gohome`
- Publishes JSON status text to `/xarm/autonomous_controller/status`
- Publishes activity state to `/xarm/autonomous_controller/active`

## Run

```bash
ros2 launch autonomous_controller autonomous_controller.launch.py
```

## Important note

The default waypoint file is `xarm_planner/config/eef_waypoints_example.yaml`. `use_cartesian=true` in that YAML is currently treated as sequential waypoint execution.
