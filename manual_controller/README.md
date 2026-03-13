# manual_controller

`manual_controller` は `xarm_api_bridge` が publish する `xarm_msgs/msg/ApiRequest` を購読し、`manual-commands` の `gripper` 操作を `trajectory_msgs/msg/JointTrajectory` に変換して `ros2_control` の gripper trajectory controller へ送る C++ ノードです。

## 想定用途

- `xarm_moveit_config` で立ち上げたアーム/グリッパーを GUI から開閉したい
- `xarm_api` の `/xarm/set_gripper_position` サービスが無い構成でも gripper を操作したい

## 起動例

```bash
ros2 launch manual_controller manual_controller.launch.py \
  api_signal_topic:=/xarm/api_requests \
  gripper_controller_name:=xarm_gripper_traj_controller \
  gripper_joint_name:=drive_joint
```

## 主なパラメータ

- `api_signal_topic` default: `/xarm/api_requests`
- `unit_id_filter` default: `""`
- `gripper_controller_name` default: `xarm_gripper_traj_controller`
- `trajectory_topic` default: `""` (`/<controller_name>/joint_trajectory` を使用)
- `gripper_joint_name` default: `drive_joint`
- `open_position` default: `0.0`
- `close_position` default: `0.85`
- `command_duration_sec` default: `1.0`
