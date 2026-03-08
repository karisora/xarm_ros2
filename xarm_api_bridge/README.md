# xarm_api_bridge

`xarm_api_bridge` は、フロントエンドが利用する HTTP API（FastAPI）と、`xarm_api` が提供する ROS2 サービスを接続するブリッジパッケージです。

## 目的

- フロントエンドから `POST /api/v1/*` を呼ぶだけで xArm を操作できるようにする
- xArm 側の操作を ROS2 サービス呼び出しに統一する

## 主なAPI

- `GET /status`
- `GET /api/v1/state`
- `GET /api/v1/units/state`
- `GET /api/v1/connection-status`
- `POST /api/v1/unit-tasks/start`
- `POST /api/v1/commands` (`pause/resume/stop/emergency_stop/reset_state`)
- `POST /api/v1/manual-commands`
- `POST /api/v1/robot/enable`
- `POST /api/v1/control-authority`
- `POST /api/v1/arm/pose-check`
- `POST /api/v1/arm/move-initial-pose`
- `GET /api/v1/arm/move-status/{move_id}`
- `GET /api/v1/unit-tasks/{unit_task_id}`
- `POST /api/v1/unit-tasks/search`

`/api/v1/unit-tasks/{id}/timeline` と `/api/v1/events` は現時点では `501` を返します。

## 通信状態ステータス

`GET /api/v1/connection-status` で、`filtration-app` と同じ語彙の接続状態を取得できます。

- `status`: `connected` / `degraded` / `disconnected` / `idle`
- `last_updated_at`: 最後に `/xarm/robot_states` を受信した時刻
- `last_error`: 直近のROSサービス通信エラー（なければ `null`）
- `success_rate_percent`: 直近 `window_sec` 秒の成功率
- `success_count` / `failure_count`: 直近窓の通信試行回数

同じ内容は `GET /api/v1/state` の `details.connection` にも含まれます。

## ROSサービス対応

このブリッジは既定で以下を使用します（`hw_ns=xarm` の場合）。

- `/xarm/motion_enable`
- `/xarm/set_mode`
- `/xarm/set_state`
- `/xarm/clean_error`
- `/xarm/clean_warn`
- `/xarm/move_gohome`
- `/xarm/get_servo_angle`
- `/xarm/set_servo_angle`
- `/xarm/set_gripper_position`
- `/xarm/set_tgpio_digital`（任意）

状態監視には `/xarm/robot_states` を購読します。

## 前提

`xarm_api` 側で利用するサービスを有効化しておく必要があります。最低限、以下は `true` 推奨です。

- `motion_enable`
- `set_mode`
- `set_state`
- `clean_error`
- `clean_warn`
- `get_servo_angle`
- `set_servo_angle`
- `move_gohome`
- `set_gripper_position`（グリッパ操作を使う場合）
- `set_tgpio_digital`（pump/waste/servo を IO 連携する場合）

## 起動方法

1. xArm ドライバ起動

```bash
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.225 hw_ns:=xarm
```

2. ブリッジ起動

```bash
ros2 launch xarm_api_bridge xarm_api_bridge.launch.py \
  api_host:=127.0.0.1 \
  api_port:=8000 \
  api_key:=your_api_key \
  unit_id:=unit-mys01 \
  hw_ns:=xarm
```

または環境変数でも指定可能です。

```bash
export FILTRATION_API_KEY=your_api_key
ros2 run xarm_api_bridge xarm_api_bridge_server
```

## 主要パラメータ

- `api_host` (string, default: `127.0.0.1`)
- `api_port` (int, default: `8000`)
- `api_key` (string, default: `""`)
- `unit_id` (string, default: `unit-xarm01`)
- `hw_ns` (string, default: `xarm`)
- `task_duration_sec` (float, default: `90.0`)
- `initial_pose_deg` (double[], default: `[0,-30,0,0,30,0]`)
- `move_joint_speed` (float, default: `0.5`)
- `move_joint_acc` (float, default: `5.0`)
- `object_centrifuge_tube_rack_installed` (bool, default: `true`)
- `object_funnel_rack_installed` (bool, default: `true`)
- `manual_pump_io` / `manual_waste_io` / `manual_servo_io` (int, default: `-1`)
