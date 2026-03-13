# xarm_ros2

このリポジトリは、xarm2のドライバー群、IKパッケージおよびモータードライバや真空ポンプなどのIO機器ドライバも含めることを想定したROS2ベースのバックエンドパッケージです。
このパッケージはDockerで構築することを前提にしています。モータードライバなどのドライバ群はsrc内に新しくディレクトリを作成してもらうことを想定しています。


## 構成概要

主に以下のパッケージを使います。

- `xarm_api`:
  xArm 本体を操作する ROS 2 サービス群を提供します。
- `xarm_msgs`:
  `ApiRequest` や `RobotMsg` など、ブリッジと制御ノードで使うメッセージ/サービス定義を提供します。
- `xarm_api_bridge`:
  FastAPI ベースの HTTP API サーバです。外部リクエストを受け取り、ROS 2 サービス呼び出しと `ApiRequest` の publish を行います。
- `master_controller`:
  `ApiRequest` を subscribe し、手動モード切替要求を監視します。必要に応じて `/xarm/set_mode`、`/xarm/set_state`、`/xarm/motion_enable` を疑似的に提供できます。

## アーキテクチャ
### API ブリッジから ROS 2 へのデータフロー
<img width="414" height="461" alt="スクリーンショット 2026-03-13 13 46 28" src="https://github.com/user-attachments/assets/3aa0f54a-1f34-492c-a518-bac58c3245ed" />

### 制御パスの考え方

1. 外部クライアントが `xarm_api_bridge` の HTTP API を呼びます。
2. `xarm_api_bridge` は受けた payload を `xarm_msgs/msg/ApiRequest` に変換し、`/xarm/api_requests` へ publish します。
3. 同時に必要な操作は `/xarm/set_mode` や `/xarm/motion_enable` などの ROS 2 サービスを呼び出します。
4. `master_controller` は `ApiRequest` を受け取り、`マニュアルモード` / `オートメーションモード` 切替要求などを監視します。テスト用途では一部サービスの疑似受け口にもなれます。
5. `xarm_api` は xArm ドライバとして実機へコマンドを送り、状態を `/xarm/robot_states` で返します。
6. `xarm_api_bridge` はその状態を見て、接続状態 API や各操作結果に反映します。
7. `manual_controller`と`autonomous_controller`のそれぞれは`master_controller`により切り替えられる。
8. 例えば自動シーケンス時は`autonomous_controller`が呼び出されそれぞれのシーケンスに応じてアームやモーターなどの動作命令を行います。動作命令は基本的にはROS2 topicをメインで利用し,statusや状態管理を行います。

## 主要インタフェース

### HTTP API

`xarm_api_bridge` は以下のような API を提供します。

- `GET /status` は認証不要
- `/api/v1/*` は `X-API-Key` または `Authorization: Bearer <token>` で認証
- `GET /status`
- `GET /api/v1/state`
- `GET /api/v1/units/state`
- `GET /api/v1/connection-status`
- `POST /api/v1/unit-tasks/start`
- `POST /api/v1/commands`
- `POST /api/v1/manual-commands`
- `POST /api/v1/robot/enable`
- `POST /api/v1/control-authority`
- `POST /api/v1/arm/pose-check`
- `POST /api/v1/arm/move-initial-pose`

### Publish される topic

- `/xarm/api_requests`
  型: `xarm_msgs/msg/ApiRequest`
  
詳細はxarm_msgs内で記述

主なフィールド:

- `unit_id`
- `api_endpoint`
- `event_type`
- `command_source`
- `job_id`
- `unit_task_id`
- `command_name`
- `payload_json`

### 利用する ROS 2 サービス

`hw_ns:=xarm` の場合、主に以下を呼びます。

- `/xarm/motion_enable`
- `/xarm/set_mode`
- `/xarm/set_state`
- `/xarm/clean_error`
- `/xarm/clean_warn`
- `/xarm/move_gohome`
- `/xarm/get_servo_angle`
- `/xarm/set_servo_angle`
- `/xarm/set_gripper_position`
- `/xarm/set_tgpio_digital`

### 購読する状態 topic

- `/xarm/robot_states`
  型: `xarm_msgs/msg/RobotMsg`

この topic をもとに、`xarm_api_bridge` は接続状態を `connected` / `degraded` / `disconnected` / `idle` で判定します。

## セットアップ

### 1. ROS 2 ワークスペースのビルド

```bash
cd ~/dev_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

必要に応じて `rosdep` も先に実行してください。

```bash
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 実機接続に必要な前提

- xArm 本体の IP アドレスが分かっていること
- `xarm_api` 側で必要なサービスが有効になっていること
- FastAPI / Uvicorn が利用できること

## 起動手順

### 実機を使う場合

1. xArm ドライバを起動

```bash
source ~/dev_ws/install/setup.bash
ros2 launch xarm_api xarm6_driver.launch.py robot_ip:=192.168.1.225 hw_ns:=xarm
```

2. API ブリッジを起動

```bash
source ~/dev_ws/install/setup.bash
ros2 launch xarm_api_bridge xarm_api_bridge.launch.py \
  api_host:=127.0.0.1 \
  api_port:=8000 \
  api_key:=your_api_key \
  unit_id:=unit-xarm01 \
  hw_ns:=xarm \
  api_signal_topic:=/xarm/api_requests
```

3. 必要なら `master_controller` を起動

```bash
source ~/dev_ws/install/setup.bash
ros2 launch master_controller master_controller.launch.py \
  api_signal_topic:=/xarm/api_requests \
  hw_ns:=xarm
```

### 実機なしで API フローだけ確認する場合

`master_controller` は `/xarm/motion_enable`、`/xarm/set_mode`、`/xarm/set_state` を疑似的に提供できます。

```bash
source ~/dev_ws/install/setup.bash
ros2 launch master_controller master_controller.launch.py emulate_mode_services:=true
```

その後、別ターミナルで API ブリッジを起動します。

```bash
source ~/dev_ws/install/setup.bash
export FILTRATION_API_KEY=your_api_key
ros2 run xarm_api_bridge xarm_api_bridge_server
```

## 動作確認例

### API サーバの疎通確認

```bash
curl http://127.0.0.1:8000/status
```

### 手動モード要求を送る例

```bash
curl -X POST http://127.0.0.1:8000/api/v1/manual-commands \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your_api_key" \
  -d '{"action":"arm_mode","mode":"manual","command_source":"gui"}'
```

このリクエストを送ると、`xarm_api_bridge` は `/xarm/api_requests` に `ApiRequest` を publish し、`master_controller` 側で manual モード要求として解釈されます。

### 接続状態の確認

```bash
curl http://127.0.0.1:8000/api/v1/connection-status \
  -H "X-API-Key: your_api_key"
```

## 主なパラメータ

### xarm_api_bridge

- `api_host` default: `127.0.0.1`
- `api_port` default: `8000`
- `api_key` default: `""`
- `unit_id` default: `unit-xarm01`
- `hw_ns` default: `xarm`
- `api_signal_topic` default: `""`
- `task_duration_sec` default: `90.0`
- `default_control_authority` default: `remote`
- `initial_pose_deg` default: `[0, -30, 0, 0, 30, 0]`
- `move_joint_speed` default: `0.5`
- `move_joint_acc` default: `5.0`
- `move_joint_timeout_sec` default: `120.0`

### master_controller

- `api_signal_topic` default: `/xarm/api_requests`
- `unit_id_filter` default: `""`
- `hw_ns` default: `xarm`
- `emulate_mode_services` default: `true`

## 関連ドキュメント

- [xarm_api_bridge/README.md](./xarm_api_bridge/README.md)
- [master_controller/README.md](./master_controller/README.md)
- [xarm_api/ReadMe.md](./xarm_api/ReadMe.md)
- [xarm_msgs/ReadMe.md](./xarm_msgs/ReadMe.md)
