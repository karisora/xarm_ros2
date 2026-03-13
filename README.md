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
- `autonomous_controller`:
  `ApiRequest` の `unit_task_start` と `pause/resume/stop` を subscribe し、GUI の工程時間データにもとづく自動シーケンスを実行します。

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

### 自動シーケンス
基本的にはこの[N1-N6](https://github.com/queeenb-com/filtration-app/blob/feat/parallel-gui-4units/docs/BE-Orchestration-Engine-Guide.md) の流れで制御する。ロボットアームと攪拌などは並列して行うため、それぞれの動作を監督して命令を出すノードが必要となる。その仕事はautonomous_controllerが行う。autonomous_controllerは自動シーケンス開始後、GUIから受け取ったそれぞれのモジュールの動作時間データなどを読み取りアームや送液などの各モジュールに命令を出力する。

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

### 1. Dockerの構築

権限付与
```bash
xhost +local:root
```

Docker build
```bash
docker build -t xarm_ros2:humble .
```


Docker run
```bash
docker run -it --rm \ --name xarm_gzclassic \ --net=host \ -e DISPLAY=$DISPLAY \ -v /tmp/.X11-unix:/tmp/.X11-unix:rw \ xarm_ros2:humble bash
```


## 起動手順, 仮
### gazebo

1. master nodeを起動

```bash
ros2 launch master_controller master_controller.launch.py
```

2. API ブリッジを起動

```bash
ros2 launch xarm_api_bridge xarm_api_bridge.launch.py api_host:=127.0.0.1 api_port:=8000 api_key:=MQWGUB1GA9rOaLCxkCGe4j4LE5cdcSxk unit_id:=unit-mys01 hw_ns:=xarm
```

3. xarm simulationを起動

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo_with_field.launch.py \
robot_type:=xarm dof:=6 add_gripper:=true \
load_controller:=true

```

ここまで起動した段階でfiltarion-appを起動すると接続されます。


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

### autonomous_controller

- `api_signal_topic` default: `/xarm/api_requests`
- `unit_id_filter` default: `""`
- `hw_ns` default: `xarm`
- `enable_robot_on_start` default: `false`
- `set_auto_mode_on_start` default: `true`
- `set_ready_state_on_start` default: `true`
- `move_home_on_start` default: `false`

## 関連ドキュメント

- [xarm_api_bridge/README.md](./xarm_api_bridge/README.md)
- [master_controller/README.md](./master_controller/README.md)
- [autonomous_controller/README.md](./autonomous_controller/README.md)
- [xarm_api/ReadMe.md](./xarm_api/ReadMe.md)
- [xarm_msgs/ReadMe.md](./xarm_msgs/ReadMe.md)
