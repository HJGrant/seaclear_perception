# PM Report: SIMULATOR_HEADLESS_VALIDATION

## 概要
- branch: `simulator/bluerov2_humble_eval`
- head commit at report time: `814d21306251b60426695a6474a3cf502a4e1c1a`
- 方針分離: GUI前までを Codex、GUI確認はユーザー担当
- headless での runtime 検証は **Docker / ROS 2 不在のため未実行**

## 実施内容
- 既存 runbook / script を確認し、headless 検証の実行経路を整理した。
- `my_ws/scripts/launch_container.sh` と `config/fastdds_client_profile.xml` の FastDDS Discovery 設定を確認した。
- `sc2_visual_servo_bringup/object_detection_ibvs.launch.py` の I/F と依存 topic を確認した。
- `scripts/run_oakd_system_check_docker.sh` / `scripts/run_oakd_system_check.sh` の headless チェック手順を確認した。
- headless validation 用 docs と artifact を新規作成した。

## 環境監査結果
- WSL distro 上では `docker` コマンド自体が利用不可。
- `/opt/ros/humble` 不在で ROS 2 runtime も未導入。
- `deployment_env/docker` は本 repo に存在せず、外部環境前提。
- よって、この環境では headless runtime 検証を実行できない。

## headless 検証経路（整理済み）
### 系統分離（要点）

| Path | 目的 | 主要 launch | 出力 topic |
| --- | --- | --- | --- |
| Phase 1 simulator | `TwistStamped` 経路の headless 検証 | `sc2_visual_servo_sim/*` | `/sc2/visual_servo/cmd_vel_body` |
| IBVS reference | 既存実機寄り IBVS 経路の参考 | `sc2_visual_servo_bringup/object_detection_ibvs.launch.py` | `/mavros/setpoint_velocity/cmd_vel` |

### 共通の前提
- container 起動: `my_ws/scripts/launch_container.sh` (Orin) / `my_ws/scripts/launch_container_x86_64.sh` (x86)
- FastDDS: `config/fastdds_client_profile.xml` が `10.1.1.200:11815` の Discovery Server を指す
- headless checks:
  - `scripts/run_oakd_system_check.sh --profile no_hw` (container 内)
  - `ros2 node list`, `ros2 topic list`, `ros2 topic hz`, `ros2 topic echo`

## GUI handoff (ユーザー側)
- `deployment_env/docker` にて Foxglove 起動
- Orin で FastDDS Discovery Server 起動
- Orin で container 起動 & build
- IBVS launch と `enable_ibvs_control` service 呼び出し

## 重要な注意点
- `object_detection_ibvs.launch.py` の `cmd_topic` は `/mavros/setpoint_velocity/cmd_vel` に固定されている。
- Phase 1 simulator の `TwistStamped` I/F とは別系統であり、同一検証に混在させない。

## 未確認項目
- `ros_gz_sim` / `ros_gz_bridge` のインストール有無
- `gz_server` / `ros_gz_sim gz_server.launch.py` の実行
- Gazebo headless のカメラセンサ稼働
- `TwistStamped` -> backend -> odom の実 runtime 収束
- rosbag 記録

## 次アクション候補
- Docker Desktop / WSL integration または Orin container で headless runtime を実行。
- `ros_gz_sim` / `ros_gz_bridge` の依存追加と `gz_server` 起動確認。
- `sc2_visual_servo_sim` 経路で `TwistStamped` 収束確認。
