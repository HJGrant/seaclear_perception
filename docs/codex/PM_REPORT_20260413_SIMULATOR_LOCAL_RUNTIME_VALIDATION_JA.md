# PM Report: SIMULATOR_LOCAL_RUNTIME_VALIDATION

## 概要
- branch: `simulator/bluerov2_humble_eval`
- head commit at report time: `814d21306251b60426695a6474a3cf502a4e1c1a`
- mainline: **local-first** (Gazebo on host, ROS2 nodes in container)
- reference: IBVS/MAVROS 経路は従経路

## 実施内容
- local-first 実行経路を runbook として分離整理した。
- host/container 間の前提条件 (DDS, network, domain) を明文化した。
- runtime 実行は **Docker/ROS/Gazebo が WSL に存在しないため未実施** と記録した。
- GUI handoff チェックリストを local-first 前提で更新した。

## runtime 実行状況
- 実行できたもの:
  - host 環境監査 (docker/ros2/colcon/gz 不在の確認)
- 実行できていないもの:
  - Gazebo 起動
  - container 内 build/launch
  - topic / tf / hz / rosbag
- 未実施理由:
  - この WSL 環境で Docker と ROS 2 runtime が無い

## mainline / reference 分離

| Path | 目的 | 主要 launch | 出力 topic |
| --- | --- | --- | --- |
| Phase 1 simulator | local-first runtime 検証 | `sc2_visual_servo_sim/*` | `/sc2/visual_servo/cmd_vel_body` |
| IBVS reference | 参考比較 | `sc2_visual_servo_bringup/object_detection_ibvs.launch.py` | `/mavros/setpoint_velocity/cmd_vel` |

## 既知の制約
- WSL に `docker`, `ros2`, `colcon`, `gz` が存在しない。
- Gazebo / ros_gz_sim / ros_gz_bridge の可否は未確認。
- `deployment_env/docker` は本 repo に存在しない。

## 次アクション
- Docker 有効ホスト or Orin container で local-first runbook を実行。
- `ros_gz_sim` / `ros_gz_bridge` の有無を確認。
- Phase 1 simulator の topic/tf/hz/rosbag を取得。
