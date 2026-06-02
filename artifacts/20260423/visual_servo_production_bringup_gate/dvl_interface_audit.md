# DVL Interface Audit

## 結論

`/waterlinked_dvl_driver/velocity_report` の正本 message type は `marine_acoustic_msgs/msg/Dvl` である。

根拠:

- [rosbag2_2026_04_22-16_47_57-diving-noimage/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/rosbag2_2026_04_22-16_47_57-diving-noimage/metadata.yaml:64)
- [rosbag2_2026_04_22-15_22_39/metadata.yaml](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/rosbag2_2026_04_22-15_22_39/metadata.yaml:82)

いずれも `/waterlinked_dvl_driver/velocity_report` を `marine_acoustic_msgs/msg/Dvl` と記録している。

## repo 内 consumer との整合

[pub_system_stats.py](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/src/deployment_env/src/visualize_urdf/visualize_urdf/pub_system_stats.py:16) は同 topic を `Dvl` で subscribe し、[line 33](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/src/deployment_env/src/visualize_urdf/visualize_urdf/pub_system_stats.py:33) で `msg.speed_gnd` を参照している。

これは今回一時的に追加した UUV-style `Dvl.msg` には存在しない field であり、local shim が正本 schema ではないことを示す。

## 失敗した案

一時的に `src/marine_acoustic_msgs` を local 追加して clean env で bag replay を通そうとしたが、bag 実 schema と不一致だった。結果として:

- minimal Python subscriber が `exit 137`
- controller process が DVL-only replay で `exit -9`

よって local shim は削除した。

## 最終方針

1. canonical interface は `marine_acoustic_msgs/msg/Dvl`
2. controller は official `marine_acoustic_msgs.msg.Dvl` が import できる環境だけ DVL subscription を作る
3. clean env では no-DVL-depth degraded mode を許容する
4. fake local replacement package は持ち込まない

## controller 実装

- import guard: [visual_servo_controller_node_implementation.py](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:47)
- staged PID DVL subscription guard: [same file](/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz/src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:505)

