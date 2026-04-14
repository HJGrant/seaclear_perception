# PM_REPORT_20260412_SIMULATOR_BLUEROV2_ORCA4_REVIEW_JA

## 結論先出し

BlueROV2 シミュレータ導入の初手は、`CentraleNantesROV/bluerov2` を土台にし、現 visual-servo 系とは `cmd_vel_body` 境界で接続する方針が最短です。`orca4` は有益ですが、今回はベース repo ではなく MAVROS / ArduSub / frame 設計の参考実装として扱うのが妥当です。

初回検証で優先すべきことは、高忠実度な実機再現ではなく「下向き camera で seabed target を見ながら現 perception/controller が閉ループするか」です。この目的に対しては、MAVROS を初手で入れるより simulator-native adapter を 1 枚入れて先に回す方が、実装コストと切り分け容易性の両面で優れます。

## 根拠

現 repo 側では、`mvp_sim.launch.py` と `blue_rov_synth_vs.launch.py` がすでに `cmd_vel_body` 境界の設計を持っています。controller 本体は bbox / `CameraInfo` / body velocity / target point を受けて `TwistStamped` を出すため、backend を差し替えれば controller 自体は大きく変えずに流用できます。既存の `cmd_vel_to_mavros_bridge.py` も backend 切替の骨格になっています。

`bluerov2` 側は Gazebo world、spawn、bridge、6 thruster command、camera/IMU/sonar を ROS 2 Humble 系で簡潔に提供しており、下向き monocular camera への変更も xacro レベルで済みます。一方で `orca4` は `/cmd_vel`、MAVROS、ArduSub、Nav2、ORB_SLAM2 を束ねた完成度の高い統合例ですが、今回必要な bbox/target-point/visual-servo の機能検証には依存が重すぎます。

## 現 repo への影響範囲

- controller/perception 本体は基本温存
- 追加が必要なのは simulator bringup、camera_info 供給、body velocity bridge、sim thruster adapter
- 将来の MAVROS 接続は adapter backend 差し替えで吸収

## 採否判断

- 採用: `bluerov2` ベース + adapter 境界固定
- 参考採用: `orca4` の `map/slam/down/odom/base_link` 設計、manager 分離、MAVROS 接続責務
- 非採用: 初手からの `orca4` 直系統合、初手からの全面 ArduSub/MAVROS 統合

## 次アクション

1. `sc2_visual_servo_sim` または同等 package を追加
2. down-camera BlueROV2 variant と seabed target world を追加
3. `cmd_vel_body -> sim_thrusters` adapter を追加
4. `sim odom -> body velocity` bridge を追加
5. remap ベースで現 perception/controller launch を simulator に接続
