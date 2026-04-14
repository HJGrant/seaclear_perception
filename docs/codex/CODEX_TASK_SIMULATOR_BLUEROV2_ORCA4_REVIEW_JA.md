# CODEX_TASK_SIMULATOR_BLUEROV2_ORCA4_REVIEW_JA

実施日: 2026-04-12
作業 branch: `simulator/bluerov2_humble_eval`

## 1. 結論

### 問いA
最短で現 repo に BlueROV2 シミュレータを接続する推奨方針は、`CentraleNantesROV/bluerov2` を基盤に採用し、`orca4` は MAVROS / ArduSub 接続と frame helper 設計の参考実装として読む案である。

理由:

- `bluerov2` は ROS 2 + Gazebo の BlueROV2 記述と水中センサ・thruster I/F を最短で提供する。`world_launch.py` と `upload_bluerov2_launch.py` で world 起動、robot spawn、bridge がすでに分離されている。`/bluerov2/cmd_thruster[1..6]` を受ける simulator-native I/F も明示されている。
- 現 repo はすでに `cmd_vel_body -> adapter` の分離を持っており、まずは simulator-native backend を追加するだけで閉ループ検証に入れる。
- `orca4` は `/cmd_vel` 前提の BaseController、MAVROS、ArduSub mode 管理、`map/slam/down/odom/base_link` の helper frame 設計が成熟しているが、Nav2 / ORB_SLAM2 / MAVROS 前提が強く、現 perception/controller の機能検証だけを目的にすると初期導入コストが高い。

主な根拠:

- `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py:99-206`
- `src/sc2_visual_servo_bringup/launch/blue_rov_synth_vs.launch.py:85-167`
- `/tmp/bluerov2_full/bluerov2_description/launch/upload_bluerov2_launch.py:22-77`
- `/tmp/bluerov2_full/bluerov2_description/urdf/thrusters.xacro:57-73`
- `/tmp/orca4/orca_base/src/base_controller.cpp:389-440`
- `/tmp/orca4/orca_bringup/launch/bringup.py:104-219`

### 問いB
現 controller/perception 実装の主契約は次のとおり。

- 検出入力は `vision_msgs/Detection2D` または `vision_msgs/Detection2DArray`
- 深度入力は `sensor_msgs/Image`
- camera intrinsics は `sensor_msgs/CameraInfo`
- target point は `geometry_msgs/PoseStamped`
- controller 出力は `geometry_msgs/TwistStamped`
- 主要 frame 仮定は `base_link` と camera optical frame

接続上の重要点:

- `target_observer_node` は `Detection2DArray` と depth image から `sc2_visual_servo_interfaces/TargetObservation` を出す。
- `target_point_estimator_node` は `Detection2D` と depth / `CameraInfo` から `PoseStamped` を出す。
- `visual_servo_controller_node` は bbox (`Detection2D`) と `CameraInfo` と速度計測 `TwistStamped` と target point `PoseStamped` を受け、`cmd_topic` に `TwistStamped` を出す。
- controller 本体は `camera_optical_frame_id -> body_frame_id` の TF を要求する。IBVS 系は camera optical ベース、最終 publish は body または `map` fallback を使う。

主な根拠:

- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py:20-79`
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_point_estimator_node.py:20-112`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:47-65`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:264-272`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:570-613`

### 問いC
MAVROS をそのまま現 controller に直結するのは不十分で、adapter 層が必要である。

必要な adapter:

- `TwistStamped(body)` を simulator-native thruster command に変換する backend
- 将来は同じ `TwistStamped(body)` を `mavros/setpoint_velocity` または RC override に変換する backend
- 必要なら `mavros/local_position/velocity_body` 相当の body velocity を simulator 側 odom から再生成する adapter
- camera optical と `base_link` を結ぶ static TF

既存実装にはすでに `cmd_vel_to_mavros_bridge.py` があり、`rc_override` / `setpoint_velocity` / `dry_run` の backend 切替を持つ。ここに simulator-native backend を追加する構成が自然である。

主な根拠:

- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavros_bridge.py:16-46`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavros_bridge.py:83-105`
- `/tmp/orca4/orca_base/src/base_controller.cpp:191-219`
- `/tmp/orca4/orca_base/README.md:100-108`

### 問いD
初回の機能検証では MAVROS を入れず、simulator-native I/F で先に閉ループ確認すべきである。

理由:

- 実装コストが最小
- 問題切り分けが容易
- 現 repo の perception/controller をほぼそのまま試せる
- MAVROS / ArduSub 導入は次段で adapter backend を差し替えればよい

補足:

- この時点で必要なのは下向き camera orientation であり、単眼化ではない
- 初回 MVP は下向き RGB + depth source を simulator 側で供給し、既存 `target_point_estimator_node` をなるべくそのまま使うべきである

ただし将来拡張のため、`cmd_vel_body` の先は adapter ノードに閉じ込める。

## 2. 監査対象

### 現 repo

- `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py`
- `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py`
- `src/sc2_visual_servo_bringup/launch/blue_rov_synth_vs.launch.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavros_bridge.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/cmd_vel_to_mavlink_stub.py`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_debug_viz_node.py`
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
- `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_point_estimator_node.py`

### 外部 repo

- `/tmp/bluerov2_full/README.md`
- `/tmp/bluerov2_full/bluerov2_description/launch/world_launch.py`
- `/tmp/bluerov2_full/bluerov2_description/launch/upload_bluerov2_launch.py`
- `/tmp/bluerov2_full/bluerov2_description/launch/state_publisher_launch.py`
- `/tmp/bluerov2_full/bluerov2_description/urdf/bluerov2.xacro`
- `/tmp/bluerov2_full/bluerov2_description/urdf/sensors.xacro`
- `/tmp/bluerov2_full/bluerov2_description/urdf/thrusters.xacro`
- `/tmp/bluerov2_full/bluerov2_description/urdf/demo_world.sdf`
- `/tmp/bluerov2_full/bluerov2_control/launch/cascaded_pids_launch.py`
- `/tmp/bluerov2_full/bluerov2_control/launch/wrench_launch.py`
- `/tmp/bluerov2_full/bluerov2_control/config/cascaded_pid.yaml`
- `/tmp/bluerov2_full/bluerov2_control/config/thruster_manager.yaml`
- `/tmp/orca4/orca_bringup/launch/sim_launch.py`
- `/tmp/orca4/orca_bringup/launch/bringup.py`
- `/tmp/orca4/orca_bringup/params/sim_orca_params.yaml`
- `/tmp/orca4/orca_base/src/base_controller.cpp`
- `/tmp/orca4/orca_base/src/manager.cpp`
- `/tmp/orca4/orca_base/README.md`
- `/tmp/orca4/orca_description/models/orca4/model.sdf`

## 3. 現 repo I/F 監査

### 3.1 現 node 契約一覧

| 現 node 名 | 入力 topic | 入力 message type | 出力 topic | 出力 message type | frame 前提 | simulator で代替すべき source | 備考 |
|---|---|---|---|---|---|---|---|
| `target_observer_node` | `/detections_output` | `vision_msgs/Detection2DArray` | `/sc2/visual_servo/target_obs` | `sc2_visual_servo_interfaces/TargetObservation` | detection header frame を保持 | detector bridge or synthetic detector | depth image があれば ROI median depth を `depth_m` に入れる |
| `target_observer_node` | `/oak/stereo/image_raw` | `sensor_msgs/Image` | 同上 | 同上 | optical frame 前提 | simulator depth or synthetic depth | `use_depth=false` で depth なし運用も可 |
| `target_point_estimator_node` | `/best_detection` | `vision_msgs/Detection2D` | `/sc2/visual_servo/target_point` | `geometry_msgs/PoseStamped` | `oak_rgb_camera_optical_frame` 既定、`CameraInfo.header.frame_id` 優先 | simulator detection + depth + camera_info | optical frame convention に合わせて `x, y, z` を組み替える |
| `target_point_estimator_node` | `/oak/stereo/image_raw` | `sensor_msgs/Image` | `/sc2/visual_servo/target_point_status` | `std_msgs/String` | optical frame | simulator depth | hold-last-valid 実装あり |
| `target_point_estimator_node` | `/oak/stereo/camera_info` | `sensor_msgs/CameraInfo` | `/sc2/visual_servo/target_point` | `geometry_msgs/PoseStamped` | camera frame 必須 | simulator `camera_info` publisher | intrinsics fallback あり |
| `visual_servo_controller_node` | `/best_detection` | `vision_msgs/Detection2D` | `/sc2/visual_servo/cmd_vel_body` または MAVROS topic | `geometry_msgs/TwistStamped` | `camera_optical_frame_id -> body_frame_id` TF 必須 | simulator detector output | bbox は `TargetObservation` ではなく `Detection2D` を直接参照 |
| `visual_servo_controller_node` | `/sc2/visual_servo/target_point` | `geometry_msgs/PoseStamped` | `/sc2/visual_servo/status` | `std_msgs/String` | target point は camera frame | simulator target point estimator | staged PID の Z 測定源に使用 |
| `visual_servo_controller_node` | `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | `/sc2/visual_servo/velocity_optical` | `geometry_msgs/TwistStamped` | body velocity 前提 | simulator odom->body velocity adapter | 実機依存が強い |
| `visual_servo_controller_node` | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | `/sc2/visual_servo/target_locked` | `std_msgs/String` | camera intrinsics 必須 | simulator `camera_info` | IBVS Jacobian 用 |
| `cmd_vel_to_mavros_bridge` | `/sc2/visual_servo/cmd_vel_body` | `geometry_msgs/TwistStamped` | `/mavros/rc/override` or `/mavros/setpoint_velocity/cmd_vel_unstamped` | `mavros_msgs/OverrideRCIn` or `geometry_msgs/Twist` | body frame | simulator-native backend に差し替え | adapter 境界として再利用可能 |

### 3.2 launch と既存 simulator 線

- `mvp_sim.launch.py` は `target_observer_node` / `target_point_estimator_node` / `visual_servo_controller_node` / `cmd_vel_to_mavlink_stub` を `cmd_vel_body` で直列接続しており、既に「MAVROS なしで controller を回す」系の最小線を持つ。
- `blue_rov_synth_vs.launch.py` は `cmd_vel_to_mavros_bridge` を launch しており、adapter backend 切替の既存土台になっている。
- `object_detection_ibvs.launch.py` は依然として `cmd_topic=/mavros/setpoint_velocity/cmd_vel` を既定に持ち、実機寄りである。

根拠:

- `src/sc2_visual_servo_bringup/launch/mvp_sim.launch.py:79-206`
- `src/sc2_visual_servo_bringup/launch/blue_rov_synth_vs.launch.py:64-167`
- `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py:67-95`

### 3.3 実機依存箇所

- `visual_servo_controller_node_implementation.py` の既定 `cmd_topic=/mavros/setpoint_velocity/cmd_vel`
- 同ファイルの既定 `vel_measurement_topic=/mavros/local_position/velocity_body`
- `_publish_cmd()` の `map <- body_frame_id` TF lookup fallback
- `cmd_vel_to_mavros_bridge.py` の MAVROS topic と RC override channel map

## 4. `CentraleNantesROV/bluerov2` 監査

### 4.1 そのまま使える点

- `world_launch.py` は Gazebo world と spawn を分離しており、world 差し替えが容易。
- `upload_bluerov2_launch.py` は robot state publisher、spawn、joint state、pose ground truth、IMU、sonar、camera、tilt、thruster topic bridge をまとめている。
- `thrusters.xacro` は 6 基の thruster plugin を定義し、topic `thruster[1..6]/cmd` を受ける。
- `upload_bluerov2_launch.py` で ROS 側 topic は `cmd_thruster1..6` に揃えられており、simulator-native backend の出口として扱いやすい。
- `sensors.xacro` は camera、IMU、sonar を既に持つ。

根拠:

- `/tmp/bluerov2_full/bluerov2_description/launch/world_launch.py:6-24`
- `/tmp/bluerov2_full/bluerov2_description/launch/upload_bluerov2_launch.py:22-77`
- `/tmp/bluerov2_full/bluerov2_description/urdf/thrusters.xacro:10-75`
- `/tmp/bluerov2_full/bluerov2_description/urdf/sensors.xacro:7-229`

### 4.2 下向き camera 化のしやすさ

しやすい。理由:

- カメラは `sensors.xacro` の `camera` / `camera_optical` と `tilt` joint で定義されている。
- 現状は base_link 前方搭載 + tilt 関節で pitch 変更する構成なので、最初の MVP では次のどちらかで十分。

1. `tilt` 初期角を下向き固定にする
2. `camera` link の joint origin / optical joint を固定下向きに変更し、新たに `down_camera_optical_frame` を生やす

推奨は 2。理由は、固定下向き camera の TF が明確になり、現 controller の `camera_optical_frame_id` にそのまま渡せるため。

根拠:

- `/tmp/bluerov2_full/bluerov2_description/urdf/sensors.xacro:115-143`
- `/tmp/bluerov2_full/bluerov2_description/urdf/sensors.xacro:198-229`
- `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py:57-58`

### 4.3 seabed target world

`demo_world.sdf` はほぼ空の水面 world で、seabed target を置くには world を追加で作る必要がある。

最小追加内容:

- seabed plane or box
- 高コントラスト target mesh か texture plane
- 十分な照明
- 必要なら camera の near/far と underwater 視認性の調整

### 4.4 今の repo と直結できる topic / frame

- `image`: simulator camera image を detector 入力へ remap 可
- `odom`: ground truth / body velocity 推定元として利用可
- `cmd_thruster1..6`: adapter 出力先に最適

不足:

- `CameraInfo` bridge が launch に含まれていない
- depth camera ではなく RGB camera
- `base_link` と camera optical の frame 名が現 repo 既定とずれる

## 5. `orca4` 監査

### 5.1 参考になる点

- `BaseController` は `/cmd_vel` を受け、`/mavros/vision_pose/pose`、`/mavros/rc/override`、`/mavros/setpoint_position/global` に役割分離している。
- `manager.cpp` は ArduSub の arm / mode / message interval / base control takeover を明確に管理している。
- `sim_orca_params.yaml` と `bringup.py` は `map/slam/down/odom/base_link/left_camera_link` を明示している。
- `bringup.py` は down-facing camera 用に `slam -> down` static TF を追加している。
- `model.sdf` は stereo camera を物理モデルに埋め込み、ArduPilotPlugin と thruster command path を接続している。

根拠:

- `/tmp/orca4/orca_base/src/base_controller.cpp:42-47`
- `/tmp/orca4/orca_base/src/base_controller.cpp:389-440`
- `/tmp/orca4/orca_base/src/manager.cpp:49-176`
- `/tmp/orca4/orca_bringup/params/sim_orca_params.yaml:13-19`
- `/tmp/orca4/orca_bringup/launch/bringup.py:168-187`
- `/tmp/orca4/orca_description/models/orca4/model.sdf:344-418`
- `/tmp/orca4/orca_description/models/orca4/model.sdf:420-518`

### 5.2 そのまま採用すべきでない点

- ArduSub + MAVROS + Nav2 + ORB_SLAM2 を前提にしており、今回の bbox / target point / visual servo の最小検証には重い。
- `/cmd_vel` は `geometry_msgs/Twist` 前提で、現 repo の `TwistStamped body` と直接一致しない。
- BaseController は depth hold と RC override を同時に束ねるため、現 controller の簡易閉ループ検証には責務が広すぎる。

### 5.3 抽出すべき設計要素

- helper frame 設計: `slam -> down`, `base_link -> camera`
- backend 分離: 高位 `/cmd_vel` と MAVROS / ArduSub 送信を分離
- mode 管理: 実機段階で manager を別ノードに隔離

## 6. 比較表

| 案名 | ベース repo | ROS 2 Humble 適合性 | MAVROS 適合性 | 下向き camera 対応難易度 | 現 repo 再利用性 | 追加 adapter 必要性 | 想定リスク | 推奨度 |
|---|---|---|---|---|---|---|---|---|
| 案1: `bluerov2` + simulator-native 閉ループ | `CentraleNantesROV/bluerov2` | 高 | 低 | 低 | 高 | 中 | depth / camera_info 不足を補う必要 | 高 |
| 案2: `bluerov2` + adapter 境界を先に固定 | `CentraleNantesROV/bluerov2` | 高 | 中 | 低 | 高 | 高 | adapter 設計を誤ると二重実装 | 最高 |
| 案3: `orca4` ベースへ寄せる | `clydemcqueen/orca4` | 中 | 高 | 低 | 低 | 高 | Nav2 / ORB_SLAM2 / MAVROS 前提で初手が重い | 低 |

採用:

- 推奨案は案2
- Phase 0 相当の最初の動作確認は案1の軽量版でよい

## 7. 推奨アーキテクチャ

```text
[Simulator / Sensor Layer]
  Gazebo BlueROV2 world
  - down RGB camera -> /sim/down/image_raw
  - depth source -> /sim/down/depth/image_raw
  - camera_info publisher -> /sim/down/camera_info
  - pose/odom -> /sim/odom
  - seabed target object

          |
          | remap / small bridge
          v

[Perception / Controller Layer]
  detector -> /best_detection or /detections_output
  target_observer_node -> /sc2/visual_servo/target_obs
  target_point_estimator_node -> /sc2/visual_servo/target_point
  visual_servo_controller_node -> /sc2/visual_servo/cmd_vel_body

          |
          | backend adapter boundary
          v

[Adapter / Autopilot Layer]
  phase-1 debug:  direct_velocity_backend
  phase-1 eval:   dynamic_velocity_backend
  future:         cmd_vel_to_mavros_bridge / MAVROS backend
  future:         manager / ArduSub backend
```

## 8. 最小実装スコープ

1. `bluerov2` を vendor せず参照実装として読み、現 repo に simulator bringup package を追加する。
2. 下向き RGB camera と depth source を持つ BlueROV2 model variant を作る。
3. seabed target object を置いた world を追加する。
4. `image_raw` / depth / `camera_info` を現 perception launch へ remap する。
5. detector は Phase 1A synthetic と Phase 1B actual detector に分ける。
6. `/sc2/visual_servo/cmd_vel_body` は上位境界として固定し、backend は direct と dynamic を分離する。
7. `base_link` と down camera frame 群の static TF を追加する。

最低 topic set:

- image: `/sim/down/image_raw`
- depth: `/sim/down/depth/image_raw`
- camera_info: `/sim/down/camera_info`
- detection: `/best_detection` または `/detections_output`
- target point: `/sc2/visual_servo/target_point`
- cmd vel body: `/sc2/visual_servo/cmd_vel_body`
- sim odom/body velocity: `/sim/odom`, `/sim/body_velocity`

backend 契約:

- 上位 visual servoing は `geometry_msgs/TwistStamped` のみ publish
- `direct_velocity_backend`: デバッグ用
- `dynamic_velocity_backend`: 評価用
- 正式評価は `dynamic_velocity_backend`

## 9. 次の実装タスク案

1. `sc2_visual_servo_sim` パッケージを追加し、`sim_bluerov2_mvp.launch.py` を作る。
2. backend 境界を一般化し、`direct_velocity_backend` と `dynamic_velocity_backend` を分ける。
3. simulator body velocity を `TwistStamped` へ変換する `sim_body_velocity_bridge` を追加する。
4. down-camera variant と seabed-target world を追加する。
5. `CameraInfo` publisher と depth source を用意し、現 `target_point_estimator_node` へ接続する。
6. Phase 1A synthetic detection launch と Phase 1B actual detector launch を併置する。

## 10. 採否判断

- `orca4` は全面採用しない
- `orca4` のうち採るのは frame helper、backend 分離、MAVROS/ArduSub manager 分離
- 直近実装は `bluerov2` ベース + `TwistStamped` 出力固定 + dynamic backend 優先で進める
