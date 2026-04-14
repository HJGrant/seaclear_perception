# SIMULATOR_PHASE1_SPEC_AND_MVP_IMPLEMENT

実施日: 2026-04-12
作業 branch: `simulator/bluerov2_humble_eval`

## 1. 目的

`CentraleNantesROV/bluerov2` をベースに、下向き camera を持つ BlueROV2 simulator と現 `sc2_visual_servo_*` を接続し、まずは MAVROS なしで閉ループ MVP を成立させる。

重要条件:

- 下向きであることが要件であり、単眼化は要件ではない
- 初回 MVP は下向き RGB + depth source を simulator 側で供給する
- 上位 visual-servoing / perception の最終出力 I/F は `geometry_msgs/msg/TwistStamped` に固定する

## 2. 必須修正方針

1. `monocular` を前提にしない
2. `cmd_vel_body -> raw thrusters` の直接変換を正式 backend にしない
3. frame 契約を `odom -> base_link -> down_camera_link -> down_camera_optical_frame` で明示する
4. Phase 1A と 1B を acceptance criteria まで分ける

## 3. backend 契約

### 上位 I/F

- publish topic: `/sc2/visual_servo/cmd_vel_body`
- type: `geometry_msgs/msg/TwistStamped`
- frame: `base_link`

上位 node は thrust / RC override / MAVROS 固有 topic を publish しない。

### backend 種別

#### `direct_velocity_backend`

- 用途: デバッグ、frame/sign 切り分け
- 入力: `TwistStamped`
- 出力: simulator vehicle への近似的速度反映
- 評価用途: 補助のみ

#### `dynamic_velocity_backend`

- 用途: 正式評価
- 入力: `TwistStamped`, `Odometry`, `tf`
- 制御: body velocity setpoint から内部で wrench / thrust に変換
- 出力: Gazebo thruster plugin compatible topic

## 4. `auv_control` / `thruster_manager` 流用判断

dynamic backend の第一候補として採用価値が高い。

根拠:

- `auv_control` は `cmd_vel (TwistStamped)` を subscribe: `/tmp/auv_control/src/controller_io.cpp:85-89`
- `auv_control` は `odom (Odometry)` を subscribe: `/tmp/auv_control/src/controller_io.cpp:96-99`
- `auv_control` は `tf` を使って frame 変換する: `/tmp/auv_control/src/controller_io.cpp:236-244`
- `auv_control` は allocator を通して Gazebo 互換の `cmd_<joint>` を publish する: `/tmp/auv_control/src/controller_io.cpp:216-233`
- README でも `cmd_vel` / `odom` / `/tf` 入力と `<thruster>_cmd` 出力が明記されている: `/tmp/auv_control/README.md:13-24`
- `thruster_manager` は `Wrench` から thrust を解く standalone node として利用できる: `/tmp/thruster_manager/src/thruster_manager_node.cpp:17-54`, `/tmp/thruster_manager/src/thruster_manager_node.cpp:58-85`

推奨構成:

`visual_servo_controller_node -> TwistStamped(body) -> dynamic_velocity_backend(auv_control) -> Gazebo thruster topics`

## 5. frame 契約

### frame chain

```text
odom
  └─ base_link
      └─ down_camera_link
          └─ down_camera_optical_frame
```

### 軸規約

- `odom`: ENU
- `base_link`: FLU
  - `+x`: forward
  - `+y`: left
  - `+z`: up
- `down_camera_optical_frame`: optical convention
  - `+x`: image right
  - `+y`: image down
  - `+z`: optical forward

下向き camera では optical `+z` が物理的には vehicle 下向きになる。image `u/v` 誤差が body のどの軸へ写るかは static TF で固定し、左右・前後・高度の単体テストで検証する。

## 6. Phase 分離

### Phase 1A

- detector: synthetic bbox / ideal target
- 目的: loop wiring, frame/sign validation
- acceptance:
  - topics alive
  - `target_point` finite
  - center error decreases in 3 initial offsets

### Phase 1B

- detector: actual detector on simulator RGB
- 目的: bbox jitter robustness
- acceptance:
  - detector produces usable bbox
  - center error still trends downward
  - lighting / altitude table completed

## 7. 実装範囲

- simulator 用 package 追加
- down-facing camera variant 追加
- seabed target world 追加
- `camera_info` と depth source の供給
- body velocity bridge
- backend adapter
- Phase 1 launch
- 最低限の再現手順と可視化

## 8. 受け入れ条件

1. `ros2 launch ...` 1 本で simulator + perception/controller MVP が起動する
2. `image_raw`, `camera_info`, depth, detection, target_point, cmd topic が流れる
3. `target_point` が有限値で安定 publish される
4. 初期オフセット 3 ケースで target center 誤差が減少する
5. frame / sign 誤りがないことを左右・前後・高度の単体テストで確認する
6. lighting 2 条件、高度 3 条件の簡易表を出す
7. 今回は MAVROS 未使用を明記する
8. 次段で MAVROS backend に差し替えるための I/F 境界を文書化する

## 9. 実装優先順

1. frame / sign 契約の固定
2. `direct_velocity_backend` で sign 検証
3. `dynamic_velocity_backend` を `auv_control` ベースで接続
4. Phase 1A synthetic detection
5. Phase 1B actual detector
