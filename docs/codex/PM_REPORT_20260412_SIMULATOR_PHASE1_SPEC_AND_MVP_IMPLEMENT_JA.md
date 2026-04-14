# PM_REPORT_20260412_SIMULATOR_PHASE1_SPEC_AND_MVP_IMPLEMENT_JA

## 結論

Phase 1 実装は、`bluerov2` ベース、MAVROS 後回し、`TwistStamped` 出力固定で進める。camera 要件は「下向き」であり、depth source を残す。正式評価 backend は raw thruster 直結ではなく、body velocity から動的に thrust へ落とす `dynamic_velocity_backend` とする。

## 設計修正点

- `monocular` 前提を撤回し、downward RGB + depth に修正
- raw thruster 直結案を正式案から外し、`auv_control` / `thruster_manager` の利用を優先
- frame 契約を `odom -> base_link -> down_camera_link -> down_camera_optical_frame` で固定
- synthetic detection と actual detector を Phase 1A / 1B に分離

## 根拠

- `auv_control` は `cmd_vel(TwistStamped)`、`odom(Odometry)`、`tf` を入力に取り、Gazebo 互換の thruster command を出力できる
- `thruster_manager` は `Wrench` から Gazebo plugin 向け thrust を生成できる
- 現 visual-servo 側は `TwistStamped` 出力に固定できる

## 次アクション

1. frame/sign 図と単体テスト仕様を固定
2. down camera + depth simulator bringup
3. `direct_velocity_backend` と `dynamic_velocity_backend` を分離実装
4. Phase 1A / 1B launch と評価表を追加
