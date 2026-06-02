# Parameter Guide

## 1. 実機でチューニングが必要なもの

- `target_class`: model / deployment target class の SoT に合わせる。
- `min_confidence`: high-confidence association threshold。bag replay で false positive / miss を見る。
- `low_confidence`: recovery threshold。低すぎると誤 association が増える。
- `base_gate_px`: target motion, frame rate, bbox jitter に合わせる。
- `r_gate_max_scale`: motion-aware gate の最大広がり。
- `t_stale_sec`: HOLD 継続時間と stale 判定。
- `n_confirm`: CONFIRMED までの連続 match 数。
- `hold_max_misses`: HOLD から LOST へ落とす miss budget。
- `lost_max_misses`: ESCALATE までの miss budget。
- `v_ref_mps`: vehicle speed normalization。
- `omega_ref_radps`: yaw-rate normalization。
- `beta_v`: linear speed gate contribution。
- `beta_omega`: yaw-rate gate contribution。
- `selector_switch_margin`: previous selected track から他 track へ切り替えるために必要な score 差。
- `selector_selected_track_bonus`: previous selected confirmed/fresh track に加える continuity bonus。
- `hold_maintain_z_kp_scale`: HOLD maintain vertical command の強さ。
- `hold_maintain_xy_damping`: HOLD 中の body-frame x/y velocity damping gain。
- `hold_maintain_yaw_damping`: HOLD 中の body yaw-rate damping gain。
- `hold_maintain_velocity_timeout_ms`: body velocity feedback freshness timeout。

## 2. 仮初期値のままでも動作確認に使えるもの

- `alpha_uv_predict = 0.0`: last-observation hold。初期確認には保守的。
- `confidence_ema_alpha = 0.35`: selection 用 confidence smoothing。
- `max_tracks = 8`: small field test には十分な track table size。
- `enable_motion_aware_gate = true`: odom/IMU がある bag/run では gate expansion 確認に使える。
- `hold_maintain_enable = true`: DVL が fresh なら zero-only HOLD を避ける。
- `hold_maintain_velocity_damping_enable = true`: body velocity が fresh なら XY/yaw の zero-only HOLD を避ける。
- `selector_switch_margin = 0.05`, `selector_selected_track_bonus = 0.05`: 初期確認用の continuity hysteresis。

## 3. 今回は固定または未使用のもの

- `use_depth = false`: P1-P4 主経路では target_point/depth path を使わない。
- `launch_target_point = false`: compatibility path として残すだけ。
- `/best_detection`: controller 互換 input として残すが primary algorithm には使わない。
- CameraInfo intrinsics: 配線/画像サイズ確認用。水中精密幾何の truth とはしない。

## Notes

初期値は仮である。特に threshold と gate は、`/detections_output` replay で ID switch、HOLD duration、low-confidence recovery の失敗/成功を見て調整する必要がある。
