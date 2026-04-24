# traceability_matrix

| 主張 / acceptance 項目 | 根拠 | 判定 |
|---|---|---|
| multi-track table が入っている | `tracker_selector_core.py:13-174` | 一致 |
| high-confidence first / low-confidence second の two-stage association がある | `tracker_selector_core.py:210-255` | 一致 |
| selector continuity bonus / switch margin が実装されている | `tracker_selector_core.py:279-330`, `target_observer_node.py:69-70,374-380`, `object_detection_ibvs.launch.py:105-106` | 一致 |
| `REACQUIRE` を独立 state にしない方針 | state 定義が `TENTATIVE/CONFIRMED/HOLD/LOST/ESCALATE` のみ (`tracker_selector_core.py:6-10`)。復帰は `update_track_with_match()` が `HOLD -> CONFIRMED` (`115-148`) | 一致 |
| `TargetObservation` に selector 状態が載る | `TargetObservation.msg:12-20`, `target_observer_node.py:287-325` | 一致 |
| stale confirmed track が miss budget 中は `HOLD` に残る | `update_track_with_miss()` (`150-174`) | 一致 |
| controller が invalid/stale 時に visual correction を止める | `hold_maintain_core.py:25-35`, `visual_servo_controller_node_implementation.py:1160-1182` | 一致 |
| HOLD が zero command only ではない | `hold_maintain_core.py:38-49`, `visual_servo_controller_node_implementation.py:781-830` | 一致 |
| controller dependency metadata が import と整合する | `sc2_visual_servo_controller/package.xml:12-23` と controller import 群 | 部分一致 |
| bringup dependency metadata が launch import と整合する | `sc2_visual_servo_bringup/package.xml:12-21`, `object_detection_ibvs.launch.py:1-47` | 一致 |
| clean env で core build/install が通る | docker `colcon build --packages-select state_estimation`, `sc2_visual_servo_controller`, `sc2_visual_servo_bringup`; `packages-up-to sc2_visual_servo_bringup` は `oakd_isaac_ros` で失敗 | 部分一致 |
| production launch が通る | `ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false` -> `PackageNotFoundError: oakd_isaac_ros` | 不一致 |
| `/sc2/visual_servo/target_obs` live publish | docker harness 実行 `obs_count=5940`, `valid_obs_count=5934` | 一致 |
| `selected_valid=false` / stale で HOLD enter | docker harness `first_hold={... track_state='HOLD' selected_valid=False ...}` | 一致 |
| HOLD maintain が非ゼロ command を出す | docker harness `first_nonzero_hold_cmd={'vx': -0.08, 'vy': 0.04, 'vz': 0.12, 'yaw': -0.045}` | 一致 |
| 複数 detection 条件で selected track が不自然に飛びすぎない | docker harness `phase1_ids=[1]`, `phase2_ids=[1]`, `phase3_ids=[2]` | 一致 |

