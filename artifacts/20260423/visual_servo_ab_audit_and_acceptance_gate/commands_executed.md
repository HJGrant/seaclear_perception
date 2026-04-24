# commands_executed

主要な確認コマンドのみ記録する。

1. source doc / source audit
   - `sed -n '1,260p' docs/codex/CODEX_TASK_20260423_VISUAL_SERVO_AB_AUDIT_AND_ACCEPTANCE_GATE_JA.md`
   - `rg --files | rg 'THIRD_PARTY_NOTE_VISUAL_SERVO_TRACKING_POLICY_20260422_v3\\.md|PM_REPORT_20260422_DUAL_BAG_ANALYSIS_AUDIT_JA\\.md|PM_REPORT_20260422_TF_FORENSIC_WITH_MANUAL_STATIC_JA\\.md|PM_REPORT_20260423_TRACKER_REWORK_DETAILED_REVIEW_JA\\.md|VS_SPEC_MEMO_P0_UPDATED_20260422\\.md'`

2. clean env build
   - `docker run ... colcon build --base-paths src --packages-select state_estimation --cmake-args -G Ninja --event-handlers console_direct+`
   - `docker run ... colcon build --base-paths src --packages-select sc2_visual_servo_controller --cmake-args -G Ninja --event-handlers console_direct+`
   - `docker run ... colcon build --base-paths src --packages-select sc2_visual_servo_bringup --event-handlers console_direct+`
   - `docker run ... colcon build --packages-up-to sc2_visual_servo_bringup --cmake-args -G Ninja --event-handlers console_direct+`

3. runtime failure reproduction
   - `docker run ... python3 -c "import marine_acoustic_msgs, state_estimation; print('ok')"`
   - `docker run ... ros2 run sc2_visual_servo_controller visual_servo_controller_node --ros-args -p use_target_observation:=true -p controller_mode:=staged_pid -p depth_estimation_mode:=DVL`
   - `docker run ... ros2 launch sc2_visual_servo_bringup object_detection_ibvs.launch.py launch_foxglove:=false`

4. live runtime harness
   - `docker run ... python3 /ws/artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/runtime_harness.py`

5. package / launch / source inspection
   - `nl -ba src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py`
   - `nl -ba src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py`
   - `nl -ba src/sc2_visual_servo_controller/sc2_visual_servo_controller/hold_maintain_core.py`
   - `nl -ba src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py`
   - `nl -ba src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py`
   - `nl -ba src/sc2_visual_servo_controller/package.xml`
   - `nl -ba src/sc2_visual_servo_bringup/package.xml`

