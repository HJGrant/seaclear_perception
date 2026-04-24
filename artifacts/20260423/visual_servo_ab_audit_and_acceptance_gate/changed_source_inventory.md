# changed_source_inventory

対象は tracker/selector/HOLD gate の実装、依存、launch、テストである。分類はこの機能系列での最終状態ベース。

| path | 区分 | 役割 | 監査メモ |
|---|---|---|---|
| `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg` | 編集 | observer->controller 契約 | `selected_valid`, `selected_track_id`, `track_state`, `reason_code`, `age_frames`, `miss_count`, `freshness` を持つ |
| `src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py` | 新規 | track table / association / selector continuity | multi-track, two-stage association, HOLD/LOST 遷移, continuity bias |
| `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py` | 編集 | `/detections_output` から `TargetObservation` を publish | selector continuity param 配線, HOLD publish, health timeout hold |
| `src/sc2_visual_servo_perception/package.xml` | 編集 | perception import 依存 | `nav_msgs` を含み `target_observer_node.py` の import と整合 |
| `src/sc2_visual_servo_perception/test/test_tracker_selector_core.py` | 新規 | pure unit test | continuity / HOLD / low-confidence recovery を検証 |
| `src/sc2_visual_servo_perception/test/test_target_observer_integration.py` | 編集 | observer integration test | confirm -> low-confidence recovery -> HOLD を検証 |
| `src/sc2_visual_servo_controller/sc2_visual_servo_controller/hold_maintain_core.py` | 新規 | HOLD maintain の純粋関数 | selected invalid/freshness gate, velocity damping |
| `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py` | 編集 | staged PID + HOLD gate 本体 | visual invalid/stale で hold maintain に切替、clean env fallback import を追加 |
| `src/sc2_visual_servo_controller/package.xml` | 編集 | controller import 依存 | `std_srvs`, `sensor_msgs`, `vision_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `state_estimation`, `marine_acoustic_msgs` を宣言 |
| `src/sc2_visual_servo_controller/test/test_hold_maintain_core.py` | 新規 | HOLD 純粋関数 test | invalid gate と damping clamp を検証 |
| `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py` | 編集 | production bringup wiring | tracker/selector path を primary にし continuity params を設定 |
| `src/sc2_visual_servo_bringup/package.xml` | 編集 | bringup runtime 依存 | `ament_index_python`, `oakd_isaac_ros` を追加 |

補足:

- `docs` 指定の開始前文書のうち、以下は repo 内で発見できなかった。  
  `THIRD_PARTY_NOTE_VISUAL_SERVO_TRACKING_POLICY_20260422_v3.md`  
  `PM_REPORT_20260422_DUAL_BAG_ANALYSIS_AUDIT_JA.md`  
  `PM_REPORT_20260422_TF_FORENSIC_WITH_MANUAL_STATIC_JA.md`  
  `PM_REPORT_20260423_TRACKER_REWORK_DETAILED_REVIEW_JA.md`  
  `VS_SPEC_MEMO_P0_UPDATED_20260422.md`

