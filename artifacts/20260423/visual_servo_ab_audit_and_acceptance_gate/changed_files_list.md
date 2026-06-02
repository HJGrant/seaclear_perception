# changed_files_list

共有対象の最終版 source 一覧。

| path | 区分 | 変更理由 |
|---|---|---|
| `src/sc2_visual_servo_interfaces/msg/TargetObservation.msg` | 編集 | observer->controller selector contract を保持 |
| `src/sc2_visual_servo_perception/sc2_visual_servo_perception/tracker_selector_core.py` | 新規 | multi-track / two-stage association / continuity / HOLD state |
| `src/sc2_visual_servo_perception/sc2_visual_servo_perception/target_observer_node.py` | 編集 | `/detections_output` primary observer, HOLD publish, continuity param |
| `src/sc2_visual_servo_perception/package.xml` | 編集 | `nav_msgs` import 依存整合 |
| `src/sc2_visual_servo_perception/test/test_tracker_selector_core.py` | 新規 | selector continuity / HOLD / recovery unit tests |
| `src/sc2_visual_servo_perception/test/test_target_observer_integration.py` | 編集 | observer integration coverage |
| `src/sc2_visual_servo_controller/sc2_visual_servo_controller/hold_maintain_core.py` | 新規 | HOLD damping/gate pure logic |
| `src/sc2_visual_servo_controller/sc2_visual_servo_controller/visual_servo_controller_node_implementation.py` | 編集 | HOLD gate 本体、DVL fallback import |
| `src/sc2_visual_servo_controller/package.xml` | 編集 | controller dependency metadata |
| `src/sc2_visual_servo_controller/test/test_hold_maintain_core.py` | 新規 | HOLD pure tests |
| `src/sc2_visual_servo_bringup/launch/object_detection_ibvs.launch.py` | 編集 | tracker/selector primary launch wiring |
| `src/sc2_visual_servo_bringup/package.xml` | 編集 | launch runtime dependency metadata |

