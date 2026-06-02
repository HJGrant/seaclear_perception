# Environment Summary

## Workspace

- repo: `/home/nobu/work/2025/SeaClear/oakd_isaac_ros_lrz`
- task date: `2026-04-23`

## Clean ROS environment

- runtime/build environment: docker image `acb2bd610943`
- ROS underlay: `/opt/ros/humble`
- GPU: unavailable in this container session

## Start-of-task references

読了:

- `PLANS.md`
- `docs/codex/CODEX_TASK_20260423_VISUAL_SERVO_AB_AUDIT_AND_ACCEPTANCE_GATE_JA.md`
- `artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/traceability_matrix.md`
- `artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/code_review_findings.md`
- `artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/runtime_validation_log.md`
- `artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/topic_wiring_table.md`
- `artifacts/20260423/visual_servo_ab_audit_and_acceptance_gate/code_snapshot/` 配下

未発見:

- `THIRD_PARTY_NOTE_VISUAL_SERVO_TRACKING_POLICY_20260422_v3.md`

## Clean env で不足していた主要 package

- `oakd_isaac_ros`
- `depthai_ros_driver`
- `depthai_descriptions`
- `isaac_ros_dnn_image_encoder`
- `isaac_ros_tensor_rt`
- `isaac_ros_yolov8`
- `isaac_ros_image_proc`

## DVL interface 事情

- recorded bag metadata 上の canonical topic type は `marine_acoustic_msgs/msg/Dvl`
- clean env には official `marine_acoustic_msgs` package がない
- local shim package を一度作ったが bag schema と不一致で runtime kill を再現したため削除した

