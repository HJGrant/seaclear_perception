# Artifact Manifest

Directory:

`artifacts/20260423/visual_servo_tracker_selector_task_ab_fixes/`

## Required Files

- `IMPLEMENTATION_REPORT_JA.md`
- `CHANGE_SUMMARY_JA.md`
- `TEST_REPORT_JA.md`
- `DEPENDENCY_FIX_NOTES_JA.md`
- `ALGORITHM_AS_IMPLEMENTED_JA.md`
- `INTENDED_VS_IMPLEMENTED_DIFF_JA.md`
- `KNOWN_LIMITATIONS_JA.md`
- `PARAMETER_GUIDE_JA.md`
- `CONFIG_INITIAL_VALUES.csv`
- `TOPIC_MESSAGE_INTERFACE.csv`
- `ARTIFACT_MANIFEST.md`
- `SELECTOR_CONTINUITY_DESIGN_JA.md`
- `REACQUIRE_POLICY_NOTE_JA.md`

## Verification Summary

- `python3 -m py_compile`: success
- perception pure Python unit tests: 7 tests passed
- controller pure Python direct-call tests: 10 tests passed
- `python3 -m pytest`: not run because system Python has no `pytest`
- ROS/colcon integration: not run because `ros2` / `colcon` are unavailable on this host
