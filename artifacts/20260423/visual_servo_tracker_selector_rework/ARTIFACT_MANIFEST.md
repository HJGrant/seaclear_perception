# Artifact Manifest

Directory: `artifacts/20260423/visual_servo_tracker_selector_rework/`

## Required reports

- `IMPLEMENTATION_REPORT_JA.md`
- `TEST_REPORT_JA.md`
- `KNOWN_LIMITATIONS_JA.md`
- `ALGORITHM_AS_IMPLEMENTED_JA.md`
- `INTENDED_VS_IMPLEMENTED_DIFF_JA.md`
- `PARAMETER_GUIDE_JA.md`
- `CHANGE_SUMMARY_JA.md`
- `ARTIFACT_MANIFEST.md`

## CSV

- `CONFIG_INITIAL_VALUES.csv`
- `TOPIC_MESSAGE_INTERFACE.csv`

## Source snapshot

`repo_snapshot/` contains copied source files after this rework for review.

## Verification

- `python3 -m py_compile`: success
- pure Python unit test `test_tracker_selector_core.py`: 5 tests passed
- pure Python direct-call controller tests: 10 tests passed
- `python3 -m pytest`: not run because system Python has no `pytest`
- ROS/colcon tests: not run because required tools are unavailable on host
