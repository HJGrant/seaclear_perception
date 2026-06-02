# Change Summary

## Perception

Added:

- `tracker_selector_core.py`

Changed:

- `target_observer_node.py`

Details:

- replaced single active track with `self._tracks`
- added greedy track x detection assignment
- added two-stage high/low confidence association
- added track-table selection after association
- added pruning by track quality and max track count

## Controller

Changed:

- `visual_servo_controller_node_implementation.py`

Added:

- `hold_maintain_core.py`

Details:

- added `hold_maintain_enable`
- added `hold_maintain_z_kp_scale`
- added body velocity damping parameters for HOLD
- added `_compute_hold_maintain_command`
- `selected_valid=false` now stops visual correction and uses DVL/altitude vertical maintain plus velocity damping when available

## Launch

Changed:

- `object_detection_ibvs.launch.py`

Details:

- added `max_tracks`
- added HOLD maintain parameters
- keeps target_point disabled by default

## Tests

Added:

- `test_tracker_selector_core.py`
- `test_hold_maintain_core.py`

Updated:

- `test_target_observer_integration.py`

## Still unchanged

- detector
- `/best_detection` publisher
- `target_point_estimator_node.py`
- precise camera/TF geometry
