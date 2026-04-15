# Recommendation

Date: 2026-04-15

## Perception Readiness

The perception-side target_point changes are now test-backed in a ROS Humble container.

At this point, `target_point.pose.position.z` is sufficiently fixed as the staged PID primary vertical source for the next integration step.

Remaining perception validation before water/vehicle tests:

1. Run the same tests after a normal `colcon build` inside the target container.
2. Record a short bag and inspect `target_point_status`.
3. Verify real target range against a known distance.
4. Tune `max_uv_jump_px` and `max_depth_jump_m` if normal motion is rejected too often.

## DVL Fallback Recommendation

Use conservative fallback, not the current full-authority fallback.

Recommended next controller change:

- add `staged_pid_dvl_fallback_vz_scale`
- scale z command only while `z_source == dvl_fallback`
- keep all current visual gate requirements
- do not use DVL fallback to declare target-distance reached
- preserve immediate return to target_point when fresh

This is safer than current behavior because it acknowledges that DVL altitude is not target-relative.

## Next Priority

Recommended order:

1. Implement conservative DVL fallback policy with compare coverage.
2. Run container tests after colcon build so generated message packages are available.
3. Add short bag-based perception validation.
4. Then proceed to search/recovery.
5. Defer nvblox/fusion node until the default RGBD target_point path and DVL fallback are stable.

Depth fusion is still the right medium-term direction, but search/recovery will be easier to debug after DVL fallback authority is explicit and bounded.
