# Target Point Field Semantics

Date: 2026-04-15

Branch: `grapple_integration`

Current base before this fix: `39e192bcf870dc36291958f813e3ae10d21e2a5d`

## Decision

`/sc2/visual_servo/target_point` remains a `geometry_msgs/msg/PoseStamped`, but its position fields are now fixed as camera optical-frame target-relative coordinates:

| field | meaning | unit | controller relevance |
| --- | --- | --- | --- |
| `pose.position.x` | metric horizontal target offset from optical center, computed from representative `u` and depth | m | diagnostic / future fusion |
| `pose.position.y` | metric vertical target offset from optical center, computed from representative `v` and depth | m | diagnostic / future fusion |
| `pose.position.z` | target-relative range/depth from local RGBD target ROI | m | staged PID vertical primary source |

The staged PID controller should continue to read `target_point.pose.position.z` for target-relative distance.

## Previous Current Behavior

Before this fix, current `target_point_estimator_node.py` built the pose as:

```text
position.x = metric x from u/depth
position.y = representative range/depth
position.z = -metric y from v/depth
```

That made `position.z` a vertical image-plane offset rather than target-relative distance. It conflicted with the controller Phase 2 assumption that `target_point.pose.position.z` is the target-relative vertical/range source.

## Adopted Convention

For representative pixel/depth `(u_rep, v_rep, z_rep)` and camera intrinsics `(fx, fy, cx, cy)`:

```text
x = (u_rep - cx) / fx * z_rep
y = (v_rep - cy) / fy * z_rep
z = z_rep
```

The output pose is:

```text
pose.position.x = x
pose.position.y = y
pose.position.z = z
```

This is the smallest correction because it keeps the controller interface unchanged and makes the perception publisher supply the field the controller already consumes.

## Controller Alignment

`VisualServoControllerNode._resolve_staged_pid_vertical_measurement()` reads:

```python
self._latest_target_point.pose.position.z
```

After this fix, that value is target-relative range in meters. DVL remains a controller fallback only when target_point is stale or unavailable.

## Remaining Hardware Check

The range field semantics are now fixed in software. Hardware testing still needs to verify the vehicle command sign convention for positive z velocity.
