# Recommendation

Date: 2026-04-15

## Is `target_point` Now Suitable As Controller Primary?

Yes for the next controlled tests. The field convention is now explicit and test-covered: `target_point.pose.position.z` is target-relative range. The estimator also has temporal smoothing, large-jump rejection, stale depth handling, and short-gap hold.

This does not mean the depth estimate is final. It is still RGBD ROI based, so it can fail with bad stereo depth, poor ROI, specular surfaces, or occlusion.

## DVL Fallback

Keep DVL as fallback, not primary.

DVL altitude is floor/terrain-relative, not target-relative. It is useful for safety and continuity when target_point is stale, but it should not replace fresh target-relative ROI depth. The controller behavior should remain:

```text
fresh target_point -> use target_point.pose.position.z
target_point stale/unavailable and fresh DVL -> use DVL fallback
target_point recovers -> return to target_point
```

The next improvement should add sanity checks when both are fresh, rather than hard-switching the primary source to DVL.

## nvblox

Do not put nvblox into the default path in this commit.

Reason:

- current default launch already has a direct RGBD path
- nvblox topic/frame/organized cloud assumptions need runtime validation
- ROI-to-nvblox association needs explicit testing
- adding it now would mix field semantics repair with a new depth source

Recommended next nvblox step: add an explicit opt-in `source_mode=nvblox_back_projected_depth` or separate fusion node prototype with recorded data.

## Search / Recovery Ordering

Before implementing full search/recovery, do one short perception validation pass in the ROS container:

1. run the new perception tests with ROS sourced
2. record `target_point_status` during representative target motion
3. verify no excessive `REJECTED_LARGE_JUMP` under normal motion
4. verify `position.z` range matches expected target-relative distance on a known setup

Then proceed to search/recovery. Search behavior depends heavily on whether target_point stale/noise semantics are reliable.

## Fusion Node Option

A separate `depth_fusion_node` is still the cleaner medium-term design if DVL/RGBD/nvblox are all used:

| option | recommendation |
| --- | --- |
| extend current estimator | good for bbox/RGBD filtering and field semantics; implemented now |
| new depth fusion node | best for DVL/RGBD/nvblox arbitration after source semantics are stable |
| controller-only fallback | acceptable short-term; avoid putting complex filtering into controller |

A 1D Kalman-style filter can be useful only if the observation models are separated. RGBD/nvblox can observe target-relative range; DVL observes altitude to floor/terrain. Treating both as the same measurement without geometry assumptions would be wrong.
