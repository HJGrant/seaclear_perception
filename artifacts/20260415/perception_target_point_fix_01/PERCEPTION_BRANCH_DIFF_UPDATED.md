# dev/aoki_perception_module vs Current Update

Date: 2026-04-15

Compared reference branch: `dev/aoki_perception_module` at `fd7ebc8874ab8b76c0228bb235ebeec930e5b787`

## Integration Table

| feature | in `dev/aoki_perception_module` | now in current | files | status | comment |
| --- | --- | --- | --- | --- | --- |
| bbox temporal EMA | yes | yes | `target_point_estimator_node.py` | integrated | ported with `filter_alpha_uv` / `filter_alpha_size` |
| representative `(u,v,depth)` EMA | yes | yes | `target_point_estimator_node.py` | integrated | ported with `filter_alpha_uv` / `filter_alpha_depth` |
| large bbox jump rejection | yes | yes | `target_point_estimator_node.py` | integrated | rejects center/size jumps above `max_uv_jump_px` |
| large depth jump rejection | yes | yes | `target_point_estimator_node.py` | integrated | rejects jumps above `max_depth_jump_m` and holds last valid pose |
| hold-last-valid helper | yes | yes | `target_point_estimator_node.py` | integrated | centralized helper replaces duplicated inline hold path |
| stale depth age check | no explicit current default | yes | `target_point_estimator_node.py` | current improvement | added `max_depth_age_ms` so depth freshness is independent from bbox freshness |
| `target_point.pose.position.z` as target-relative range | no | yes | `target_point_estimator_node.py` | current improvement | dev/aoki still used a remapped pose convention that did not make `.z` the range |
| `target_obs_topic` legacy parameter | yes | yes | `target_point_estimator_node.py` | integrated | restored as deprecated alias because `mvp_sim.launch.py` still passes it |
| nvblox back-projected PointCloud2 source mode | yes | no | `target_point_estimator_node.py` | not integrated | deferred because the default experiment path uses RGBD local depth and nvblox topic/frame validation is not done |
| nvblox test | yes | no | perception tests | not integrated | should follow after nvblox default path or explicit source mode is selected |

## Summary

The short-term stabilizers from `dev/aoki_perception_module` were integrated into the current default RGBD ROI path. The pose field convention was deliberately changed beyond dev/aoki because the controller needs `PoseStamped.position.z` to mean target-relative range.

nvblox support was not copied in this step. It needs a separate source-mode decision and validation of topic availability, organized point cloud layout, frame convention, and ROI association.
