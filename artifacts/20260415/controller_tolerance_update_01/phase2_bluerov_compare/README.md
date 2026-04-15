# Phase 2 Vertical Compare Artifacts

Primary report: `PHASE2_REAL_CONTROLLER_COMPARE_RESULT.md`.

Files:

- `PHASE2_REAL_CONTROLLER_COMPARE_RESULT.md`: human-readable result and interpretation.
- `phase2_real_controller_compare.json`: full row-by-row log.
- `cases/*.json`: per-case row logs.

Default source behavior under the standard launch command:

- primary: `/sc2/visual_servo/target_point`
- fallback: DVL altitude while target_point is stale/unavailable
- recovery: target_point becomes primary again as soon as it is fresh

Run:

```bash
uv run python tools/compare_visual_servo_controller_nodes_phase2_bluerov.py
```
