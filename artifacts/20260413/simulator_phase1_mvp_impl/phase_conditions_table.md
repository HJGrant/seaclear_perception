# Phase Conditions Table

## Phase split

| Phase | Detector source | Purpose | Status in this task |
| --- | --- | --- | --- |
| Phase 1A | synthetic bbox | frame/sign/topic/backend loop wiring | implemented, runtime pending |
| Phase 1B | rendered RGB detector | detector path integration | implemented, runtime pending |

## Evaluation sweep to execute in ROS runtime

| Dimension | Conditions |
| --- | --- |
| Altitude | `initial_z_m=-1.0`, `-1.5`, `-2.0` |
| Lighting | `bright`, `dim` |
| Initial offset | `(x,y)=(+1.0,+0.6)`, `(+1.0,-0.6)`, `(0.3,+0.8)` |
| Initial yaw | `0.0`, `0.4` |

## Acceptance-oriented matrix

| Case ID | Phase | Backend | Lighting | Altitude | Initial offset | Intended judgment |
| --- | --- | --- | --- | --- | --- | --- |
| A1 | 1A | dynamic | bright | -1.0 | (+1.0,+0.6) | success target |
| A2 | 1A | dynamic | bright | -1.5 | (+1.0,-0.6) | success target |
| A3 | 1A | dynamic | dim | -2.0 | (+0.3,+0.8) | success target |
| B1 | 1B | dynamic | bright | -1.0 | (+1.0,+0.6) | success target |
| B2 | 1B | dynamic | dim | -1.5 | (+1.0,-0.6) | stability check |
| D1 | 1A | direct | bright | -1.5 | (+1.0,+0.6) | debug only |
