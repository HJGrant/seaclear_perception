# Test Matrix: OAK-D-PoE + VSLAM

**Purpose:** Keep a minimal, reproducible matrix for configuration testing.

## Configs
| ID | Params file | What it enables | Expected outcome | Notes |
|---|---|---|---|---|
| A | `oakd_stereo_only_30hz.yaml` | stereo raw only (baseline) | stable, near 30 Hz at low load | Use to detect pure disconnect issues |
| A400 | `oakd_stereo_only_30hz_400p.yaml` | stereo raw only @400p | stable 30 Hz | Current best baseline |
| B | `oakd_stereo_rect_30hz.yaml` | adds rectify/rect topics | should remain stable | rect topics may depend on driver pipeline |
| C | `oakd_stereo_rect_synced_30hz.yaml` | adds synced rect pair | might trigger driver issues | historically associated with crashes |
| D | `oakd_rgb_only_30hz.yaml` | RGB only | stable at target FPS | used to isolate RGB pipeline |
| E | `oakd_rgb_plus_stereo_30hz.yaml` | RGB + stereo | goal config | only after baseline is stable |

## Standard Run Procedure
Run inside the Isaac ROS dev container:
```bash
src/oakd_isaac_ros/scripts/diag_oakd_run_and_monitor.sh \
  --params-file <one of above> \
  --enable-vslam <true|false> \
  --iface eno1 \
  --duration-sec 300
```

## What to record
- Hz/bw for left/right `image_raw` and both `camera_info`
- `dmesg` link events
- NIC drops/errors (`ip -s`, `ethtool -S`)
- vslam topics (`odometry`, status)
- Any `X_LINK_ERROR` / `ping was missed` / `No Data` / crashes

