# Project Status: OAK-D-PoE + Isaac ROS Visual SLAM (Orin)

**Last updated:** 2026-02-23

## Goal
Run **Isaac ROS Visual SLAM (cuVSLAM)** on Jetson Orin using **OAK-D-PoE** as a stereo camera, and ultimately support **simultaneous RGB + stereo** publishing without destabilizing the system.

## Current Situation (What works today)
### Stable stereo baseline exists
- Using **Stereo-only, 400p**, both left/right `image_raw` and both `camera_info` streams can be sustained at **~30 Hz** with consistent bandwidth (~7.7 MB/s per mono stream).
- NIC/link statistics during the stable 400p run show **no increasing RX/TX errors or drops**.

### Known failure modes (still observed)
1. **Device disconnect / stream stop** in some configurations:
   - Log signatures: `ping was missed`, `X_LINK_ERROR`, `No Data` (DepthAI diagnostics).
   - After a disconnect event, the driver frequently **does not auto-recover**; restart is required.
2. **Host-side throughput ceiling at higher resolutions**:
   - At higher payload sizes (e.g., ~720p mono), observed `image_raw` publish rates plateau around **25–27 Hz** even when `camera_info` remains ~30 Hz.
   - Strongly suggests a **host-side bottleneck (ROS publish/DDS/CPU/memory copy)** rather than the device being unable to generate 30 Hz.
3. **ROS 2 graph/daemon instability**:
   - Symptoms: `ros2 topic list` works but `ros2 node list`/`ros2 param get` fails (`Node not found`).
   - Workaround: restart `ros2 daemon` and clear daemon cache.
4. **Visual SLAM integration issues**:
   - cuVSLAM initializes, but odometry may not respond to motion or may jump (e.g., ~100 m).
   - Likely causes under investigation:
     - image_transport republish misconfiguration (unexpected transport selection such as compressedDepth for mono8)
     - camera + vslam in the same single-thread container causing scheduling delays
     - TF/baseline sign or frame mismatch
     - intermittent frame timing gaps detected by vslam (`Delta ... above threshold`)

## Primary Hypotheses
1. **Performance bottleneck at higher resolution is on the host** (publish/DDS/copies), not the OAK-D sensor.
2. **Disconnect events** are not explained by NIC error counters alone; likely **device/driver state** or PoE/initialization behavior. Need correlation via `dmesg` link events + depthai logs.
3. VSLAM “not moving / jumps” is primarily **integration (topics, transport, TF)** rather than feature scarcity.

## Priority Tasks
### P0 — Make VSLAM integration correct on stable 400p baseline
- Ensure vslam receives the intended topics (raw vs republished) reliably.
- Validate TF chain and baseline sign.
- Confirm odometry updates with motion (no large jumps).

### P1 — Improve diagnostics robustness
- Ensure all diagnosis runs capture:
  - publisher node names (`ros2 topic info -v`)
  - applied params (`ros2 param dump`)
  - container/host identity (host vs container) to avoid mixing measurements.
- Make diagnosis script robust to `Ctrl+C` (trap cleanup).

### P2 — Characterize and mitigate higher-resolution bottleneck
- Repeat diagnosis at 720p mono and compare CPU/DDS load.
- Try CycloneDDS vs FastDDS; try changing QoS; avoid unnecessary republish.

### P3 — Reintroduce RGB alongside stereo
- Add RGB back with bandwidth controls (resolution/FPS/encoding) after stereo+vslam is correct.

## How to Run (canonical)
1. Start dev container:
```bash
cd /mnt/nova_ssd/workspaces/oakd_isaac_ros
src/isaac_ros_common/scripts/run_dev.sh --isaac_ros_dev_dir /mnt/nova_ssd/workspaces/oakd_isaac_ros
```
2. Build + source in container:
```bash
cd /workspaces/oakd_isaac_ros
colcon build --merge-install
source /workspaces/oakd_isaac_ros/install/setup.bash
```
3. Diagnosis runs (run inside container):
```bash
src/oakd_isaac_ros/scripts/diag_oakd_run_and_monitor.sh \
  --params-file src/oakd_isaac_ros/config/oak_d/oakd_stereo_only_30hz_400p.yaml \
  --enable-vslam false \
  --iface eno1 \
  --duration-sec 300
```

## Artifacts
- See `docs/oakd-vslam/attachments/` for zipped diagnosis logs and reference snapshots.
