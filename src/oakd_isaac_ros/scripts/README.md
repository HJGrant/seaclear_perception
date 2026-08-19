## Viewing a stream (no latency measurement)

`view_stream.py` just picks the right port for you and opens the video --
useful day to day when you don't need actual latency numbers, only to look
at the stream. Runs on the host, no ROS needed (needs `gst-launch-1.0` and
PyYAML -- `sudo apt install python3-yaml` if missing):

```bash
python3 src/oakd_isaac_ros/scripts/view_stream.py rgb
python3 src/oakd_isaac_ros/scripts/view_stream.py mask_overlay
python3 src/oakd_isaac_ros/scripts/view_stream.py depth
```

It reads the port straight out of `config/system_params.yaml` (`rtp.<stream>.port`),
so it stays correct if you ever change the ports there. Pass `--port N` to
bypass the config file entirely (e.g. on a host that doesn't have this repo
checked out), or `--config PATH` to point at a config file somewhere else.

# RTP glass-to-glass latency test

Measures true end-to-end latency (camera capture -> ISP -> encode -> network
-> decode -> received frame) for the RGB RTP stream, using
`latency_stamp.py` (sender) and `latency_measure.py` (receiver).

## How it works

`latency_stamp.py` burns the current wall-clock time (ms since epoch) into
the top-left corner of each RGB frame as a row of black/white blocks (not
text -- robust to H264 compression, unlike OCR), before it reaches the
encoder. `latency_measure.py` decodes that block on the receiving end and
reports `(receiver's clock right now) - (decoded capture time)`.

This is only meaningful if sender and receiver share a clock. On localhost
that's automatic. Sender (Jetson) and receiver (topside host) on the real
tether are different machines, so their clocks must be synchronized first
(chrony/NTP) -- otherwise the reported number is just however far apart
their clocks are, plus or minus the real latency.

## Prerequisites

- **On the Jetson**: the normal perception pipeline running
  (`ros2 launch oakd_isaac_ros perception.launch.py`), so `/oak/rgb/image_raw`
  is actually publishing.
- **On the topside host**: `python3-gi` and GStreamer (already present if
  `gst-launch-1.0` works). `latency_measure.py` needs no ROS install.
- **Clock sync** (skip for localhost testing) -- see below.

## Syncing the Jetson and host clocks

Skip this for localhost testing (sender and receiver share a clock
automatically). For the real Jetson + tether test, the Jetson likely has no
direct internet access over the tether, so the simplest setup is: the
topside host runs its own local time server, and the Jetson syncs against
*it* directly, rather than both trying to reach a public NTP pool.

**On the topside host** (192.168.2.94) -- make it a local chrony server:
```bash
sudo apt install chrony
sudo tee -a /etc/chrony/chrony.conf <<'EOF'
allow 192.168.2.0/24
local stratum 10
EOF
sudo systemctl restart chrony
```
(`local stratum 10` lets chrony serve time even if the host itself has no
upstream NTP source right now -- fine for our purposes, we only care about
the two machines agreeing with each other, not absolute correctness.)

**On the Jetson** (at the host OS level, not inside the container --
containers share the host kernel's clock, so this is sufficient):
```bash
sudo apt install chrony
sudo sed -i 's/^pool .*/server 192.168.2.94 iburst/' /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

**Verify on the Jetson**, wait ~30s, then:
```bash
chronyc tracking
```
Look for `Reference ID` matching the host and `System time` offset
converging to a small number (well under a millisecond, ideally). If it's
still off by tens of ms after a minute, don't trust the latency numbers yet
-- that offset shows up directly as error in what `latency_measure.py`
reports.

## Running the test

All commands below run inside the Jetson's container unless noted.

1. Make sure the shell you use for step 2 has the *same* ROS environment as
   the shell running the main pipeline (same `source install/setup.bash`,
   same `ROS_DOMAIN_ID`/FASTRTPS config -- don't `export`/`unset` DDS
   discovery variables in just this one shell, or the stamp node won't
   discover the rest of the graph even though it starts up fine).

2. Start the stamper:
   ```bash
   python3 src/oakd_isaac_ros/scripts/latency_stamp.py
   ```
   It subscribes to `/oak/rgb/image_raw` and republishes the stamped copy on
   `/oak/rgb/image_raw_stamped`.

3. In another shell (same environment), send that topic out over RTP to the
   topside host, on a throwaway port so it doesn't collide with the real
   production RGB stream (8550):
   ```bash
   ros2 run ros_deep_learning video_output --ros-args \
     -r image_in:=/oak/rgb/image_raw_stamped \
     -p resource:=rtp://192.168.2.94:9999 -p codec:=h264 -p headless:=true
   ```
   Confirm it's actually getting frames -- the log should show
   `allocated CUDA memory for ... image conversion` and
   `gstEncoder -- new caps: ...` shortly after starting. If it just sits at
   `"waiting for messages"`, no frames are reaching the encoder yet (check
   `ros2 topic hz /oak/rgb/image_raw_stamped` to isolate whether the stamper
   or the camera itself is the gap).

4. On the topside host (192.168.2.94):
   ```bash
   python3 src/oakd_isaac_ros/scripts/latency_measure.py 9999
   ```
   Prints per-frame and rolling 30-frame avg/max latency in ms. Ignore the
   first frame or two -- an initial keyframe/warm-up transient isn't
   representative of steady state.

## Gotchas we hit getting this working the first time

- **`cv_bridge`/`cv2` import errors mentioning NumPy 1.x vs 2.x**: the
  container's `numpy` needs to be `<2` for the compiled `cv_bridge`/`cv2`
  bindings to work (`pip install "numpy<2"`). Nothing in this workspace's
  own code imports `cupy` (which wants numpy>=2), so this downgrade is safe
  here -- but double check before baking it into `Dockerfile.oakd`.
- **`ModuleNotFoundError: No module named 'gi'` on the receiver**: if the
  host uses a conda/miniforge Python by default, `apt install python3-gi`
  only installs into the *system* Python. Run the receiver with
  `/usr/bin/python3` explicitly, not whatever `python3` resolves to.
- **`cv2.VideoCapture(..., cv2.CAP_GSTREAMER)` fails to open**: pip's
  `opencv-python` wheels aren't built with GStreamer support. That's why
  `latency_measure.py` talks to GStreamer directly via PyGObject instead of
  through OpenCV's video capture.
- **Sender's `video_output` log never gets past "waiting for messages"**:
  no frames are reaching it -- check `ros2 topic hz` on
  `/oak/rgb/image_raw` and `/oak/rgb/image_raw_stamped` to find where the
  chain is actually broken before suspecting the network.
- **`ros2 topic info <topic> --verbose` shows 1 publisher but your own test
  node isn't listed as a subscriber**, even though it printed its startup
  log with no errors: DDS discovery mismatch, not a code bug. Usually
  caused by the test node's shell having different `ROS_DOMAIN_ID` /
  `FASTRTPS_DEFAULT_PROFILES_FILE` / `ROS_LOCALHOST_ONLY` than the shell
  running the main pipeline. Use a fresh shell with the same environment,
  don't override discovery variables ad hoc.
- **`rtp://127.0.0.1:<port>` silently sends nowhere** when sender and
  receiver are on opposite sides of a container boundary: a container's own
  `127.0.0.1` is its own network namespace unless run with
  `--network host`. Use the actual destination IP instead (this is why the
  Jetson test above targets `192.168.2.94` directly rather than localhost).
