#!/usr/bin/env python3
"""Manual human-click tracking override for the visual-servo pipeline.

Displays the mask_overlay RTP stream (same H264 feed view_stream.py views --
low-bandwidth, built for exactly this kind of constrained field link, unlike
subscribing to the raw ROS2 Image topic directly, which doesn't hold up over
a real-world link) and, while you hold the left mouse button down (and
optionally drag), continuously publishes the cursor's pixel position as the
target centroid -- overriding the real object detector entirely until you
let go. See visual_servo_controller_node_implementation.py's
_human_override_cb, and the guard at the top of _detection_cb that makes
real detections stand down while an override stays fresh.

The override point itself is a tiny, infrequent ROS2 publish (a few
PointStamped messages a second) -- that's negligible bandwidth and fine over
DDS even on a constrained link; only the video feed needs the RTP/H264
treatment. /mask_overlay is drawn in the 640x640 YOLO input frame, which
is also the pixel frame the controller/KF use, but the RTP node may resize
it (rtp.mask_overlay.width/height in system_params.yaml), so clicks are
rescaled from the decoded frame's size to --detection-width/height before
publishing.

The controller's centroid KF estimate (the ImageMarker it publishes on
centroid_kf_marker_topic, only while centroid_kf_enable is true) is drawn on
top of the stream, scaled back into the displayed frame, so you can see the
filter follow your clicks. It's hidden once no marker has arrived for
--kf-marker-timeout seconds.

Needs: gstreamer with an H264 decoder, and OpenCV built with GStreamer
support (`python3 -c "import cv2; print('GStreamer' in cv2.getBuildInformation())"`).
Still needs ROS network/discovery reachability to the robot to publish the
override point (same FASTRTPS discovery-server setup as everything else in
this stack; `unset FASTRTPS_DEFAULT_PROFILES_FILE` for local/dev use).

Usage:
    python3 manual_tracking_viewer.py
    python3 manual_tracking_viewer.py --port 8552   # skip config entirely
    python3 manual_tracking_viewer.py --override-topic /sc2/visual_servo/human_override_point
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

# In the Isaac ROS dev container, apt's python3-opencv (which has GStreamer
# support -- confirmed via cv2.getBuildInformation()) is ABI-compiled against
# apt's numpy 1.x, but a pip-installed numpy 2.x earlier on sys.path shadows
# it and breaks cv2's import entirely ("_ARRAY_API not found"). Prioritizing
# the apt dist-packages dir fixes it without touching anything system-wide --
# this only affects this process, not the main controller node (a separate
# process, with its own numpy usage unaffected by this).
_APT_DIST_PACKAGES = '/usr/lib/python3/dist-packages'
if os.path.isdir(_APT_DIST_PACKAGES):
    # Already present further down sys.path (hence pip's numpy winning the
    # resolution) -- re-inserting at the front, not skipping when merely
    # present, is what actually fixes the priority. A duplicate entry is
    # harmless.
    sys.path.insert(0, _APT_DIST_PACKAGES)

import cv2
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import ImageMarker

from view_stream import DEFAULT_CONFIG, port_for_stream

WINDOW_NAME = 'manual_tracking_viewer (hold+drag left click to override)'
STREAM = 'mask_overlay'


def _build_capture(port: int) -> cv2.VideoCapture:
    caps = (
        'application/x-rtp, media=(string)video, clock-rate=(int)90000, '
        'encoding-name=(string)H264, payload=(int)96'
    )
    # appsink (not autovideosink, what view_stream.py uses) so OpenCV
    # actually gets frame buffers to display/click on, not just a GStreamer-
    # owned window. drop=true + max-buffers=1 + sync=false: always show the
    # latest frame rather than buffering/catching up, which would add
    # click-to-display lag on a real control feed (drop does nothing without
    # a max-buffers limit). avdec_h264 rather than decodebin, which picks
    # VA-API on some hosts -- same reason as view_stream.py.
    pipeline = (
        f'udpsrc port={port} caps="{caps}" ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
        'video/x-raw,format=BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(
            'Failed to open the GStreamer capture pipeline -- check gstreamer/OpenCV '
            'GStreamer support, and that the port is correct.',
            file=sys.stderr,
        )
        sys.exit(1)
    return cap


class ManualOverridePublisher(Node):
    def __init__(self, override_topic: str, kf_marker_topic: str):
        super().__init__('manual_tracking_viewer')
        self._pub = self.create_publisher(PointStamped, override_topic, 10)
        self.get_logger().info(f'Publishing override on {override_topic} while held.')

        # Latest KF centroid marker and its local receive time (monotonic,
        # so staleness doesn't depend on robot/topside clock sync).
        self.kf_marker = None
        self.kf_marker_rx_time = None
        self.create_subscription(ImageMarker, kf_marker_topic, self._kf_marker_cb, 10)
        self.get_logger().info(f'Drawing KF estimate from {kf_marker_topic}.')

    def _kf_marker_cb(self, msg: ImageMarker) -> None:
        if msg.action == ImageMarker.REMOVE:
            self.kf_marker = None
            return
        self.kf_marker = msg
        self.kf_marker_rx_time = time.monotonic()

    def publish(self, x: float, y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        self._pub.publish(msg)


def _draw_kf_marker(frame, node, scale_x, scale_y, timeout_sec) -> None:
    """Draw the controller's KF centroid estimate, mapped from the detection
    frame back into the displayed frame (inverse of the click rescale)."""
    marker = node.kf_marker
    if marker is None or time.monotonic() - node.kf_marker_rx_time > timeout_sec:
        return
    # A diverged filter publishes NaN/inf -- nothing meaningful to draw.
    if not all(math.isfinite(v) for v in (marker.position.x, marker.position.y, marker.scale)):
        return
    center = (int(round(marker.position.x / scale_x)), int(round(marker.position.y / scale_y)))
    radius = max(2, int(round(marker.scale / min(scale_x, scale_y))))
    c = marker.outline_color
    # ImageMarker colors are RGBA floats; OpenCV wants BGR 0-255. Fall back
    # to the controller's orange if the color is unset (all zeros).
    color = (int(c.b * 255), int(c.g * 255), int(c.r * 255)) if (c.r or c.g or c.b) else (0, 140, 255)
    cv2.circle(frame, center, radius, color, thickness=2)
    cv2.putText(frame, 'KF', (center[0] + radius + 3, center[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                         help=f'Path to system_params.yaml (default: {DEFAULT_CONFIG})')
    parser.add_argument('--port', type=int, default=None,
                         help='Override the mask_overlay RTP port instead of reading it from config')
    parser.add_argument('--override-topic', default='/sc2/visual_servo/human_override_point')
    # Must match the controller's image_width/image_height
    # (object_detection_ibvs.yaml) -- the YOLO input frame detections are in.
    parser.add_argument('--detection-width', type=float, default=640.0)
    parser.add_argument('--detection-height', type=float, default=640.0)
    parser.add_argument('--kf-marker-topic', default='/sc2/visual_servo/centroid_kf_marker')
    # The controller publishes the marker every tick (10 Hz), so this is a
    # few missed ticks -- long enough to ride out jitter, short enough that a
    # stopped controller/KF doesn't leave a frozen circle on screen.
    parser.add_argument('--kf-marker-timeout', type=float, default=0.5,
                         help='Hide the KF marker after this many seconds without an update')
    args = parser.parse_args()

    port = args.port if args.port is not None else port_for_stream(STREAM, args.config)
    cap = _build_capture(port)

    rclpy.init()
    node = ManualOverridePublisher(args.override_topic, args.kf_marker_topic)

    mouse_down = False
    mouse_xy = (0, 0)

    def on_mouse(event, x, y, flags, _param):
        nonlocal mouse_down, mouse_xy
        if event == cv2.EVENT_LBUTTONDOWN:
            mouse_down = True
            mouse_xy = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and mouse_down:
            mouse_xy = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            mouse_down = False

    # WINDOW_AUTOSIZE (not WINDOW_NORMAL) keeps the window fixed at the
    # decoded frame's native size -- WINDOW_NORMAL lets the OS stretch it on
    # resize, which would silently scale click coordinates away from true
    # pixel coordinates.
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    print(f'Viewing {STREAM} on port {port}. Hold left click and drag over the '
          'target; release to hand control back to the object detector. q/Ctrl-C to quit.')

    frame_size = None
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue

            frame_h, frame_w = frame.shape[:2]
            if frame_size != (frame_w, frame_h):
                frame_size = (frame_w, frame_h)
                scale_x = args.detection_width / frame_w
                scale_y = args.detection_height / frame_h
                print(f'Stream is {frame_w}x{frame_h}; clicks are rescaled to the '
                      f'{args.detection_width:.0f}x{args.detection_height:.0f} detection frame.')
                if abs(scale_x - scale_y) > 1e-3:
                    print('Warning: stream aspect ratio differs from the detection '
                          'frame -- the image is stretched; set rtp.mask_overlay.width/'
                          'height to the detection size for an undistorted view.')

            # Each displayed frame is also the override-publish tick while
            # held -- tied to the stream's own delivery rate rather than a
            # separate timer, and still comfortably faster than the
            # controller's human_override_timeout_ms (default 200ms).
            _draw_kf_marker(frame, node, scale_x, scale_y, args.kf_marker_timeout)

            if mouse_down:
                node.publish(mouse_xy[0] * scale_x, mouse_xy[1] * scale_y)
                cv2.drawMarker(
                    frame, mouse_xy, (0, 140, 255),
                    markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2,
                )

            cv2.imshow(WINDOW_NAME, frame)
            rclpy.spin_once(node, timeout_sec=0.0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        # rclpy's own SIGINT handler may already have shut the context down.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
