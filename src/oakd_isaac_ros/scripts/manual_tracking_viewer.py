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
treatment. mask_overlay's RTP stream carries native resolution (confirmed
640x640, no resize in the encode pipeline), so clicks on the decoded frame
land on the same pixel coordinates the detector/KF use -- no rescaling
needed.

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
import os
import sys
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
    # owned window. drop=true + sync=false: always show the latest frame
    # rather than buffering/catching up, which would add click-to-display
    # lag on a real control feed.
    pipeline = (
        f'udpsrc port={port} caps="{caps}" ! '
        'rtph264depay ! decodebin ! videoconvert ! '
        'video/x-raw,format=BGR ! '
        'appsink drop=true sync=false'
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
    def __init__(self, override_topic: str):
        super().__init__('manual_tracking_viewer')
        self._pub = self.create_publisher(PointStamped, override_topic, 10)
        self.get_logger().info(f'Publishing override on {override_topic} while held.')

    def publish(self, x: float, y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                         help=f'Path to system_params.yaml (default: {DEFAULT_CONFIG})')
    parser.add_argument('--port', type=int, default=None,
                         help='Override the mask_overlay RTP port instead of reading it from config')
    parser.add_argument('--override-topic', default='/sc2/visual_servo/human_override_point')
    args = parser.parse_args()

    port = args.port if args.port is not None else port_for_stream(STREAM, args.config)
    cap = _build_capture(port)

    rclpy.init()
    node = ManualOverridePublisher(args.override_topic)

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

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue

            # Each displayed frame is also the override-publish tick while
            # held -- tied to the stream's own delivery rate rather than a
            # separate timer, and still comfortably faster than the
            # controller's human_override_timeout_ms (default 200ms).
            if mouse_down:
                node.publish(mouse_xy[0], mouse_xy[1])
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
