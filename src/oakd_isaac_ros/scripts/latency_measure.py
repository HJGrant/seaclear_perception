#!/usr/bin/env python3
"""Reads the timestamp block scripts/latency_stamp.py burns into each RGB
frame and reports glass-to-glass latency: (this machine's clock right now)
minus (decoded capture time). Talks to GStreamer directly via PyGObject
(the same GStreamer install `gst-launch-1.0` uses) rather than through
OpenCV's VideoCapture -- pip's opencv-python wheels are built without
GStreamer support, so cv2.VideoCapture(..., cv2.CAP_GSTREAMER) fails on a
plain `pip install opencv-python` even though the system's own GStreamer
works fine. No ROS needed, so this can run on a topside/receiver machine
that isn't running ROS at all.

Only meaningful if sender and receiver share a clock. Automatically true
for localhost testing (127.0.0.1). Over the real tether with the Jetson as
sender, the two machines need synchronized clocks (chrony/NTP) or the
number reported here is meaningless -- it'll just be however far apart
their clocks are, plus or minus the real latency.

Requires PyGObject + the GStreamer app plugin (already present on most
systems that have gst-launch-1.0 working -- if not:
`sudo apt install python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0`).

Usage: python3 latency_measure.py [port]
"""
import sys
import time

import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

NUM_BITS = 44
BLOCK_SIZE = 16
ORIGIN = (10, 10)


def decode_timestamp(frame):
    x0, y0 = ORIGIN
    value = 0
    for bit_index in range(NUM_BITS):
        x = x0 + bit_index * BLOCK_SIZE
        block = frame[y0:y0 + BLOCK_SIZE, x:x + BLOCK_SIZE]
        bit = 1 if block.mean() > 128 else 0
        value = (value << 1) | bit
    return value


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8550
    Gst.init(None)

    pipeline_str = (
        f'udpsrc port={port} caps="application/x-rtp, media=(string)video, '
        f'clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" '
        f'! rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=BGR '
        f'! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1'
    )
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name('sink')

    latencies = []

    def on_new_sample(sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value('width')
        height = structure.get_value('height')

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape(height, width, 3)
            now_ms = int(time.time() * 1000)
            ts_ms = decode_timestamp(frame)
            latency_ms = now_ms - ts_ms
            latencies.append(latency_ms)
            if len(latencies) % 30 == 0:
                recent = latencies[-30:]
                print(
                    f'latency: {latency_ms:5d} ms   '
                    f'(30-frame avg {sum(recent) / len(recent):6.1f} ms, '
                    f'max {max(recent):5d} ms)'
                )
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    appsink.connect('new-sample', on_new_sample)

    pipeline.set_state(Gst.State.PLAYING)
    print(f'Listening on port {port}... (Ctrl-C to stop)')

    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == '__main__':
    main()
