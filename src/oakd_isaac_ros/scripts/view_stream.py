#!/usr/bin/env python3
"""Host-side RTP stream viewer.

Wraps the gst-launch-1.0 receive pipeline we've been typing by hand all
session, picking the right port automatically from config/system_params.yaml
so you don't have to remember/retype it per stream. Runs entirely on the
receiving (topside) machine -- no ROS needed.

Usage:
    python3 view_stream.py rgb
    python3 view_stream.py mask_overlay
    python3 view_stream.py depth
    python3 view_stream.py rgb --config /path/to/system_params.yaml
    python3 view_stream.py rgb --port 8550   # skip config entirely
"""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print('PyYAML not found -- install it with: sudo apt install python3-yaml')
    sys.exit(1)

STREAMS = ('rgb', 'mask_overlay', 'depth')

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'system_params.yaml'


def port_for_stream(stream, config_path):
    if not config_path.exists():
        print(f'Config file not found: {config_path}')
        print('Pass --port explicitly if this host doesn\'t have the repo checked out.')
        sys.exit(1)

    with open(config_path) as f:
        params = yaml.safe_load(f)

    rtp_cfg = params.get('rtp', {})
    stream_cfg = rtp_cfg.get(stream)
    if stream_cfg is None or 'port' not in stream_cfg:
        print(f'No rtp.{stream}.port entry found in {config_path}')
        sys.exit(1)

    if not stream_cfg.get('enabled', True):
        print(f'Warning: rtp.{stream}.enabled is false in {config_path} -- '
              f'the Jetson probably isn\'t sending this stream right now.')

    return stream_cfg['port']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stream', choices=STREAMS, help='Which stream to view')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                         help=f'Path to system_params.yaml (default: {DEFAULT_CONFIG})')
    parser.add_argument('--port', type=int, default=None,
                         help='Override the port instead of reading it from config')
    args = parser.parse_args()

    if shutil.which('gst-launch-1.0') is None:
        print('gst-launch-1.0 not found on this machine.')
        sys.exit(1)

    port = args.port if args.port is not None else port_for_stream(args.stream, args.config)

    caps = (
        'application/x-rtp, media=(string)video, clock-rate=(int)90000, '
        'encoding-name=(string)H264, payload=(int)96'
    )
    # Software decode (avdec_h264, from gstreamer1.0-libav) rather than
    # decodebin: decodebin picks VA-API on some hosts, which silently never
    # outputs frames. The jitter buffer + large socket buffer absorb bursty
    # delivery over the tether; latency=200 trades ~0.2 s delay for fewer
    # dropped frames.
    cmd = [
        'gst-launch-1.0',
        'udpsrc', f'port={port}', 'buffer-size=8388608', f'caps={caps}',
        '!', 'rtpjitterbuffer', 'latency=200',
        '!', 'rtph264depay',
        '!', 'h264parse',
        '!', 'queue',
        '!', 'avdec_h264', 'max-threads=4',
        '!', 'videoconvert',
        '!', 'autovideosink', 'sync=false',
    ]

    print(f'Viewing "{args.stream}" on port {port} (Ctrl-C to stop)')
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
