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

# Software decode (avdec_h264, from gstreamer1.0-libav) rather than
# decodebin: decodebin picks VA-API on some hosts, which silently never
# outputs frames. The jitter buffer + large socket buffer absorb bursty
# delivery over the tether; latency=200 trades ~0.2 s delay for fewer
# dropped frames. Kept free of spaces inside elements so it can also be
# split() into gst-launch-1.0 arguments.
RECEIVE_PIPELINE = (
    'udpsrc port={port} buffer-size=8388608 '
    'caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 '
    '! rtpjitterbuffer latency=200 ! rtph264depay ! h264parse ! queue '
    '! avdec_h264 max-threads=4 ! videoconvert'
)


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

    port = args.port if args.port is not None else port_for_stream(args.stream, args.config)

    print(f'Viewing "{args.stream}" on port {port} (Ctrl-C to stop)')
    try:
        run_with_fps_overlay(port)
    except ImportError:
        print('python3-gi / GStreamer introspection not found -- running without '
              'the FPS overlay (install with: sudo apt install python3-gi '
              'gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0)')
        run_gst_launch(port)


def run_with_fps_overlay(port):
    """Run the pipeline in-process so the measured FPS can be drawn top-right."""
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GLib', '2.0')
    from gi.repository import Gst, GLib

    Gst.init(None)
    # fpsdisplaysink's built-in overlay can't be repositioned (it's created
    # late, after properties are applied), so measure with it but draw the
    # number with our own textoverlay.
    pipeline = Gst.parse_launch(
        f'{RECEIVE_PIPELINE.format(port=port)} '
        '! textoverlay name=fps_text text="-- FPS" halignment=right valignment=top '
        'font-desc="Sans Bold 20" shaded-background=true '
        '! fpsdisplaysink name=fps_sink video-sink=autovideosink '
        'text-overlay=false signal-fps-measurements=true sync=false'
    )
    fps_text = pipeline.get_by_name('fps_text')
    pipeline.get_by_name('fps_sink').connect(
        'fps-measurements',
        lambda _sink, fps, _droprate, _avgfps: fps_text.set_property('text', f'{fps:.1f} FPS'),
    )

    loop = GLib.MainLoop()

    def on_message(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f'ERROR: {err.message}\n{dbg}')
            loop.quit()
        elif msg.type == Gst.MessageType.EOS:
            loop.quit()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect('message', on_message)

    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)


def run_gst_launch(port):
    if shutil.which('gst-launch-1.0') is None:
        print('gst-launch-1.0 not found on this machine.')
        sys.exit(1)
    cmd = (['gst-launch-1.0'] + RECEIVE_PIPELINE.format(port=port).split()
           + ['!', 'autovideosink', 'sync=false'])
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
