#!/usr/bin/python3
"""Topside RTP stream diagnostics.

Receives one RTP/H264 stream exactly like view_stream.py, but prints once per
second what happens at each stage, so you can see where frames get lost:

  net pkt/s   RTP packets arriving at the socket
  lost        packets missing (gaps in RTP sequence numbers) -> network loss
  kern drop   UDP receive-buffer overflows in this host's kernel (all sockets)
  jb lost/late  packets the jitter buffer gave up on / received too late
  sent fps    distinct frames seen on the wire (RTP timestamps) ~= what the Orin sent
  asm fps     complete frames reassembled by the depayloader
  dec fps     frames the decoder produced
  shown fps   frames the display sink rendered (dropped = rendered too late)
  dec warn    decoder warnings (corrupt/missing reference -> grey/smeared frames)

Usage (system python, not conda):
    /usr/bin/python3 stream_diag.py rgb
    /usr/bin/python3 stream_diag.py mask_overlay
    /usr/bin/python3 stream_diag.py rgb --no-display   # rule out the display
    /usr/bin/python3 stream_diag.py rgb --port 8550 --latency 200

Stop the normal viewer first -- only one program can receive on a port.
Needs: python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 gstreamer1.0-libav
"""
import argparse
import sys
import time
from pathlib import Path

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from view_stream import DEFAULT_CONFIG, STREAMS, port_for_stream  # noqa: E402

CAPS = 'application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96'


def kernel_udp_rcvbuf_errors():
    try:
        lines = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')]
        return int(lines[1][lines[0].index('RcvbufErrors')])
    except (OSError, ValueError, IndexError):
        return None


class Stats:
    def __init__(self):
        self.pkts = 0
        self.lost = 0
        self.last_seq = None
        self.timestamps = set()
        self.assembled = 0
        self.decoded = 0
        self.dec_warnings = 0
        self.total_pkts = 0
        self.total_lost = 0

    def on_rtp(self, data):
        seq = (data[2] << 8) | data[3]
        self.pkts += 1
        self.total_pkts += 1
        self.timestamps.add(data[4:8])
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if gap < 0x8000:  # ignore reordered/duplicate packets
                self.lost += gap
                self.total_lost += gap
        self.last_seq = seq


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('stream', choices=STREAMS)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--latency', type=int, default=200, help='jitter buffer latency in ms')
    parser.add_argument('--no-display', action='store_true',
                        help='decode but do not display (fakesink) to rule out the display')
    parser.add_argument('--duration', type=int, default=0,
                        help='stop after this many seconds (0 = run until Ctrl-C)')
    args = parser.parse_args()

    port = args.port if args.port is not None else port_for_stream(args.stream, args.config)
    sink = 'fakesink' if args.no_display else 'autovideosink'

    Gst.init(None)
    pipeline = Gst.parse_launch(
        f'udpsrc name=src port={port} buffer-size=8388608 caps={CAPS} '
        f'! rtpjitterbuffer name=jb latency={args.latency} '
        '! rtph264depay name=depay ! h264parse ! queue '
        '! avdec_h264 name=dec max-threads=4 ! videoconvert '
        f'! fpsdisplaysink name=sink video-sink={sink} text-overlay=false sync=false'
    )
    st = Stats()

    def rtp_probe(pad, info):
        buf = info.get_buffer()
        if buf.get_size() >= 8:
            st.on_rtp(buf.extract_dup(0, 8))
        return Gst.PadProbeReturn.OK

    def count(attr):
        def probe(pad, info):
            setattr(st, attr, getattr(st, attr) + 1)
            return Gst.PadProbeReturn.OK
        return probe

    pipeline.get_by_name('src').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, rtp_probe)
    pipeline.get_by_name('depay').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, count('assembled'))
    pipeline.get_by_name('dec').get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, count('decoded'))

    jb = pipeline.get_by_name('jb')
    fps_sink = pipeline.get_by_name('sink')
    loop = GLib.MainLoop()

    def on_message(_bus, msg):
        if msg.type == Gst.MessageType.WARNING and msg.src.get_name() == 'dec':
            st.dec_warnings += 1
        elif msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f'ERROR: {err.message}\n{dbg}')
            loop.quit()

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect('message', on_message)

    prev = {'t': time.time(), 'kern': kernel_udp_rcvbuf_errors(),
            'rendered': 0, 'dropped': 0, 'jb_lost': 0, 'jb_late': 0}
    header = ('  time | net pkt/s  lost  kern drop | jb lost  late | sent fps  asm fps  '
              'dec fps  shown fps  dropped | dec warn')
    print(f'Receiving "{args.stream}" on port {port} (display: {"off" if args.no_display else "on"}). '
          'Ctrl-C to stop.\n' + header)

    def report():
        now = time.time()
        dt = max(now - prev['t'], 1e-3)
        kern = kernel_udp_rcvbuf_errors()
        kern_d = '   n/a' if kern is None or prev['kern'] is None else f'{kern - prev["kern"]:6d}'
        jbs = jb.get_property('stats')
        jb_lost = jbs.get_value('num-lost') if jbs else 0
        jb_late = jbs.get_value('num-late') if jbs else 0
        rendered = fps_sink.get_property('frames-rendered')
        dropped = fps_sink.get_property('frames-dropped')
        print(f'{time.strftime("%H:%M:%S")} | {st.pkts / dt:9.0f} {st.lost:5d}     {kern_d} | '
              f'{jb_lost - prev["jb_lost"]:7d} {jb_late - prev["jb_late"]:5d} | '
              f'{len(st.timestamps) / dt:8.1f} {st.assembled / dt:8.1f} {st.decoded / dt:8.1f} '
              f'{(rendered - prev["rendered"]) / dt:10.1f} {dropped - prev["dropped"]:8d} | '
              f'{st.dec_warnings:8d}')
        prev.update(t=now, kern=kern, rendered=rendered, dropped=dropped,
                    jb_lost=jb_lost, jb_late=jb_late)
        st.pkts = st.lost = st.assembled = st.decoded = st.dec_warnings = 0
        st.timestamps.clear()
        return True

    GLib.timeout_add(1000, report)
    if args.duration > 0:
        GLib.timeout_add(args.duration * 1000 + 100, loop.quit)
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        if st.total_pkts:
            pct = 100.0 * st.total_lost / (st.total_pkts + st.total_lost)
            print(f'\nTotal: {st.total_pkts} packets received, {st.total_lost} lost ({pct:.2f}%)')


if __name__ == '__main__':
    main()
