"""Air side (Radxa): camera H.265 passed through untouched, plus one KLV record per frame, muxed
into MPEG-TS and sent down the link.

    camera source ! h265parse --probe--> mpegtsmux ! sink
                                 |           ^
                                 +-> appsrc (KLV, same PTS)

The probe runs on every access unit leaving h265parse. It takes the arrival time, subtracts the
calibrated camera latency to estimate exposure time, interpolates telemetry to that time and
pushes the record into the mux with the frame's PTS. Nothing here decodes video.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time

import gi
import yaml

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

from ..meta import UNUSABLE, FrameMeta, pack  # noqa: E402
from .telemetry import Telemetry  # noqa: E402

log = logging.getLogger("bv_link.air")

# config-interval=-1 repeats VPS/SPS/PPS before every keyframe, so the ground can join mid-stream.
# alignment=7 packs 7 TS packets (1316 bytes) per UDP datagram, under a 1500-byte MTU.
PIPELINE = (
    "{source} ! h265parse name=parse config-interval=-1 "
    "! video/x-h265,stream-format=byte-stream,alignment=au ! queue ! mux. "
    "appsrc name=klv is-live=true format=time do-timestamp=false caps=meta/x-klv,parsed=true "
    "! queue ! mux. "
    "mpegtsmux name=mux alignment=7 ! {sink}"
)


class Sender:
    def __init__(self, source: str, sink: str, camera_latency_s: float, telemetry: Telemetry):
        self.telemetry = telemetry
        self.camera_latency_s = camera_latency_s
        self.pipeline = Gst.parse_launch(PIPELINE.format(source=source, sink=sink))
        self.klv = self.pipeline.get_by_name("klv")
        parse_src = self.pipeline.get_by_name("parse").get_static_pad("src")
        parse_src.add_probe(Gst.PadProbeType.BUFFER, self._on_frame)
        # Offset from monotonic to Unix time, fixed at start so a clock step mid-flight
        # (NTP/chrony correction) cannot make exposure times jump.
        self.monotonic_to_unix = time.time() - time.monotonic()
        self.frames = 0
        self.unusable = 0

    def _on_frame(self, pad: Gst.Pad, info: Gst.PadProbeInfo) -> Gst.PadProbeReturn:
        arrived = time.monotonic()
        buf = info.get_buffer()
        if buf.pts == Gst.CLOCK_TIME_NONE:
            return Gst.PadProbeReturn.OK
        exposure = arrived - self.camera_latency_s
        s = self.telemetry.state_at(exposure)
        record = FrameMeta(
            frame_seq=self.frames & 0xFFFFFFFF,
            t_exposure_us=round((exposure + self.monotonic_to_unix) * 1e6),
            lat_deg=s.lat_deg, lon_deg=s.lon_deg, alt_msl_m=s.alt_msl_m, alt_rel_m=s.alt_rel_m,
            q_vehicle=s.q_vehicle, q_gimbal=s.q_gimbal, gimbal_flags=s.gimbal_flags, status=s.status,
        )
        klv = Gst.Buffer.new_wrapped(pack(record))
        klv.pts = buf.pts
        klv.duration = buf.duration
        self.klv.emit("push-buffer", klv)
        self.frames += 1
        if s.status & UNUSABLE:
            self.unusable += 1
        return Gst.PadProbeReturn.OK

    def run(self, stats_period_s: int) -> int:
        loop = GLib.MainLoop()
        exit_code = 0

        def on_message(_bus: Gst.Bus, message: Gst.Message) -> None:
            nonlocal exit_code
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                log.error("pipeline error: %s (%s)", error.message, debug)
                exit_code = 1
                loop.quit()
            elif message.type == Gst.MessageType.EOS:
                log.info("end of stream")
                loop.quit()

        def log_stats() -> bool:
            log.info("frames=%d unusable_meta=%d telemetry=%s", self.frames, self.unusable,
                     self.telemetry.stats())
            return True

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_message)
        GLib.timeout_add_seconds(stats_period_s, log_stats)
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, lambda: loop.quit() or False)
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, lambda: loop.quit() or False)

        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            loop.run()
        finally:
            self.pipeline.set_state(Gst.State.NULL)
        return exit_code


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="air YAML config")
    args = parser.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    Gst.init(None)

    mav = cfg["mavlink"]
    telemetry = Telemetry(clock_window_s=mav["clock_window_s"], max_hold_s=mav["max_hold_s"])
    stop = threading.Event()
    threading.Thread(target=telemetry.run, name="mavlink", daemon=True,
                     args=(mav["url"], mav["baud"], mav["rates_hz"], mav["source_system"],
                           mav["source_component"], stop)).start()

    sender = Sender(cfg["camera"]["source"], cfg["sink"], cfg["camera"]["latency_ms"] / 1000.0, telemetry)
    exit_code = sender.run(cfg["stats_period_s"])
    stop.set()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
