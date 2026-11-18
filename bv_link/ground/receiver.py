"""Ground side (Jetson): receive MPEG-TS, decode H.265 in hardware, pair each frame with its KLV
record by PTS.

    source ! tsdemux -+-> h265parse ! decoder ! appsink (video)  --pulled by Receiver.frames()
                      +-> appsink (KLV)  --callback stores records by PTS

tsdemux gives both streams PTS from the same TS clock, and the air side stamped each record with
its frame's PTS, so the pair shares one PTS exactly. KLV usually arrives first because the video
still has to decode; frames() waits up to meta_wait_s for a late record.
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from collections.abc import Iterator
from dataclasses import asdict, dataclass
from pathlib import Path

import gi
import numpy as np
import yaml

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import Gst, GstVideo  # noqa: E402

from ..meta import FrameMeta, unpack  # noqa: E402

log = logging.getLogger("bv_link.ground")

PTS_TOLERANCE_NS = 1_000_000  # decoders should keep PTS exact; allow 1 ms in case one rounds


@dataclass
class Frame:
    image: np.ndarray        # H x W x 3, BGR, uint8
    meta: FrameMeta | None   # None if no record arrived for this frame
    pts_ns: int
    received_unix_s: float


class Receiver:
    def __init__(self, source: str, decoder: str, meta_wait_s: float, drop_frames: bool,
                 max_pending: int = 600):
        self.decoder = decoder
        self.meta_wait_s = meta_wait_s
        self.drop_frames = drop_frames
        self.max_pending = max_pending
        self.pipeline = Gst.parse_launch(f"{source} ! tsdemux name=demux")
        self.pipeline.get_by_name("demux").connect("pad-added", self._on_pad_added)
        self._video_sink: Gst.Element | None = None
        self._pending: dict[int, FrameMeta] = {}
        self._cond = threading.Condition()
        self.stats = {"frames": 0, "matched": 0, "bad_klv": 0}

    def start(self) -> None:
        self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self) -> None:
        self.pipeline.set_state(Gst.State.NULL)

    def _on_pad_added(self, _demux: Gst.Element, pad: Gst.Pad) -> None:
        kind = pad.query_caps(None).get_structure(0).get_name()
        if kind == "video/x-h265":
            # With drop_frames, a consumer slower than the stream always gets the newest frame
            # instead of an ever-growing backlog.
            drop = "max-buffers=2 drop=true" if self.drop_frames else "max-buffers=30 drop=false"
            desc = f"queue ! h265parse ! {self.decoder} ! appsink name=video sync=false {drop}"
        elif kind == "meta/x-klv":
            desc = "queue ! appsink name=klv sync=false emit-signals=true"
        else:
            log.warning("ignoring stream %s", kind)
            return
        branch = Gst.parse_bin_from_description(desc, True)
        self.pipeline.add(branch)
        branch.sync_state_with_parent()
        pad.link(branch.get_static_pad("sink"))
        if kind == "meta/x-klv":
            branch.get_by_name("klv").connect("new-sample", self._on_klv)
        else:
            with self._cond:
                self._video_sink = branch.get_by_name("video")
                self._cond.notify_all()
        log.info("linked %s", kind)

    def _on_klv(self, sink: Gst.Element) -> Gst.FlowReturn:
        buf = sink.emit("pull-sample").get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        data = bytes(info.data)
        buf.unmap(info)
        try:
            record = unpack(data)
        except ValueError as error:
            self.stats["bad_klv"] += 1
            log.warning("bad KLV packet: %s", error)
            return Gst.FlowReturn.OK
        with self._cond:
            self._pending[buf.pts] = record
            while len(self._pending) > self.max_pending:
                del self._pending[next(iter(self._pending))]
            self._cond.notify_all()
        return Gst.FlowReturn.OK

    def frames(self, timeout_s: float = 1.0) -> Iterator[Frame]:
        """Yields decoded frames with their metadata. Blocks; stops when the pipeline ends."""
        with self._cond:
            while self._video_sink is None:
                self._cond.wait()
        sink = self._video_sink
        while True:
            sample = sink.emit("try-pull-sample", int(timeout_s * Gst.SECOND))
            if sample is None:
                if sink.get_property("eos"):
                    return
                continue
            received = time.time()
            buf = sample.get_buffer()
            image = _to_bgr(sample)
            record = self._take_meta(buf.pts)
            self.stats["frames"] += 1
            self.stats["matched"] += record is not None
            yield Frame(image=image, meta=record, pts_ns=buf.pts, received_unix_s=received)

    def _take_meta(self, pts: int) -> FrameMeta | None:
        deadline = time.monotonic() + self.meta_wait_s
        with self._cond:
            key = self._find(pts)
            while key is None and (remaining := deadline - time.monotonic()) > 0:
                self._cond.wait(remaining)
                key = self._find(pts)
            record = self._pending.pop(key) if key is not None else None
            # Anything older belongs to frames that were dropped or already handled.
            for stale in [k for k in self._pending if k < pts]:
                del self._pending[stale]
        return record

    def _find(self, pts: int) -> int | None:
        if pts in self._pending:
            return pts
        near = [k for k in self._pending if abs(k - pts) <= PTS_TOLERANCE_NS]
        return min(near, key=lambda k: abs(k - pts)) if near else None


def _to_bgr(sample: Gst.Sample) -> np.ndarray:
    """Copies a BGRx sample into a contiguous BGR array, respecting row stride padding."""
    caps = sample.get_caps()
    try:
        info = GstVideo.VideoInfo.new_from_caps(caps)
    except AttributeError:  # GStreamer < 1.20
        info = GstVideo.VideoInfo()
        info.from_caps(caps)
    height, width, stride = info.height, info.width, info.stride[0]
    buf = sample.get_buffer()
    ok, mapped = buf.map(Gst.MapFlags.READ)
    if not ok:
        raise RuntimeError("could not map video buffer")
    try:
        rows = np.frombuffer(mapped.data, dtype=np.uint8, count=height * stride).reshape(height, stride)
        return np.ascontiguousarray(rows[:, :width * 4].reshape(height, width, 4)[:, :, :3])
    finally:
        buf.unmap(mapped)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="ground YAML config")
    parser.add_argument("--record", type=Path, help="write meta.jsonl (every frame) and JPEGs here")
    parser.add_argument("--record-every", type=int, default=30, help="save every Nth frame as JPEG")
    args = parser.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    Gst.init(None)

    meta_log = None
    if args.record:
        import cv2
        args.record.mkdir(parents=True, exist_ok=True)
        meta_log = open(args.record / "meta.jsonl", "a")

    receiver = Receiver(cfg["source"], cfg["decoder"], cfg["meta_wait_ms"] / 1000.0, cfg["drop_frames"])
    receiver.start()
    last_stats = time.monotonic()
    try:
        for frame in receiver.frames():
            if meta_log is not None:
                meta_log.write(json.dumps({"pts_ns": frame.pts_ns, "received_unix_s": frame.received_unix_s,
                                           "meta": asdict(frame.meta) if frame.meta else None}) + "\n")
                if receiver.stats["frames"] % args.record_every == 0:
                    cv2.imwrite(str(args.record / f"{frame.pts_ns}.jpg"), frame.image)
            if time.monotonic() - last_stats >= cfg["stats_period_s"]:
                last_stats = time.monotonic()
                # Only meaningful if the air and ground clocks are synced (chrony/GPS).
                age_ms = ((frame.received_unix_s * 1e6 - frame.meta.t_exposure_us) / 1000.0
                          if frame.meta else None)
                log.info("stats=%s size=%dx%d exposure_to_ground_ms=%s status=%s", receiver.stats,
                         frame.image.shape[1], frame.image.shape[0],
                         None if age_ms is None else round(age_ms, 1),
                         None if frame.meta is None else frame.meta.status)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        if meta_log is not None:
            meta_log.close()


if __name__ == "__main__":
    main()
