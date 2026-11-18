"""MAVLink telemetry, timestamped on the local clock and interpolated to any past time.

Every sample is stamped with the sender's own clock (time_boot_ms), mapped to local monotonic
time through a per-component ClockMap. The flight controller and the gimbal boot at different
times, so each (system, component) gets its own map.
"""

from __future__ import annotations

import bisect
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

from .. import meta
from .clock import ClockMap

log = logging.getLogger(__name__)

Vec = tuple[float, ...]
IDENTITY: meta.Quat = (1.0, 0.0, 0.0, 0.0)

MSG_ATTITUDE_QUATERNION = 31
MSG_GLOBAL_POSITION_INT = 33
MSG_GIMBAL_DEVICE_ATTITUDE_STATUS = 285
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_AUTOPILOT_INVALID = 8


def lerp(a: Vec, b: Vec, f: float) -> Vec:
    return tuple(x + f * (y - x) for x, y in zip(a, b))


def slerp(q0: Vec, q1: Vec, f: float) -> Vec:
    """Spherical interpolation between unit quaternions (w, x, y, z).

    q and -q are the same rotation. If the dot product is negative, flipping q1 makes the
    interpolation take the short way round instead of spinning almost 360 degrees. When the two
    are nearly equal, sin(theta) is close to zero, so normalized lerp is used instead (the error
    is negligible at that separation).
    """
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-x for x in q1)
        dot = -dot
    if dot > 0.9995:
        q = lerp(q0, q1, f)
        norm = math.sqrt(sum(x * x for x in q))
        return tuple(x / norm for x in q)
    theta = math.acos(dot)
    s = math.sin(theta)
    w0 = math.sin((1.0 - f) * theta) / s
    w1 = math.sin(f * theta) / s
    return tuple(w0 * a + w1 * b for a, b in zip(q0, q1))


class Series:
    """Time-ordered samples on the local clock, interpolated with the given function."""

    def __init__(self, interp: Callable[[Vec, Vec, float], Vec], keep: int = 1000):
        self.interp = interp
        self.keep = keep
        self._t: list[float] = []
        self._v: list[Vec] = []

    def append(self, t: float, value: Vec) -> None:
        # The clock offset estimate can step slightly, which can map a new sample to just before
        # the previous one. Dropping it keeps the series sorted for bisect.
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._v.append(value)
        if len(self._t) > 2 * self.keep:
            del self._t[:-self.keep]
            del self._v[:-self.keep]

    def sample(self, t: float, max_hold_s: float) -> tuple[Vec | None, bool]:
        """Returns (value, held). value is None if no sample lies within max_hold_s of t."""
        if not self._t:
            return None, False
        i = bisect.bisect_left(self._t, t)
        if i < len(self._t) and self._t[i] == t:
            return self._v[i], False
        if 0 < i < len(self._t):
            t0, t1 = self._t[i - 1], self._t[i]
            return self.interp(self._v[i - 1], self._v[i], (t - t0) / (t1 - t0)), False
        nearest = 0 if i == 0 else -1
        if abs(t - self._t[nearest]) > max_hold_s:
            return None, False
        return self._v[nearest], True

    def rate_hz(self, window_s: float = 2.0) -> float:
        if len(self._t) < 2:
            return 0.0
        start = bisect.bisect_left(self._t, self._t[-1] - window_s)
        span = self._t[-1] - self._t[start]
        return (len(self._t) - 1 - start) / span if span > 0 else 0.0


@dataclass(frozen=True)
class FrameState:
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    alt_rel_m: float
    q_vehicle: meta.Quat
    q_gimbal: meta.Quat
    gimbal_flags: int
    status: int


class Telemetry:
    def __init__(self, clock_window_s: float = 10.0, max_hold_s: float = 0.2):
        self.clock_window_s = clock_window_s
        self.max_hold_s = max_hold_s
        self.position = Series(lerp)    # lat_deg, lon_deg, alt_msl_m, alt_rel_m
        self.attitude = Series(slerp)   # vehicle quaternion
        self.gimbal = Series(slerp)     # gimbal quaternion
        self.gimbal_flags = 0
        self.target_system: int | None = None
        self._clocks: dict[tuple[int, int], ClockMap] = {}
        self._lock = threading.Lock()

    def ingest(self, msg, local_s: float) -> None:
        """Buffers one MAVLink message received at local_s (time.monotonic())."""
        kind = msg.get_type()
        if kind not in ("ATTITUDE_QUATERNION", "GLOBAL_POSITION_INT", "GIMBAL_DEVICE_ATTITUDE_STATUS"):
            return
        source = (msg.get_srcSystem(), msg.get_srcComponent())
        if self.target_system is not None and source[0] != self.target_system:
            return
        remote_s = msg.time_boot_ms / 1000.0
        with self._lock:
            clock = self._clocks.get(source)
            if clock is None:
                clock = self._clocks[source] = ClockMap(self.clock_window_s)
            clock.update(remote_s, local_s)
            t = clock.to_local(remote_s)
            if kind == "ATTITUDE_QUATERNION":
                self.attitude.append(t, (msg.q1, msg.q2, msg.q3, msg.q4))
            elif kind == "GLOBAL_POSITION_INT":
                self.position.append(t, (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3,
                                         msg.relative_alt * 1e-3))
            else:
                self.gimbal.append(t, tuple(msg.q))
                self.gimbal_flags = msg.flags

    def state_at(self, t: float) -> FrameState:
        """Vehicle and gimbal state at local monotonic time t."""
        with self._lock:
            position, position_held = self.position.sample(t, self.max_hold_s)
            attitude, attitude_held = self.attitude.sample(t, self.max_hold_s)
            gimbal, gimbal_held = self.gimbal.sample(t, self.max_hold_s)
            gimbal_flags = self.gimbal_flags

        status = 0
        status |= meta.POSITION_HELD if position_held else 0
        status |= meta.ATTITUDE_HELD if attitude_held else 0
        status |= meta.GIMBAL_HELD if gimbal_held else 0
        if position is None:
            status |= meta.NO_POSITION
            position = (0.0, 0.0, 0.0, 0.0)
        if attitude is None:
            status |= meta.NO_ATTITUDE
            attitude = IDENTITY
        if gimbal is None:
            status |= meta.NO_GIMBAL
            gimbal = IDENTITY
        return FrameState(*position, q_vehicle=tuple(attitude), q_gimbal=tuple(gimbal),
                          gimbal_flags=gimbal_flags, status=status)

    def stats(self) -> dict:
        with self._lock:
            return {
                "position_hz": round(self.position.rate_hz(), 1),
                "attitude_hz": round(self.attitude.rate_hz(), 1),
                "gimbal_hz": round(self.gimbal.rate_hz(), 1),
                "clock_offsets_s": {f"{s}/{c}": round(m.offset_s, 4)
                                    for (s, c), m in self._clocks.items() if m.ready},
            }

    def run(self, url: str, baud: int, rates_hz: dict[str, float], source_system: int,
            source_component: int, stop: threading.Event) -> None:
        """Reads MAVLink until stop is set, reconnecting on errors. Run it in its own thread."""
        from pymavlink import mavutil

        while not stop.is_set():
            try:
                conn = mavutil.mavlink_connection(url, baud=baud, source_system=source_system,
                                                  source_component=source_component)
                self._wait_for_autopilot(conn, stop)
                if stop.is_set():
                    return
                self._request_rates(conn, rates_hz)
                while not stop.is_set():
                    msg = conn.recv_match(blocking=True, timeout=1.0)
                    if msg is not None:
                        self.ingest(msg, time.monotonic())
            except Exception:
                log.exception("MAVLink connection on %s failed; retrying in 1 s", url)
                stop.wait(1.0)

    def _wait_for_autopilot(self, conn, stop: threading.Event) -> None:
        log.info("waiting for an autopilot heartbeat")
        while not stop.is_set():
            hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            # Gimbals and cameras heartbeat too; only the autopilot reports a real autopilot type.
            if hb is not None and hb.autopilot != MAV_AUTOPILOT_INVALID:
                conn.target_system, conn.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                self.target_system = conn.target_system
                log.info("autopilot is system %d component %d", conn.target_system, conn.target_component)
                return

    @staticmethod
    def _request_rates(conn, rates_hz: dict[str, float]) -> None:
        # Gimbal status comes from the gimbal device or PX4's gimbal manager, so that request goes
        # to every component (0) rather than the autopilot alone.
        requests = [(MSG_ATTITUDE_QUATERNION, rates_hz["attitude"], conn.target_component),
                    (MSG_GLOBAL_POSITION_INT, rates_hz["position"], conn.target_component),
                    (MSG_GIMBAL_DEVICE_ATTITUDE_STATUS, rates_hz["gimbal"], 0)]
        for msg_id, hz, component in requests:
            conn.mav.command_long_send(conn.target_system, component, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                       msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
