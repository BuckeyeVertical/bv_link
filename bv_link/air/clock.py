"""Maps a MAVLink component's boot clock (time_boot_ms) onto this machine's time.monotonic()."""

from __future__ import annotations

from collections import deque


class ClockMap:
    """Sliding-window minimum offset estimator.

    Each message has a remote timestamp r and a local arrival time a. With true clock offset c
    and transport delay d >= d_min:

        a = r + c + d   =>   a - r = c + d >= c + d_min

    Jitter only ever adds delay, so the minimum of (a - r) over a window picks the least-delayed
    message and estimates c + d_min. A mean would be biased by every slow read. d_min on a serial
    link is a fixed few hundred microseconds; it is constant, so the camera latency calibration
    absorbs it along with everything else.

    The window keeps the estimate following crystal drift between the two clocks (50 ppm is
    0.5 ms per 10 s). A remote time that jumps backwards means the component rebooted, which
    restarts its boot clock, so the estimate is reset.
    """

    def __init__(self, window_s: float = 10.0, reboot_jump_s: float = 1.0):
        self.window_s = window_s
        self.reboot_jump_s = reboot_jump_s
        # (local_s, offset_s) with offsets strictly increasing front to back, so the front is
        # always the minimum over the window (monotonic deque, O(1) amortized per update).
        self._candidates: deque[tuple[float, float]] = deque()
        self._last_remote: float | None = None

    def update(self, remote_s: float, local_s: float) -> None:
        if self._last_remote is not None and remote_s < self._last_remote - self.reboot_jump_s:
            self._candidates.clear()
        self._last_remote = remote_s

        offset = local_s - remote_s
        while self._candidates and self._candidates[-1][1] >= offset:
            self._candidates.pop()
        self._candidates.append((local_s, offset))
        while self._candidates[0][0] < local_s - self.window_s:
            self._candidates.popleft()

    @property
    def ready(self) -> bool:
        return bool(self._candidates)

    @property
    def offset_s(self) -> float:
        return self._candidates[0][1]

    def to_local(self, remote_s: float) -> float:
        return remote_s + self.offset_s
