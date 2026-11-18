import random

import pytest

from bv_link.air.clock import ClockMap


def test_min_filter_ignores_jitter():
    """Remote clock 1000 s behind local; delay is 0.3 ms plus up to 20 ms of one-sided jitter."""
    rng = random.Random(0)
    clock = ClockMap(window_s=10.0)
    true_offset, d_min = 1000.0, 0.0003
    for i in range(500):          # 5 s at 100 Hz
        remote = i * 0.01
        clock.update(remote, remote + true_offset + d_min + rng.uniform(0.0, 0.020))
    assert clock.offset_s == pytest.approx(true_offset + d_min, abs=0.0005)


def test_tracks_drift():
    """Remote clock runs 100 ppm slow; the windowed estimate should follow it."""
    clock = ClockMap(window_s=5.0)
    for i in range(6000):         # 60 s at 100 Hz
        local = i * 0.01
        clock.update(local * (1 - 100e-6), local)
    # After 60 s the true offset has grown to 6 ms; a stale estimate would be stuck near 0.
    assert clock.offset_s == pytest.approx(60 * 100e-6, abs=0.0006)


def test_reboot_resets_estimate():
    clock = ClockMap()
    for i in range(100):
        clock.update(500.0 + i * 0.01, 10.0 + i * 0.01)          # offset -490
    clock.update(0.0, 11.5)                                        # boot clock restarted
    assert clock.offset_s == pytest.approx(11.5)


def test_old_minimum_ages_out():
    clock = ClockMap(window_s=1.0)
    clock.update(0.0, 1.0)            # unusually fast: offset 1.0
    for i in range(1, 300):
        clock.update(i * 0.01, i * 0.01 + 1.005)
    assert clock.offset_s == pytest.approx(1.005)
