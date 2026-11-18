import math
from types import SimpleNamespace

import pytest

from bv_link import meta
from bv_link.air.telemetry import Series, Telemetry, lerp, slerp


def yaw_quat(deg: float) -> tuple[float, float, float, float]:
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def test_slerp_halfway_yaw():
    assert slerp(yaw_quat(0), yaw_quat(90), 0.5) == pytest.approx(yaw_quat(45))


def test_slerp_takes_short_way_across_hemispheres():
    # -q is the same rotation as q; interpolating 10 -> 30 deg must still give 20 deg.
    q1 = tuple(-x for x in yaw_quat(30))
    out = slerp(yaw_quat(10), q1, 0.5)
    expected = yaw_quat(20)
    assert out == pytest.approx(expected) or out == pytest.approx(tuple(-x for x in expected))


def test_slerp_nearly_equal_stays_unit():
    q = slerp(yaw_quat(0), yaw_quat(0.001), 0.3)
    assert math.sqrt(sum(x * x for x in q)) == pytest.approx(1.0)


def test_series_interpolates_holds_and_expires():
    s = Series(lerp)
    s.append(1.0, (0.0,))
    s.append(2.0, (10.0,))
    assert s.sample(1.25, 0.2) == ((2.5,), False)
    assert s.sample(2.0, 0.2) == ((10.0,), False)
    assert s.sample(2.1, 0.2) == ((10.0,), True)
    assert s.sample(0.9, 0.2) == ((0.0,), True)
    assert s.sample(2.5, 0.2) == (None, False)


def test_series_drops_out_of_order():
    s = Series(lerp)
    s.append(1.0, (0.0,))
    s.append(0.99, (5.0,))
    assert s.sample(1.0, 0.2) == ((0.0,), False)


def _msg(kind, system, component, time_boot_ms, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
                           get_srcComponent=lambda: component, time_boot_ms=time_boot_ms, **fields)


def test_state_at_uses_each_components_own_clock():
    """FC booted 100 s before the Radxa's reference; gimbal booted 5 s after. Both 2 ms delay."""
    tel = Telemetry(max_hold_s=0.2)
    tel.target_system = 1
    for i in range(101):                      # 1 s at 100 Hz
        local = 50.0 + i * 0.01
        fc_ms = round((local - 50.0 + 100.0) * 1000)
        gimbal_ms = round((local - 50.0 + 5.0) * 1000)
        tel.ingest(_msg("ATTITUDE_QUATERNION", 1, 1, fc_ms, q1=yaw_quat(i)[0], q2=0.0, q3=0.0,
                        q4=yaw_quat(i)[3]), local + 0.002)
        tel.ingest(_msg("GLOBAL_POSITION_INT", 1, 1, fc_ms, lat=400000000 + i, lon=-830000000,
                        alt=250000, relative_alt=30000 + i), local + 0.002)
        tel.ingest(_msg("GIMBAL_DEVICE_ATTITUDE_STATUS", 1, 154, gimbal_ms, q=list(yaw_quat(-i)),
                        flags=12), local + 0.002)
        tel.ingest(_msg("ATTITUDE_QUATERNION", 2, 1, fc_ms, q1=0.0, q2=1.0, q3=0.0, q4=0.0),
                   local)  # another vehicle: ignored

    # Halfway between samples 40 and 41, shifted by the constant 2 ms transport delay.
    s = tel.state_at(50.405 + 0.002)
    assert s.status == 0
    assert s.q_vehicle == pytest.approx(yaw_quat(40.5), abs=1e-6)
    assert s.q_gimbal == pytest.approx(yaw_quat(-40.5), abs=1e-6)
    assert s.lat_deg == pytest.approx(40.00000405, abs=1e-9)
    assert s.alt_rel_m == pytest.approx(30.0405)
    assert s.gimbal_flags == 12


def test_state_at_flags_missing_data():
    tel = Telemetry()
    s = tel.state_at(10.0)
    assert s.status == meta.NO_POSITION | meta.NO_ATTITUDE | meta.NO_GIMBAL
    assert s.q_vehicle == (1.0, 0.0, 0.0, 0.0)
