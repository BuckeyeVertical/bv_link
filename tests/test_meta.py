import pytest

from bv_link import meta


def _record(**overrides) -> meta.FrameMeta:
    fields = dict(frame_seq=42, t_exposure_us=1_763_400_000_123_456, lat_deg=40.0012345,
                  lon_deg=-83.0123456, alt_msl_m=250.5, alt_rel_m=30.25,
                  q_vehicle=(1.0, 0.0, 0.0, 0.0), q_gimbal=(0.5, 0.5, 0.5, 0.5),
                  gimbal_flags=0b1100, status=meta.GIMBAL_HELD)
    fields.update(overrides)
    return meta.FrameMeta(**fields)


def test_round_trip():
    record = _record()
    packet = meta.pack(record)
    assert packet[:16] == meta.KEY
    assert packet[16] == 74          # short-form BER length
    assert len(packet) == 16 + 1 + 74
    out = meta.unpack(packet)
    assert out.frame_seq == record.frame_seq
    assert out.t_exposure_us == record.t_exposure_us
    assert out.lat_deg == record.lat_deg     # f64: exact
    assert out.lon_deg == record.lon_deg
    assert out.alt_rel_m == pytest.approx(record.alt_rel_m)
    assert out.q_gimbal == pytest.approx(record.q_gimbal)
    assert out.gimbal_flags == record.gimbal_flags
    assert out.status == record.status


def test_long_form_length_is_read():
    assert meta._read_ber_length(bytes([0x82, 0x01, 0x00]), 0) == (256, 3)
    assert meta._ber_length(256) == bytes([0x82, 0x01, 0x00])


@pytest.mark.parametrize("mutate, message", [
    (lambda p: b"\x00" * 16 + p[16:], "not a bv_link"),
    (lambda p: p[:-1], "truncated"),
    (lambda p: p[:17] + b"\x63" + p[18:], "version"),
])
def test_rejects_bad_packets(mutate, message):
    with pytest.raises(ValueError, match=message):
        meta.unpack(mutate(meta.pack(_record())))
