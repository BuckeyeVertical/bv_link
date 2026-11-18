"""Per-frame metadata record, wrapped as KLV so it can ride in MPEG-TS next to the video.

The air side sends one record per video frame, carrying the same PTS as that frame. Body layout
(little-endian, 74 bytes):

    u8     version
    u32    frame_seq       increments per frame on the air side (wraps at 2^32)
    i64    t_exposure_us   estimated exposure time, Unix microseconds on the air clock
    f64    lat_deg, lon_deg
    f32    alt_msl_m       GLOBAL_POSITION_INT.alt
    f32    alt_rel_m       above home, not above ground
    4xf32  q_vehicle       w, x, y, z; body FRD -> NED (ATTITUDE_QUATERNION)
    4xf32  q_gimbal        w, x, y, z; the frame depends on gimbal_flags (GIMBAL_DEVICE_ATTITUDE_STATUS)
    u32    gimbal_flags    GIMBAL_DEVICE_FLAGS bitmask exactly as the gimbal reported it
    u8     status          STATUS_* bits below
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

# Project-specific 16-byte KLV key. Any 16 bytes work as long as both ends agree; this is not a
# registered SMPTE/MISB key, so off-the-shelf MISB 0601 parsers will skip it.
KEY = bytes.fromhex("b7c1a3e05d2f4c8e9a61f0d4e2b8c753")
VERSION = 1

# Status bits. "Held": the frame time fell outside the buffered samples, so the nearest sample
# (within max_hold_s) was used instead of interpolating. "No": nothing within max_hold_s; the
# fields are zero (position) or identity (quaternions) and must not be used.
POSITION_HELD = 1 << 0
ATTITUDE_HELD = 1 << 1
GIMBAL_HELD = 1 << 2
NO_POSITION = 1 << 3
NO_ATTITUDE = 1 << 4
NO_GIMBAL = 1 << 5
UNUSABLE = NO_POSITION | NO_ATTITUDE | NO_GIMBAL

_BODY = struct.Struct("<BIqddff4f4fIB")

Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class FrameMeta:
    frame_seq: int
    t_exposure_us: int
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    alt_rel_m: float
    q_vehicle: Quat
    q_gimbal: Quat
    gimbal_flags: int
    status: int


def pack(meta: FrameMeta) -> bytes:
    body = _BODY.pack(VERSION, meta.frame_seq, meta.t_exposure_us, meta.lat_deg, meta.lon_deg,
                      meta.alt_msl_m, meta.alt_rel_m, *meta.q_vehicle, *meta.q_gimbal,
                      meta.gimbal_flags, meta.status)
    return KEY + _ber_length(len(body)) + body


def unpack(data: bytes) -> FrameMeta:
    if data[:16] != KEY:
        raise ValueError("not a bv_link KLV packet")
    length, start = _read_ber_length(data, 16)
    body = data[start:start + length]
    if len(body) != length:
        raise ValueError(f"truncated KLV packet: {len(body)} of {length} bytes")
    if body[0] != VERSION:
        raise ValueError(f"metadata version {body[0]}, expected {VERSION}")
    if length != _BODY.size:
        raise ValueError(f"metadata body is {length} bytes, expected {_BODY.size}")
    v = _BODY.unpack(body)
    return FrameMeta(frame_seq=v[1], t_exposure_us=v[2], lat_deg=v[3], lon_deg=v[4],
                     alt_msl_m=v[5], alt_rel_m=v[6], q_vehicle=tuple(v[7:11]),
                     q_gimbal=tuple(v[11:15]), gimbal_flags=v[15], status=v[16])


def _ber_length(n: int) -> bytes:
    """KLV length field: short form below 128, otherwise 0x80 | byte count, then big-endian bytes."""
    if n < 0x80:
        return bytes([n])
    raw = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(raw)]) + raw


def _read_ber_length(data: bytes, i: int) -> tuple[int, int]:
    """Returns (length, index of the first value byte)."""
    first = data[i]
    if first < 0x80:
        return first, i + 1
    count = first & 0x7F
    return int.from_bytes(data[i + 1:i + 1 + count], "big"), i + 1 + count
