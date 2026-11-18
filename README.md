# bv_link

Gimbal camera video from the drone to the ground Jetson, with every frame tagged with the drone
and gimbal state at the moment it was exposed.

```
Radxa (air)                                                 Jetson (ground)
camera RTSP H.265 --passthrough--> MPEG-TS  ~~Herelink~~>   tsdemux -> nvv4l2decoder -> frame
FC MAVLink -> interpolate at exposure -> KLV (same PTS)     KLV -----------------------> meta
```

The Radxa never decodes video. It repacks the camera's H.265 and adds one 74-byte KLV record per
frame, stamped with that frame's PTS. Because the pairing happens on the drone, link latency and
WiFi jitter cannot misalign a frame and its state.

## Layout

| Path | Runs on | What |
|---|---|---|
| `bv_link/meta.py` | both | The per-frame record and its KLV encoding. Both ends must run the same version. |
| `bv_link/air/clock.py` | Radxa | Maps each MAVLink component's boot clock onto local time (min-filter). |
| `bv_link/air/telemetry.py` | Radxa | MAVLink reader, buffers, lerp/slerp to an arbitrary past time. |
| `bv_link/air/sender.py` | Radxa | GStreamer passthrough + KLV mux. `bv-link-air` |
| `bv_link/ground/receiver.py` | Jetson | Demux, hardware decode, PTS pairing. `bv-link-ground` |
| `config/` | | `air.yaml`/`ground.yaml` for hardware, `*_sim.yaml` for a laptop loopback test. |

## Install

GStreamer and PyGObject come from the OS, so the venv needs system site packages.

```bash
sudo apt install python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

```bash
python3 -m venv --system-site-packages .venv && source .venv/bin/activate && pip install -e ".[dev]"
```

The Jetson already has `nvv4l2decoder` from JetPack. For the laptop test also install
`gstreamer1.0-libav` (software H.265 decode).

## Run

On the Radxa, after filling in the `CHECK` lines in `config/air.yaml`:

```bash
bv-link-air --config config/air.yaml
```

On the Jetson:

```bash
bv-link-ground --config config/ground.yaml --record runs/$(date +%F_%H%M%S)
```

In code, `bv_autonomy` consumes frames like this:

```python
receiver = Receiver(cfg["source"], cfg["decoder"], 0.05, drop_frames=True)
receiver.start()
for frame in receiver.frames():   # frame.image (BGR), frame.meta (FrameMeta or None)
    ...
```

Check `frame.meta.status` before using a record. `meta.UNUSABLE` bits mean that a value is
missing. `*_HELD` bits mean the nearest sample was used instead of an interpolated one.

## Laptop loopback test (no hardware)

Start PX4 SITL, then run each command in its own terminal:

```bash
bv-link-ground --config config/ground_sim.yaml
```

```bash
bv-link-air --config config/air_sim.yaml
```

You should see the ground stats count `matched` equal to `frames`.

## Frames of reference (for geolocation)

- `q_vehicle`: body FRD to NED, from `ATTITUDE_QUATERNION`.
- `q_gimbal`: from `GIMBAL_DEVICE_ATTITUDE_STATUS`. Roll and pitch are relative to the horizon.
  Yaw is relative to the vehicle's heading unless `gimbal_flags` has
  `GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME`, in which case it is relative to north. Check the flags;
  do not assume.
- `alt_rel_m` is height above home, not above the ground. Over sloped terrain, use the terrain map.

## Not done yet

1. **Camera latency calibration.** `camera.latency_ms` is 0. Measure it by yawing fast at a
   textured scene and cross-correlating optical-flow rotation with gyro yaw rate. Measure it at
   the sender's probe point, since that is where arrival time is taken.
2. **Zoom/FOV per frame.** The record has no field for it. Add one if the camera zooms in flight.
3. **Herelink IP passthrough** is assumed, not yet verified on hardware.
4. **The GStreamer code is untested.** Only `meta`, `clock` and `telemetry` have unit tests.
