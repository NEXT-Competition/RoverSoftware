# Wiring and calibration

- **motor1 → Fusion HAT channel 0** (left), **motor2 → channel 1** (right).
- The right motor is mounted mirrored, so `right.inverted = True`.
- **An ESC takes the same PWM a servo does.** Neutral pulse = stop, longer =
  forward, shorter = reverse.
- **"90 is centre"? Not here.** The Fusion HAT's `Servo.angle()` runs about
  `−90 … +90` with the *middle* (0) as neutral; the 0–180 convention is for
  positional servos. Neutral need not be 0 — this rover's ESC stops at `5.0`.

In code, throttle `-1 … +1` maps to a **symmetric throw about `neutral_angle`**
— an equal swing on each side, where
`throw = min(max_angle - neutral_angle, neutral_angle - min_angle)`. This keeps
the normal and mirrored motors starting together and matching speed even when
neutral is not centred.

## Bring-up, in order

```bash
# 1. confirm a channel moves — WHEELS OFF THE GROUND
python tools/servo_sweep.py --channel 0

# 2. find this ESC's neutral and endpoints, then copy them into the
#    motor's card in Settings → Hardware
python tools/esc_calibrate.py --channel 0

# 3. verify the radio: watch frames in, send test frames out
python tools/xbee_monitor.py --port /dev/serial0 --baud 9600
#    type  d 0.5 0  to send a drive frame; blank line for test telemetry;
#    q to quit. Raw bytes printed => baud or wiring mismatch.
```

Run the monitor on the robot to confirm the base station's commands arrive; run
it (or the base station) on the other end to confirm your sends are received.

## The tools

| Tool | What it is for |
|---|---|
| `servo_sweep.py` | Raw servo sweep — the Fusion HAT hello-world |
| `esc_calibrate.py` | Interactive single-channel ESC bring-up |
| `xbee_monitor.py` | Watch and inject XBee frames to prove the link |
| `gps_monitor.py` | GPS bring-up: fix quality, track angle |
| `imu_monitor.py` | BNO085 heading and calibration status |
| `imu_selftest.py` | Confirms the IMU answers and calibrates |
| `detector_selftest.py` | Vision bring-up and standoff calibration, either backend |
| `fetch_tiles.py` | Build an offline `.mbtiles` cache before you lose signal |

## Vision backends

Two interchangeable detectors, chosen with `RS_VISION_BACKEND` or
`--vision-backend`:

- **`imx500`** — the Raspberry Pi AI Camera runs the network on the sensor
  itself, so the Pi spends no CPU on inference and every model reports real
  sized boxes. `sudo apt install python3-picamera2 imx500-all`.
- **`edge_impulse`** — a compiled `.eim` run on the Pi's CPU; works with any
  camera. Drop a model at `RS_VISION_MODEL`. Export a YOLO-style
  (`object_detection`) model — FOMO reports centroids, not sized boxes, so it
  can align but never approach.

`auto` uses the AI Camera when one is attached, else Edge Impulse. See
[Training and converting a detector](../model-conversion.md) for how the `.rpk`
is produced.
