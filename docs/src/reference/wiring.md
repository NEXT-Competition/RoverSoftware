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

## Wheel encoders (optional)

A throttle is a wish, not a speed. Two motors handed the same pulse turn at
different rates, so the rover curves while the dashboard insists it is going
straight. An encoder measures what the wheel actually did, and the robot can
then hold the two sides together.

**The digital pins, not the PWM channels.** The Fusion HAT has both, and they
are different buses. `channel` above is a PWM output — where an ESC or a servo
goes. `encoder_a`/`encoder_b` are the HAT's **digital** pins, which are Pi GPIO
lines broken out on the HAT header and numbered as BCM, so the number silkscreened
on the board is the number to enter. Confusing the two produces an encoder that
counts nothing, and it is the first mistake everybody makes here.

A quadrature encoder has four wires, and all four matter: ground, **supply**,
A and B. A Hall-effect encoder is an active device — with no supply its outputs
never drive, both pins sit at the internal pull-up, and the count stays at
exactly 0 with no other symptom. That is the most common bring-up fault here.

**Power it from 3.3 V.** Many encoders accept 3.3–5 V; take the lower one,
because a Pi GPIO is *not* 5 V tolerant and A/B are wired straight to it. A 5 V
encoder with no choice in the matter needs a level shifter.

Internal pull-ups are enabled for you, so an open-collector output needs no
resistors of its own.

**Check the published resolution before you measure.** Some motors specify it
exactly — a goBILDA 5203 Yellow Jacket, for instance, gives "1993.6 PPR at the
Output Shaft", already counted the way this decoder counts (all four edges), and
already including the gearbox. Enter it directly. Multiply only if there is
further reduction between the output shaft and the wheel. Measuring is the
fallback for a motor whose datasheet you do not trust, not a ritual.

The pins are read through the same `fusion_hat` library that already drives
the motors — no separate GPIO package and no daemon. One catch: `fusion_hat`
reads pins through `RPi.GPIO`, and the **stock `RPi.GPIO` cannot arm GPIO
interrupts on a current Raspberry Pi OS kernel**. It was last released in 2019
and still does edge detection through `/sys/class/gpio`, whose numbering the
kernel has since rebased. Setting pin direction works, so the motors are fine
and an encoder fails with a bare `Failed to add edge detection` that looks for
all the world like a pin conflict.

Swap it for the drop-in replacement, once per robot (they conflict, so the old
one comes off first):

```bash
just encoder-gpio
# or by hand, on the Pi:
sudo apt remove -y python3-rpi.gpio && sudo apt install -y python3-rpi-lgpio
```

`rpi-lgpio` presents the same API on top of lgpio and goes through the GPIO
character device. Check which one you have with:

```bash
python3 -c "import sys, RPi.GPIO; print('lgpio-backed:', 'lgpio' in sys.modules)"
```

Then, **wheels off the ground and the drivetrain unpowered**:

```bash
# 4. prove the wiring, and MEASURE counts-per-rev
python tools/encoder_monitor.py --pins 17,27
#    turn the wheel forward by hand: the count must go UP. If it goes down,
#    that is the "Count inverted" toggle, not a wiring fault.
#    Then zero it, turn the wheel exactly one full turn, and read the count —
#    that number is Counts per rev on the actuator's card.
```

Enter the pins and that number in **Settings → Hardware → the motor → Encoder**,
save, and restart the robot (pins are claimed at start-up, like PWM channels).
The driving view then shows a `wheels` row. Set **Settings → Robot → Wheel speed
matching → Mode** to `match` and drive a straight line: the gap should collapse
toward zero. `match` needs no calibration at all and only acts while you are
driving straight; `velocity` also works in turns but first wants **Max wheel
RPM**, which you get by driving flat out and reading that same row.

## The tools

| Tool | What it is for |
|---|---|
| `servo_sweep.py` | Raw servo sweep — the Fusion HAT hello-world |
| `esc_calibrate.py` | Interactive single-channel ESC bring-up |
| `encoder_monitor.py` | Encoder bring-up, and the counts-per-rev measurement |
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
