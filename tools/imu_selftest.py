#!/usr/bin/env python3
"""Standalone BNO055 self-test — verify the IMU hardware in isolation.

Where tools/imu_monitor.py *streams* live heading (for figure-8 calibration),
this runs a one-shot DIAGNOSTIC of the sensor by itself — independent of the
rest of the robot stack — and prints a PASS/FAIL summary. Run it first when
bringing up a new board or chasing a wiring/I2C problem.

    python tools/imu_selftest.py
    python tools/imu_selftest.py --address 0x29

Checks:
  1. Driver libraries installed (adafruit-circuitpython-bno055 / Blinka).
  2. I2C bus reachable and the BNO055 present at the expected address.
  3. Sensor initializes — the driver validates the chip ID / power-on self-test.
  4. NDOF mode engages and readings are live and sane: |acceleration| ~ 9.8 m/s^2
     at rest, magnetometer producing a field, euler heading valid, temperature
     plausible.
  5. Calibration snapshot (sys/gyro/accel/mag, 0-3) — informational.

Off-hardware (no sensor libraries) it prints a clear note and exits 0, so it's
safe to run on a dev laptop.
"""

import argparse
import math
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable, matching the other tools.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:  # low-level access for the bus scan + raw reads (guarded like servo_sweep.py)
    import board
    import busio
    import adafruit_bno055
except Exception:
    board = busio = adafruit_bno055 = None


def _mag3(v):
    return math.sqrt(sum(c * c for c in v))


def _avg(vs):
    if not vs:
        return None
    n = len(vs)
    return tuple(sum(v[i] for v in vs) / n for i in range(len(vs[0])))


def _span_mag(vs):
    """Range (max-min) of the per-sample vector magnitudes — how jumpy the reads
    are. Large spread => corrupted/jittery reads (bus); small => a stable offset."""
    ms = [_mag3(v) for v in vs]
    return max(ms) - min(ms) if ms else 0.0


def _fmt(v):
    if v is None:
        return "None"
    if isinstance(v, (int, float)):
        return f"{v:.2f}"
    return "(" + ", ".join(f"{c:.2f}" if isinstance(c, (int, float)) else str(c)
                           for c in v) + ")"


def main():
    p = argparse.ArgumentParser(description="BNO055 isolation self-test")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x28,
                   help="I2C address (default 0x28; 0x29 if ADR high)")
    p.add_argument("--samples", type=int, default=10,
                   help="raw-reading samples to average for the sanity checks")
    p.add_argument("--raw", action="store_true",
                   help="print every raw sample (spot corrupted/jittery I2C reads)")
    args = p.parse_args()

    if adafruit_bno055 is None:
        print("adafruit_bno055 not found — install on the Pi: "
              "pip install adafruit-circuitpython-bno055\n"
              "(no sensor to self-test on a dev laptop without the libraries)")
        return 0

    results = []  # (name, ok)

    def check(name, ok, detail=""):
        results.append((name, ok))
        mark = "PASS" if ok else "FAIL"
        print(f"[{mark}] {name}" + (f" — {detail}" if detail else ""))
        return ok

    # 2. I2C bus + device presence.
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        check("I2C bus init", False, str(e))
        return _summary(results)
    while not i2c.try_lock():
        pass
    try:
        addrs = i2c.scan()
    finally:
        i2c.unlock()
    present = args.address in addrs
    check("BNO055 present on I2C", present,
          f"0x{args.address:02x} found" if present else
          f"0x{args.address:02x} NOT found; bus has {[hex(a) for a in addrs]}")

    # 3. Init (the driver reads/validates the chip ID and runs the POST) + NDOF.
    try:
        sensor = adafruit_bno055.BNO055_I2C(i2c, address=args.address)
        sensor.mode = adafruit_bno055.NDOF_MODE
        sleep(0.05)
        check("Sensor init + NDOF mode", True,
              "chip ID validated by driver; NDOF mode set")
    except Exception as e:
        check("Sensor init + NDOF mode", False, str(e))
        return _summary(results)  # nothing else is meaningful without a sensor

    # 4. Live readings + sanity. Average a few samples, skipping None tuples the
    # sensor emits briefly right after a mode switch. We also track the spread
    # (range of the per-sample magnitudes) so you can tell a systematic offset
    # (stable but wrong -> power / clone chip) from corrupted reads (jumpy ->
    # I2C clock stretching or wiring). Run with --raw to see every sample.
    acc, mag, eul, temp = [], [], [], []
    for i in range(max(1, args.samples)):
        a, m, e, t = sensor.acceleration, sensor.magnetic, sensor.euler, sensor.temperature
        if a and None not in a:
            acc.append(a)
        if m and None not in m:
            mag.append(m)
        if e and None not in e:
            eul.append(e)
        if isinstance(t, (int, float)):
            temp.append(t)
        if args.raw:
            print(f"  sample {i:2d}: accel={_fmt(a)} mag={_fmt(m)} "
                  f"euler={_fmt(e)} temp={_fmt(t)}")
        sleep(0.05)

    check("Readings are live", bool(acc and mag and eul),
          f"acc={len(acc)} mag={len(mag)} euler={len(eul)} samples")

    if acc:
        am = _mag3(_avg(acc))
        check("Accelerometer sane (|a| ~ 9.8 m/s^2 at rest)", 6.0 <= am <= 13.0,
              f"|a|={am:.2f} m/s^2 (spread {_span_mag(acc):.2f})")
    if mag:
        bm = _mag3(_avg(mag))
        # Earth's field is ~25-65 uT; board/hard-iron offsets raise that, but a
        # reading this far out means interference or a corrupted read, not physics.
        check("Magnetometer plausible (earth field ~25-65 uT)", 5.0 <= bm <= 150.0,
              f"|B|={bm:.1f} uT (spread {_span_mag(mag):.1f})")
    if temp:
        t = sum(temp) / len(temp)
        check("Temperature plausible", -20.0 <= t <= 85.0,
              f"{t:.0f} C (range {min(temp):.0f}..{max(temp):.0f})")

    # 5. Calibration snapshot (informational — a fresh board reads 0s until you
    # calibrate, so this is not a pass/fail check).
    try:
        s, gy, ac, mg = sensor.calibration_status
        print(f"[INFO] calibration sys={s} gyro={gy} accel={ac} mag={mg} (0-3). "
              "Not calibrated yet? Run tools/imu_monitor.py and do figure-8s.")
    except Exception:
        pass

    return _summary(results)


def _summary(results):
    fails = [n for n, ok in results if not ok]
    print()
    if fails:
        print(f"SELF-TEST FAILED: {len(fails)}/{len(results)} checks failed -> "
              f"{', '.join(fails)}")
        _troubleshoot(fails)
        return 1
    print(f"SELF-TEST PASSED: all {len(results)} checks OK")
    return 0


def _troubleshoot(fails):
    """Different failures point at different causes. Accel/temperature garbage
    means bad I2C reads (bus); a lone, stable magnetometer failure means a real
    magnetic field (interference), not the bus."""
    bus_checks = {
        "Accelerometer sane (|a| ~ 9.8 m/s^2 at rest)",
        "Temperature plausible",
    }
    mag_check = "Magnetometer plausible (earth field ~25-65 uT)"
    bus_failed = [f for f in fails if f in bus_checks]

    if bus_failed:
        print("\nAccel/temperature garbage means corrupted I2C reads:")
        print("  1. I2C clock stretching — the Pi's hardware I2C mishandles it, and")
        print("       100 kHz (the DEFAULT) is not slow enough. Drop the bus right down:")
        print("       set 'dtparam=i2c_arm_baudrate=10000' in /boot/firmware/config.txt,")
        print("       then reboot. (Older Pi OS: /boot/config.txt.)")
        print("  2. To keep full speed instead, use a software I2C bus")
        print("       (dtoverlay=i2c-gpio), which handles clock stretching correctly.")
        print("  3. Power/wiring — solid 3.3V, short leads, common ground; a brown-out")
        print("       skews accel & temperature. --raw jitter => bus/wiring; stable-but-")
        print("       wrong => power or a counterfeit BNO055 (common on GY-BNO055 clones).")

    if mag_check in fails and not bus_failed:
        print("\nMagnetometer is the only failure and the reads are stable, so it's a")
        print("REAL field the sensor sees — magnetic interference, not a bus fault:")
        print("  1. Mount the BNO055 AWAY from the drive motors/ESCs, magnets, speakers,")
        print("       battery/current wiring and ferrous metal — motor magnets dwarf")
        print("       earth's ~50 uT and can saturate the magnetometer (~1300 uT range).")
        print("  2. Confirm it: hold the bare board away from the rover and re-run — |B|")
        print("       should drop to ~25-65 uT. If it does, it's a mounting-location fix.")
        print("  3. Then calibrate: run tools/imu_monitor.py and do figure-8s (mag -> 3).")


if __name__ == "__main__":
    sys.exit(main())
