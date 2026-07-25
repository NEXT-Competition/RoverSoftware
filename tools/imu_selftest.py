#!/usr/bin/env python3
"""Standalone BNO085 self-test — verify the IMU hardware in isolation.

Where tools/imu_monitor.py *streams* live heading (for figure-8 calibration),
this runs a one-shot DIAGNOSTIC of the sensor by itself — independent of the
rest of the robot stack — and prints a PASS/FAIL summary. Run it first when
bringing up a new board or chasing a wiring/I2C problem.

    python tools/imu_selftest.py
    python tools/imu_selftest.py --address 0x4b

Checks:
  1. Driver libraries installed (adafruit-circuitpython-bno08x / Blinka).
  2. I2C bus reachable and the BNO085 present at the expected address.
  3. Sensor initializes — the driver opens the SHTP channel and validates the
     product-id report.
  4. Reports enable and readings are live and sane: |acceleration| ~ 9.8 m/s^2
     at rest, magnetometer producing a field, and a unit-norm orientation
     quaternion.
  5. Calibration snapshot (fused-orientation level, 0-3) — informational.

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
    import adafruit_bno08x
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
except Exception:
    board = busio = adafruit_bno08x = None
    BNO08X_I2C = None
    BNO_REPORT_ACCELEROMETER = BNO_REPORT_MAGNETOMETER = None
    BNO_REPORT_GYROSCOPE = BNO_REPORT_ROTATION_VECTOR = None


def _mag(v):
    return math.sqrt(sum(c * c for c in v))


def _avg(vs):
    if not vs:
        return None
    n = len(vs)
    return tuple(sum(v[i] for v in vs) / n for i in range(len(vs[0])))


def _span_mag(vs):
    """Range (max-min) of the per-sample vector magnitudes — how jumpy the reads
    are. Large spread => corrupted/jittery reads (bus/wiring); small => stable."""
    ms = [_mag(v) for v in vs]
    return max(ms) - min(ms) if ms else 0.0


def _fmt(v):
    if v is None:
        return "None"
    if isinstance(v, (int, float)):
        return f"{v:.2f}"
    return "(" + ", ".join(f"{c:.2f}" if isinstance(c, (int, float)) else str(c)
                           for c in v) + ")"


def main():
    p = argparse.ArgumentParser(description="BNO085 isolation self-test")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x4A,
                   help="I2C address (default 0x4a; 0x4b if DI/AD0 high)")
    p.add_argument("--samples", type=int, default=10,
                   help="raw-reading samples to average for the sanity checks")
    p.add_argument("--raw", action="store_true",
                   help="print every raw sample (spot corrupted/jittery I2C reads)")
    args = p.parse_args()

    if adafruit_bno08x is None:
        print("adafruit_bno08x not found — install on the Pi: "
              "pip install adafruit-circuitpython-bno08x\n"
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
    check("BNO085 present on I2C", present,
          f"0x{args.address:02x} found" if present else
          f"0x{args.address:02x} NOT found; bus has {[hex(a) for a in addrs]}")

    # 3. Init (opens the SHTP channel, validates the product-id report) + enable
    # the reports we need. The sensor sends nothing until each feature is enabled.
    try:
        sensor = BNO08X_I2C(i2c, address=args.address)
        sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        sensor.enable_feature(BNO_REPORT_MAGNETOMETER)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        sleep(0.2)  # first reports take a beat to start flowing
        check("Sensor init + reports enabled", True,
              "SHTP up; accel/mag/gyro/rotation-vector subscribed")
    except Exception as e:
        check("Sensor init + reports enabled", False, str(e))
        return _summary(results)  # nothing else is meaningful without a sensor

    # 4. Live readings + sanity. Average a few samples, skipping the None tuples
    # the sensor emits briefly right after enabling a feature. We also track the
    # spread (range of per-sample magnitudes) so you can tell a systematic offset
    # (stable but wrong -> power / wiring) from corrupted reads (jumpy -> bus).
    acc, mag, quat = [], [], []
    for i in range(max(1, args.samples)):
        try:
            a, m, q = sensor.acceleration, sensor.magnetic, sensor.quaternion
        except Exception as e:
            if args.raw:
                print(f"  sample {i:2d}: read error {e}")
            sleep(0.05)
            continue
        if a and None not in a:
            acc.append(a)
        if m and None not in m:
            mag.append(m)
        if q and None not in q:
            quat.append(q)
        if args.raw:
            print(f"  sample {i:2d}: accel={_fmt(a)} mag={_fmt(m)} quat={_fmt(q)}")
        sleep(0.05)

    check("Readings are live", bool(acc and mag and quat),
          f"acc={len(acc)} mag={len(mag)} quat={len(quat)} samples")

    if acc:
        am = _mag(_avg(acc))
        check("Accelerometer sane (|a| ~ 9.8 m/s^2 at rest)", 6.0 <= am <= 13.0,
              f"|a|={am:.2f} m/s^2 (spread {_span_mag(acc):.2f})")
    if mag:
        bm = _mag(_avg(mag))
        # Earth's field is ~25-65 uT; board/hard-iron offsets raise that, but a
        # reading this far out means interference or a corrupted read, not physics.
        check("Magnetometer plausible (earth field ~25-65 uT)", 5.0 <= bm <= 150.0,
              f"|B|={bm:.1f} uT (spread {_span_mag(mag):.1f})")
    if quat:
        # A valid orientation quaternion is unit-norm. A norm far from 1 means the
        # rotation vector isn't producing a real fused orientation yet.
        qn = _mag(_avg(quat))
        check("Orientation quaternion valid (|q| ~ 1)", 0.9 <= qn <= 1.1,
              f"|q|={qn:.3f}")

    # 5. Calibration snapshot (informational — a fresh board reads low until you
    # calibrate, so this is not a pass/fail check).
    try:
        level = sensor.calibration_status
        print(f"[INFO] calibration level={level} (0-3). "
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
    """Different failures point at different causes. A missing device or dead
    reads is wiring/mode-strap/power; a lone, stable magnetometer failure means a
    real magnetic field (interference), not the bus."""
    present_check = "BNO085 present on I2C"
    accel_check = "Accelerometer sane (|a| ~ 9.8 m/s^2 at rest)"
    live_check = "Readings are live"
    mag_check = "Magnetometer plausible (earth field ~25-65 uT)"
    wiring_failed = any(f in (present_check, accel_check, live_check) for f in fails)

    if wiring_failed:
        print("\nNo device / dead / garbage reads point at wiring or the interface mode:")
        print("  1. Mode straps — the BNO08x only speaks I2C when PS0/PS1 select it")
        print("       (both LOW on most breakouts). If they're strapped for UART/SPI")
        print("       the chip won't answer on I2C at all.")
        print("  2. Address — 0x4A default, 0x4B if DI/AD0 is pulled high. Re-run with")
        print("       --address 0x4b if the scan shows the other one.")
        print("  3. Power/wiring — solid 3.3V, short leads, common ground; a brown-out")
        print("       skews accel. --raw jitter => bus/wiring; stable-but-wrong => power.")
        print("  NOTE: unlike the BNO055, the BNO08x does NOT clock-stretch, so the")
        print("       old dtparam=i2c_arm_baudrate=10000 hack is unnecessary — and if")
        print("       it's still set it only slows this bus down.")

    if mag_check in fails and not wiring_failed:
        print("\nMagnetometer is the only failure and the reads are stable, so it's a")
        print("REAL field the sensor sees — magnetic interference, not a bus fault:")
        print("  1. Mount the BNO085 AWAY from the drive motors/ESCs, magnets, speakers,")
        print("       battery/current wiring and ferrous metal — motor magnets dwarf")
        print("       earth's ~50 uT and can saturate the magnetometer.")
        print("  2. Confirm it: hold the bare board away from the rover and re-run — |B|")
        print("       should drop to ~25-65 uT. If it does, it's a mounting-location fix.")
        print("  3. Then calibrate: run tools/imu_monitor.py and do figure-8s (level -> 3).")


if __name__ == "__main__":
    sys.exit(main())
