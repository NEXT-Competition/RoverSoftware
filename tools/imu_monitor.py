#!/usr/bin/env python3
"""Live BNO085 heading / yaw-rate / calibration monitor — IMU bring-up.

Use this to confirm the IMU is wired and reading before trusting it for
navigation, and to run the first-power-up magnetometer calibration: move the
rover in a few figure-8s until the calibration level reaches 3.

    python tools/imu_monitor.py
    python tools/imu_monitor.py --address 0x4b --offset 90 --invert

The calibration level is 0-3 (3 = fully calibrated). heading() only reports a
value once it reaches the driver's min_calib; below that it prints
"(uncalibrated -> GPS fallback)", which is exactly how the rover behaves.

Once the level reaches 3 the driver saves the calibration to the BNO08x's own
on-chip flash (there is no file — the chip remembers it across power cycles), so
the robot boots pre-calibrated. Use --no-save for a monitor-only dry run.

Off-hardware (no adafruit-circuitpython-bno08x / Blinka): prints a clear note
and exits, so this is safe to run on a laptop.
"""

import argparse
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/imu_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.sensors.bno085 import IMU, adafruit_bno08x


def main():
    p = argparse.ArgumentParser(description="BNO085 IMU live monitor / calibration")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x4A,
                   help="I2C address (default 0x4a; 0x4b if DI/AD0 high)")
    p.add_argument("--offset", type=float, default=0.0,
                   help="heading offset (deg) to align yaw with North")
    p.add_argument("--invert", action="store_true", help="flip yaw sign to CW-positive")
    p.add_argument("--min-calib", type=int, default=1,
                   help="min calibration level (0-3) before heading is reported")
    p.add_argument("--rate", type=float, default=5.0, help="print rate (Hz)")
    p.add_argument("--no-save", action="store_true",
                   help="don't persist calibration to the chip's flash (monitor only)")
    args = p.parse_args()

    if adafruit_bno08x is None:
        print("adafruit_bno08x not found — install on the Pi: "
              "pip install adafruit-circuitpython-bno08x\n"
              "(nothing to monitor on a dev laptop without the sensor libraries)")
        return

    imu = IMU(i2c_address=args.address, heading_offset_deg=args.offset,
              invert=args.invert, min_calib=args.min_calib,
              persist_calibration=not args.no_save)
    imu.start()
    dest = "off" if args.no_save else "BNO085 flash"
    print(f"Move the rover in figure-8s until the calibration level reaches 3 "
          f"(auto-save -> {dest}). Ctrl-C to stop.\n")
    period = 1.0 / args.rate if args.rate > 0 else 0.2
    try:
        while True:
            heading = imu.heading()
            rate = imu.yaw_rate()
            calib = imu.calibration()
            h = f"{heading:6.1f}°" if heading is not None else "  --   (uncalibrated -> GPS fallback)"
            r = f"{rate:+6.1f}°/s" if rate is not None else "  --  "
            print(f"heading={h}  yaw_rate={r}  calib={calib}/3")
            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
