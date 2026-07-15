#!/usr/bin/env python3
"""Live BNO055 heading / yaw-rate / calibration monitor — IMU bring-up.

Use this to confirm the IMU is wired and reading before trusting it for
navigation, and to run the first-power-up magnetometer calibration: move the
rover in a few figure-8s until the mag/sys calibration levels reach 3.

    python tools/imu_monitor.py
    python tools/imu_monitor.py --address 0x29 --offset 90 --invert

Calibration levels are 0-3 (3 = fully calibrated). heading() only reports a
value once sys and mag reach the driver's min_calib; below that it prints
"(uncalibrated -> GPS fallback)", which is exactly how the rover behaves.

Off-hardware (no adafruit-circuitpython-bno055 / Blinka): prints a clear note
and exits, so this is safe to run on a laptop.
"""

import argparse
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/imu_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.sensors.bno055 import IMU, adafruit_bno055


def main():
    p = argparse.ArgumentParser(description="BNO055 IMU live monitor / calibration")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x28,
                   help="I2C address (default 0x28; 0x29 if ADR high)")
    p.add_argument("--offset", type=float, default=0.0,
                   help="heading offset (deg) to align yaw with North")
    p.add_argument("--invert", action="store_true", help="flip yaw sign to CW-positive")
    p.add_argument("--min-calib", type=int, default=1,
                   help="min sys/mag calibration (0-3) before heading is reported")
    p.add_argument("--rate", type=float, default=5.0, help="print rate (Hz)")
    args = p.parse_args()

    if adafruit_bno055 is None:
        print("adafruit_bno055 not found — install on the Pi: "
              "pip install adafruit-circuitpython-bno055\n"
              "(nothing to monitor on a dev laptop without the sensor libraries)")
        return

    imu = IMU(i2c_address=args.address, heading_offset_deg=args.offset,
              invert=args.invert, min_calib=args.min_calib)
    imu.start()
    print("Move the rover in figure-8s until mag/sys reach 3. Ctrl-C to stop.\n")
    period = 1.0 / args.rate if args.rate > 0 else 0.2
    try:
        while True:
            heading = imu.heading()
            rate = imu.yaw_rate()
            sys_l, gyro_l, accel_l, mag_l = imu.calibration()
            h = f"{heading:6.1f}°" if heading is not None else "  --   (uncalibrated -> GPS fallback)"
            r = f"{rate:+6.1f}°/s" if rate is not None else "  --  "
            print(f"heading={h}  yaw_rate={r}  "
                  f"calib[sys={sys_l} gyro={gyro_l} accel={accel_l} mag={mag_l}]")
            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
