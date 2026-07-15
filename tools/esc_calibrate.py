#!/usr/bin/env python3
"""Interactive ESC calibration / bring-up for a single channel.

Bidirectional ESCs usually need a one-time throttle-range calibration and a
known neutral point. Use this to nudge one channel, find the throttle values
that mean full-reverse / stop / full-forward, then copy the resulting angles
into robot/config.py.

    python tools/esc_calibrate.py --channel 0

Always start at 0 (neutral) so the ESC arms before you command motion.
Run this with the robot wheels off the ground.
"""

import argparse
import os
import sys

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/esc_calibrate.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import MotorConfig  # noqa: E402
from robot.drive.motor import ESCMotor, HAVE_HW  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--channel", type=int, default=0)
    args = p.parse_args()

    if not HAVE_HW:
        print("(!) fusion_hat not found - running against the mock servo.")

    motor = ESCMotor(MotorConfig(channel=args.channel))
    print(f"Channel {args.channel} ready. Commands:")
    print("  <number in [-1, 1]>  set throttle")
    print("  s / <enter>          stop (neutral)")
    print("  q                    quit\n")

    while True:
        try:
            raw = input("throttle> ").strip().lower()
        except EOFError:
            break
        if raw in ("q", "quit", "exit"):
            break
        if raw in ("s", "stop", ""):
            motor.stop()
            print("  neutral")
            continue
        try:
            t = float(raw)
        except ValueError:
            print("  enter a number in [-1, 1], 's', or 'q'")
            continue
        motor.set_throttle(t)
        print(f"  throttle={t:+.2f}")

    motor.stop()
    print("stopped.")


if __name__ == "__main__":
    main()
