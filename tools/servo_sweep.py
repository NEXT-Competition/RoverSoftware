#!/usr/bin/env python3
"""Raw servo sweep on one channel - the Fusion HAT 'hello world', adapted.

Handy to confirm a channel is wired and moving before dealing with ESC arming.
Wheels off the ground, please.

    python tools/servo_sweep.py --channel 0
"""

import argparse
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable so the mock fallback's
# `import robot` works even when run as `python tools/servo_sweep.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from fusion_hat.servo import Servo
except Exception:
    from robot.drive.motor import Servo  # mock fallback


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--channel", type=int, default=0)
    p.add_argument("--cycles", type=int, default=0, help="0 = forever")
    args = p.parse_args()

    servo = Servo(args.channel)
    n = 0
    try:
        while args.cycles == 0 or n < args.cycles:
            for i in range(-90, 91, 10):
                servo.angle(i)
                sleep(0.1)
            for i in range(90, -91, -10):
                servo.angle(i)
                sleep(0.1)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        servo.angle(0)  # back to neutral


if __name__ == "__main__":
    main()
