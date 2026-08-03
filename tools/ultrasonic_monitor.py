#!/usr/bin/env python3
"""Live ultrasonic monitor — wiring proof and collision-threshold calibration.

Two jobs, and as with the encoders the second is the one you cannot skip.

    1. PROVE THE WIRING. Hold a book a foot in front of the module and watch the
       distance move. An ultrasonic is the one sensor on the rover whose failure
       is INVISIBLE — a dead module and a clear path both read as silence — so
       "I saw it track my hand" is the only evidence that it works.

           python tools/ultrasonic_monitor.py --pins 27,22

    2. MEASURE THE STOPPING DISTANCE. Drive the rover at a wall at the speed it
       actually runs at, and see how far past the command it travels. That
       number, plus the distance the sensor sits behind the bumper, IS
       `ultrasonic.stop_m` on the settings page. Guessing it gives you a rover
       that either stops a metre early or arrives at the wall having already
       decided it was fine.

Both readings are printed: `raw` is a single ping, `filtered` is the median the
collision guard actually acts on (robot/control/collision.py). Watching them
side by side is how you size `samples` — if raw spikes and filtered doesn't, the
filter is doing its job; if both jump, it is a real echo off something.

    --- ping ---  echoes 143/150     raw 0.42 m   filtered 0.43 m   [SLOW]

The tag on the right is what the guard would do to a forward command at that
distance right now, using the same code path the robot runs, so you can walk the
rover forward by hand and see exactly where it would refuse.

This tool does NOT touch the motors, so it is safe with the drivetrain powered
down — which is how you should do it. Off-hardware (no `fusion_hat`) it says so
and exits, so it is safe to run on a laptop.
"""

import argparse
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/ultrasonic_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import UltrasonicConfig
from robot.control.collision import CollisionGuard
from robot.control.commands import DriveCommand
from robot.sensors.ultrasonic import Ultrasonic


def _parse_pins(text: str) -> tuple:
    try:
        trig, echo = (int(part.strip()) for part in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(
            "expected two BCM pin numbers, e.g. --pins 27,22") from None
    if trig == echo:
        raise argparse.ArgumentTypeError(
            "TRIG and ECHO must be different pins — one is an output and the "
            "other an input")
    return trig, echo


def main():
    p = argparse.ArgumentParser(
        description="Ultrasonic rangefinder monitor and collision-threshold "
                    "calibration")
    p.add_argument("--pins", type=_parse_pins, default=(27, 22),
                   metavar="TRIG,ECHO",
                   help="BCM GPIO pins for TRIG and ECHO (default 27,22)")
    p.add_argument("--interval", type=float, default=0.06,
                   help="seconds between pings (default 0.06; the HC-SR04 "
                        "datasheet asks for at least this much)")
    p.add_argument("--samples", type=int, default=3,
                   help="median filter width in samples (1 = unfiltered)")
    p.add_argument("--max-m", type=float, default=4.0,
                   help="ignore echoes further away than this (default 4 m)")
    p.add_argument("--min-m", type=float, default=0.03,
                   help="ignore echoes closer than this (default 0.03 m)")
    p.add_argument("--stop-m", type=float, default=UltrasonicConfig.stop_m,
                   help="the guard's stop distance, for the tag on the right")
    p.add_argument("--slow-m", type=float, default=UltrasonicConfig.slow_m,
                   help="the guard's slow-down distance")
    p.add_argument("--rate", type=float, default=5.0, help="print rate (Hz)")
    args = p.parse_args()

    trig, echo = args.pins
    sonar = Ultrasonic(trig_pin=trig, echo_pin=echo, min_m=args.min_m,
                       max_m=args.max_m, interval=args.interval,
                       samples=args.samples, name="monitor")
    if not sonar.start():
        # start() has already said what it could not do. What is left is the
        # advice, which depends on which of the two failures it was.
        print("\nThe ultrasonic is read through the Fusion HAT library, the "
              "same one that drives the motors:")
        print("    just bootstrap        (or SunFounder's install.sh)")
        print(f"\nIf the pins are claimed by something else: the robot service "
              f"takes\nGPIO {trig}/{echo} at start-up whenever robot.env or a "
              f"layout names them.")
        print("    sudo systemctl stop roversoftware-robot")
        return 1

    # The real guard, on a config built from the flags, so the tag below is the
    # decision the robot would make and not a second implementation of it.
    cfg = UltrasonicConfig(enabled=True, trig_pin=trig, echo_pin=echo,
                           stop_m=args.stop_m, slow_m=args.slow_m)
    guard = CollisionGuard(cfg, sonar.distance_m)
    forward = DriveCommand.tank(1.0, 1.0)

    print(f"Pinging TRIG=GPIO{trig} ECHO=GPIO{echo}. Ctrl-C to stop.")
    print(f"Guard: slow from {args.slow_m:.2f} m, stop at {args.stop_m:.2f} m.")
    print("Wave something in front of it — a distance that never appears is a "
          "sensor that\nis not wired, not a field with nothing in it.\n")

    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            raw, filtered = sonar.raw_m(), sonar.distance_m()
            pings, echoes = sonar.counts()
            limited = guard.apply(forward)
            line = (f"echoes {echoes:>5}/{pings:<5}  "
                    f"raw {_metres(raw)}   filtered {_metres(filtered)}   "
                    f"forward x{(limited.left + limited.right) / 2.0:4.2f}  "
                    f"[{guard.state.upper()}]")
            print(f"{line:<90}", end="\r", flush=True)
            sleep(period)
    except KeyboardInterrupt:
        pings, echoes = sonar.counts()
        print(f"\n\n{echoes} echoes from {pings} pings.")
        if echoes == 0:
            print("\nNot one echo. In order of likelihood:")
            print("  1. TRIG and ECHO are swapped, or on pins other than "
                  f"GPIO {trig}/{echo}.")
            print("  2. The module has no 5 V supply. An HC-SR04 needs 5 V to "
                  "transmit;")
            print("     it will sit there powered by nothing and never answer.")
            print("  3. ECHO is wired straight to a Pi GPIO. That pin returns "
                  "5 V, and a")
            print("     Pi is not 5 V tolerant — use the HAT's own ultrasonic "
                  "port, or a")
            print("     divider. A pin damaged this way reads nothing forever "
                  "after.")
            print("  4. Nothing has been within "
                  f"{args.max_m:.1f} m of it this whole time.")
        elif echoes < pings // 2:
            print("\nMost pings heard nothing. That is normal facing an open "
                  "room, and a\nproblem facing a wall — an angled or soft "
                  "surface bounces the ping away\nrather than back, which is "
                  "the obstacle class this sensor cannot see.")
    finally:
        sonar.stop()
    return 0


def _metres(value) -> str:
    """A distance, or a dash. A dash means NO ECHO, which is not zero."""
    return "  --  " if value is None else f"{value:5.2f}m"


if __name__ == "__main__":
    raise SystemExit(main())
