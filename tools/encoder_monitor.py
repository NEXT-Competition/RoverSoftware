#!/usr/bin/env python3
"""Live wheel-encoder monitor — encoder bring-up and counts-per-rev calibration.

Two jobs, and the second is the one you cannot skip.

    1. PROVE THE WIRING. Turn a wheel by hand and watch the count move. Forward
       should count UP; if it counts down, that is `encoder_invert` on the
       Hardware tab, not a wiring fault. If it counts erratically in both
       directions, the two channels are swapped or a pull-up is missing.

    2. MEASURE counts_per_rev. Nothing else can do this for you: the number on
       the encoder is cycles per revolution of the MOTOR, this decoder counts
       four edges per cycle, and the gearbox ratio between the motor and the
       wheel is frequently not the one printed on the gearbox. So:

           python tools/encoder_monitor.py --pins 17,27
           (press z to zero, turn the wheel exactly one full turn, read `count`)

       That number goes in Hardware -> the actuator -> Counts per rev.

Reads the pins directly rather than through a RobotConfig, so it works before
any layout has been saved — and it does NOT touch the motors, so it is safe to
run with the drivetrain powered down and the wheels turned by hand. Which is how
you should do it: a wheel spun by a motor is harder to stop at exactly one turn.

Off-hardware (no pigpio, no lgpio, no pigpiod running) it says which one is
missing and exits, so this is safe to run on a laptop.
"""

import argparse
import os
import sys
from time import monotonic, sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/encoder_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.sensors.encoder import Encoder, backend


def _parse_pins(text: str) -> tuple:
    try:
        a, b = (int(part.strip()) for part in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(
            "expected two BCM pin numbers, e.g. --pins 17,27") from None
    if a == b:
        raise argparse.ArgumentTypeError(
            "A and B must be different pins — a quadrature encoder needs two "
            "phases to have a phase relationship")
    return a, b


def main():
    p = argparse.ArgumentParser(
        description="Quadrature wheel-encoder monitor and counts-per-rev calibration")
    p.add_argument("--pins", type=_parse_pins, default=(17, 27), metavar="A,B",
                   help="BCM GPIO pins for channels A and B (default 17,27)")
    p.add_argument("--cpr", type=float, default=0.0,
                   help="counts per wheel revolution, if you already know it — "
                        "with it, RPM and revolutions are printed too")
    p.add_argument("--invert", action="store_true",
                   help="flip the counting direction")
    p.add_argument("--rate", type=float, default=5.0, help="print rate (Hz)")
    p.add_argument("--window", type=float, default=0.1,
                   help="speed measurement window in seconds (see the settings page)")
    args = p.parse_args()

    if backend() is None:
        # backend() has already printed which libraries it looked for.
        print("\nOn a Pi 4 or older:  pip install pigpio && "
              "sudo systemctl enable --now pigpiod")
        print("On a Pi 5:           pip install lgpio")
        return 1

    a, b = args.pins
    # cpr defaults to 1 so `count` is still meaningful before it is measured —
    # RPM is then in "counts per minute", which is why it is only printed when a
    # real value was supplied.
    enc = Encoder(pin_a=a, pin_b=b, counts_per_rev=args.cpr or 1.0,
                  invert=args.invert, name="encoder", window=args.window)
    if not enc.start():
        print("[encoder] could not claim the pins — is something else using them?")
        return 1

    print(f"Reading GPIO {a} (A) / {b} (B). Ctrl-C to stop.")
    print("Turn the wheel one full revolution by hand and read `count`; that is "
          "the counts-per-rev to enter on the Hardware tab.\n")
    period = 1.0 / max(args.rate, 0.1)
    try:
        while True:
            enc.sample(monotonic())
            line = f"count {enc.ticks:+8d}"
            if args.cpr > 0:
                rpm = enc.rpm() or 0.0
                line += f"   {enc.ticks / args.cpr:+7.2f} rev   {rpm:+8.1f} rpm"
            if enc.missed:
                # Not a warning until it moves: a single miss at start-up is the
                # decoder finding its footing. A number that CLIMBS is real.
                line += f"   missed {enc.missed}"
            print(line, end="\r", flush=True)
            sleep(period)
    except KeyboardInterrupt:
        print(f"\n\nfinal count: {enc.ticks}")
        if args.cpr <= 0:
            print("If you turned the wheel exactly one revolution, that IS your "
                  "counts-per-rev.")
        if enc.missed:
            print(f"{enc.missed} transitions could not be decoded. A handful is "
                  "noise; a lot means the edges are arriving faster than the Pi "
                  "can service them, or a channel is floating.")
    finally:
        enc.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
