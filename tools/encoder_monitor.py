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

Off-hardware (no `fusion_hat`) it says so and exits, so this is safe to run on a
laptop.
"""

import argparse
import os
import sys
import tempfile
from time import monotonic, sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/encoder_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.sensors.encoder import Encoder, backend


def _move_somewhere_writable() -> None:
    """Get out of a read-only directory before claiming any pins.

    On a Pi 5 the RPi.GPIO under fusion_hat is the lgpio shim, and lgpio puts
    its edge-notification FIFO in the CURRENT WORKING DIRECTORY. It cannot
    create one in a root-owned directory, and the failure surfaces much later
    as a flat "Failed to add edge detection" from add_event_detect — with
    nothing in it about directories. Setting the pin direction needs no such
    file, so the pins look claimable right up until the moment they aren't.

    The obvious reading of that message is that something else holds the pins,
    which sends you hunting a conflict that does not exist. Since the natural
    way to run this tool is `python3 encoder_monitor.py` from wherever it was
    installed — /opt/roversoftware/tools, owned by root — that is the common
    case, not the corner case. So move, rather than explain.
    """
    try:
        if os.access(os.getcwd(), os.W_OK):
            return
    except OSError:
        pass  # cwd deleted out from under us; the fallback handles it too
    os.chdir(tempfile.mkdtemp(prefix="encoder-monitor-"))


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

    _move_somewhere_writable()

    if backend() is None:
        # backend() has already printed what it could not open.
        print("\nThe encoder pins are read through the Fusion HAT library, the "
              "same one that drives the motors:")
        print("    just bootstrap        (or SunFounder's install.sh)")
        return 1

    a, b = args.pins
    # cpr defaults to 1 so `count` is still meaningful before it is measured —
    # RPM is then in "counts per minute", which is why it is only printed when a
    # real value was supplied.
    enc = Encoder(pin_a=a, pin_b=b, counts_per_rev=args.cpr or 1.0,
                  invert=args.invert, name="encoder", window=args.window)
    if not enc.start():
        # The library's own message is "Failed to add edge detection", which
        # names no cause. The backend appends the package hint above when it
        # applies; what is left is a genuine contest for the pins.
        print(f"\nIf the pins really are claimed by something else: the robot "
              f"service takes\nGPIO {a}/{b} at start-up whenever a layout or "
              f"robot.env names them.")
        print("    sudo systemctl stop roversoftware-robot")
        print("Otherwise check no second copy of this tool is still running.")
        return 1

    print(f"Reading GPIO {a} (A) / {b} (B). Ctrl-C to stop.")
    print("Turn the wheel one full revolution by hand and read `count`; that is "
          "the counts-per-rev to enter on the Hardware tab.\n")
    period = 1.0 / max(args.rate, 0.1)
    # Every distinct (A, B) pair we have ever seen. A healthy quadrature encoder
    # visits all four; how many are missing says exactly what is wrong, and a
    # count stuck at zero says nothing at all.
    seen = set()
    try:
        while True:
            enc.sample(monotonic())
            state = enc.levels()
            if state is not None:
                seen.add(state)
            line = f"count {enc.ticks:+8d}"
            if args.cpr > 0:
                rpm = enc.rpm() or 0.0
                line += f"   {enc.ticks / args.cpr:+7.2f} rev   {rpm:+8.1f} rpm"
            if state is not None:
                line += f"   A{state[0]} B{state[1]}   states {len(seen)}/4"
            if enc.missed:
                # Not a warning until it moves: a single miss at start-up is the
                # decoder finding its footing. A number that CLIMBS is real.
                line += f"   missed {enc.missed}"
            print(line, end="\r", flush=True)
            sleep(period)
    except KeyboardInterrupt:
        print(f"\n\nfinal count: {enc.ticks}")
        _explain_states(seen, a, b)
        if enc.ticks and args.cpr <= 0:
            print("If you turned the wheel exactly one revolution, that IS your "
                  "counts-per-rev.")
        if enc.missed:
            print(f"{enc.missed} transitions could not be decoded. A handful is "
                  "noise; a lot means the edges are arriving faster than the Pi "
                  "can service them, or a channel is floating.")
    finally:
        enc.stop()
    return 0


def _explain_states(seen: set, pin_a: int, pin_b: int) -> None:
    """Turn "which of the four states did we see" into the actual fault.

    A count that never moves is the least informative symptom in this whole
    stack — still wheel, unpowered encoder, wrong pins and a dead channel all
    look identical. The levels tell them apart, so say so rather than making
    someone infer it.
    """
    if len(seen) >= 4:
        return
    moved = {
        "a": len({s[0] for s in seen}) > 1,
        "b": len({s[1] for s in seen}) > 1,
    }
    if not seen:
        return
    if not moved["a"] and not moved["b"]:
        held = next(iter(seen))
        print(f"\nNEITHER channel ever changed — both sat at A{held[0]} B{held[1]}.")
        print("Nothing was counted because nothing arrived. In order:")
        print("  1. The encoder has no power. A Hall-effect encoder is an active")
        print("     device: its 4-pin connector is ground, supply, A and B, and")
        print("     with the supply missing the outputs never drive. Both pins")
        print("     reading 1 is exactly what our internal pull-ups do on their own.")
        print("  2. Power it from 3.3 V, NOT 5 V. The encoder accepts either, but a")
        print("     Pi GPIO is not 5 V tolerant and A/B would be driven at 5 V.")
        print(f"  3. A and B are on pins other than GPIO {pin_a}/{pin_b}.")
        print("  4. The gearbox is not back-driving, so the shaft never turned.")
        return
    if moved["a"] != moved["b"]:
        dead, pin = ("B", pin_b) if moved["a"] else ("A", pin_a)
        print(f"\nOnly one channel moved: {dead} (GPIO {pin}) never changed.")
        print("One live channel and one dead one gives a count near zero, because")
        print("without a phase relationship every transition looks like a missed")
        print(f"one. Check that wire and its pin. States seen: {sorted(seen)}")
        return
    print(f"\nBoth channels moved but only {len(seen)} of 4 states appeared: "
          f"{sorted(seen)}.")
    print("Turn the wheel further — a full revolution visits all four many times.")


if __name__ == "__main__":
    raise SystemExit(main())
