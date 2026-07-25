#!/usr/bin/env python3
"""Live Adafruit GPS fix monitor — GPS bring-up.

Use this to confirm the receiver is wired, fixed, and reporting a usable track
angle before trusting it for navigation:

    python tools/gps_monitor.py
    python tools/gps_monitor.py --port /dev/ttyAMA0 --baud 9600 --rate-ms 200

What to look for, in order:

  1. "waiting for fix" for the first minute or two outdoors is normal (cold
     start). Indoors it will never fix — take it outside with a clear sky view.
     Nothing at all after several minutes usually means the wrong port, the wrong
     baud, or TX/RX swapped.
  2. sats and hdop are the fix-quality numbers: 6+ satellites and HDOP under ~2
     is a good fix. 3 satellites and HDOP of 12 will wander tens of metres and
     will steer the rover badly.
  3. TRACK ANGLE is the heading the rover navigates on when there's no IMU, and
     it only exists while MOVING. Standing still it reads "--"; walk or drive the
     rover in a straight line and it should settle within a few degrees of the
     direction of travel. If it never appears, you're below --min-move.

Off-hardware (no adafruit-circuitpython-gps / no serial device): prints a clear
note and exits, so this is safe to run on a laptop.
"""

import argparse
import os
import sys
from time import sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/gps_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.sensors.gps import GPS, adafruit_gps, serial


def main():
    p = argparse.ArgumentParser(description="Adafruit GPS live fix monitor")
    p.add_argument("--port", default="/dev/ttyAMA0", help="serial port (Pi UART)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--rate-ms", type=int, default=1000,
                   help="ms between fixes (PMTK220); below ~200 won't fit 9600 baud")
    p.add_argument("--min-move", type=float, default=0.5,
                   help="m/s below which the track angle is treated as noise")
    p.add_argument("--rate", type=float, default=2.0, help="print rate (Hz)")
    args = p.parse_args()

    if adafruit_gps is None or serial is None:
        print("adafruit_gps/pyserial not found — install on the Pi: "
              "pip install pyserial adafruit-circuitpython-gps\n"
              "(nothing to monitor on a dev laptop without the GPS libraries)")
        return

    gps = GPS(args.port, args.baud, min_move_mps=args.min_move,
              update_rate_ms=args.rate_ms)
    gps.start()
    if not gps.is_running():  # start() already printed why (bad port / missing dep)
        return
    print("Drive or walk the rover in a straight line to see the track angle. "
          "Ctrl-C to stop.\n")
    period = 1.0 / args.rate if args.rate > 0 else 0.5
    try:
        while True:
            fix = gps.pose()
            t = gps.telemetry()
            if fix is None:
                print(f"waiting for fix...  sats={t['sats']} "
                      f"hdop={t.get('hdop', '--')}")
            else:
                lat, lon, heading = fix
                h = f"{heading:6.1f}°" if heading is not None else "  --   (not moving)"
                print(f"{lat:11.6f}, {lon:12.6f}  track={h}  "
                      f"speed={t['speed']:5.2f} m/s  sats={t['sats']} "
                      f"hdop={t.get('hdop', '--')} alt={t.get('alt', '--')} "
                      f"fix={t['fix']}")
            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        gps.stop()


if __name__ == "__main__":
    main()
