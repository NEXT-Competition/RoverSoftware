#!/usr/bin/env python3
"""Launch the base-station dashboard (run on your Mac or a Raspberry Pi).

    # Full simulation - no hardware needed, great for development:
    python run_basestation.py --sim
    #   then open http://127.0.0.1:8000

    # Talk to real robots over the XBee radio:
    python run_basestation.py --port /dev/tty.usbserial-XXXX --baud 57600

Defaults come from the environment first (so the systemd service can be
configured via /etc/roversoftware/basestation.env), then CLI flags override:

    UC_XBEE_PORT, UC_XBEE_BAUD, UC_WEB_HOST, UC_WEB_PORT, UC_SIM,
    UC_SIM_ROBOTS, UC_SIM_ORIGIN, UC_NO_CONTROLLER, UC_TILES,
    UC_TILES_MBTILES, UC_TILES_UPSTREAM, UC_TILES_OFFLINE,
    UC_DRIVE_HZ, UC_UI_HZ
"""

import argparse
import os
import time

import uvicorn

from basestation.app import build_app
from basestation.fleet import FleetManager


def _env(name, default):
    return os.environ.get(name, default)


def _envbool(name):
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def main():
    p = argparse.ArgumentParser(description="RoverSoftware base station")
    p.add_argument("--port", default=_env("UC_XBEE_PORT", None),
                   help="XBee serial port for real robots (e.g. /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=int(_env("UC_XBEE_BAUD", 9600)))
    p.add_argument("--sim", action="store_true", default=_envbool("UC_SIM"),
                   help="run the built-in simulator with fake robots instead of a radio")
    p.add_argument("--robots", type=int, default=int(_env("UC_SIM_ROBOTS", 3)),
                   help="number of simulated robots (with --sim)")
    p.add_argument("--origin", default=_env("UC_SIM_ORIGIN", "37.7749,-122.4194"),
                   help="simulator origin 'lat,lon' (with --sim)")
    p.add_argument("--host", default=_env("UC_WEB_HOST", "127.0.0.1"))
    p.add_argument("--web-port", type=int, default=int(_env("UC_WEB_PORT", 8000)))
    p.add_argument("--no-controller", action="store_true", default=_envbool("UC_NO_CONTROLLER"),
                   help="skip gamepad input (touch-only base station)")
    p.add_argument("--tiles", default=_env("UC_TILES", "https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
                   help="map tile URL the browser loads; set to /tiles/{z}/{x}/{y}.png to serve offline")
    p.add_argument("--tiles-mbtiles", default=_env("UC_TILES_MBTILES", None),
                   help="offline .mbtiles cache served at /tiles/... (build with tools/fetch_tiles.py)")
    p.add_argument("--tiles-upstream",
                   default=_env("UC_TILES_UPSTREAM", "https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
                   help="online source used to fill /tiles cache misses (unless --tiles-offline)")
    p.add_argument("--tiles-offline", action="store_true", default=_envbool("UC_TILES_OFFLINE"),
                   help="never fetch missing tiles online; serve only what's cached")
    p.add_argument("--drive-hz", type=float, default=float(_env("UC_DRIVE_HZ", 30)),
                   help="max drive-command send rate over the radio (lower for slow/9600 links)")
    p.add_argument("--ui-hz", type=float, default=float(_env("UC_UI_HZ", 30)),
                   help="dashboard refresh rate pushed to the browser")
    args = p.parse_args()

    fleet = FleetManager()

    def on_msg(msg):
        fleet.update_from_telemetry(msg, time.monotonic())

    if args.sim:
        from basestation.simulator import SimulatedFleet
        lat, lon = (float(x) for x in args.origin.split(","))
        link = SimulatedFleet(on_msg, n_robots=args.robots, origin=(lat, lon))
        print(f"[base] SIMULATOR: {args.robots} fake robots @ {lat},{lon}")
    elif args.port:
        from robot.comms.xbee_link import XBeeLink
        link = XBeeLink(args.port, args.baud, on_msg)
        print(f"[base] XBee link on {args.port} @ {args.baud}")
    else:
        p.error("no data source: pass --port <serial> to talk to real robots, "
                "or --sim to run the simulator.")

    controller = None
    if not args.no_controller:
        try:
            from basestation.controller_input import ControllerReader
            controller = ControllerReader()
        except Exception as e:
            print(f"[base] gamepad disabled: {e}")

    app = build_app(fleet, link, controller,
                    {"tiles": args.tiles, "drive_hz": args.drive_hz, "ui_hz": args.ui_hz,
                     "tiles_mbtiles": args.tiles_mbtiles, "tiles_upstream": args.tiles_upstream,
                     "tiles_offline": args.tiles_offline})
    print(f"[base] dashboard -> http://{args.host}:{args.web_port}")
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
