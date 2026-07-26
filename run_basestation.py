#!/usr/bin/env python3
"""Launch the base-station dashboard (run on your Mac or a Raspberry Pi).

    # Full simulation - no hardware needed, great for development:
    python run_basestation.py --sim
    #   then open http://127.0.0.1:8000

    # Talk to real robots over the XBee radio:
    python run_basestation.py --port /dev/tty.usbserial-XXXX --baud 57600

Defaults come from the environment first (so the systemd service can be
configured via /etc/roversoftware/basestation.env), then CLI flags override:

    RS_XBEE_PORT, RS_XBEE_BAUD, RS_WEB_HOST, RS_WEB_PORT, RS_SIM,
    RS_SIM_ROBOTS, RS_SIM_ORIGIN, RS_NO_CONTROLLER, RS_TILES,
    RS_TILES_MBTILES, RS_TILES_UPSTREAM, RS_TILES_OFFLINE,
    RS_DRIVE_HZ, RS_UI_HZ, RS_VIDEO_ENABLED, RS_VIDEO_PORT, RS_VIDEO_HZ
"""

import argparse
import os
import time

import uvicorn

from basestation.app import build_app
from basestation.fleet import FleetManager

# Aerial/satellite imagery is the useful basemap for driving a rover: you steer
# by the terrain you can actually see (grass, gravel, tree lines), not by street
# names. Esri World Imagery needs no API key, so this works on a fresh clone.
# Tiles are JPEG, which the whole /tiles path handles (see basestation/tiles.py).
# Swap in another provider with --tiles-upstream / RS_TILES_UPSTREAM, e.g.
# MapTiler satellite: https://api.maptiler.com/tiles/satellite-v2/{z}/{x}/{y}.jpg?key=KEY
SATELLITE_TILES = (
    "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Imagery/MapServer/tile/{z}/{y}/{x}"
)


def _env(name, default):
    return os.environ.get(name, default)


def _envbool(name):
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def main():
    p = argparse.ArgumentParser(description="RoverSoftware base station")
    p.add_argument("--port", default=_env("RS_XBEE_PORT", None),
                   help="XBee serial port for real robots (e.g. /dev/ttyUSB0)")
    # Matches robot/config.py CommsConfig.baud. These two MUST agree: the link
    # still passes traffic when they don't, but the slower side's serial buffer
    # backs up and command latency grows without bound while you drive.
    p.add_argument("--baud", type=int, default=int(_env("RS_XBEE_BAUD", 57600)))
    p.add_argument("--sim", action="store_true", default=_envbool("RS_SIM"),
                   help="run the built-in simulator with fake robots instead of a radio")
    p.add_argument("--robots", type=int, default=int(_env("RS_SIM_ROBOTS", 3)),
                   help="number of simulated robots (with --sim)")
    p.add_argument("--origin", default=_env("RS_SIM_ORIGIN", "37.7749,-122.4194"),
                   help="simulator origin 'lat,lon' (with --sim)")
    p.add_argument("--host", default=_env("RS_WEB_HOST", "127.0.0.1"))
    p.add_argument("--web-port", type=int, default=int(_env("RS_WEB_PORT", 8000)))
    p.add_argument("--no-controller", action="store_true", default=_envbool("RS_NO_CONTROLLER"),
                   help="skip gamepad input (touch-only base station)")
    p.add_argument("--tiles", default=_env("RS_TILES", SATELLITE_TILES),
                   help="map tile URL the browser loads; set to /tiles/{z}/{x}/{y}.png to serve offline")
    p.add_argument("--tiles-mbtiles", default=_env("RS_TILES_MBTILES", None),
                   help="offline .mbtiles cache served at /tiles/... (build with tools/fetch_tiles.py)")
    p.add_argument("--tiles-upstream",
                   default=_env("RS_TILES_UPSTREAM", SATELLITE_TILES),
                   help="online source used to fill /tiles cache misses (unless --tiles-offline)")
    p.add_argument("--tiles-offline", action="store_true", default=_envbool("RS_TILES_OFFLINE"),
                   help="never fetch missing tiles online; serve only what's cached")
    # An airtime budget, not a feel knob. A drive frame is ~62 B ≈ 11 ms at
    # 57600; 5 Hz of telemetry already costs ~26% of a half-duplex channel, so
    # 15 Hz lands near 40% utilisation with headroom for retries. The old
    # default of 30 oversubscribed a 9600 link 2x — which is what "laggy steering
    # that gets worse the longer you hold the stick" actually was.
    p.add_argument("--drive-hz", type=float, default=float(_env("RS_DRIVE_HZ", 15)),
                   help="max drive-command send rate over the radio (lower for slow/9600 links)")
    p.add_argument("--ui-hz", type=float, default=float(_env("RS_UI_HZ", 30)),
                   help="dashboard refresh rate pushed to the browser")
    p.add_argument("--no-video", dest="video", action="store_false",
                   default=os.environ.get("RS_VIDEO_ENABLED", "1").strip().lower()
                   in ("1", "true", "yes", "on"),
                   help="disable the FPV video receiver (frees the UDP port)")
    p.add_argument("--video-port", type=int, default=int(_env("RS_VIDEO_PORT", 5005)),
                   help="UDP port the robots stream FPV video to")
    p.add_argument("--video-hz", type=float, default=float(_env("RS_VIDEO_HZ", 20)),
                   help="max MJPEG frame rate served to browsers")
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

    video_rx = None
    if args.video:
        from robot.comms.video_udp import VideoReceiver
        video_rx = VideoReceiver(port=args.video_port)
        print(f"[base] FPV video receiver on udp/{args.video_port}")

    app = build_app(fleet, link, controller,
                    {"tiles": args.tiles, "drive_hz": args.drive_hz, "ui_hz": args.ui_hz,
                     "tiles_mbtiles": args.tiles_mbtiles, "tiles_upstream": args.tiles_upstream,
                     "tiles_offline": args.tiles_offline, "video_hz": args.video_hz},
                    video_rx=video_rx)
    print(f"[base] dashboard -> http://{args.host}:{args.web_port}")
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
