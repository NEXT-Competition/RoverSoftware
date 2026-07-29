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
    RS_DRIVE_HZ, RS_UI_HZ, RS_VIDEO_ENABLED, RS_VIDEO_PORT, RS_VIDEO_HZ,
    RS_BULK_IP, RS_BULK_PORT

...and finally the dashboard's own saved settings (RS_BASE_SETTINGS) are loaded
over the top: the gamepad mapping and the link/UI rates are editable from the
Settings page, and what the operator saved there should outrank a default that
was baked into a service file months ago. See basestation/settings.py.
"""

import argparse
import os
import time

import uvicorn

from basestation.app import build_app
from basestation.fleet import FleetManager
from basestation.places import PlaceStore
from basestation.settings import SettingsStore

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
    # Bulk transfers (config snapshots, layouts, routines) over WiFi instead of
    # the radio; see robot/comms/ip_link.py. Robots dial in, so this only needs
    # the port. Robots that aren't connected keep using the radio, so leaving it
    # on costs nothing.
    p.add_argument("--no-bulk-ip", dest="bulk_ip", action="store_false",
                   default=os.environ.get("RS_BULK_IP", "1").strip().lower()
                   in ("1", "true", "yes", "on"),
                   help="disable the WiFi bulk-transfer listener (everything on the radio)")
    p.add_argument("--bulk-port", type=int, default=int(_env("RS_BULK_PORT", 5006)),
                   help="TCP port robots connect to for config/layout/routine transfers")
    # ---- commanding in words (basestation/command/) ----
    # All optional. With no model server and no faster-whisper installed, the
    # command view still recognises stop, rover names, modes and the camera by
    # keyword — so none of these flags gate the base station starting.
    p.add_argument("--no-voice", dest="voice", action="store_false",
                   default=os.environ.get("RS_VOICE", "1").strip().lower()
                   in ("1", "true", "yes", "on"),
                   help="start with the command language model switched off "
                        "(the keyword fast path still works, and it can be "
                        "turned back on from the command view)")
    p.add_argument("--llm-url", default=_env("RS_LLM_URL", "http://127.0.0.1:1234/v1"),
                   help="OpenAI-compatible endpoint for the local command model "
                        "(LM Studio's default is http://127.0.0.1:1234/v1)")
    p.add_argument("--llm-model", default=_env("RS_LLM_MODEL", None),
                   help="model id to use; default is whichever loaded model "
                        "looks like a Gemma, else the first one")
    p.add_argument("--stt-model", default=_env("RS_STT_MODEL", "base.en"),
                   help="faster-whisper model for speech recognition "
                        "(tiny.en | base.en | small.en). base.en is the default "
                        "because tiny.en starts confusing rover numbers")
    args = p.parse_args()

    fleet = FleetManager()

    # CLI/env values are the baseline; anything the operator has since changed
    # from the dashboard is loaded over the top (basestation/settings.py).
    settings = SettingsStore(defaults={
        "base.drive_hz": args.drive_hz,
        "base.ui_hz": args.ui_hz,
        "base.video_hz": args.video_hz,
        "base.tiles": args.tiles,
    })

    # Named field positions ("bucket A", "start"), captured from the map. Owned
    # by the base station and shared by the whole fleet — see basestation/places.py.
    places = PlaceStore()

    def on_msg(msg):
        fleet.handle(msg, time.monotonic())

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
            controller = ControllerReader(hz=settings.base.controller_hz,
                                          mapping=settings.mapping())
        except Exception as e:
            print(f"[base] gamepad disabled: {e}")

    video_rx = None
    if args.video:
        from robot.comms.video_udp import VideoReceiver
        video_rx = VideoReceiver(port=args.video_port)
        print(f"[base] FPV video receiver on udp/{args.video_port}")

    # Config snapshots, layouts and routine documents come in over WiFi from any
    # robot that can reach us, and go back out the same way. Robots that can't
    # are unaffected — their transfers stay on the radio. Not started under
    # --sim: simulated robots are in-process and have no socket to dial in on.
    ip_server = None
    if args.bulk_ip and not args.sim:
        from robot.comms.ip_link import IPServer
        ip_server = IPServer(port=args.bulk_port, on_message=on_msg)

    # web_cfg now carries only what is fixed for the process's lifetime (the
    # tile cache is opened once); the live rates and the basemap URL come from
    # `settings`, which the dashboard can edit.
    # The local command model. Constructed unconditionally and never contacted
    # here: it is a separate application the operator may not have launched yet,
    # and the base station coming up must not depend on it. The first health
    # check happens in the background after startup (see build_app).
    from basestation.command.llm import LMStudio
    from basestation.command.stt import Transcriber
    llm = LMStudio(args.llm_url, model=args.llm_model)
    transcriber = Transcriber(model_name=args.stt_model)

    app = build_app(fleet, link, controller,
                    {"tiles_mbtiles": args.tiles_mbtiles,
                     "tiles_upstream": args.tiles_upstream,
                     "tiles_offline": args.tiles_offline,
                     "tiles": args.tiles,
                     "voice": args.voice},
                    video_rx=video_rx, settings=settings, ip_server=ip_server,
                    places=places, llm=llm, transcriber=transcriber)
    print(f"[base] dashboard -> http://{args.host}:{args.web_port}")
    print(f"[base] voice: model {args.llm_url} ({'on' if args.voice else 'off'}), "
          f"speech {args.stt_model}")
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
