"""FastAPI base-station bridge: browser <-> radio over a WebSocket, plus tiles.

The dashboard UI is served separately by the Deno front door (roversoftware-ui),
which reverse-proxies /ws and /tiles to this process. This app serves no HTML.


Wiring:
    link (XBee or simulator)  --telemetry-->  FleetManager
    browser  --WebSocket actions-->  dispatch()  --commands-->  link
    controller thread  --on_drive/on_action-->  selected robot  --> link
    broadcaster task  --fleet snapshot @10Hz-->  all browsers

Commands to robots are addressed with a "to" field so one radio channel serves
the whole fleet.
"""

from __future__ import annotations

import asyncio
import time
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse

from .fleet import FleetManager
from .tiles import TileStore, attribution_for, content_type


def build_app(fleet: FleetManager, link, controller, web_cfg: dict, video_rx=None) -> FastAPI:
    app = FastAPI(title="RoverSoftware base station")
    clients: Set[WebSocket] = set()

    # Offline map tiles: serves /tiles/{z}/{x}/{y}.png from a local .mbtiles cache,
    # optionally filling misses from an upstream server. Constructed even when no
    # cache is configured (it just proxies upstream, or returns blanks offline).
    tile_store = TileStore(
        mbtiles_path=web_cfg.get("tiles_mbtiles"),
        upstream_url=web_cfg.get("tiles_upstream"),
        allow_upstream=not web_cfg.get("tiles_offline", False),
    )
    # The browser usually loads tiles from our /tiles/... proxy, so credit the
    # source we actually fetch from (falling back to the URL the browser loads
    # when tiles come straight off a public server).
    tiles_attribution = (attribution_for(web_cfg.get("tiles_upstream"))
                         or attribution_for(web_cfg.get("tiles")))

    def dispatch(robot_id, msg: dict) -> None:
        if robot_id:
            link.send({**msg, "to": robot_id})

    # ---- gamepad -> currently selected robot ----
    # Rate-limit drive frames so we don't flood a slow XBee link (at 9600 baud a
    # 20 Hz stream backs the serial buffer up and latency grows without bound).
    # Send when the command meaningfully changes, capped at DRIVE_MAX_HZ, plus a
    # periodic keepalive so the robot's command_timeout failsafe doesn't trip
    # while the stick is held steady.
    DRIVE_EPS = 0.01
    DRIVE_MIN_INTERVAL = 1.0 / float(web_cfg.get("drive_hz", 30))
    DRIVE_KEEPALIVE = 0.25  # seconds; must stay below the robot's command_timeout
    _drive = {"throttle": None, "steer": None, "t": -1.0}

    def on_drive(throttle: float, steer: float) -> None:
        rid = fleet.selected
        if not rid:
            return
        now = time.monotonic()
        dt = now - _drive["t"]
        changed = (_drive["throttle"] is None
                   or abs(throttle - _drive["throttle"]) > DRIVE_EPS
                   or abs(steer - _drive["steer"]) > DRIVE_EPS)
        if (changed and dt >= DRIVE_MIN_INTERVAL) or dt >= DRIVE_KEEPALIVE:
            _drive.update(throttle=throttle, steer=steer, t=now)
            dispatch(rid, {"type": "drive", "throttle": round(throttle, 3), "steer": round(steer, 3)})

    def on_action(name: str) -> None:
        rid = fleet.selected
        if name == "estop":
            dispatch(rid, {"type": "estop"})
        elif name == "clear":
            dispatch(rid, {"type": "clear_estop"})
        elif name.startswith("mode:"):
            dispatch(rid, {"type": "mode", "mode": name.split(":", 1)[1]})

    if controller is not None:
        controller.on_drive = on_drive
        controller.on_action = on_action

    # ---- browser actions -> robots ----
    def handle_action(data: dict) -> None:
        action = data.get("action")
        rid = data.get("robot_id") or fleet.selected
        if action == "select":
            fleet.select(data.get("robot_id"))
        elif action == "mode":
            dispatch(rid, {"type": "mode", "mode": data.get("mode", "teleop")})
        elif action == "estop":
            dispatch(rid, {"type": "estop"})
        elif action == "clear_estop":
            dispatch(rid, {"type": "clear_estop"})
        elif action == "route":
            dispatch(rid, {"type": "route", "waypoints": data.get("waypoints", [])})
        elif action == "drive":
            dispatch(rid, {"type": "drive",
                           "throttle": float(data.get("throttle", 0)),
                           "steer": float(data.get("steer", 0))})
        elif action in ("arm_shooter", "disarm_shooter", "fire"):
            # Pass-through: the robot owns every firing rule (arm latch, dwell,
            # cooldown, magazine). Duplicating any of that here would give two
            # sources of truth for when it's safe to shoot, and the base station
            # is the one that can be out of date or disconnected.
            dispatch(rid, {"type": action})

    async def broadcast_loop() -> None:
        ui_period = 1.0 / float(web_cfg.get("ui_hz", 30))
        try:
            while True:
                snap = fleet.snapshot(time.monotonic())
                snap["controller"] = {
                    "connected": getattr(controller, "connected", False) if controller else False,
                    "name": getattr(controller, "name", None) if controller else None,
                }
                snap["tiles"] = web_cfg.get("tiles")
                snap["tiles_maxzoom"] = tile_store.maxzoom
                snap["tiles_attribution"] = tiles_attribution
                # Which robots currently have a live feed, so the UI shows the
                # FPV panel only when there's actually something to show.
                snap["video"] = video_rx.robots() if video_rx is not None else []
                for ws in list(clients):
                    try:
                        await ws.send_json(snap)
                    except Exception:
                        clients.discard(ws)
                await asyncio.sleep(ui_period)
        except asyncio.CancelledError:
            pass

    @app.on_event("startup")
    async def _startup():
        link.start()
        if controller is not None:
            controller.start()
        if video_rx is not None:
            video_rx.start()
        app.state.broadcaster = asyncio.create_task(broadcast_loop())

    @app.on_event("shutdown")
    async def _shutdown():
        task = getattr(app.state, "broadcaster", None)
        if task:
            task.cancel()
        if controller is not None:
            controller.stop()
        if video_rx is not None:
            video_rx.stop()
        link.stop()

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        clients.add(ws)
        try:
            while True:
                data = await ws.receive_json()
                handle_action(data)
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(ws)

    @app.get("/tiles/{z}/{x}/{y}.png")
    async def tiles(z: int, x: int, y: int):
        # Offload the (possibly blocking) DB read + upstream fetch to a thread.
        data = await asyncio.to_thread(tile_store.get, z, x, y)
        if data is None:
            return Response(status_code=204)  # uncached + offline -> blank tile
        # The .png in the route is Leaflet's template, not a promise: satellite
        # imagery comes back as JPEG, so label each tile by what it really is.
        return Response(content=data, media_type=content_type(data),
                        headers={"Cache-Control": "public, max-age=86400"})

    @app.get("/video/{robot_id}.mjpg")
    async def video(robot_id: str):
        # MJPEG (multipart/x-mixed-replace) is browser-native in an <img> and low
        # latency. We just relay the freshest JPEG the UDP receiver has for this
        # robot; missed/partial frames were already dropped upstream.
        if video_rx is None:
            return Response(status_code=404)
        period = 1.0 / max(float(web_cfg.get("video_hz", 20)), 1.0)

        async def frames():
            try:
                while True:
                    jpeg = video_rx.latest(robot_id)
                    if jpeg is not None:
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                               + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    await asyncio.sleep(period)
            except asyncio.CancelledError:  # client closed the stream
                pass

        return StreamingResponse(
            frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/")
    async def root():
        # The dashboard is served by the Deno touch UI (roversoftware-ui), which
        # reverse-proxies /ws and /tiles here. This process is purely the bridge
        # — no HTML/static assets. Handy hint if someone hits the bridge port.
        return Response(
            "roversoftware bridge — WebSocket at /ws, map tiles at /tiles/{z}/{x}/{y}.png.\n"
            "The dashboard is served by the roversoftware-ui service.\n",
            media_type="text/plain",
        )

    return app
