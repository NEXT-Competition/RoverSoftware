"""FastAPI base-station app: serves the dashboard, bridges browser <-> radio.

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
from pathlib import Path
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from .fleet import FleetManager
from .tiles import TileStore

STATIC_DIR = Path(__file__).parent / "static"


def build_app(fleet: FleetManager, link, controller, web_cfg: dict) -> FastAPI:
    app = FastAPI(title="uc-chassis base station")
    clients: Set[WebSocket] = set()

    # Offline map tiles: serves /tiles/{z}/{x}/{y}.png from a local .mbtiles cache,
    # optionally filling misses from an upstream server. Constructed even when no
    # cache is configured (it just proxies upstream, or returns blanks offline).
    tile_store = TileStore(
        mbtiles_path=web_cfg.get("tiles_mbtiles"),
        upstream_url=web_cfg.get("tiles_upstream"),
        allow_upstream=not web_cfg.get("tiles_offline", False),
    )

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
        app.state.broadcaster = asyncio.create_task(broadcast_loop())

    @app.on_event("shutdown")
    async def _shutdown():
        task = getattr(app.state, "broadcaster", None)
        if task:
            task.cancel()
        if controller is not None:
            controller.stop()
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
        return Response(content=data, media_type="image/png",
                        headers={"Cache-Control": "public, max-age=86400"})

    @app.get("/")
    async def index():
        return FileResponse(STATIC_DIR / "index.html")

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    return app
