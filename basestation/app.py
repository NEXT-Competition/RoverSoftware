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

--- Two outbound channels, not one ---
`{"type":"fleet"}` is the hot path: small, at ui_hz, and it carries only what
moves. `{"type":"settings"}` is the cold one: the base station's own settings,
the gamepad layout, and each robot's ~2.4 KB tunable config, sent on connect and
then only when something actually changes. Restating a config nobody is looking
at 30 times a second would be the single largest thing on this socket.
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse

from robot.comms.doc_transfer import split

from .fleet import FleetManager
from .places import PlaceStore
from .settings import SettingsStore
from .tiles import TileStore, attribution_for, content_type

# Document fragments handed to the radio per broadcast cycle (30 Hz by default,
# so ~60 frames a second of headroom). Sized to empty a layout in well under a
# second without ever giving the link more than it can write.
DOC_FRAMES_PER_CYCLE = 2


def build_app(fleet: FleetManager, link, controller, web_cfg: dict, video_rx=None,
              settings: SettingsStore | None = None, ip_server=None) -> FastAPI:
    app = FastAPI(title="RoverSoftware base station")
    clients: Set[WebSocket] = set()
    # Clients with the gamepad mapping editor open (see watch_gamepad).
    watchers: Set[WebSocket] = set()
    # Editable base-station settings. Falls back to a non-persisting store built
    # from web_cfg so an embedder that doesn't pass one still gets a working app
    # (and the tests don't write to a developer's home directory).
    settings = settings or SettingsStore(
        defaults={f"base.{k}": v for k, v in web_cfg.items()
                  if k in ("drive_hz", "ui_hz", "video_hz", "tiles")},
        load=False,
    )
    # Named field positions, fleet-wide (basestation/places.py). Same fallback
    # rule as settings: an embedder that passes nothing still gets a working
    # app, and the tests don't write to a developer's home directory.
    places = places or PlaceStore(load=False)

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

    def dispatch_bulk(robot_id, msg: dict) -> bool:
        """Send a bulk frame over WiFi if that robot is on it; else the radio.

        Returns True when WiFi took it, which is also the signal that there is
        no airtime to pace against — see drain_documents. A robot at the far end
        of the field simply isn't in `ip_server`, so it keeps getting documents
        over the radio with no special handling here.
        """
        if not robot_id:
            return False
        if ip_server is not None and ip_server.send({**msg, "to": robot_id}):
            return True
        dispatch(robot_id, msg)
        return False

    # Outbound document fragments, paced rather than dispatched in a loop. The
    # robot's own outbox does this in the other direction and for the same
    # reason: XBeeLink drops a frame the radio isn't draining, so a 10-fragment
    # layout fired at once is the shape that arrives incomplete.
    _doc_queue: "deque[tuple]" = deque()
    _txid = {"n": 0}

    def send_document(robot_id, action: str, doc: dict, save: bool) -> None:
        if not robot_id:
            return
        _txid["n"] += 1
        mtype = "put_layout" if action == "set_layout" else "put_routines"
        for frame in split(doc, mtype, txid=f"B{_txid['n']}", save=save):
            _doc_queue.append((robot_id, frame))

    async def drain_documents() -> None:
        # The pacing exists to protect radio airtime. A fragment that went over
        # WiFi cost none, so it doesn't count against the budget and the next
        # one goes immediately — a layout push over WiFi completes in one cycle
        # instead of being metered out over a second.
        budget = DOC_FRAMES_PER_CYCLE
        while _doc_queue and budget > 0:
            robot_id, frame = _doc_queue.popleft()
            if not dispatch_bulk(robot_id, frame):
                budget -= 1

    # ---- gamepad -> currently selected robot ----
    # Rate-limit drive frames so we don't flood a slow XBee link (at 9600 baud a
    # 20 Hz stream backs the serial buffer up and latency grows without bound).
    # Send when the command meaningfully changes, capped at DRIVE_MAX_HZ, plus a
    # periodic keepalive so the robot's command_timeout failsafe doesn't trip
    # while the stick is held steady.
    DRIVE_EPS = 0.01
    # Read from settings on every frame, not captured once: --drive-hz is now
    # also a dashboard slider, and a rate that took a service restart to change
    # is exactly the knob nobody adjusts when the link is struggling.
    DRIVE_KEEPALIVE = 0.25  # seconds; must stay below the robot's command_timeout
    _drive = {"throttle": None, "steer": None, "t": -1.0}

    def on_drive(throttle: float, steer: float) -> None:
        rid = fleet.selected
        if not rid:
            return
        now = time.monotonic()
        dt = now - _drive["t"]
        min_interval = 1.0 / max(settings.base.drive_hz, 1.0)
        changed = (_drive["throttle"] is None
                   or abs(throttle - _drive["throttle"]) > DRIVE_EPS
                   or abs(steer - _drive["steer"]) > DRIVE_EPS)
        if (changed and dt >= min_interval) or dt >= DRIVE_KEEPALIVE:
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
        elif name in ("arm_shooter", "disarm_shooter", "fire"):
            # A bindable button reaches the same pass-through the on-screen
            # controls use; the robot still owns every firing rule.
            dispatch(rid, {"type": name})

    if controller is not None:
        controller.on_drive = on_drive
        controller.on_action = on_action
        controller.set_mapping(settings.mapping())

    # ---- settings changes -> the things that cached them ----
    # Marked dirty so the broadcaster pushes one settings frame instead of one
    # per edited field: a slider drag is dozens of updates a second.
    _settings_dirty = {"v": True}
    # Outcome of the last set_settings, echoed once so the page can show what
    # was clamped or refused. Robot config results ride on the robot's own
    # entry (fleet.configs); this is the base station's equivalent.
    _settings_result: dict = {"v": None}
    # Same idea for the places list, so a refused coordinate is reported rather
    # than silently dropped from the map.
    _places_result: dict = {"v": None}

    def on_settings_change(applied: dict) -> None:
        if controller is not None and any(p.startswith("controller.") for p in applied):
            controller.set_mapping(settings.mapping())
        if "base.trail_max" in applied:
            fleet.trail_max = settings.base.trail_max
        _settings_dirty["v"] = True

    settings.on_change = on_settings_change
    fleet.trail_max = settings.base.trail_max

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
        # ---- configuration ----
        elif action == "get_config":
            # Explicit, never polled: the reply is ~2.4 KB and the radio is
            # shared with telemetry. The settings page asks once when opened.
            dispatch(rid, {"type": "get_config"})
        elif action == "set_config":
            values = data.get("config")
            if isinstance(values, dict) and values:
                dispatch(rid, {"type": "set_config", "config": values,
                               "save": bool(data.get("save", True))})
        # ---- documents (hardware layout, FSM routines) ----
        elif action in ("get_layout", "get_routines", "get_fields"):
            # Explicit, never polled — same rule as get_config. These are
            # kilobytes on a radio shared with telemetry, and the editors ask
            # once when they open.
            dispatch(rid, {"type": action})
        elif action in ("set_layout", "set_routines"):
            doc = data.get("doc")
            if isinstance(doc, dict):
                send_document(rid, action, doc, bool(data.get("save", True)))
        elif action in ("select_routine", "routine_cmd", "routine_event"):
            # Pass-through: the robot owns every rule about what a routine may
            # do. Duplicating any of it here would give two sources of truth,
            # and the base station is the one that can be disconnected.
            dispatch(rid, {k: v for k, v in data.items() if k != "action"}
                     | {"type": action})
        elif action == "jog":
            dispatch(rid, {"type": "jog", "mech": data.get("mech"),
                           "actuator": data.get("actuator"),
                           "power": float(data.get("power", 0))})
        elif action == "set_settings":
            # Base-station settings and gamepad mapping: applied locally, no
            # radio involved. The result (clamped values, what needs a restart)
            # rides back on the next settings frame, so the page reports a
            # refused value the same way it reports one a robot refused.
            _settings_result["v"] = settings.apply(data.get("settings") or {})
            _settings_dirty["v"] = True
        elif action == "set_places":
            # Named field positions, edited from the map. Local like
            # set_settings — no radio, no robot involved. A place only ever
            # reaches a robot as the plain lat/lon the editor resolved it into
            # when a routine was saved, so nothing here has to be pushed out.
            _places_result["v"] = places.replace(data.get("places") or [])
            _settings_dirty["v"] = True

    def settings_frame() -> dict:
        """The cold channel: everything the settings page edits or displays."""
        return {
            "type": "settings",
            "settings": settings.snapshot(),
            "settings_result": _settings_result["v"],
            # Named field positions. Cold channel because they change when
            # somebody saves one, not thirty times a second — but they ride
            # here rather than in `settings` because they are a list the map
            # draws, not a flat whitelist of scalars the settings page edits.
            "places": places.snapshot(),
            "places_result": _places_result["v"],
            "configs": fleet.configs(),
            # Layouts, routines and the field descriptors for whatever actuators
            # the operator declared. Cold channel with the configs, for the same
            # reason: kilobytes that change on Save, not thirty times a second.
            "documents": fleet.documents(),
            # Live gamepad axes/buttons, so the mapping editor can offer
            # "press the button you want" instead of asking for an index.
            "gamepad": controller.state() if controller is not None else None,
        }

    async def send_to(ws: WebSocket, msg: dict) -> None:
        try:
            await ws.send_json(msg)
        except Exception:
            clients.discard(ws)
            watchers.discard(ws)

    async def broadcast_loop() -> None:
        # Config revisions we've already pushed; a change means a robot answered
        # a get_config/set_config and the open settings page should see it.
        seen_revs: dict = {}
        try:
            while True:
                snap = fleet.snapshot(time.monotonic())
                snap["controller"] = {
                    "connected": getattr(controller, "connected", False) if controller else False,
                    "name": getattr(controller, "name", None) if controller else None,
                }
                snap["tiles"] = settings.base.tiles
                snap["tiles_maxzoom"] = tile_store.maxzoom
                snap["tiles_attribution"] = tiles_attribution
                # The bridge does NOT rate-limit browser {action:"drive"} frames
                # (see handle_action), so the on-screen joystick has to throttle
                # itself. Ship the server's budget instead of letting the client
                # hardcode a copy: a --drive-hz that the touch UI ignored is
                # exactly how the radio ended up oversubscribed.
                snap["drive_hz"] = settings.base.drive_hz
                # Which robots currently have a live feed, so the UI shows the
                # FPV panel only when there's actually something to show.
                snap["video"] = video_rx.robots() if video_rx is not None else []
                for ws in list(clients):
                    await send_to(ws, snap)

                # Outbound document fragments, a couple per cycle.
                await drain_documents()

                # Cold channel: only when something changed. Every revision
                # counter, so a saved layout or routine pushes the editors an
                # update the same way a config edit already does.
                revs = fleet.doc_revs()
                if revs != seen_revs:
                    seen_revs = revs
                    _settings_dirty["v"] = True
                if _settings_dirty["v"]:
                    _settings_dirty["v"] = False
                    frame = settings_frame()
                    for ws in list(clients):
                        await send_to(ws, frame)

                # Raw gamepad state streams only to pages that asked for it —
                # it is useless anywhere but the mapping editor, and it would
                # otherwise be dead weight on every driving client's socket.
                if watchers and controller is not None:
                    gp = {"type": "gamepad", "gamepad": controller.state()}
                    for ws in list(watchers):
                        await send_to(ws, gp)

                await asyncio.sleep(1.0 / max(settings.base.ui_hz, 1.0))
        except asyncio.CancelledError:
            pass

    @app.on_event("startup")
    async def _startup():
        link.start()
        if ip_server is not None:
            ip_server.start()
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
        if ip_server is not None:
            ip_server.stop()
        link.stop()

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        clients.add(ws)
        # Settings up front so a page that opens straight onto the settings tab
        # renders filled in, rather than blank until the next edit.
        await send_to(ws, settings_frame())
        try:
            while True:
                data = await ws.receive_json()
                # A frame that isn't an object would raise out of the loop and
                # drop the socket — which, mid-drive, is the operator losing
                # the dashboard because something sent a stray array.
                if not isinstance(data, dict):
                    continue
                if data.get("action") == "watch_gamepad":
                    (watchers.add if data.get("on") else watchers.discard)(ws)
                    continue
                handle_action(data)
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(ws)
            watchers.discard(ws)

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
        period = 1.0 / max(settings.base.video_hz, 1.0)

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
