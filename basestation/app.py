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

(There is a third, `{"type":"command"}`, but it is event-driven rather than a
channel: it goes out when somebody says something, and is silent otherwise.)

--- Voice, and the sockets that carry it ---
This endpoint takes binary frames as well as JSON. A binary frame is raw 16 kHz
mono PCM from a held microphone button (basestation/command/stt.py explains the
format and why it isn't Opus); everything else is unchanged. Speech recognition
and the local language model both block for hundreds of milliseconds, so they
run on threads — the broadcaster must keep pushing telemetry at 30 Hz while
somebody is mid-sentence, or the map freezes every time an order is given.
"""

from __future__ import annotations

import asyncio
import json
import os
import time
from collections import deque
from typing import Any, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse

from robot.comms.doc_transfer import split

from .command import CommandExecutor
from .command.stt import Transcriber, Utterance
from .fleet import FleetManager
from .places import PlaceStore
from .settings import SettingsStore
from .tiles import TileStore, attribution_for, content_type

def build_app(fleet: FleetManager, link, controller, web_cfg: dict, video_rx=None,
              settings: SettingsStore | None = None, ip_server=None,
              places: PlaceStore | None = None, llm=None,
              transcriber: "Transcriber | None" = None) -> FastAPI:
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
        """Offer one bulk frame to a link. False means "not now, keep it".

        WiFi first: a robot on the LAN costs no airtime at all, so its fragments
        go as fast as we can hand them over. Otherwise the radio, which takes a
        frame only when it has the airtime for it (robot/comms/airtime.py). A
        robot at the far end of the field simply isn't in `ip_server`, so it
        keeps getting documents over the radio with no special handling here.

        The False-means-keep-it rule is the whole point. Half a layout is not a
        smaller layout, and a fragment dropped because the radio was busy is one
        nobody re-requests.
        """
        if not robot_id:
            return True  # nowhere to send it; drop it rather than queue forever
        if ip_server is not None and ip_server.send({**msg, "to": robot_id}):
            return True
        send_bulk = getattr(link, "send_bulk", None)
        if send_bulk is None:
            # The simulator and any other in-process link: no wire, no pacing.
            link.send({**msg, "to": robot_id})
            return True
        return bool(send_bulk({**msg, "to": robot_id}))

    # Outbound document fragments, metered rather than dispatched in a loop. The
    # robot's own outbox does this in the other direction and for the same
    # reason: a radio handed more than it can write drops frames, and a
    # 10-fragment layout fired at once is the shape that arrives incomplete.
    # Bounded for the same reason the robot's outbox is: a link that has stopped
    # taking frames must not turn a queue into a leak. The oldest go first —
    # they belong to a save the operator has long since given up on.
    _doc_queue: "deque[tuple]" = deque(maxlen=512)
    _txid = {"n": 0}

    def send_document(robot_id, action: str, doc: dict, save: bool) -> None:
        if not robot_id:
            return
        _txid["n"] += 1
        mtype = "put_layout" if action == "set_layout" else "put_routines"
        for frame in split(doc, mtype, txid=f"B{_txid['n']}", save=save):
            _doc_queue.append((robot_id, frame))

    async def drain_documents() -> None:
        # Frames leave the queue only once a link has taken them, so a cycle
        # where the radio has no airtime left simply defers the rest to the next
        # one. Over WiFi nothing is ever refused and a whole layout goes in a
        # single cycle.
        while _doc_queue and dispatch_bulk(*_doc_queue[0]):
            _doc_queue.popleft()

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

    # ---- spoken and typed orders -> the same handle_action above ----
    # Constructed here rather than passed in fully wired because the executor's
    # whole point is that it can only do what handle_action can do; giving it
    # any other sink would be the bug this design exists to prevent.
    #
    # `ui` actions never reach a robot — they move the operator's screen (show a
    # camera, switch views). They ride out to the browsers on the command frame
    # instead, inside the outcome the UI is already rendering.
    executor = CommandExecutor(fleet, places, dispatch=handle_action,
                               llm=llm, enabled=bool(web_cfg.get("voice", True)))
    stt = transcriber or Transcriber(
        model_name=os.environ.get("RS_STT_MODEL", "base.en"))

    # Outcomes queued from worker threads and drained by the broadcaster. A
    # thread cannot touch a WebSocket, and `asyncio.run_coroutine_threadsafe`
    # would need the loop handle plumbed through — a deque drained at ui_hz puts
    # a spoken command on screen within 33 ms, which is well under the time it
    # took to say it.
    _command_out: "deque[dict]" = deque(maxlen=32)

    def command_frame(outcome=None) -> dict:
        frame = {"type": "command", **executor.status(),
                 "stt": stt.status()}
        if outcome is not None:
            frame["outcome"] = outcome.to_json()
        return frame

    executor.on_outcome = lambda outcome: _command_out.append(command_frame(outcome))

    async def run_command(text: str, source: str) -> None:
        """Recognise off the event loop — the model can block for a second."""
        await asyncio.to_thread(executor.recognise, text, source)

    async def run_utterance(pcm: bytes) -> None:
        """Transcribe held audio, then treat it exactly like typed text.

        The transcript is pushed to the browsers before recognition starts, so
        the operator sees what was heard while the model is still thinking. When
        a command goes wrong, knowing whether the microphone or the model was at
        fault is most of the debugging.
        """
        vocab = executor.vocabulary()
        # Bias Whisper toward the words that are currently legal. Proper nouns
        # are exactly what it gets wrong, and these are all proper nouns.
        hints = [*vocab.robots, *(p.get("name") or p["id"] for p in vocab.places),
                 *vocab.labels, *[m.replace("_", " ") for m in vocab.modes]]
        try:
            text, conf = await asyncio.to_thread(stt.transcribe, pcm, hints)
        except Exception as e:  # STTUnavailable, or a model that failed to load
            _command_out.append({"type": "command", **executor.status(),
                                 "stt": stt.status(),
                                 "heard": "", "stt_error": str(e)})
            return
        _command_out.append({"type": "command", **executor.status(),
                             "stt": stt.status(),
                             "heard": text, "heard_conf": round(conf, 2)})
        if text:
            await run_command(text, "voice")

    def handle_command(data: dict) -> Optional[Any]:
        """The command actions. Returns a coroutine to await, or None.

        Split out from handle_action because these are the only actions that are
        asynchronous — everything else is a synchronous write to the radio.
        """
        action = data.get("action")
        if action == "command_text":
            text = str(data.get("text") or "")
            return run_command(text, str(data.get("source") or "text"))
        if action == "command_intent":
            # A structured intent, from a UI button or a client that already
            # knows what it wants. Skips recognition, not the confirm gate.
            executor.execute(str(data.get("intent") or ""), data.get("args") or {},
                             source=str(data.get("source") or "ui"))
        elif action == "command_confirm":
            executor.confirm(str(data.get("id") or ""))
        elif action == "command_cancel":
            executor.cancel(str(data.get("id") or ""))
        elif action == "command_enable":
            executor.enabled = bool(data.get("on", True))
            _command_out.append(command_frame())
        elif action == "command_check":
            # "Is the model there yet?" — cheap, explicit, and never polled: it
            # is a button in the command view, pressed after starting LM Studio.
            return _check_model()
        return None

    async def _check_model() -> None:
        await asyncio.to_thread(executor.check_model)
        _command_out.append(command_frame())

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

                # Command outcomes queued by worker threads. Everyone sees them,
                # not just whoever spoke: on a two-tablet setup, the person who
                # did not give the order is the one who needs to know it was given.
                while _command_out:
                    frame = _command_out.popleft()
                    for ws in list(clients):
                        await send_to(ws, frame)

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

        # Warm the voice stack in the background. Both of these can take
        # seconds — loading ~150 MB of Whisper weights, and a round trip to a
        # model server that may not be running — and neither may delay the
        # bridge coming up. A base station that won't accept a teleop connection
        # because it is downloading a speech model is a base station that misses
        # its match.
        async def _warm() -> None:
            await asyncio.to_thread(stt.preload)
            await asyncio.to_thread(executor.check_model)
            _command_out.append(command_frame())

        app.state.warm = asyncio.create_task(_warm())

    @app.on_event("shutdown")
    async def _shutdown():
        for name in ("broadcaster", "warm"):
            task = getattr(app.state, name, None)
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
        # One microphone buffer per socket, so two tablets holding their mic
        # buttons at once don't interleave audio into one another's command.
        utterance = Utterance()
        # Settings up front so a page that opens straight onto the settings tab
        # renders filled in, rather than blank until the next edit.
        await send_to(ws, settings_frame())
        await send_to(ws, command_frame())
        try:
            while True:
                # receive() rather than receive_json(): this socket now carries
                # binary microphone frames as well as JSON actions, and
                # receive_json on a binary frame raises and drops the socket —
                # which mid-drive is the operator losing the dashboard.
                message = await ws.receive()
                if message.get("type") == "websocket.disconnect":
                    break

                chunk = message.get("bytes")
                if chunk is not None:
                    # Raw 16 kHz mono PCM while the mic button is held. Dropped
                    # silently when no utterance is open — a late frame arriving
                    # after the release is normal, not an error.
                    utterance.feed(chunk)
                    continue

                text = message.get("text")
                if text is None:
                    continue
                try:
                    data = json.loads(text)
                except (TypeError, ValueError):
                    continue
                # A frame that isn't an object would raise out of the loop and
                # drop the socket — which, mid-drive, is the operator losing
                # the dashboard because something sent a stray array.
                if not isinstance(data, dict):
                    continue

                action = data.get("action")
                if action == "watch_gamepad":
                    (watchers.add if data.get("on") else watchers.discard)(ws)
                    continue
                if action == "mic":
                    # Push-to-talk. The button going down opens a buffer; going
                    # up closes it and starts recognition on a thread, so the
                    # socket is free to keep receiving telemetry acks and — the
                    # point of the whole exercise — a following "stop".
                    if data.get("on"):
                        utterance.begin()
                    elif utterance.active or utterance.seconds:
                        pcm = utterance.end()
                        asyncio.create_task(run_utterance(pcm))
                    continue
                if isinstance(action, str) and action.startswith("command_"):
                    pending = handle_command(data)
                    if pending is not None:
                        asyncio.create_task(pending)
                    continue

                handle_action(data)
        except WebSocketDisconnect:
            pass
        except RuntimeError:
            # Starlette raises this if receive() is called after the client has
            # already gone. Same outcome as a clean disconnect.
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

    # Reachable for tests and for anything embedding the bridge. Not a second
    # way in: everything it can do, it does by calling handle_action above.
    app.state.commands = executor
    app.state.stt = stt
    return app
