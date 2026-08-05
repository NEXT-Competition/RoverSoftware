"""Fleet state: tracks every robot the base station has heard from.

Telemetry frames (from robots or the simulator) flow in via
`update_from_telemetry`; the web layer reads `snapshot()` to push the whole
fleet to the browser. Thread-safe: telemetry arrives on the link's reader
thread while the web loop reads snapshots.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from robot.comms.doc_transfer import Reassembler

ONLINE_TIMEOUT = 3.0  # seconds without telemetry before a robot is "offline"

# Chunked documents a robot sends, and the verdicts it returns on ones we sent.
_DOC_TYPES = ("layout", "routines", "scripts", "fields")
_DOC_RESULTS = {"layout_result": "layout", "routines_result": "routines",
                "scripts_result": "scripts"}
# The same documents named by what we SEND, which is what a delivery failure
# knows about — there is no reply to key on when nothing was delivered.
_DOC_SENDS = {"put_layout": "layout", "put_routines": "routines",
              "put_scripts": "scripts"}

# How many console lines from a running script are kept per robot. A ring
# buffer, because a script printing in a loop is a normal thing to write and a
# base station that grew a list for the rest of the match is not.
SCRIPT_CONSOLE_MAX = 500


@dataclass
class RobotState:
    robot_id: str
    mode: str = "unknown"
    estop: bool = False
    left: float = 0.0
    right: float = 0.0
    battery: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    heading: Optional[float] = None
    # Vision summary from the robot's object detector: {ok, fps, label, conf, ex,
    # size, age}. Opaque here on purpose — the robot owns the shape; this layer
    # just forwards it. Note fields must be listed here AND in snapshot() or they
    # never reach the browser.
    vision: Optional[dict] = None
    imu_calib: Optional[int] = None  # BNO085 fused-orientation calibration level 0-3
    # GPS fix health from the robot: {fix, sats, speed, hdop, alt, track,
    # track_age}. Opaque here for the same reason as `vision` above. This is how
    # you tell "the rover is lost" from "the rover has 3 satellites", and
    # track_age is how you tell a live heading from one held since the last time
    # it moved.
    gps: Optional[dict] = None
    # Shooter summary {armed, shots, ready, cool}, present only while the robot
    # is in shooter_align. Unlike the fields above this one is NOT sticky — see
    # update_from_telemetry for why a stale arm indicator would be dangerous.
    shooter: Optional[dict] = None
    # Layout mechanisms {name: {kind, ...}} and the live FSM state
    # {id, state, t, drive, done}. Both follow `shooter`'s non-sticky rule for
    # the same reason: the robot omits them when they don't apply, and a stale
    # copy would show a state machine still running one the robot has left.
    mech: Optional[dict] = None
    routine: Optional[dict] = None
    # Whether the operator's Python is still going, and why it stopped if it
    # isn't. Rides the hot frame rather than the bulk link the console takes,
    # because a rover out of WiFi range still has to be able to say this.
    script: Optional[dict] = None
    # Measured wheel speed and what the speed-matching loop did about it:
    # {rpm: {actuator: rpm}, mode, tl, tr, fault}. Non-sticky like the two
    # above — the robot omits it entirely on a build with no encoders, and a
    # held-over RPM readout is a speedometer that lies about a stopped rover.
    enc: Optional[dict] = None
    # Distance to whatever is straight ahead and what the collision guard is
    # doing about it: {d, state, mute, off}. Non-sticky like the blocks above,
    # and here the reason is the sharpest of them: a held-over "0.4 m" from a
    # sensor that has stopped reporting is the dashboard telling an operator
    # there is a wall where there may be nothing, or nothing where there is a
    # wall. Absent entirely on a build with no ultrasonic fitted.
    sonar: Optional[dict] = None
    # Live closed-loop traces {loop_path: {sp, e, o, p, i, d, m, sat}}, present
    # only while the robot has nav.pid_trace on AND is in a mode that runs a
    # loop. Non-sticky like the two above: a frozen curve left on screen after
    # the loop stopped is a graph that lies about what the robot is doing.
    pid: Optional[dict] = None
    # The robot's WiFi state, as it last reported it: {ok, ssid, ip, signal,
    # networks, error}. Sticky, unlike the blocks above, and for the opposite
    # reason — this is an ANSWER to something the operator asked, so it has to
    # stay on screen until the next answer replaces it. It never contains a
    # credential (robot/comms/wifi.py).
    wifi: Optional[dict] = None
    wifi_rev: int = 0
    last_seen: float = 0.0
    trail: List[Tuple[float, float]] = field(default_factory=list)
    # How many points have EVER been appended to `trail`, which is not the same
    # as how many it holds — the oldest are dropped once it reaches trail_max.
    # This is what lets the hot frame carry only the points added since the last
    # broadcast instead of restating the whole breadcrumb thirty times a second:
    # a client that knows the count it is holding can tell "I am two points
    # behind" from "I have missed some", and only the second needs a resend.
    # Monotonic for the life of the process, so it never repeats a value.
    trail_seq: int = 0
    # The robot's tunable parameters (robot/tuning.py), flat dotted paths ->
    # values, as last reported by the robot itself. Merged rather than replaced:
    # a robot answers `get_config` with everything but acknowledges a `set_config`
    # with only the fields it applied, so the base station keeps the union.
    # Empty until someone asks — it costs ~2.4 KB of radio airtime, so it is
    # fetched on demand, never polled.
    config: Dict[str, object] = field(default_factory=dict)
    # Bumped on every config change so the web layer can push the settings page
    # an update without diffing, and without putting 2.4 KB in every 30 Hz frame.
    config_rev: int = 0
    # Result of the last set_config: {"rejected": {...}, "restart": [...],
    # "save_error": str|None}. Shown next to the fields the operator just edited.
    config_result: Optional[dict] = None
    # Documents, each with its own revision so saving a routine doesn't push a
    # layout back over the radio. Reassembled from fragments, never merged —
    # half a layout is not a smaller layout (see robot/comms/doc_transfer.py).
    layout: Optional[dict] = None
    layout_rev: int = 0
    layout_result: Optional[dict] = None
    routines: Optional[dict] = None
    routines_rev: int = 0
    routines_result: Optional[dict] = None
    # Operator-written Python, and what the last save of it was told.
    scripts: Optional[dict] = None
    scripts_rev: int = 0
    scripts_result: Optional[dict] = None
    # What a running script has printed, and the values it asked to be watched.
    # Not a document — it is a live stream off the bulk link — so it gets its
    # own revision and its own frame rather than riding with the editors'
    # kilobytes. See FleetManager.script_console.
    console: List[str] = field(default_factory=list)
    console_rev: int = 0
    watch: Dict[str, object] = field(default_factory=dict)
    # Descriptors for the tunable fields the dashboard's schema.ts cannot know
    # about in advance, because the operator invented them.
    fields: List[dict] = field(default_factory=list)
    fields_rev: int = 0

    def online(self, now: float) -> bool:
        return self.last_seen > 0 and (now - self.last_seen) < ONLINE_TIMEOUT


class FleetManager:
    def __init__(self, trail_max: int = 400):
        self._robots: Dict[str, RobotState] = {}
        self._selected: Optional[str] = None
        self._lock = threading.Lock()
        self.trail_max = trail_max
        # robot_id -> document type -> Reassembler
        self._reassemblers: Dict[str, Dict[str, Reassembler]] = {}

    def _ensure(self, robot_id: str) -> RobotState:
        st = self._robots.get(robot_id)
        if st is None:
            st = RobotState(robot_id=robot_id)
            self._robots[robot_id] = st
            if self._selected is None:
                self._selected = robot_id  # auto-select the first robot we meet
        return st

    def update_from_telemetry(self, msg: dict, now: float) -> None:
        if msg.get("type") != "telemetry":
            return
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return
        # Blocks the robot is deliberately HOLDING BACK this frame, because it
        # sends them on a slower tier than the rest (robot/robot.py::_telemetry).
        # This is the difference between "the sensor stopped answering" and "you
        # already have this" — which for the non-sticky blocks below is the
        # difference between clearing a reading and keeping it. Absent on a robot
        # running older code, which is exactly the old behaviour: nothing is
        # held, so every block is read straight off the frame.
        keep = msg.get("keep") or ()
        with self._lock:
            st = self._ensure(robot_id)
            st.mode = msg.get("mode", st.mode)
            st.estop = bool(msg.get("estop", st.estop))
            st.left = float(msg.get("left", st.left))
            st.right = float(msg.get("right", st.right))
            if "battery" in msg and msg["battery"] is not None:
                st.battery = float(msg["battery"])
            if "heading" in msg and msg["heading"] is not None:
                st.heading = float(msg["heading"])
            if "vision" not in keep and msg.get("vision") is not None:
                st.vision = msg["vision"]
            # Assigned unconditionally rather than only when present, unlike the
            # sticky fields above it. The robot sends null the moment the IMU
            # stops answering, and the whole point of that null is to take the
            # calibration pips off the screen — a sticky copy would hold them
            # there, which is the reassurance-about-a-dead-sensor this is meant
            # to prevent. Naming it in `keep` is how the robot says "unchanged"
            # WITHOUT saying "gone", which a bare omission cannot express.
            if "imu_calib" not in keep:
                calib = msg.get("imu_calib")
                st.imu_calib = int(calib) if calib is not None else None
            if "gps" not in keep and msg.get("gps") is not None:
                st.gps = msg["gps"]
            # Assigned unconditionally, breaking the "only overwrite when present"
            # pattern above on purpose. The robot omits this field entirely once
            # shooter_align is no longer active, and a sticky copy would leave the
            # UI showing ARMED for a mode the robot has already left — the one
            # piece of stale telemetry here that could get someone hurt.
            st.shooter = msg.get("shooter")
            # Non-sticky like `shooter`, and on the slow tier — so unlike it,
            # this one has to distinguish an omission that means "the layout has
            # no mechanisms" from one that means "nothing about them changed".
            if "mech" not in keep:
                st.mech = msg.get("mech")
            st.routine = msg.get("routine")
            # Assigned unconditionally, like `routine` above it: the robot omits
            # this entirely once `script` is no longer the active mode, and a
            # sticky copy would leave the rail showing a run that ended.
            st.script = msg.get("script")
            st.pid = msg.get("pid")
            st.enc = msg.get("enc")
            st.sonar = msg.get("sonar")
            if msg.get("lat") is not None and msg.get("lon") is not None:
                st.lat, st.lon = float(msg["lat"]), float(msg["lon"])
                st.trail.append((st.lat, st.lon))
                st.trail_seq += 1
                if len(st.trail) > self.trail_max:
                    del st.trail[: len(st.trail) - self.trail_max]
            st.last_seen = now

    def handle(self, msg: dict, now: float) -> None:
        """Route one inbound frame from the radio (or the simulator).

        The single entry point for everything a robot sends, so callers don't
        have to know which frame types exist.
        """
        mtype = msg.get("type")
        if mtype == "config":
            self.update_from_config(msg)
        elif mtype == "wifi":
            self.update_from_wifi(msg)
        elif mtype == "script_output":
            self.update_from_script_output(msg)
        elif mtype in _DOC_TYPES or mtype in _DOC_RESULTS:
            self.update_from_document(msg)
        else:
            self.update_from_telemetry(msg, now)

    def update_from_wifi(self, msg: dict) -> Optional[str]:
        """Absorb a {"type":"wifi"} frame — a scan, a status, or the verdict on
        a join attempt.

        Replaced whole rather than merged: unlike a config frame these are not
        partial views of one state, they are separate answers to separate
        questions, and merging a failed join into a successful scan would leave
        a panel showing both at once.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        with self._lock:
            st = self._ensure(robot_id)
            st.wifi = {k: v for k, v in msg.items()
                       if k not in ("type", "from", "to", "robot_id")}
            st.wifi_rev += 1
        return robot_id

    def update_from_config(self, msg: dict) -> Optional[str]:
        """Absorb a {"type":"config"} frame from a robot; returns its id.

        The payload is MERGED, because it arrives in two flavours: the full
        snapshot a robot sends when asked, and the applied-fields subset it
        acknowledges an edit with. Merging makes both correct and means a
        rejected field keeps showing the robot's real value, not the operator's
        wish.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        values = msg.get("config")
        with self._lock:
            st = self._ensure(robot_id)
            if isinstance(values, dict):
                st.config.update(values)
            st.config_result = {
                "rejected": msg.get("rejected") or {},
                "restart": msg.get("restart") or [],
                "save_error": msg.get("save_error"),
                # Cleared explicitly: a real answer from the robot is proof the
                # link is back, and a stale "not on WiFi" banner sitting above a
                # page that has just updated is worse than no banner at all.
                "error": None,
            }
            st.config_rev += 1
        return robot_id

    def note_unreachable(self, robot_id: Optional[str], reason: str) -> None:
        """Record that a config command could not be delivered.

        Written into the same `config_result` the robot's own answers land in,
        because it is the same question the page is asking — "what happened to
        my edit?" — and the answer wants to appear in the same place. The rev
        bump is what pushes it: `configs()` only reports robots whose rev has
        moved, so this also makes the entry exist for a robot that has never
        managed to send a config at all.
        """
        if not robot_id:
            return
        with self._lock:
            st = self._ensure(robot_id)
            st.config_result = {"rejected": {}, "restart": [],
                                "save_error": None, "error": reason}
            st.config_rev += 1

    def note_doc_unreachable(self, robot_id: Optional[str], mtype: str,
                             reason: str) -> None:
        """Same, for a layout or routine document that could not be sent.

        Shaped exactly like the robot's own verdict on a document it rejected,
        so the editors need no second code path for "it never got there".
        """
        field_name = _DOC_SENDS.get(mtype)
        if not robot_id or field_name is None:
            return
        with self._lock:
            st = self._ensure(robot_id)
            setattr(st, f"{field_name}_result", {
                "ok": False, "errors": [reason], "warnings": [],
                "save_error": None, "restart_required": False,
            })
            setattr(st, f"{field_name}_rev", getattr(st, f"{field_name}_rev") + 1)

    def update_from_document(self, msg: dict) -> Optional[str]:
        """Absorb a document fragment, or a robot's verdict on one we sent.

        Unlike `config`, nothing is merged. Documents arrive as numbered
        fragments and are reassembled whole (robot/comms/doc_transfer.py) — a
        half-arrived layout is not a smaller layout, it is a robot with one
        drive motor. One reassembler per robot per document type, so a layout
        and a routine save can't interleave.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        mtype = msg.get("type")
        with self._lock:
            st = self._ensure(robot_id)
            if mtype in _DOC_RESULTS:
                field_name = _DOC_RESULTS[mtype]
                setattr(st, f"{field_name}_result", {
                    "ok": bool(msg.get("ok")),
                    "errors": msg.get("errors") or [],
                    "warnings": msg.get("warnings") or [],
                    "save_error": msg.get("save_error"),
                    "restart_required": bool(msg.get("restart_required")),
                })
                setattr(st, f"{field_name}_rev",
                        getattr(st, f"{field_name}_rev") + 1)
                return robot_id

            rx = self._reassemblers.setdefault(robot_id, {})
            doc = rx.setdefault(mtype, Reassembler()).feed(msg)
            if doc is None:
                return None
            if mtype == "fields":
                st.fields = doc.get("fields") or []
                st.fields_rev += 1
            elif mtype == "layout":
                st.layout = doc
                st.layout_rev += 1
            elif mtype == "routines":
                st.routines = doc
                st.routines_rev += 1
            elif mtype == "scripts":
                st.scripts = doc
                st.scripts_rev += 1
        return robot_id

    def update_from_script_output(self, msg: dict) -> Optional[str]:
        """Absorb a `script_output` frame: console lines and watched values.

        Not a document and not chunked — each frame is a self-contained batch
        of whatever the script printed in the last quarter second, so a frame
        lost to a WiFi hiccup costs those lines and nothing else. That is the
        right trade for output: the alternative is a reassembly buffer whose
        failure mode is a console that goes permanently blank.
        """
        robot_id = msg.get("from") or msg.get("robot_id")
        if not robot_id:
            return None
        lines = msg.get("lines") or []
        watch = msg.get("watch") or {}
        if not lines and not watch:
            return None
        with self._lock:
            st = self._ensure(robot_id)
            if lines:
                st.console.extend(str(line) for line in lines)
                if len(st.console) > SCRIPT_CONSOLE_MAX:
                    del st.console[: len(st.console) - SCRIPT_CONSOLE_MAX]
            if watch:
                st.watch = dict(watch)
            st.console_rev += 1
        return robot_id

    def clear_console(self, robot_id: str) -> None:
        """Throw away a robot's console. The operator's own "clear" button, and
        what a fresh run starts from — output from the last attempt sitting
        above this one's is how a fixed bug looks unfixed."""
        with self._lock:
            st = self._ensure(robot_id)
            st.console = []
            st.watch = {}
            st.console_rev += 1

    def script_console(self) -> Dict[str, dict]:
        """Every robot's script console, for the code editor's output pane."""
        with self._lock:
            return {
                st.robot_id: {"lines": list(st.console), "watch": dict(st.watch),
                              "rev": st.console_rev}
                for st in self._robots.values()
                if st.console_rev
            }

    def console_revs(self) -> Dict[str, int]:
        with self._lock:
            return {st.robot_id: st.console_rev for st in self._robots.values()}

    def documents(self) -> Dict[str, dict]:
        """Per-robot layout, routines and field descriptors, for the editors.

        On the cold channel with the configs, and for the same reason: these are
        kilobytes that change when someone presses Save, not 30 times a second.
        """
        with self._lock:
            return {
                st.robot_id: {
                    "layout": st.layout,
                    "layout_rev": st.layout_rev,
                    "layout_result": st.layout_result,
                    "routines": st.routines,
                    "routines_rev": st.routines_rev,
                    "routines_result": st.routines_result,
                    "scripts": st.scripts,
                    "scripts_rev": st.scripts_rev,
                    "scripts_result": st.scripts_result,
                    "fields": st.fields,
                    "fields_rev": st.fields_rev,
                }
                for st in self._robots.values()
                if st.layout_rev or st.routines_rev or st.scripts_rev
                or st.fields_rev
            }

    def doc_revs(self) -> Dict[str, tuple]:
        """Every revision counter, so the broadcaster can push on any change."""
        with self._lock:
            return {
                st.robot_id: (st.config_rev, st.layout_rev, st.routines_rev,
                              st.scripts_rev, st.fields_rev, st.wifi_rev)
                for st in self._robots.values()
            }

    def wifi(self) -> Dict[str, dict]:
        """Per-robot WiFi state, for the Network page.

        On the cold channel with the configs: a scan result is a few hundred
        bytes that changes when somebody presses Scan, and restating it thirty
        times a second would spend the dashboard's bandwidth on an answer nobody
        asked twice.

        The revision rides along so the dashboard can tell a NEW answer from the
        same one being re-pushed by an unrelated change. Without it, a Scan the
        rover never answered looks identical to one it answered with the same
        result, and the button either spins forever or stops spinning too early.
        """
        with self._lock:
            return {st.robot_id: {"rev": st.wifi_rev, **st.wifi}
                    for st in self._robots.values() if st.wifi is not None}

    def configs(self) -> Dict[str, dict]:
        """Per-robot config + the last edit's result, for the settings page.

        Pushed only when `config_rev` moves — see basestation/app.py. It is not
        part of the fleet snapshot on purpose: at 30 Hz, 2.4 KB per robot is a
        lot of bytes to spend restating a config nobody is looking at.
        """
        with self._lock:
            return {
                st.robot_id: {
                    "rev": st.config_rev,
                    "config": dict(st.config),
                    "result": st.config_result,
                }
                for st in self._robots.values()
                if st.config_rev > 0
            }

    def config_revs(self) -> Dict[str, int]:
        with self._lock:
            return {st.robot_id: st.config_rev for st in self._robots.values()}

    @property
    def selected(self) -> Optional[str]:
        with self._lock:
            return self._selected

    def select(self, robot_id: Optional[str]) -> None:
        with self._lock:
            if robot_id in self._robots:
                self._selected = robot_id

    def mode_of(self, robot_id: Optional[str]) -> Optional[str]:
        """The mode a robot last reported, or None if we have never heard from
        it. Its own accessor because the drive gate asks this per frame, and
        building a whole snapshot to read one string is not what that is for."""
        if not robot_id:
            return None
        with self._lock:
            st = self._robots.get(robot_id)
            return st.mode if st is not None else None

    def trails(self) -> Dict[str, dict]:
        """Every robot's breadcrumb in full, with the count it is current as of.

        The cold counterpart to the deltas `snapshot` emits: sent once when a
        browser connects and again only if one asks to resync. A client cannot
        start appending to a trail it has never seen, and reconstructing one from
        deltas would mean keeping every point the base station has ever dropped.
        """
        with self._lock:
            return {st.robot_id: {"trail": list(st.trail), "seq": st.trail_seq}
                    for st in self._robots.values() if st.trail}

    def snapshot(self, now: float, trail_cursors: Optional[Dict[str, int]] = None
                 ) -> dict:
        """The hot frame. `trail_cursors` maps robot_id -> the trail_seq the
        caller last sent out; each robot then carries only the points appended
        since, under `trail_add`.

        Passing None omits breadcrumb data altogether, which is what the callers
        that only want current state (the command layer) actually want — the
        trail is for drawing a map, and restating it is the single largest thing
        this frame used to carry.
        """
        with self._lock:
            robots = [
                {
                    "robot_id": st.robot_id,
                    "mode": st.mode,
                    "estop": st.estop,
                    "left": round(st.left, 3),
                    "right": round(st.right, 3),
                    "battery": st.battery,
                    "lat": st.lat,
                    "lon": st.lon,
                    "heading": st.heading,
                    "vision": st.vision,
                    "imu_calib": st.imu_calib,
                    "gps": st.gps,
                    "shooter": st.shooter,
                    "mech": st.mech,
                    "routine": st.routine,
                    "script": st.script,
                    "pid": st.pid,
                    "enc": st.enc,
                    "sonar": st.sonar,
                    "online": st.online(now),
                    "age": round(now - st.last_seen, 2) if st.last_seen else None,
                    **self._trail_delta(st, trail_cursors),
                }
                for st in self._robots.values()
            ]
            return {"type": "fleet", "selected": self._selected, "robots": robots}

    def _trail_delta(self, st: RobotState, cursors: Optional[Dict[str, int]]) -> dict:
        """The breadcrumb points `cursors` has not seen yet, and the count they
        bring it to. Called with the lock held.

        `trail_seq` is sent even when there is nothing to add, because it is what
        a client checks itself against: if the arithmetic doesn't work out it has
        missed points and asks for a full trail, which is the only failure this
        scheme can have and the only one it needs to detect.
        """
        if cursors is None:
            return {}
        behind = st.trail_seq - cursors.get(st.robot_id, 0)
        if behind <= 0:
            add: List[Tuple[float, float]] = []
        else:
            # Never more than we still hold: a client further behind than
            # trail_max cannot be caught up by appending, and asks for the
            # whole trail instead once it sees the arithmetic fail.
            add = st.trail[-behind:] if behind <= len(st.trail) else list(st.trail)
        return {"trail_seq": st.trail_seq, "trail_add": add}
