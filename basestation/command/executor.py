"""One place a command becomes an action, whoever said it.

Voice, typed text and MCP all arrive here. That is not code reuse for its own
sake — it is how the confirmation gate, the whitelist and the audit log stay
true for every caller. An MCP client that could bypass the gate by taking a
different path would make the gate decorative.

--- The three phases ---
    recognise()  transcript -> intent name + args   (fast path, then the model)
    execute()    intent -> Outcome                  (validate, gate, dispatch)
    confirm()    a pending id -> Outcome            (a human said yes)

`recognise` is the only phase that involves a language model. `execute` is the
only one that reaches a robot. Splitting them is what lets MCP skip straight to
`execute` with a structured intent, and lets the fast path skip the model.

--- The log ---
Every outcome is appended to a bounded history and pushed to every connected
browser. A fleet being commanded by voice, with a model in the loop and possibly
an AI on MCP, must leave a record of what was heard and what was sent; without
it, "why did rover2 just turn" has no answer.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from . import fastpath
from .intents import Authority, Intent, IntentError, plan
from .llm import LLMError, LMStudio, Plan
from .vocabulary import Vocabulary, build as build_vocabulary

# Confirmations expire. A "fire?" card left on screen from two matches ago,
# tapped by someone tidying up, is the exact accident the gate exists to stop.
CONFIRM_TTL = 45.0

# Bounded so a long day doesn't grow the process. Fifty is more than fits on
# screen and more than anyone scrolls back through mid-match.
HISTORY_MAX = 50


@dataclass
class Outcome:
    """What happened to one command, in the shape the UI and MCP both read."""

    status: str
    """done | pending | refused | unheard | error.

    `unheard` is distinct from `refused` on purpose: "I didn't understand you"
    and "you may not do that" need different responses from the operator, and
    collapsing them is how a voice interface becomes untrustworthy."""

    say: str
    """One sentence for the operator. Always populated, including on failure."""

    intent: Optional[str] = None
    args: Dict[str, Any] = field(default_factory=dict)
    actions: List[Dict[str, Any]] = field(default_factory=list)
    """Exactly what was (or would be) dispatched. Shown in the UI's detail row —
    the operator's proof that the sentence and the command agree."""

    confirm_id: Optional[str] = None
    source: str = "voice"
    """voice | text | mcp | fastpath — who said it. Kept because "an AI did that"
    is the first thing you want to know when a rover moves unexpectedly."""

    transcript: str = ""
    route: str = ""
    """fastpath | model | direct — which path recognised it, for debugging a
    model that has started mis-classifying."""

    at: float = 0.0
    ms: Optional[int] = None
    """Wall time recognition took. A model slower than a second is unusable for
    driving, and the operator should be able to see that rather than infer it."""

    def to_json(self) -> Dict[str, Any]:
        return {
            "status": self.status, "say": self.say, "intent": self.intent,
            "args": self.args, "actions": self.actions,
            "confirm_id": self.confirm_id, "source": self.source,
            "transcript": self.transcript, "route": self.route,
            "at": round(self.at, 3), "ms": self.ms,
        }


@dataclass
class Pending:
    id: str
    intent: Intent
    actions: List[Dict[str, Any]]
    sentence: str
    source: str
    at: float


class CommandExecutor:
    """Owns the vocabulary, the gate, the history — and nothing else.

    Deliberately holds no sockets and no radio. It is handed a `dispatch`
    callable (in practice `basestation/app.py::handle_action`) and a `ui`
    callable for the actions that only move the operator's screen. Everything
    it emits is something the dashboard could have emitted itself.
    """

    def __init__(self, fleet, places, dispatch: Callable[[Dict[str, Any]], None],
                 ui: Optional[Callable[[Dict[str, Any]], None]] = None,
                 llm: Optional[LMStudio] = None, enabled: bool = True):
        self._fleet = fleet
        self._places = places
        self._dispatch = dispatch
        self._ui = ui or (lambda _msg: None)
        self.llm = llm or LMStudio()
        self.enabled = enabled
        """False when the operator has turned the model off (or it was never
        configured). The fast path still runs — stop must work regardless."""

        self._lock = threading.Lock()
        self._pending: Dict[str, Pending] = {}
        self._history: List[Outcome] = []
        self._seq = 0
        self._model_ok = False
        self._model_note = "not checked yet"
        self.on_outcome: Optional[Callable[[Outcome], None]] = None
        """Set by the app so every outcome reaches the browsers immediately,
        rather than on the next 30 Hz frame."""

    # --- state ------------------------------------------------------------

    def vocabulary(self) -> Vocabulary:
        return build_vocabulary(self._fleet, self._places, time.monotonic())

    def check_model(self) -> Dict[str, Any]:
        """Ping the model server and remember the answer for the status pill."""
        ok, note = self.llm.health()
        self._model_ok, self._model_note = ok, note
        return self.status()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            self._expire(time.monotonic())
            pending = [{"id": p.id, "intent": p.intent.name, "say": p.sentence,
                        "source": p.source, "at": round(p.at, 3),
                        "expires_in": round(CONFIRM_TTL - (time.monotonic() - p.at), 1)}
                       for p in self._pending.values()]
            history = [o.to_json() for o in self._history]
        return {
            "enabled": self.enabled,
            "model_ok": self._model_ok,
            "model": self.llm.model or None,
            "model_note": self._model_note,
            "base_url": self.llm.base_url,
            "pending": pending,
            "history": history,
            "vocabulary": self.vocabulary().snapshot(),
        }

    # --- phase 1: what did they say? -------------------------------------

    def recognise(self, transcript: str, source: str = "voice") -> Outcome:
        """Transcript -> an executed or pending Outcome.

        The order is load-bearing. Fast path first so a stop never waits on a
        model; a stop found *anywhere* in the sentence next, so "uh stop stop
        stop" is not sent off to be thought about; only then the model.
        """
        began = time.monotonic()
        text = (transcript or "").strip()
        if not text:
            return self._record(Outcome("unheard", "I didn't hear anything.",
                                        source=source, transcript=text,
                                        at=time.time()))

        vocab = self.vocabulary()

        hit = fastpath.match(text, vocab)
        if hit is not None:
            name, args = hit
            return self.execute(name, args, source=source, transcript=text,
                                route="fastpath", vocab=vocab,
                                ms=int((time.monotonic() - began) * 1000))

        # Not a clean fast-path match, but there is a stop in there somewhere.
        # Stop wins over understanding the rest of the sentence.
        if fastpath.is_stop(text):
            return self.execute("estop", {}, source=source, transcript=text,
                                route="fastpath", vocab=vocab,
                                ms=int((time.monotonic() - began) * 1000))

        if not self.enabled:
            return self._record(Outcome(
                "unheard",
                "The command model is turned off — only stop, rover names, modes "
                "and the camera are recognised.",
                source=source, transcript=text, route="fastpath", at=time.time()))

        try:
            decision: Plan = self.llm.classify(text, vocab)
            self._model_ok, self._model_note = True, self.llm.model
        except LLMError as e:
            self._model_ok, self._model_note = False, str(e)
            return self._record(Outcome(
                "error",
                f"{e}. Stop, rover names, modes and the camera still work.",
                source=source, transcript=text, route="model", at=time.time(),
                ms=int((time.monotonic() - began) * 1000)))

        ms = int((time.monotonic() - began) * 1000)
        if decision.intent is None:
            return self._record(Outcome(
                "unheard", decision.say or "I didn't catch a command in that.",
                source=source, transcript=text, route="model",
                at=time.time(), ms=ms))

        return self.execute(decision.intent, decision.args, source=source,
                            transcript=text, route="model", vocab=vocab, ms=ms,
                            say=decision.say)

    # --- phase 2: turn it into actions -----------------------------------

    def execute(self, name: str, args: Any, *, source: str = "mcp",
                transcript: str = "", route: str = "direct",
                vocab: Optional[Vocabulary] = None, ms: Optional[int] = None,
                say: str = "") -> Outcome:
        """Validate an intent against live state, then gate it or run it.

        This is the entry point MCP uses directly — a structured intent needs no
        recognition — which is exactly why the gate lives here and not in
        `recognise`.
        """
        vocab = vocab or self.vocabulary()
        now = time.time()
        try:
            intent, actions, sentence = plan(name, args, vocab)
        except IntentError as e:
            return self._record(Outcome(
                "refused", str(e), intent=name,
                args=args if isinstance(args, dict) else {},
                source=source, transcript=transcript, route=route,
                at=now, ms=ms))

        # The model's own sentence reads better, but the planner's is the
        # accurate one — it names the rover the command actually resolved to,
        # which may not be the one the model wrote down. So the model's line is
        # what gets spoken back, and the planner's is what the confirm card and
        # the log show. Never the model's on a confirm.
        spoken = say or sentence

        if intent.authority is Authority.CONFIRM:
            with self._lock:
                self._seq += 1
                pid = f"c{self._seq}"
                self._pending[pid] = Pending(pid, intent, actions, sentence,
                                             source, time.monotonic())
                self._expire(time.monotonic())
            return self._record(Outcome(
                "pending", f"Confirm: {sentence}?", intent=intent.name,
                args=args if isinstance(args, dict) else {}, actions=actions,
                confirm_id=pid, source=source, transcript=transcript,
                route=route, at=now, ms=ms))

        self._run(actions)
        # `status` asks a question, so the reply IS the result — an operator who
        # said "what is rover2 doing" and heard "status of rover2" learned
        # nothing. Overrides the model's sentence too: only live telemetry can
        # answer this, and the model does not have it.
        if intent.name == "status":
            spoken = self.narrate(actions[0].get("robot_id"))
        return self._record(Outcome(
            "done", spoken or sentence, intent=intent.name,
            args=args if isinstance(args, dict) else {}, actions=actions,
            source=source, transcript=transcript, route=route, at=now, ms=ms))

    def narrate(self, robot_id: Optional[str] = None) -> str:
        """What a rover (or the fleet) is doing, as a sentence to read back.

        Mirrors the wording in basestation-ui/src/components/CommandDock.tsx
        `activity()` on purpose — hearing something different from what the dock
        says is on screen would make the operator doubt one of them.
        """
        snap = self._fleet.snapshot(time.monotonic())
        rows = snap.get("robots") or []
        if robot_id:
            rows = [r for r in rows if r.get("robot_id") == robot_id]
            if not rows:
                return f"I have nothing from {robot_id}."
        if not rows:
            return "No rovers are on the air."

        def one(r: Dict[str, Any]) -> str:
            rid = r.get("robot_id")
            if r.get("estop"):
                return f"{rid} is e-stopped"
            if not r.get("online"):
                age = r.get("age")
                return f"{rid} is silent" + (f" for {age:.0f} seconds" if age else "")
            mode = str(r.get("mode") or "teleop")
            if mode == "routine":
                state = (r.get("routine") or {}).get("state")
                return f"{rid} is running a routine" + (f", in {state}" if state else "")
            if mode == "object_align":
                seen = (r.get("vision") or {}).get("label")
                return f"{rid} is aligning" + (f" on the {seen}" if seen else ", nothing in view")
            if mode == "waypoint":
                return f"{rid} is driving a route"
            if mode == "shooter_align":
                return f"{rid} is aiming"
            return f"{rid} is in teleop"

        return ". ".join(one(r) for r in rows) + "."

    # --- phase 3: a human said yes ---------------------------------------

    def confirm(self, confirm_id: str, *, source: str = "ui") -> Outcome:
        now = time.monotonic()
        with self._lock:
            self._expire(now)
            entry = self._pending.pop(confirm_id, None)
        if entry is None:
            return self._record(Outcome(
                "refused", "That confirmation has expired — say it again.",
                confirm_id=confirm_id, source=source, at=time.time()))
        self._run(entry.actions)
        return self._record(Outcome(
            "done", entry.sentence, intent=entry.intent.name,
            actions=entry.actions, confirm_id=confirm_id,
            source=entry.source, route="confirmed", at=time.time()))

    def cancel(self, confirm_id: str) -> Outcome:
        with self._lock:
            entry = self._pending.pop(confirm_id, None)
        return self._record(Outcome(
            "refused", f"Cancelled: {entry.sentence}" if entry else "Nothing to cancel.",
            confirm_id=confirm_id, source="ui", at=time.time()))

    # --- plumbing ---------------------------------------------------------

    def _run(self, actions: List[Dict[str, Any]]) -> None:
        """Hand each action to the bridge, or to the UI if it never leaves here.

        A `ui` key marks an action that moves the operator's screen and nothing
        else — showing a camera, switching views. Keeping those in the same list
        is what lets "pull up rover2's camera" be one intent that both selects
        the rover over the radio and points the screen at it.
        """
        for action in actions:
            if "ui" in action:
                self._ui(action)
            else:
                self._dispatch(action)

    def _expire(self, now: float) -> None:
        for pid in [p.id for p in self._pending.values() if now - p.at > CONFIRM_TTL]:
            self._pending.pop(pid, None)

    def _record(self, outcome: Outcome) -> Outcome:
        if not outcome.at:
            outcome.at = time.time()
        with self._lock:
            self._history.append(outcome)
            del self._history[:-HISTORY_MAX]
        if self.on_outcome is not None:
            try:
                self.on_outcome(outcome)
            except Exception:
                # A broken listener must never take down a command that already
                # reached the robot.
                pass
        return outcome

    # --- read-only answers, for MCP and the status intent -----------------

    def describe_fleet(self) -> Dict[str, Any]:
        """Everything an AI needs to decide what to command, in one call.

        Shaped for reading rather than for the wire: an MCP client should not
        have to know that `ex` is a horizontal error in normalised units.
        """
        snap = self._fleet.snapshot(time.monotonic())
        out = []
        for r in snap.get("robots") or []:
            vision = r.get("vision") or {}
            entry = {
                "robot_id": r.get("robot_id"),
                "mode": r.get("mode"),
                "estopped": bool(r.get("estop")),
                "online": bool(r.get("online")),
                "seconds_since_telemetry": r.get("age"),
                "position": ({"lat": r.get("lat"), "lon": r.get("lon"),
                              "heading_deg": r.get("heading")}
                             if r.get("lat") is not None else None),
                "battery_percent": r.get("battery"),
            }
            if vision:
                entry["vision"] = {
                    "detector_running": bool(vision.get("ok")),
                    "sees": vision.get("label"),
                    "confidence": vision.get("conf"),
                    "centred": (abs(vision["ex"]) < 0.1
                                if vision.get("ex") is not None else None),
                }
            if r.get("routine"):
                entry["routine"] = r["routine"]
            if r.get("shooter"):
                entry["shooter"] = r["shooter"]
            out.append(entry)
        return {"selected": snap.get("selected"), "robots": out,
                "places": [{"name": p.get("name") or p["id"], "id": p["id"],
                            "lat": p["lat"], "lon": p["lon"], "kind": p.get("kind")}
                           for p in (self._places.snapshot() if self._places else [])]}
