"""What the operator is allowed to say right now.

Not a fixed word list — a snapshot of live state. The robots are whichever ones
have sent telemetry, the places are whatever somebody saved on the map, and the
routines are whatever was loaded onto the robot being addressed. That is the
point: "send rover2 to bucket A" only means something if rover2 is on the air and
bucket A is on the map, and the model should be told so rather than guessing.

The vocabulary is used three times over, and it has to be one object so those
three never disagree:

  * in the prompt handed to the local model (llm.py), as the list of legal values
  * in the executor's validation (executor.py), as the list it checks against
  * in the UI, so the operator can read what they can say (state/command.ts)

--- Resolving what was heard ---
Speech recognition does not return ids. It returns "rover two", "Rover-2",
"bucket a", "the bucket". `resolve_robot` and `resolve_place` close that gap with
a deliberately small ladder of rules — exact, normalised, spoken-number, prefix,
then a token-overlap score — and they return None rather than a bad guess when
two candidates tie. A wrong rover is worse than a re-ask.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

# Modes the robot's ControlManager arbitrates between. Mirrors the MODES list in
# basestation-ui/src/components/ModeControls.tsx — if one grows, so does the
# other, and tests/test_command_vocabulary.py asserts the overlap.
MODES: Tuple[str, ...] = ("teleop", "object_align", "shooter_align", "waypoint", "routine")

# How an operator actually says a mode out loud. The model is given these, and
# the resolver accepts them, because nobody says "object underscore align".
MODE_ALIASES: Dict[str, str] = {
    "teleop": "teleop",
    "tele op": "teleop",
    "manual": "teleop",
    "driving": "teleop",
    "drive": "teleop",
    "stick": "teleop",
    "object align": "object_align",
    "object": "object_align",
    "align": "object_align",
    "track": "object_align",
    "tracking": "object_align",
    "shooter align": "shooter_align",
    "shooter": "shooter_align",
    "aim": "shooter_align",
    "aiming": "shooter_align",
    "waypoint": "waypoint",
    "waypoints": "waypoint",
    "route": "waypoint",
    "auto": "waypoint",
    "navigate": "waypoint",
    "routine": "routine",
    "sequence": "routine",
    "program": "routine",
}

# Spoken numbers, so "rover two" reaches rover2. Stops at twelve on purpose: a
# fleet larger than that is not being commanded by voice.
_SPOKEN_NUMBERS: Dict[str, str] = {
    "zero": "0", "one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
    "six": "6", "seven": "7", "eight": "8", "nine": "9", "ten": "10",
    "eleven": "11", "twelve": "12",
}

# Homophones Whisper produces for small fleets — "rover to", "rover for".
#
# These are NOT in the table above, and the distinction cost a bug worth
# recording: expanding them globally rewrote "switch to rover 3" as "switch 2
# rover 3", which no longer starts with the lead-in "switch to" and so stopped
# parsing. They are guesses about a rover's *name*, so they are only ever
# applied inside resolve_robot, on a second attempt, after the honest reading
# has failed.
_HOMOPHONES: Dict[str, str] = {"to": "2", "too": "2", "for": "4", "fore": "4",
                               "won": "1", "ate": "8", "ford": "4"}

_WORD = re.compile(r"[a-z0-9]+")


def normalise(text: str) -> str:
    """Lowercase, strip punctuation, collapse whitespace, expand spoken numbers.

    Applied to both sides of every comparison so "Rover-2!" and "rover two" land
    on the same string. Number expansion happens per-token, so "bucket one"
    becomes "bucket 1" but "money" is left alone.
    """
    out: List[str] = []
    for token in _WORD.findall((text or "").lower()):
        out.append(_SPOKEN_NUMBERS.get(token) or token)
    return " ".join(out)


def _squash(text: str) -> str:
    """`normalise`, then remove the spaces too — "rover 2" -> "rover2".

    Robot ids are written without a separator but always spoken with one, so
    this is the comparison that actually matches them.
    """
    return normalise(text).replace(" ", "")


def _score(said: str, candidate: str) -> float:
    """How well a spoken phrase matches a name, 0..1. 0 means "not a match".

    Token overlap rather than edit distance: an operator says "bucket A" for a
    place named "bucket A far side", and the useful signal is that the words
    they said appear in the name — not that the strings differ by 9 characters.

    The gate on `said` coverage is the important part, and it is there because
    of a real failure: without it, "select rover1" scored 0.5 against a rover
    named `rover1` and the fast path happily reported the operator had asked for
    a rover called "select rover1". Requiring that most of what was *said* is
    part of the name is what distinguishes naming a thing from talking about it.
    """
    said_tokens = set(normalise(said).split())
    cand_tokens = set(normalise(candidate).split())
    if not said_tokens or not cand_tokens:
        return 0.0
    shared = said_tokens & cand_tokens
    if not shared:
        return 0.0
    said_cov = len(shared) / len(said_tokens)
    cand_cov = len(shared) / len(cand_tokens)
    if said_cov < 0.6:
        return 0.0  # they said a sentence, not this name
    return (said_cov + cand_cov) / 2.0


def _singular(said: str) -> Optional[str]:
    """"cones" -> "cone", or None if the phrase wasn't plural.

    Operators pluralise the thing they are naming constantly — "track the
    cones", "stop the rovers" — while the detector's label and the saved place
    are singular. Deliberately naive: this only has to undo the two endings
    English actually uses out loud, and a wrong singular simply fails to match
    rather than matching something else.
    """
    tokens = normalise(said).split()
    if not tokens:
        return None
    last = tokens[-1]
    if len(last) > 3 and last.endswith("es") and last[-3] in "sxzo":
        tokens[-1] = last[:-2]
    elif len(last) > 2 and last.endswith("s") and not last.endswith("ss"):
        tokens[-1] = last[:-1]
    else:
        return None
    return " ".join(tokens)


def _best(said: str, candidates: Sequence[Tuple[str, str]]) -> Optional[str]:
    """Pick one of `(key, display)` for what was said, or None if it's a coin flip.

    The ladder, strictest first. Each rung returns only on a *unique* hit, so an
    ambiguous phrase falls through to the next rule rather than resolving to
    whichever candidate happened to be first in the list.
    """
    hit = _best_exact(said, candidates)
    if hit is not None:
        return hit
    # Retry singularised, and only then. Running it first would let "rovers"
    # mean a rover; running it last means every literal reading has already had
    # its chance, and the uniqueness rules still apply to this pass.
    singular = _singular(said)
    return _best_exact(singular, candidates) if singular else None


def _best_exact(said: str, candidates: Sequence[Tuple[str, str]]) -> Optional[str]:
    if not said or not candidates:
        return None
    said_n, said_s = normalise(said), _squash(said)

    for key_of in (
        # 1. exact id, 2. normalised display name, 3. space-insensitive
        lambda k, d: k.lower() == said.strip().lower(),
        lambda k, d: normalise(d) == said_n or normalise(k) == said_n,
        lambda k, d: _squash(d) == said_s or _squash(k) == said_s,
        # 4. the spoken phrase is a prefix of the name ("bucket" -> "bucket A")
        lambda k, d: _squash(d).startswith(said_s) or _squash(k).startswith(said_s),
    ):
        hits = [k for k, d in candidates if key_of(k, d)]
        if len(hits) == 1:
            return hits[0]
        if len(hits) > 1:
            return None  # genuinely ambiguous — make the operator repeat it

    # 5. token overlap, and only when one candidate clearly wins. The margin
    #    stops "bucket" from silently picking between "bucket A" and "bucket B"
    #    — they tie, so nobody wins and the operator is asked to repeat it.
    scored = sorted(((_score(said, d), k) for k, d in candidates), reverse=True)
    if scored and scored[0][0] >= 0.5 and (len(scored) == 1 or scored[0][0] - scored[1][0] >= 0.2):
        return scored[0][1]
    return None


@dataclass
class Vocabulary:
    """Everything sayable, as of one instant.

    Built per command rather than cached: a rover that just came online should be
    addressable on the very next sentence, and a place saved thirty seconds ago
    should already be a destination.
    """

    robots: List[str] = field(default_factory=list)
    """robot_ids currently known to the fleet, in snapshot order."""

    online: List[str] = field(default_factory=list)
    """The subset actually reporting telemetry. Commanding an offline rover is
    allowed — the radio may just be lossy — but the executor says so in its
    reply, because "nothing happened" and "it never heard you" look identical."""

    selected: Optional[str] = None
    """Who an order goes to when the operator doesn't name anyone."""

    places: List[Dict[str, Any]] = field(default_factory=list)
    """Saved field positions (basestation/places.py). Each has id/name/lat/lon/kind."""

    routines: Dict[str, List[str]] = field(default_factory=dict)
    """robot_id -> the routine ids loaded on it, from the documents cold channel."""

    labels: List[str] = field(default_factory=list)
    """Object classes the detector can see, gathered from the robots' vision
    status. This is what makes "align to the bucket" resolvable into a
    `vision.target_label` — without it, the target is whatever the model
    invented."""

    modes: Tuple[str, ...] = MODES

    # --- resolution -------------------------------------------------------

    def resolve_robot(self, said: Optional[str]) -> Optional[str]:
        """Spoken rover name -> robot_id, or None (including "ask again")."""
        if not said:
            return None
        if normalise(said) in ("all", "everyone", "everybody", "all rovers",
                              "fleet", "all robots", "the fleet", "every rover"):
            return "*"
        candidates = [(r, r) for r in self.robots]
        hit = _best(said, candidates)
        if hit is not None:
            return hit
        # Second attempt with the homophones applied. Only for rovers, only once
        # the plain reading has failed, so "rover to" can still reach rover2
        # without "switch to" being rewritten out from under the parser.
        guess = " ".join(_HOMOPHONES.get(t, t) for t in normalise(said).split())
        return _best(guess, candidates) if guess != normalise(said) else None

    def resolve_place(self, said: Optional[str]) -> Optional[Dict[str, Any]]:
        """Spoken place name -> the place dict, or None."""
        if not said:
            return None
        key = _best(said, [(p["id"], p.get("name") or p["id"]) for p in self.places])
        return next((p for p in self.places if p["id"] == key), None) if key else None

    def resolve_mode(self, said: Optional[str]) -> Optional[str]:
        """Spoken mode -> a canonical mode name, or None.

        Checks the alias table before the fuzzy ladder: "drive" must mean teleop,
        not lose a token-overlap contest to some other mode.
        """
        if not said:
            return None
        n = normalise(said).replace("_", " ")
        if n in MODE_ALIASES:
            return MODE_ALIASES[n]
        if n.replace(" ", "_") in self.modes:
            return n.replace(" ", "_")
        return _best(said, [(m, m.replace("_", " ")) for m in self.modes])

    def resolve_label(self, said: Optional[str]) -> Optional[str]:
        """Spoken object name -> a detector class label, or None.

        Falls back to the normalised phrase when the fleet has not told us what
        its model can see (no robot online, or vision disabled). A label the
        detector doesn't have is harmless — robot/sensors/detector.py logs that
        nothing will ever match — and refusing here would make the feature
        unusable on a bench with no camera attached.
        """
        if not said:
            return None
        hit = _best(said, [(l, l) for l in self.labels]) if self.labels else None
        return hit or normalise(said) or None

    def resolve_routine(self, robot_id: Optional[str], said: Optional[str]) -> Optional[str]:
        if not said:
            return None
        ids = self.routines.get(robot_id or "", [])
        return _best(said, [(r, r.replace("_", " ")) for r in ids])

    # --- for the model and the UI ----------------------------------------

    def as_prompt_context(self) -> str:
        """The live half of the system prompt — see llm.py::build_messages."""
        lines = [
            f"robots: {', '.join(self.robots) or '(none on the air)'}",
            f"currently selected: {self.selected or '(none)'}",
        ]
        if self.places:
            lines.append("places: " + ", ".join(
                f"{p.get('name') or p['id']}" for p in self.places))
        else:
            lines.append("places: (none saved yet)")
        if self.labels:
            lines.append(f"detector object labels: {', '.join(self.labels)}")
        routines = sorted({r for ids in self.routines.values() for r in ids})
        if routines:
            lines.append(f"routines: {', '.join(routines)}")
        lines.append(f"modes: {', '.join(self.modes)}")
        return "\n".join(lines)

    def snapshot(self) -> Dict[str, Any]:
        """The shape the UI reads (basestation-ui/src/state/command.ts)."""
        return {
            "robots": list(self.robots),
            "online": list(self.online),
            "selected": self.selected,
            "places": [{"id": p["id"], "name": p.get("name") or p["id"]} for p in self.places],
            "routines": {k: list(v) for k, v in self.routines.items()},
            "labels": list(self.labels),
            "modes": list(self.modes),
        }


def build(fleet, places_store, now: float) -> Vocabulary:
    """Read the live vocabulary off the fleet and the place store.

    Tolerant by construction: a half-populated snapshot (a robot that has sent
    one telemetry frame and nothing else) must still yield a usable vocabulary,
    because that is exactly the moment somebody is trying to talk to it.
    """
    snap = fleet.snapshot(now)
    robots = snap.get("robots") or []
    ids = [r.get("robot_id") for r in robots if r.get("robot_id")]

    labels: List[str] = []
    for r in robots:
        vision = r.get("vision") or {}
        label = vision.get("label")
        if label and label not in labels:
            labels.append(label)

    routines: Dict[str, List[str]] = {}
    for rid, docs in (fleet.documents() or {}).items():
        doc = (docs or {}).get("routines") or {}
        found = [r.get("id") for r in (doc.get("routines") or []) if r.get("id")]
        if found:
            routines[rid] = found

    return Vocabulary(
        robots=ids,
        online=[r["robot_id"] for r in robots if r.get("online")],
        selected=snap.get("selected"),
        places=places_store.snapshot() if places_store is not None else [],
        routines=routines,
        labels=labels,
    )
