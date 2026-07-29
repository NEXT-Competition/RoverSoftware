"""Named field positions, shared by the whole fleet.

A place is a point somebody stood a robot on and gave a name to: "bucket A",
"start", "the gap in the fence". They live HERE, on the base station, and not in
any robot's routine document, because of what they are for — one rover is driven
up to a bucket in teleop and saves where it is, and every OTHER rover has to be
able to use that position afterwards. A place stored on the robot that captured
it would be a note in the wrong notebook.

That decision is the whole design:

  * fleet-wide, so a waypoint captured by rover1 is on rover2's route without
    anyone re-typing coordinates;
  * persisted here, so it survives a browser reload, a different tablet, and the
    laptop being closed between matches — a field position that has to be
    re-driven every morning is one nobody trusts;
  * NOT sent to robots as places. The editor resolves a place into plain lat/lon
    when it saves a routine (see the `places` key in state/places.ts), so the
    robot keeps running exactly the coordinates it was given and needs no new
    vocabulary. The radio budget in robot/routine/schema.py is unchanged.

Mirrors basestation/settings.py in shape — a bounded whitelist, coerced rather
than trusted, saved atomically, and never raising — because both are edited from
the same page and a corrupt file must leave the base station launchable rather
than dead before a match.
"""

from __future__ import annotations

import json
import os
import re
import threading
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_PATH = "~/.config/roversoftware/places.json"

# The cap is about the cold channel, not about storage: every place rides in the
# settings frame to every connected browser. Sixty-four named points is far more
# than a field has and still under a couple of kilobytes.
MAX_PLACES = 64
MAX_NAME = 40

# Same alphabet the robot uses for routine and state ids (robot/routine/schema.py).
# Nothing on the robot reads a place id today, but ids that are already legal
# there mean this can move onto the robot later without a migration.
_ID_RE = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")

# What a place is, so the map can draw it differently and an operator can find
# the buckets among the markers. Free-form would become a typo per rover.
KINDS = ("bucket", "marker", "start", "hazard")
DEFAULT_KIND = "marker"

# ~1 cm. Beyond this we would be storing GPS noise and paying for it in every
# settings frame.
_PRECISION = 7


def places_path() -> str:
    return os.path.expanduser(os.environ.get("RS_BASE_PLACES", DEFAULT_PATH))


def slugify(name: str, taken: Tuple[str, ...] = ()) -> str:
    """A legal, unique id from whatever the operator typed.

    Ids are generated rather than asked for: the operator is standing in a field
    naming a bucket, and a form that refuses "Bucket A (far)" until they supply
    a slug is a form they fill in wrong under time pressure.
    """
    base = re.sub(r"[^a-z0-9]+", "_", str(name or "").strip().lower()).strip("_")
    if not base or not base[0].isalpha():
        base = f"p_{base}" if base else "place"
    base = base[:32]
    if base not in taken:
        return base
    for n in range(2, 1000):
        suffix = f"_{n}"
        candidate = base[: 32 - len(suffix)] + suffix
        if candidate not in taken:
            return candidate
    return base  # 998 places named the same thing; the cap fired long ago


def _coord(value: Any, lo: float, hi: float) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or not (lo <= number <= hi):  # NaN, or off the planet
        return None
    return round(number, _PRECISION)


def clean(raw: Any, taken: Tuple[str, ...] = ()) -> Tuple[Optional[Dict[str, Any]], str]:
    """Coerce one place. Returns (place, reason-it-was-refused)."""
    if not isinstance(raw, dict):
        return None, "expected an object"
    lat = _coord(raw.get("lat"), -90.0, 90.0)
    lon = _coord(raw.get("lon"), -180.0, 180.0)
    if lat is None or lon is None:
        return None, "needs a valid lat and lon"
    name = str(raw.get("name", "") or "").strip()[:MAX_NAME]
    pid = str(raw.get("id", "") or "").strip()
    if not _ID_RE.match(pid) or pid in taken:
        pid = slugify(name or pid, taken)
    if not name:
        name = pid
    kind = str(raw.get("kind", "") or "").strip()
    return {
        "id": pid,
        "name": name,
        "lat": lat,
        "lon": lon,
        "kind": kind if kind in KINDS else DEFAULT_KIND,
    }, ""


class PlaceStore:
    """Thread-safe list of places, backed by a JSON file.

    Read from the web loop and written from a WebSocket handler, hence the lock.
    Edits replace the whole list rather than patching one entry: the editor
    already holds the whole list, and a per-entry protocol would need ordering,
    delete semantics and conflict rules to express what one array assignment
    says exactly.
    """

    def __init__(self, path: Optional[str] = None, load: bool = True):
        self.path = path or places_path()
        self._places: List[Dict[str, Any]] = []
        self._lock = threading.RLock()
        if load:
            saved = self._read_file()
            if saved:
                self._places = saved
                print(f"[base] places: {len(saved)} from {self.path}")

    # --- internals ----------------------------------------------------------

    def _read_file(self) -> List[Dict[str, Any]]:
        """Saved places, or []. Never raises — see the module docstring."""
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except FileNotFoundError:
            return []
        except Exception as e:
            print(f"[base] ignoring unreadable {self.path}: {e}")
            return []
        if isinstance(data, dict):
            data = data.get("places")
        if not isinstance(data, list):
            print(f"[base] ignoring {self.path}: expected a list of places")
            return []
        return self._coerce(data)[0]

    def _coerce(self, raw: Any) -> Tuple[List[Dict[str, Any]], Dict[str, str]]:
        """Whole-list coercion, dropping what can't be salvaged. Caller locks."""
        out: List[Dict[str, Any]] = []
        rejected: Dict[str, str] = {}
        if not isinstance(raw, list):
            return out, {"*": "expected a list"}
        for index, item in enumerate(raw):
            if len(out) >= MAX_PLACES:
                rejected[str(index)] = f"at most {MAX_PLACES} places"
                continue
            place, reason = clean(item, tuple(p["id"] for p in out))
            if place is None:
                rejected[str(index)] = reason
                continue
            out.append(place)
        return out, rejected

    def _save(self) -> Optional[str]:
        """Write atomically. Returns an error string rather than raising: a
        read-only home directory must not undo an edit that already took
        effect in memory."""
        tmp = f"{self.path}.tmp"
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump({"places": self.snapshot()}, fh, indent=2)
                fh.write("\n")
            os.replace(tmp, self.path)
            return None
        except Exception as e:
            try:
                os.unlink(tmp)
            except Exception:
                # Broad on purpose. This is the cleanup arm of a function whose
                # whole contract is "returns an error, never raises", and the
                # path that got us here can fail unlink in ways that aren't
                # OSError (a null byte in the path raises ValueError). Letting
                # the tidy-up throw would take the base station down over a
                # temp file nobody was going to read.
                pass
            return f"{e}"

    # --- public -------------------------------------------------------------

    def snapshot(self) -> List[Dict[str, Any]]:
        """A copy, safe to hand to the broadcaster while an edit is in flight."""
        with self._lock:
            return [dict(p) for p in self._places]

    def replace(self, raw: Any, save: bool = True) -> Dict[str, Any]:
        """Replace the whole list. Returns a result dict for the client."""
        with self._lock:
            places, rejected = self._coerce(raw)
            self._places = places
            error = self._save() if save else None
        if error:
            print(f"[base] places applied but NOT saved: {error}")
        return {"count": len(places), "rejected": rejected, "save_error": error}
