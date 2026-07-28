"""Where routine documents live on the robot.

Same contract as tuning.py's override file and layout.py's: atomic write, never
fatal on a corrupt read, and an env var so a dev run can point somewhere
writable. Routines are the one document that IS hot-swappable — they are data
the engine reads rather than hardware a constructor owns — so this is also what
a `put_routines` frame writes through to.
"""

from __future__ import annotations

import json
import os
from typing import Optional

DEFAULT_ROUTINES_PATH = "/var/lib/roversoftware/routines.json"


def routines_path() -> str:
    return os.environ.get("RS_ROUTINES_FILE", DEFAULT_ROUTINES_PATH)


def empty_doc() -> dict:
    return {"version": 1, "routines": []}


def load(path: Optional[str] = None) -> Optional[dict]:
    """Read the saved routines, or None if there aren't any / it's unreadable."""
    path = path or routines_path()
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return None
    except Exception as e:
        print(f"[routines] ignoring unreadable {path}: {e}")
        return None
    if not isinstance(data, dict):
        print(f"[routines] ignoring {path}: expected an object")
        return None
    return data


def save(doc: dict, path: Optional[str] = None) -> Optional[str]:
    """Persist routines. Returns an error string on failure, never raises."""
    path = path or routines_path()
    tmp = f"{path}.tmp"
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=2, sort_keys=True)
            fh.write("\n")
        os.replace(tmp, path)
        return None
    except Exception as e:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        return f"{e}"
