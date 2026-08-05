"""Where scripts live on the robot between power cycles.

The same contract as `routine/store.py`, `layout.py` and `tuning.py`: atomic
write, never fatal on a corrupt read, and an env var so a dev run can point
somewhere writable. Scripts hot-swap exactly as routines do — they are text the
runner compiles, not hardware a constructor owns — so this is also what a
`put_scripts` frame writes through to.
"""

from __future__ import annotations

import json
import os
from typing import Optional

DEFAULT_SCRIPTS_PATH = "/var/lib/roversoftware/scripts.json"


def scripts_path() -> str:
    return os.environ.get("RS_SCRIPTS_FILE", DEFAULT_SCRIPTS_PATH)


def empty_doc() -> dict:
    return {"version": 1, "scripts": []}


def load(path: Optional[str] = None) -> Optional[dict]:
    """Read the saved scripts, or None if there aren't any / it's unreadable."""
    path = path or scripts_path()
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return None
    except Exception as e:
        print(f"[scripts] ignoring unreadable {path}: {e}")
        return None
    if not isinstance(data, dict):
        print(f"[scripts] ignoring {path}: expected an object")
        return None
    return data


def save(doc: dict, path: Optional[str] = None) -> Optional[str]:
    """Persist scripts. Returns an error string on failure, never raises."""
    path = path or scripts_path()
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
