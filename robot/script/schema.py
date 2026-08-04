"""Parsing and validating a script document.

Same contract as `routine/schema.py`, and for the same reasons: nothing raises,
everything is bounded, and a document that does not validate is never installed
— the robot keeps the last set that was good rather than ending up with no way
to run anything.

One thing this validator does that the routine one cannot: it COMPILES the
code. A syntax error is caught here, at save time, with a line number, and
reported to the editor as a refusal. The alternative is a script that saves
cleanly and then dies the moment somebody presses Run, at the field, which is
the worst possible moment to discover a missing colon.

Compiling is not running. `compile()` builds a code object and executes nothing,
so a document full of `os.system(...)` is compiled and stored here without
anything happening — it is refused at RUN time by the import whitelist
(runtime.py). The distinction matters: validation must be safe to perform on
anything that arrives over the wire.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

VERSION = 1

# Caps. The binding constraint is the document reassembler's ceiling
# (comms/doc_transfer.py::MAX_CHARS, 32 KB), which exists so anything that can
# put bytes on the radio cannot make a rover run out of memory. Scripts ride
# WiFi rather than the radio, but they arrive through the same reassembler, so
# the whole document has to fit under it with room for the JSON envelope.
MAX_SCRIPTS = 8
MAX_CODE_BYTES = 16384   # ~400 lines; far past any rover script anyone writes
MAX_DOC_BYTES = 28672    # under the 32 KB reassembly cap, envelope included
MAX_NAME_LEN = 60

_ID_RE = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")

# A script with nothing in it is legal — it is what "New script" creates, and
# refusing it would mean you cannot save until you have finished writing. It is
# reported as a warning so the editor can say so without blocking the save.


@dataclass
class Script:
    id: str
    name: str
    code: str
    # Compiled at validation time and kept, so pressing Run does not re-parse
    # and cannot fail with a syntax error the save already accepted. The runner
    # compiles again into its own unit for the traceback filename; this one is
    # the proof that it will.
    ok: bool = True


@dataclass
class ParseResult:
    scripts: Dict[str, Script] = field(default_factory=dict)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _check_id(value: Any, errors: List[str]) -> str:
    text = str(value or "").strip()
    if not _ID_RE.match(text):
        errors.append(f"script id {text!r} is invalid: lower-case letters, "
                      "digits, underscore and hyphen, starting with a letter")
        return ""
    return text


def _check_code(code: str, sid: str, errors: List[str],
                warnings: List[str]) -> bool:
    """Is this Python that will at least start? Compiles it; runs nothing."""
    size = len(code.encode("utf-8"))
    if size > MAX_CODE_BYTES:
        errors.append(f"script {sid!r}: {size} bytes of code, at most "
                      f"{MAX_CODE_BYTES} allowed")
        return False
    if "\x00" in code:
        errors.append(f"script {sid!r}: the code contains a null byte")
        return False
    if not code.strip():
        warnings.append(f"script {sid!r} is empty — it will run and finish "
                        "immediately")
        return True
    try:
        compile(code, f"<{sid}>", "exec")
    except SyntaxError as e:
        # The line number is the whole value of doing this here: an editor can
        # put a marker on it, and an operator can fix it without a round trip.
        errors.append(f"script {sid!r}: line {e.lineno}: {e.msg}")
        return False
    except ValueError as e:
        errors.append(f"script {sid!r}: {e}")
        return False
    return True


def _parse_script(raw: Any, errors: List[str],
                  warnings: List[str]) -> Optional[Script]:
    if not isinstance(raw, dict):
        errors.append("scripts: a script must be an object")
        return None
    sid = _check_id(raw.get("id"), errors)
    if not sid:
        return None
    code = raw.get("code", "")
    if not isinstance(code, str):
        errors.append(f"script {sid!r}: 'code' must be a string")
        return None
    # Normalise line endings. A file pasted or imported from Windows arrives
    # with \r\n, which compiles fine but makes every reported column and every
    # `line.strip()` in a traceback one character wrong.
    code = code.replace("\r\n", "\n").replace("\r", "\n")
    name = str(raw.get("name", "") or sid)[:MAX_NAME_LEN]
    if not _check_code(code, sid, errors, warnings):
        return None
    return Script(id=sid, name=name, code=code)


def parse(doc: Any) -> ParseResult:
    """Validate and compile a script document. Never raises."""
    result = ParseResult()

    if not isinstance(doc, dict):
        result.errors.append("scripts: expected an object")
        return result

    version = doc.get("version", VERSION)
    if version != VERSION:
        result.errors.append(f"scripts: unsupported version {version!r} "
                             f"(this robot speaks version {VERSION})")
        return result

    encoded = len(json.dumps(doc, separators=(",", ":")).encode("utf-8"))
    if encoded > MAX_DOC_BYTES:
        result.errors.append(
            f"scripts: {encoded} bytes in total, at most {MAX_DOC_BYTES} "
            "allowed — delete a script, or move some of it into fewer lines")
        return result

    raw_scripts = doc.get("scripts", [])
    if not isinstance(raw_scripts, list):
        result.errors.append("scripts: 'scripts' must be a list")
        return result
    if len(raw_scripts) > MAX_SCRIPTS:
        result.errors.append(f"scripts: {len(raw_scripts)} scripts, at most "
                             f"{MAX_SCRIPTS} allowed")
        return result

    for raw in raw_scripts:
        script = _parse_script(raw, result.errors, result.warnings)
        if script is None:
            continue
        if script.id in result.scripts:
            result.errors.append(f"scripts: duplicate script id {script.id!r}")
            continue
        result.scripts[script.id] = script

    return result
