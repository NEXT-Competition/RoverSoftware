"""The local language model, reached through LM Studio's OpenAI-compatible API.

Its entire job is *classification*: given a sentence and the list of things this
fleet can do right now, name one intent and fill in its arguments. It does not
generate commands, it does not get told the wire protocol, and its output is
never trusted — `intents.plan()` rebuilds the command from validated pieces (see
intents.py). Treating it as a classifier rather than an agent is what makes a 4B
model safe enough to point at a moving robot.

--- Why LM Studio, and why it may be absent ---
LM Studio serves `/v1/chat/completions` on localhost, which means no cloud, no
API key, and no internet — the same constraint that put offline map tiles in
tiles.py. It also means it is a separate application a person has to have
launched. Every failure here is reported as "the model isn't running", never
raised: the fast path (fastpath.py) still stops rovers with the model dead, and
an operator has to be able to tell the difference between "I misheard you" and
"nothing is listening".

--- Structured output ---
We ask for a JSON schema (`response_format: json_schema`), which LM Studio
enforces with grammar-constrained sampling — the model *cannot* emit a malformed
object or an intent name outside the enum. Servers or models that don't support
it fall back to prompted JSON plus a brace-scraper, because a Gemma that returns
```json fences is the normal case, not an error.
"""

from __future__ import annotations

import json
import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .intents import INTENTS, Authority
from .vocabulary import Vocabulary

DEFAULT_BASE_URL = "http://127.0.0.1:1234/v1"
# LM Studio accepts any string when exactly one model is loaded, but naming it
# matters when several are: asking a 20B reasoning model to classify "rover two"
# costs seconds. Resolved against /v1/models at connect time — see discover().
DEFAULT_MODEL_HINT = "gemma"

# Short, because the answer is one small object and a model that starts
# explaining itself has already gone wrong.
MAX_TOKENS = 200
# Deterministic. Two identical orders must not classify differently, and there is
# no creativity wanted anywhere in this task.
TEMPERATURE = 0.0


@dataclass
class Plan:
    """What the model decided. `intent` is None when it declined to guess."""
    intent: Optional[str]
    args: Dict[str, Any]
    say: str
    """A one-line reply for the operator. Shown in the transcript, and the only
    thing the model is allowed to write in its own words."""
    raw: str = ""
    """The unparsed completion, kept for the debug row in the command view — a
    model that is failing is much easier to fix when you can see what it said."""


class LLMError(RuntimeError):
    """The model could not be reached or could not be understood."""


def _schema() -> Dict[str, Any]:
    """The grammar the model is constrained to.

    `intent` is an enum of exactly the registry's keys plus "none", so an
    unrecognised sentence has a legal way to say so instead of inventing a verb.
    `args` is deliberately loose — the values are strings an operator said, and
    validating them is the vocabulary's job, not the sampler's.
    """
    return {
        "type": "object",
        "properties": {
            "intent": {"type": "string", "enum": [*INTENTS.keys(), "none"]},
            "args": {
                "type": "object",
                "properties": {
                    "robot": {"type": "string"},
                    "mode": {"type": "string"},
                    "target": {"type": "string"},
                    "place": {"type": "string"},
                    "places": {"type": "array", "items": {"type": "string"}},
                    "routine": {"type": "string"},
                    "view": {"type": "string"},
                    "mech": {"type": "string"},
                    "actuator": {"type": "string"},
                    "throttle": {"type": "number"},
                    "steer": {"type": "number"},
                    "power": {"type": "number"},
                },
                "additionalProperties": False,
            },
            "say": {"type": "string"},
        },
        "required": ["intent", "args", "say"],
        "additionalProperties": False,
    }


def _system_prompt(vocab: Vocabulary) -> str:
    """The menu, the live state, and the rules — assembled fresh every call.

    Rebuilt per command rather than cached because the interesting half is the
    vocabulary: a rover that came online two seconds ago has to be in the prompt
    or the model will refuse to address it.
    """
    lines = [
        "You control a fleet of ground rovers from a base station.",
        "Classify the operator's spoken order into ONE intent from this list.",
        "You never write commands — you name an intent and fill in its arguments.",
        "",
        "INTENTS:",
    ]
    for intent in INTENTS.values():
        args = ", ".join(intent.args) or "no arguments"
        lines.append(f"- {intent.name}({args}): {intent.summary}")
        if intent.examples:
            lines.append(f"    said as: {'; '.join(intent.examples[:3])}")
    lines += [
        "",
        "WHAT IS ON THE AIR RIGHT NOW:",
        vocab.as_prompt_context(),
        "",
        "RULES:",
        "- Use the operator's own words for arguments. Do not translate 'rover two'",
        "  into 'rover2' or 'bucket A' into an id — the base station resolves names.",
        "- Omit 'robot' entirely when the operator did not name one. It defaults to",
        f"  the selected rover ({vocab.selected or 'none'}).",
        "- If the order is not clearly one of the intents, answer intent 'none' and",
        "  use 'say' to ask what they meant. Guessing moves a real robot.",
        "- 'say' is one short sentence, spoken back to the operator. No markdown.",
    ]
    # Naming the gated verbs is not a safety mechanism (the executor enforces it
    # regardless), but a model that knows a confirm is coming writes a much more
    # useful 'say' line for the confirm card.
    gated = [i.name for i in INTENTS.values() if i.authority is Authority.CONFIRM]
    lines.append(f"- These need a human to confirm before running: {', '.join(gated)}.")
    return "\n".join(lines)


def _few_shot(vocab: Vocabulary) -> List[Dict[str, str]]:
    """Two worked examples, grounded in this fleet's actual names.

    A small model's accuracy on this task is mostly a function of how close its
    input looks to an example, and the highest-value thing to demonstrate is the
    two-argument case ("align to X") and the refusal — those are the two it gets
    wrong when shown nothing.
    """
    robot = (vocab.robots or ["rover1"])[0]
    place = next((p.get("name") or p["id"] for p in vocab.places), None)
    shots: List[Dict[str, str]] = [
        {"role": "user", "content": f"put {robot} in object align and track the bucket"},
        {"role": "assistant", "content": json.dumps({
            "intent": "set_mode",
            "args": {"robot": robot, "mode": "object align", "target": "bucket"},
            "say": f"{robot} to object align, tracking the bucket."})},
    ]
    if place:
        shots += [
            {"role": "user", "content": f"send {robot} over to {place}"},
            {"role": "assistant", "content": json.dumps({
                "intent": "route_to",
                "args": {"robot": robot, "places": [place]},
                "say": f"Routing {robot} to {place}."})},
        ]
    shots += [
        {"role": "user", "content": "what's the weather like tomorrow"},
        {"role": "assistant", "content": json.dumps({
            "intent": "none", "args": {},
            "say": "That isn't a rover command — say something like a mode, a rover name, or a destination."})},
    ]
    return shots


_FENCE = re.compile(r"```(?:json)?\s*(.+?)\s*```", re.S)


def _parse(text: str) -> Dict[str, Any]:
    """Get an object out of a completion, fenced or not.

    Only needed on the fallback path — with json_schema the server has already
    guaranteed a bare object — but that path is the one that runs on whatever
    someone actually has installed, so it has to cope with a model that wrapped
    its answer in a fence or added a sentence after it.
    """
    text = (text or "").strip()
    if not text:
        raise LLMError("the model returned nothing")
    fenced = _FENCE.search(text)
    if fenced:
        text = fenced.group(1).strip()
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        # Scrape the outermost braces. A model that said "Sure! {...}" is common
        # enough that refusing here would make the fallback path useless.
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise LLMError(f"the model did not return JSON: {text[:120]}")
        try:
            parsed = json.loads(text[start:end + 1])
        except json.JSONDecodeError:
            raise LLMError(f"the model returned broken JSON: {text[:120]}")
    if not isinstance(parsed, dict):
        raise LLMError("the model returned JSON that was not an object")
    return parsed


class LMStudio:
    """A thin OpenAI-compatible client. Blocking; call it off the event loop.

    urllib rather than the `openai` package or httpx on purpose: this is two
    POSTs against localhost, and the base station's dependency list is already
    carrying a robot, a radio and a web server onto a Raspberry Pi.
    """

    def __init__(self, base_url: str = DEFAULT_BASE_URL,
                 model: Optional[str] = None, timeout: float = 12.0):
        self.base_url = (base_url or DEFAULT_BASE_URL).rstrip("/")
        self.model = model or ""
        self.timeout = timeout
        self._json_schema_ok = True
        """Cleared the first time a server rejects response_format, so we stop
        paying for a failed request per command on a server that lacks it."""

    # --- transport --------------------------------------------------------

    def _post(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json",
                     # LM Studio ignores it; a proxied llama.cpp or vLLM may not.
                     "Authorization": "Bearer local"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:
            detail = e.read().decode(errors="replace")[:300]
            raise LLMError(f"model server returned {e.code}: {detail}")
        except urllib.error.URLError as e:
            raise LLMError(f"no model server at {self.base_url} ({e.reason}) — "
                           "start LM Studio and load a model")
        except TimeoutError:
            raise LLMError(f"the model took longer than {self.timeout:.0f}s")
        except OSError as e:
            raise LLMError(f"could not reach {self.base_url}: {e}")

    def discover(self) -> Optional[str]:
        """Pick a loaded model, preferring one whose id looks like the hint.

        Called on startup and again after any failure, so launching LM Studio
        *after* the base station recovers on its own rather than needing a
        restart in the middle of a match.
        """
        try:
            with urllib.request.urlopen(f"{self.base_url}/models", timeout=4.0) as resp:
                data = json.loads(resp.read().decode())
        except Exception:
            return None
        ids = [m.get("id") for m in (data.get("data") or []) if m.get("id")]
        if not ids:
            return None
        if self.model in ids:
            return self.model
        hinted = [i for i in ids if DEFAULT_MODEL_HINT in i.lower()]
        self.model = (hinted or ids)[0]
        return self.model

    def health(self) -> Tuple[bool, str]:
        """(reachable, description) — what the command view's status pill shows."""
        model = self.discover()
        if model:
            return True, model
        return False, f"no model server at {self.base_url}"

    # --- the one call that matters ---------------------------------------

    def classify(self, text: str, vocab: Vocabulary) -> Plan:
        """Sentence -> intent name + arguments. Raises LLMError on anything else."""
        if not self.model and not self.discover():
            raise LLMError(f"no model loaded at {self.base_url} — open LM Studio, "
                           "load gemma-3n-e4b, and start the local server")

        messages = [{"role": "system", "content": _system_prompt(vocab)}]
        messages += _few_shot(vocab)
        messages.append({"role": "user", "content": text})

        payload: Dict[str, Any] = {
            "model": self.model,
            "messages": messages,
            "temperature": TEMPERATURE,
            "max_tokens": MAX_TOKENS,
            "stream": False,
        }
        if self._json_schema_ok:
            payload["response_format"] = {
                "type": "json_schema",
                "json_schema": {"name": "rover_intent", "strict": True,
                                "schema": _schema()},
            }

        try:
            data = self._post("/chat/completions", payload)
        except LLMError:
            if not self._json_schema_ok:
                raise
            # One retry without the constraint. Servers that don't implement
            # json_schema reject the whole request, and the difference between
            # "your model server is broken" and "your model server is older" is
            # not something an operator should have to diagnose at a competition.
            self._json_schema_ok = False
            payload.pop("response_format", None)
            payload["messages"][0]["content"] += (
                "\n\nAnswer with a single JSON object and nothing else: "
                '{"intent": "...", "args": {...}, "say": "..."}')
            data = self._post("/chat/completions", payload)

        try:
            content = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError):
            raise LLMError(f"unexpected response shape: {json.dumps(data)[:200]}")

        parsed = _parse(content)
        name = parsed.get("intent")
        if name in (None, "", "none", "null"):
            return Plan(None, {}, str(parsed.get("say") or "I didn't catch a command in that."),
                        raw=content)
        args = parsed.get("args")
        return Plan(str(name), args if isinstance(args, dict) else {},
                    str(parsed.get("say") or ""), raw=content)
