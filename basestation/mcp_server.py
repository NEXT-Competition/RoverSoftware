#!/usr/bin/env python3
"""MCP server: let Claude — or any MCP client — command the fleet.

Run it from an MCP client's config; it speaks JSON-RPC 2.0 over stdio and talks
to a running base station over the same WebSocket the dashboard uses.

    {"mcpServers": {"rover": {
        "command": "python", "args": ["-m", "basestation.mcp_server"],
        "env": {"RS_BASE_WS": "ws://127.0.0.1:8000/ws"}}}}

--- Why a separate process, and not an endpoint on the bridge ---
Because it goes through the front door. Every tool below sends exactly the
action a browser sends, over the same socket, and lands in the same
`CommandExecutor` — so the confirmation gate, the whitelist and the audit log
apply to an AI without a single line of "unless it's MCP" anywhere. An endpoint
mounted inside the FastAPI app would have been a second path into the fleet, and
a second path is a second set of rules to get wrong.

It also means this file can run on somebody's laptop while the base station runs
on the Pi, which is the normal case: the person pointing an AI at the fleet is
not sitting at the base station.

--- Why no `mcp` SDK ---
MCP over stdio is newline-delimited JSON-RPC. Implementing it here costs about a
hundred lines and keeps the base station's dependency list — which already has
to install on a Raspberry Pi — exactly as long as it was. The same reasoning is
in pyproject.toml about torch, and in llm.py about the openai package.

--- What an AI may and may not do ---
Nothing here decides that. The tool list is *generated* from
`basestation/command/intents.py`, and each tool's authority is whatever that
registry says. Adding an intent gives every MCP client the same new verb the
voice interface gets, with the same gate, automatically. Tools whose intent is
CONFIRM return "waiting for a human" and stop — the operator taps the card in
the command view, and the AI can see the result on the next get_fleet.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from typing import Any, Dict, List, Optional

from basestation.command.intents import INTENTS, Authority

# Bumped only when the wire format changes. Clients send their own; we answer
# with ours and they negotiate down, which is the protocol's design.
PROTOCOL_VERSION = "2025-06-18"
SERVER_NAME = "roversoftware-fleet"
SERVER_VERSION = "1.0.0"

DEFAULT_WS = os.environ.get("RS_BASE_WS", "ws://127.0.0.1:8000/ws")

# How long to wait for the bridge to answer a command with an outcome. Generous
# because a command that has to cross a radio to a rover is not instant, and a
# tool that times out while the rover is actually turning is worse than one that
# waits a moment.
REPLY_TIMEOUT = 8.0


# ---------------------------------------------------------------------------
# The link to the base station.
# ---------------------------------------------------------------------------

class Bridge:
    """One WebSocket to the bridge, reconnected on demand.

    Kept open between tool calls: an AI that asks for the fleet, thinks, and
    then issues an order should not pay a connection setup for each step. But
    every call re-checks the socket, because the base station gets restarted
    between matches and an MCP client does not.
    """

    def __init__(self, url: str = DEFAULT_WS):
        self.url = url
        self._ws: Optional[Any] = None
        self._fleet: Dict[str, Any] = {}
        self._command: Dict[str, Any] = {}
        self._outcomes: "asyncio.Queue[Dict[str, Any]]" = asyncio.Queue()
        self._reader: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    async def connect(self) -> None:
        async with self._lock:
            if self._ws is not None:
                return
            try:
                import websockets
            except ImportError:
                raise RuntimeError(
                    "the `websockets` package is required — pip install websockets")
            try:
                self._ws = await asyncio.wait_for(
                    websockets.connect(self.url, max_size=4 * 1024 * 1024), timeout=6.0)
            except Exception as e:
                raise RuntimeError(
                    f"could not reach the base station at {self.url} ({e}). "
                    "Is run_basestation.py running, and is RS_BASE_WS pointing at it?")
            self._reader = asyncio.create_task(self._read_loop())
            # Wait for the first frames so the very first get_fleet has data
            # rather than an empty snapshot the model would reason from.
            for _ in range(60):
                if self._fleet and self._command:
                    break
                await asyncio.sleep(0.05)

    async def _read_loop(self) -> None:
        try:
            async for raw in self._ws:  # type: ignore[union-attr]
                if isinstance(raw, bytes):
                    continue
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                kind = msg.get("type")
                if kind == "fleet":
                    self._fleet = msg
                elif kind == "command":
                    self._command = msg
                    if msg.get("outcome"):
                        await self._outcomes.put(msg["outcome"])
        except Exception:
            pass
        finally:
            self._ws = None  # next call reconnects

    async def send(self, payload: Dict[str, Any]) -> None:
        await self.connect()
        assert self._ws is not None
        await self._ws.send(json.dumps(payload))

    async def command(self, intent: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Issue an intent and wait for the outcome the executor produced.

        Drains stale outcomes first: another operator may have spoken while this
        client was thinking, and returning *their* result as this tool's answer
        would be the worst kind of wrong — plausible and about the wrong rover.
        """
        await self.connect()
        while not self._outcomes.empty():
            self._outcomes.get_nowait()
        await self.send({"action": "command_intent", "intent": intent,
                         "args": args, "source": "mcp"})
        try:
            return await asyncio.wait_for(self._outcomes.get(), timeout=REPLY_TIMEOUT)
        except asyncio.TimeoutError:
            return {"status": "error",
                    "say": "the base station did not answer within "
                           f"{REPLY_TIMEOUT:.0f}s — it may be restarting"}

    async def fleet(self) -> Dict[str, Any]:
        await self.connect()
        return self._fleet

    async def command_state(self) -> Dict[str, Any]:
        await self.connect()
        return self._command


# ---------------------------------------------------------------------------
# Tools, generated from the intent registry.
# ---------------------------------------------------------------------------

# What each intent argument means, said once. The descriptions matter more than
# usual here: they are the only thing telling a model that `robot` takes a
# spoken name rather than an id, which is the difference between a working tool
# and one that fails on every call.
_ARG_SCHEMA: Dict[str, Dict[str, Any]] = {
    "robot": {"type": "string", "description":
              "Which rover, by name (e.g. 'rover2'). Omit to use the currently "
              "selected rover. 'all' is accepted by estop only."},
    "mode": {"type": "string", "description":
             "teleop, object_align, shooter_align, waypoint or routine."},
    "target": {"type": "string", "description":
               "Object class for the detector to track, e.g. 'bucket'. Only "
               "meaningful with object_align or shooter_align."},
    "place": {"type": "string", "description": "A saved place name (see list_places)."},
    "places": {"type": "array", "items": {"type": "string"}, "description":
               "Saved place names, visited in the order given."},
    "routine": {"type": "string", "description": "A routine id loaded on that rover."},
    "view": {"type": "string", "description": "map, command or settings."},
    "mech": {"type": "string", "description": "Mechanism name from the rover's layout."},
    "actuator": {"type": "string", "description": "One actuator within that mechanism."},
    "throttle": {"type": "number", "description": "-1 (full reverse) to 1 (full forward)."},
    "steer": {"type": "number", "description": "-1 (full left) to 1 (full right)."},
    "power": {"type": "number", "description": "-1 to 1."},
}

# Read-only tools. Hand-written because they answer questions rather than issue
# intents, and an AI needs these before it can sensibly use anything else.
_READ_TOOLS: List[Dict[str, Any]] = [
    {
        "name": "get_fleet",
        "description":
            "What every rover is doing right now: mode, e-stop state, whether it "
            "is online, position, battery, what its camera detects, and the saved "
            "places on the field. Call this before commanding anything — rover "
            "names and place names come from here.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "get_pending_confirmations",
        "description":
            "Commands waiting for a human to approve at the base station. Tools "
            "marked 'needs human confirmation' land here; they do not execute "
            "until an operator taps the card. Use this to see whether one went "
            "through.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "get_command_log",
        "description":
            "Recent orders and what came of them, whoever gave them — voice, the "
            "dashboard, or another MCP client. Useful for finding out why a rover "
            "is doing something you did not ask for.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "say",
        "description":
            "Send a plain-English order to the base station's own command "
            "recogniser, exactly as if it had been spoken. Use this only for "
            "phrasings the specific tools below cannot express — the specific "
            "tools are faster and cannot be misunderstood.",
        "inputSchema": {
            "type": "object",
            "properties": {"text": {"type": "string", "description":
                                    "e.g. 'send rover2 to bucket A then start'"}},
            "required": ["text"], "additionalProperties": False,
        },
    },
]


def _tool_for(name: str) -> Dict[str, Any]:
    intent = INTENTS[name]
    properties = {a: _ARG_SCHEMA[a] for a in intent.args if a in _ARG_SCHEMA}
    description = intent.summary
    if intent.authority is Authority.CONFIRM:
        # Said in the description, not just enforced server-side, so a model
        # plans around the gate rather than calling the tool and being confused
        # by a result that says nothing happened.
        description += (" NEEDS HUMAN CONFIRMATION: this returns a pending "
                        "request and does nothing until an operator approves it "
                        "at the base station.")
    if intent.authority is Authority.ALWAYS:
        description += " Takes effect immediately and cannot be gated."
    if intent.examples:
        description += " Spoken as: " + "; ".join(f'"{e}"' for e in intent.examples[:3])
    return {
        "name": name,
        "description": description,
        "inputSchema": {"type": "object", "properties": properties,
                        "additionalProperties": False},
    }


def tool_list() -> List[Dict[str, Any]]:
    return [*_READ_TOOLS, *(_tool_for(n) for n in INTENTS)]


# ---------------------------------------------------------------------------
# Rendering results for a model to read.
# ---------------------------------------------------------------------------

def _render_outcome(outcome: Dict[str, Any]) -> str:
    status = outcome.get("status")
    say = outcome.get("say") or ""
    if status == "done":
        return f"Done. {say}"
    if status == "pending":
        return (f"Waiting for a human. {say} An operator has to approve this at "
                "the base station; nothing has been sent to the rover yet. "
                "Check get_pending_confirmations, or get_fleet in a few seconds.")
    if status == "refused":
        return f"Refused: {say}"
    if status == "unheard":
        return f"Not understood: {say}"
    return f"Error: {say}"


async def call_tool(bridge: Bridge, name: str, args: Dict[str, Any]) -> str:
    if name == "get_fleet":
        fleet = await bridge.fleet()
        state = await bridge.command_state()
        vocab = state.get("vocabulary") or {}
        return json.dumps({
            "selected": fleet.get("selected"),
            "robots": [_render_robot(r) for r in (fleet.get("robots") or [])],
            "places": vocab.get("places") or [],
            "detector_labels": vocab.get("labels") or [],
            "routines": vocab.get("routines") or {},
        }, indent=2)

    if name == "get_pending_confirmations":
        state = await bridge.command_state()
        pending = state.get("pending") or []
        if not pending:
            return "Nothing is waiting for approval."
        return json.dumps(pending, indent=2)

    if name == "get_command_log":
        state = await bridge.command_state()
        log = state.get("history") or []
        return json.dumps([
            {"at": h.get("at"), "source": h.get("source"), "said": h.get("transcript"),
             "intent": h.get("intent"), "status": h.get("status"), "result": h.get("say")}
            for h in log[-25:]], indent=2)

    if name == "say":
        await bridge.send({"action": "command_text",
                           "text": str(args.get("text") or ""), "source": "mcp"})
        try:
            outcome = await asyncio.wait_for(bridge._outcomes.get(), timeout=REPLY_TIMEOUT)
        except asyncio.TimeoutError:
            return ("The base station did not answer. If no language model is "
                    "loaded there, only simple orders (stop, a rover name, a mode, "
                    "the camera) are recognised — use the specific tools instead.")
        return _render_outcome(outcome)

    if name in INTENTS:
        return _render_outcome(await bridge.command(name, args))

    return f"Unknown tool: {name}"


def _render_robot(r: Dict[str, Any]) -> Dict[str, Any]:
    """The wire shape, translated into words a model can reason about.

    `ex` is a normalised horizontal error and `age` is seconds since telemetry;
    an MCP client should not have to know either. This is the same translation
    `CommandExecutor.describe_fleet` does, kept here because this process reads
    the socket rather than the executor.
    """
    vision = r.get("vision") or {}
    out: Dict[str, Any] = {
        "robot_id": r.get("robot_id"),
        "mode": r.get("mode"),
        "estopped": bool(r.get("estop")),
        "online": bool(r.get("online")),
    }
    if r.get("age") is not None:
        out["seconds_since_telemetry"] = round(r["age"], 1)
    if r.get("lat") is not None:
        out["position"] = {"lat": r["lat"], "lon": r["lon"],
                           "heading_deg": r.get("heading")}
    if r.get("battery") is not None:
        out["battery_percent"] = r["battery"]
    if vision:
        out["camera"] = {
            "detector_running": bool(vision.get("ok")),
            "sees": vision.get("label") or "nothing",
            "confidence": vision.get("conf"),
            "centred": (abs(vision["ex"]) < 0.1 if vision.get("ex") is not None else None),
        }
    if r.get("routine"):
        out["routine"] = r["routine"]
    if r.get("shooter"):
        out["shooter"] = r["shooter"]
    return out


# ---------------------------------------------------------------------------
# JSON-RPC over stdio.
# ---------------------------------------------------------------------------

def _result(rpc_id: Any, payload: Dict[str, Any]) -> Dict[str, Any]:
    return {"jsonrpc": "2.0", "id": rpc_id, "result": payload}


def _error(rpc_id: Any, code: int, message: str) -> Dict[str, Any]:
    return {"jsonrpc": "2.0", "id": rpc_id, "error": {"code": code, "message": message}}


async def handle(bridge: Bridge, request: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    method = request.get("method")
    rpc_id = request.get("id")
    params = request.get("params") or {}

    if method == "initialize":
        return _result(rpc_id, {
            "protocolVersion": PROTOCOL_VERSION,
            "capabilities": {"tools": {"listChanged": False}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
            "instructions": (
                "You are commanding real ground rovers over a radio link. Call "
                "get_fleet before acting so you use real rover and place names, "
                "and again afterwards to check what happened — a command reaching "
                "the base station does not prove the rover received it. Some "
                "tools need a human to approve them; that is not an error, and "
                "retrying will only queue a second request. If something is going "
                "wrong, estop is never gated and takes effect immediately."),
        })

    # Notifications carry no id and must not be answered.
    if method in ("notifications/initialized", "notifications/cancelled"):
        return None

    if method == "ping":
        return _result(rpc_id, {})

    if method == "tools/list":
        return _result(rpc_id, {"tools": tool_list()})

    if method == "tools/call":
        name = params.get("name") or ""
        args = params.get("arguments") or {}
        try:
            text = await call_tool(bridge, name, args)
            is_error = False
        except Exception as e:
            # Reported as tool content rather than a protocol error on purpose:
            # a model can read and act on "the base station is not running",
            # whereas a JSON-RPC error is usually surfaced as a dead end.
            text, is_error = f"{e}", True
        return _result(rpc_id, {"content": [{"type": "text", "text": text}],
                                "isError": is_error})

    if method in ("resources/list", "prompts/list"):
        # Declared unsupported in `capabilities`, but clients probe anyway and an
        # error here shows up as a broken server.
        key = "resources" if method.startswith("resources") else "prompts"
        return _result(rpc_id, {key: []})

    if rpc_id is None:
        return None
    return _error(rpc_id, -32601, f"method not found: {method}")


async def serve(url: str = DEFAULT_WS) -> None:
    bridge = Bridge(url)
    loop = asyncio.get_running_loop()
    reader = asyncio.StreamReader()
    await loop.connect_read_pipe(lambda: asyncio.StreamReaderProtocol(reader), sys.stdin)

    def write(msg: Dict[str, Any]) -> None:
        sys.stdout.write(json.dumps(msg) + "\n")
        sys.stdout.flush()

    while True:
        line = await reader.readline()
        if not line:
            break
        try:
            request = json.loads(line)
        except ValueError:
            continue
        if not isinstance(request, dict):
            continue
        try:
            response = await handle(bridge, request)
        except Exception as e:
            response = _error(request.get("id"), -32603, str(e))
        if response is not None:
            write(response)


def main() -> None:
    import argparse
    p = argparse.ArgumentParser(
        description="MCP server exposing the rover fleet to any MCP client.")
    p.add_argument("--ws", default=DEFAULT_WS,
                   help="base station WebSocket (default: $RS_BASE_WS or "
                        "ws://127.0.0.1:8000/ws)")
    p.add_argument("--list-tools", action="store_true",
                   help="print the generated tool list and exit — for checking "
                        "what an AI would be able to do before connecting one")
    args = p.parse_args()

    if args.list_tools:
        for tool in tool_list():
            print(f"{tool['name']}\n    {tool['description']}\n")
        return
    try:
        asyncio.run(serve(args.ws))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
