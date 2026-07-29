"""The MCP surface: what an AI can see, what it can do, and what it can't.

The point of these is the *generation* claim. The tool list is not hand-written;
it comes from `basestation/command/intents.py`, so an intent added for the voice
interface reaches every MCP client with the same authority automatically. If
that link ever breaks — a tool that exists without an intent, or a gated intent
whose tool doesn't say so — an AI ends up with more authority than the operator
granted, and these tests are what catches it.
"""

import asyncio
import json

import pytest

from basestation import mcp_server
from basestation.command.intents import INTENTS, Authority


def rpc(bridge, method, params=None, rpc_id=1):
    return asyncio.run(mcp_server.handle(
        bridge, {"jsonrpc": "2.0", "id": rpc_id, "method": method,
                 "params": params or {}}))


class FakeBridge:
    """Stands in for the WebSocket to a running base station."""

    def __init__(self, outcome=None, fleet=None, command=None):
        self.sent = []
        self._outcome = outcome or {"status": "done", "say": "select rover2"}
        self._fleet = fleet or {
            "type": "fleet", "selected": "rover1",
            "robots": [{"robot_id": "rover1", "mode": "object_align", "estop": False,
                        "online": True, "age": 0.2, "lat": 1.0, "lon": 2.0,
                        "heading": 90.0, "battery": 88.0,
                        "vision": {"ok": True, "label": "bucket", "conf": 0.91,
                                   "ex": 0.03}}]}
        self._command = command or {
            "type": "command", "pending": [], "history": [],
            "vocabulary": {"robots": ["rover1"], "places": [{"id": "a", "name": "bucket A"}],
                           "labels": ["bucket"], "routines": {}}}

    async def connect(self):
        pass

    async def send(self, payload):
        self.sent.append(payload)

    async def command(self, intent, args):
        self.sent.append({"intent": intent, "args": args})
        return self._outcome

    async def fleet(self):
        return self._fleet

    async def command_state(self):
        return self._command


# --- protocol -------------------------------------------------------------

def test_initialize_answers_with_a_protocol_version():
    r = rpc(FakeBridge(), "initialize", {"protocolVersion": "2025-06-18"})
    assert r["result"]["protocolVersion"] == mcp_server.PROTOCOL_VERSION
    assert r["result"]["serverInfo"]["name"] == "roversoftware-fleet"
    assert r["result"]["capabilities"]["tools"] is not None


def test_initialize_warns_the_model_about_real_hardware():
    instructions = rpc(FakeBridge(), "initialize")["result"]["instructions"]
    assert "estop" in instructions
    assert "human" in instructions.lower()


def test_notifications_get_no_reply():
    """A response to a notification is a protocol violation and some clients
    treat it as a fatal desync."""
    assert asyncio.run(mcp_server.handle(
        FakeBridge(), {"jsonrpc": "2.0", "method": "notifications/initialized"})) is None


def test_ping_and_unsupported_probes_do_not_error():
    bridge = FakeBridge()
    assert rpc(bridge, "ping")["result"] == {}
    # Declared unsupported, but clients probe anyway; an error reads as broken.
    assert rpc(bridge, "resources/list")["result"] == {"resources": []}
    assert rpc(bridge, "prompts/list")["result"] == {"prompts": []}


def test_an_unknown_method_is_a_proper_json_rpc_error():
    r = rpc(FakeBridge(), "does/not/exist")
    assert r["error"]["code"] == -32601


# --- the generated tool list ----------------------------------------------

def test_every_intent_becomes_a_tool():
    names = {t["name"] for t in mcp_server.tool_list()}
    assert set(INTENTS) <= names


def test_no_tool_exists_without_an_intent_behind_it():
    """The direction that matters for safety: a hand-added tool would be a verb
    with no whitelist entry and no authority setting."""
    read_only = {t["name"] for t in mcp_server._READ_TOOLS}
    for tool in mcp_server.tool_list():
        assert tool["name"] in INTENTS or tool["name"] in read_only, tool["name"]


@pytest.mark.parametrize("name", [n for n, i in INTENTS.items()
                                  if i.authority is Authority.CONFIRM])
def test_gated_tools_say_so_in_their_description(name):
    """Enforced server-side regardless, but a model that knows the gate is
    coming plans around it instead of retrying and queueing a second request."""
    tool = next(t for t in mcp_server.tool_list() if t["name"] == name)
    assert "NEEDS HUMAN CONFIRMATION" in tool["description"]


def test_every_tool_has_a_valid_input_schema():
    for tool in mcp_server.tool_list():
        schema = tool["inputSchema"]
        assert schema["type"] == "object"
        assert schema["additionalProperties"] is False
        for prop in schema["properties"].values():
            assert "description" in prop, tool["name"]
        assert tool["description"].strip()


def test_estop_is_advertised_as_ungated():
    tool = next(t for t in mcp_server.tool_list() if t["name"] == "estop")
    assert "immediately" in tool["description"]


# --- calling tools --------------------------------------------------------

def test_a_direct_tool_reports_what_happened():
    bridge = FakeBridge()
    r = rpc(bridge, "tools/call", {"name": "select_robot", "arguments": {"robot": "rover two"}})
    assert r["result"]["isError"] is False
    assert "Done" in r["result"]["content"][0]["text"]
    assert bridge.sent == [{"intent": "select_robot", "args": {"robot": "rover two"}}]


def test_a_gated_tool_reports_that_nothing_was_sent():
    """The wording matters: a model that reads "waiting" as "failed" will retry
    and queue a second confirmation for the operator to deal with."""
    bridge = FakeBridge(outcome={"status": "pending", "say": "FIRE rover2",
                                 "confirm_id": "c1"})
    r = rpc(bridge, "tools/call", {"name": "fire", "arguments": {}})
    text = r["result"]["content"][0]["text"]
    assert "Waiting for a human" in text
    assert "nothing has been sent to the rover" in text


def test_a_refusal_is_returned_as_readable_text():
    bridge = FakeBridge(outcome={"status": "refused",
                                 "say": "no rover matching 'rover9'"})
    r = rpc(bridge, "tools/call", {"name": "select_robot", "arguments": {"robot": "rover9"}})
    assert "Refused" in r["result"]["content"][0]["text"]


def test_get_fleet_translates_the_wire_into_words():
    r = rpc(FakeBridge(), "tools/call", {"name": "get_fleet", "arguments": {}})
    data = json.loads(r["result"]["content"][0]["text"])
    robot = data["robots"][0]
    assert robot["camera"]["sees"] == "bucket"
    assert robot["camera"]["centred"] is True   # not a raw `ex` of 0.03
    assert data["places"] == [{"id": "a", "name": "bucket A"}]


def test_get_fleet_reports_a_silent_rover_as_offline():
    bridge = FakeBridge(fleet={"selected": None, "robots": [
        {"robot_id": "rover1", "mode": "teleop", "estop": False, "online": False,
         "age": 12.4}]})
    r = rpc(bridge, "tools/call", {"name": "get_fleet", "arguments": {}})
    robot = json.loads(r["result"]["content"][0]["text"])["robots"][0]
    assert robot["online"] is False
    assert robot["seconds_since_telemetry"] == 12.4


def test_pending_confirmations_are_visible_to_the_client():
    bridge = FakeBridge(command={
        "pending": [{"id": "c1", "intent": "fire", "say": "FIRE rover2",
                     "source": "mcp", "at": 1.0, "expires_in": 30.0}],
        "history": [], "vocabulary": {}})
    r = rpc(bridge, "tools/call", {"name": "get_pending_confirmations", "arguments": {}})
    assert "FIRE rover2" in r["result"]["content"][0]["text"]


def test_the_command_log_shows_who_gave_each_order():
    bridge = FakeBridge(command={
        "pending": [], "vocabulary": {},
        "history": [{"at": 1.0, "source": "voice", "transcript": "stop",
                     "intent": "estop", "status": "done", "say": "STOP rover1"},
                    {"at": 2.0, "source": "mcp", "transcript": "",
                     "intent": "fire", "status": "pending", "say": "Confirm"}]})
    r = rpc(bridge, "tools/call", {"name": "get_command_log", "arguments": {}})
    rows = json.loads(r["result"]["content"][0]["text"])
    assert [row["source"] for row in rows] == ["voice", "mcp"]


def test_an_unknown_tool_is_reported_rather_than_crashing():
    r = rpc(FakeBridge(), "tools/call", {"name": "launch_missile", "arguments": {}})
    assert "Unknown tool" in r["result"]["content"][0]["text"]


def test_an_unreachable_base_station_is_tool_content_not_a_protocol_error():
    """A model can read and act on "the base station is not running". A JSON-RPC
    error usually surfaces to it as a dead end."""

    class Dead(FakeBridge):
        async def command(self, intent, args):
            raise RuntimeError("could not reach the base station at ws://x")

    r = rpc(Dead(), "tools/call", {"name": "estop", "arguments": {}})
    assert r["result"]["isError"] is True
    assert "could not reach the base station" in r["result"]["content"][0]["text"]
