"""The local model client, against a real HTTP server.

A stub that returns canned dicts would test nothing that has ever broken here.
What breaks is the *contract*: a server that rejects `response_format`, a Gemma
that wraps its answer in a ```json fence, a model server that isn't running at
all. So these run a real socket and speak the real OpenAI-compatible shape.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from basestation.command.llm import LMStudio, LLMError
from basestation.command.vocabulary import Vocabulary


class FakeLMStudio:
    """A stand-in for LM Studio, scriptable per test."""

    def __init__(self, reply="", *, models=("gemma-3n-e4b",),
                 reject_json_schema=False, status=200):
        self.reply = reply
        self.models = models
        self.reject_json_schema = reject_json_schema
        self.status = status
        self.requests = []

        server = self  # closed over by the handler below

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass  # pytest output stays readable

            def do_GET(self):
                if self.path.endswith("/models"):
                    body = {"data": [{"id": m} for m in server.models]}
                    self._send(200, body)
                else:
                    self._send(404, {})

            def do_POST(self):
                length = int(self.headers.get("Content-Length", 0))
                payload = json.loads(self.rfile.read(length) or b"{}")
                server.requests.append(payload)
                if server.reject_json_schema and "response_format" in payload:
                    self._send(400, {"error": "response_format is not supported"})
                    return
                if server.status != 200:
                    self._send(server.status, {"error": "boom"})
                    return
                self._send(200, {"choices": [
                    {"message": {"role": "assistant", "content": server.reply}}]})

            def _send(self, code, body):
                raw = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

        self._http = HTTPServer(("127.0.0.1", 0), Handler)
        self.port = self._http.server_port
        self._thread = threading.Thread(target=self._http.serve_forever, daemon=True)
        self._thread.start()

    @property
    def base_url(self):
        return f"http://127.0.0.1:{self.port}/v1"

    def close(self):
        self._http.shutdown()
        self._http.server_close()


@pytest.fixture
def vocab():
    return Vocabulary(robots=["rover1", "rover2"], selected="rover1",
                      places=[{"id": "bucket_a", "name": "bucket A",
                               "lat": 1.0, "lon": 2.0, "kind": "bucket"}],
                      labels=["bucket"])


def _plan(intent, args, say="ok"):
    return json.dumps({"intent": intent, "args": args, "say": say})


# --- the happy path -------------------------------------------------------

def test_a_clean_answer_becomes_a_plan(vocab):
    server = FakeLMStudio(_plan("route_to", {"robot": "rover2", "places": ["bucket A"]},
                                "Routing rover2 to bucket A."))
    try:
        plan = LMStudio(server.base_url).classify("send rover2 to bucket A", vocab)
        assert plan.intent == "route_to"
        assert plan.args == {"robot": "rover2", "places": ["bucket A"]}
        assert plan.say == "Routing rover2 to bucket A."
    finally:
        server.close()


def test_the_prompt_carries_the_live_vocabulary(vocab):
    """The model can only address a rover it has been told exists. This is the
    difference between a fleet that grew a robot ten seconds ago being
    commandable and not."""
    server = FakeLMStudio(_plan("select_robot", {"robot": "rover2"}))
    try:
        LMStudio(server.base_url).classify("rover two", vocab)
        system = server.requests[0]["messages"][0]["content"]
        assert "rover1, rover2" in system
        assert "bucket A" in system
        assert "route_to" in system  # the intent menu
        assert "arm_shooter" in system and "confirm" in system.lower()
    finally:
        server.close()


def test_a_json_schema_is_requested_and_constrains_the_intent_name(vocab):
    server = FakeLMStudio(_plan("estop", {}))
    try:
        LMStudio(server.base_url).classify("stop", vocab)
        fmt = server.requests[0]["response_format"]
        assert fmt["type"] == "json_schema"
        enum = fmt["json_schema"]["schema"]["properties"]["intent"]["enum"]
        assert "route_to" in enum and "none" in enum
        assert "delete_everything" not in enum
    finally:
        server.close()


def test_it_is_deterministic(vocab):
    """Two identical orders must never classify differently. There is no
    creativity wanted anywhere in this task."""
    server = FakeLMStudio(_plan("estop", {}))
    try:
        LMStudio(server.base_url).classify("stop", vocab)
        assert server.requests[0]["temperature"] == 0.0
    finally:
        server.close()


# --- the shapes real models actually return -------------------------------

def test_a_fenced_answer_is_parsed(vocab):
    """Gemma wrapping its answer in ```json is the normal case, not an error."""
    server = FakeLMStudio(
        "```json\n" + _plan("set_mode", {"mode": "teleop"}) + "\n```",
        reject_json_schema=True)
    try:
        plan = LMStudio(server.base_url).classify("go to teleop", vocab)
        assert plan.intent == "set_mode"
    finally:
        server.close()


def test_a_chatty_answer_is_scraped(vocab):
    server = FakeLMStudio("Sure! " + _plan("estop", {}) + " Hope that helps.",
                          reject_json_schema=True)
    try:
        assert LMStudio(server.base_url).classify("stop", vocab).intent == "estop"
    finally:
        server.close()


def test_a_server_without_json_schema_is_retried_without_it(vocab):
    """Older llama.cpp and vLLM builds reject the whole request. The operator
    should not have to tell the difference between "broken" and "older"."""
    server = FakeLMStudio(_plan("select_robot", {"robot": "rover2"}),
                          reject_json_schema=True)
    try:
        client = LMStudio(server.base_url)
        assert client.classify("rover two", vocab).intent == "select_robot"
        assert len(server.requests) == 2                       # constrained, then not
        assert "response_format" not in server.requests[1]
        assert "single JSON object" in server.requests[1]["messages"][0]["content"]
        # And it remembers, so the next command costs one request rather than two.
        client.classify("rover one", vocab)
        assert len(server.requests) == 3
    finally:
        server.close()


def test_declining_to_guess_is_a_first_class_answer(vocab):
    """An intent enum with no escape hatch forces a model to invent a verb for
    "what's the weather" — which is precisely the failure that moves a robot."""
    server = FakeLMStudio(_plan("none", {}, "That isn't a rover command."))
    try:
        plan = LMStudio(server.base_url).classify("what's the weather", vocab)
        assert plan.intent is None
        assert "isn't a rover command" in plan.say
    finally:
        server.close()


# --- failure ---------------------------------------------------------------

def test_no_server_at_all_raises_something_actionable(vocab):
    client = LMStudio("http://127.0.0.1:9/v1", timeout=1.0)
    with pytest.raises(LLMError) as e:
        client.classify("stop", vocab)
    assert "LM Studio" in str(e.value)


def test_no_model_loaded_says_so(vocab):
    server = FakeLMStudio(_plan("estop", {}), models=())
    try:
        with pytest.raises(LLMError) as e:
            LMStudio(server.base_url).classify("stop", vocab)
        assert "no model" in str(e.value).lower()
    finally:
        server.close()


def test_broken_json_is_reported_not_guessed_at(vocab):
    server = FakeLMStudio("I think you want to stop the rover.")
    try:
        with pytest.raises(LLMError) as e:
            LMStudio(server.base_url).classify("stop", vocab)
        assert "did not return JSON" in str(e.value)
    finally:
        server.close()


def test_a_server_error_is_surfaced_with_its_body(vocab):
    server = FakeLMStudio(_plan("estop", {}), status=500)
    try:
        with pytest.raises(LLMError) as e:
            LMStudio(server.base_url).classify("stop", vocab)
        assert "500" in str(e.value)
    finally:
        server.close()


# --- model discovery -------------------------------------------------------

def test_discovery_prefers_a_gemma_when_several_are_loaded(vocab):
    server = FakeLMStudio(_plan("estop", {}),
                          models=("qwen3-30b", "google/gemma-3n-e4b", "phi-4"))
    try:
        client = LMStudio(server.base_url)
        assert client.discover() == "google/gemma-3n-e4b"
        ok, note = client.health()
        assert ok and "gemma" in note
    finally:
        server.close()


def test_health_reports_a_missing_server_without_raising():
    ok, note = LMStudio("http://127.0.0.1:9/v1", timeout=1.0).health()
    assert ok is False
    assert "127.0.0.1:9" in note
