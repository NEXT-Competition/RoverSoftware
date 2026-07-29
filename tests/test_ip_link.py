"""Bulk transfers over WiFi, and the fallback to the radio when there isn't any.

The contract that matters is `send() -> bool`: True means the frame went out
over IP, False means the caller must put it on the radio. Every failure mode
here — not configured, base station down, socket dropped mid-transfer — has to
come out as False rather than an exception or a silently swallowed frame,
because the fallback is what keeps the settings page working at range.
"""

import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot.comms.ip_link import IPLink, IPServer, _lines  # noqa: E402


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def pair():
    """A connected server + robot link, torn down after the test."""
    port = free_port()
    got_at_base = []
    server = IPServer(port=port, on_message=got_at_base.append, host="127.0.0.1")
    server.start()
    got_at_robot = []
    link = IPLink("127.0.0.1", port, got_at_robot.append, robot_id="rover1")
    link.start()
    assert wait_for(link.is_connected), "robot never connected"
    assert wait_for(lambda: server.is_connected("rover1")), "server never learned the id"
    yield link, server, got_at_robot, got_at_base
    link.stop()
    server.stop()


# --- framing ---------------------------------------------------------------


def test_lines_reassembles_a_split_stream():
    """TCP is a byte stream: one recv can hold half a frame, or three."""
    buf = bytearray()
    assert list(_lines(buf, b'{"a":1}\n{"b":2}\n')) == [b'{"a":1}', b'{"b":2}']
    assert list(_lines(buf, b'{"c":')) == []       # partial, held
    assert list(_lines(buf, b'3}\n')) == [b'{"c":3}']


def test_lines_resynchronises_on_a_peer_that_never_terminates():
    buf = bytearray()
    assert list(_lines(buf, b"x" * 70000)) == []
    assert len(buf) == 0  # dropped rather than grown without bound
    assert list(_lines(buf, b'{"a":1}\n')) == [b'{"a":1}']


# --- the fallback contract -------------------------------------------------


def test_unconfigured_link_declines_everything():
    """No base_host: start() is a no-op and every frame goes to the radio."""
    link = IPLink("", 5006, None)
    link.start()
    assert link.is_connected() is False
    assert link.send({"type": "config"}) is False
    link.stop()


def test_unreachable_base_station_declines(capsys):
    link = IPLink("127.0.0.1", free_port(), None, robot_id="rover1")
    link.start()
    time.sleep(0.3)  # let one connect attempt fail
    assert link.is_connected() is False
    assert link.send({"type": "config"}) is False
    link.stop()
    assert "using the radio" in capsys.readouterr().out


def test_server_declines_an_unknown_robot(pair):
    _, server, _, _ = pair
    assert server.send({"type": "put_layout", "to": "rover9"}) is False
    assert server.send({"type": "put_layout"}) is False  # unaddressed


# --- frames actually cross, unchanged --------------------------------------


def test_robot_to_base(pair):
    link, _, _, got_at_base = pair
    frame = {"type": "config", "from": "rover1", "config": {"nav.cruise_speed": 0.35}}
    assert link.send(frame) is True
    assert wait_for(lambda: got_at_base)
    assert got_at_base[0] == frame  # same dict as the radio would have carried


def test_base_to_robot(pair):
    _, server, got_at_robot, _ = pair
    assert server.send({"type": "put_layout", "to": "rover1", "part": "{}"}) is True
    assert wait_for(lambda: got_at_robot)
    assert got_at_robot[0]["type"] == "put_layout"


def test_hello_is_transport_only(pair):
    """The id handshake must not reach FleetManager as a robot message."""
    _, _, _, got_at_base = pair
    time.sleep(0.1)
    assert [m for m in got_at_base if m.get("type") == "hello"] == []


def test_a_whole_config_snapshot_crosses_in_one_go(pair):
    """The point of the exercise: 7 frames at once, none dropped."""
    link, _, _, got_at_base = pair
    for i in range(7):
        assert link.send({"type": "config", "from": "rover1", "seq": i}) is True
    assert wait_for(lambda: len(got_at_base) == 7)
    assert [m["seq"] for m in got_at_base] == list(range(7))


def test_reconnect_replaces_the_old_socket(pair):
    """A robot that drops and comes back must not leave a dead socket behind."""
    link, server, _, _ = pair
    port = link.port
    link.stop()
    assert wait_for(lambda: not server.is_connected("rover1"))
    again = IPLink("127.0.0.1", port, None, robot_id="rover1")
    again.start()
    assert wait_for(lambda: server.is_connected("rover1"))
    assert server.send({"type": "get_config", "to": "rover1"}) is True
    again.stop()


def test_server_survives_a_busy_port(capsys):
    """A port already in use must not stop the dashboard from coming up."""
    port = free_port()
    hog = socket.socket()
    hog.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    hog.bind(("127.0.0.1", port))
    hog.listen(1)
    server = IPServer(port=port, host="127.0.0.1")
    server.start()  # must not raise
    assert server.send({"type": "x", "to": "rover1"}) is False
    server.stop()
    hog.close()
    assert "stay on the radio" in capsys.readouterr().out


def test_handler_errors_do_not_kill_the_reader(pair):
    link, server, _, got_at_base = pair
    boom = {"n": 0}

    def explode(msg):
        boom["n"] += 1
        raise RuntimeError("handler blew up")

    server.on_message = explode
    link.send({"type": "config", "from": "rover1"})
    assert wait_for(lambda: boom["n"] == 1)
    server.on_message = got_at_base.append
    link.send({"type": "config", "from": "rover1", "after": True})
    assert wait_for(lambda: got_at_base)  # still reading


def test_concurrent_senders_do_not_interleave(pair):
    """Two threads sending at once must not splice one frame into another."""
    link, _, _, got_at_base = pair

    def spam(tag):
        for i in range(25):
            link.send({"type": "config", "from": "rover1", "tag": tag, "i": i})

    threads = [threading.Thread(target=spam, args=(t,)) for t in ("a", "b")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert wait_for(lambda: len(got_at_base) == 50)
    for m in got_at_base:  # every frame decoded => no torn writes
        assert m["type"] == "config" and m["tag"] in ("a", "b")
