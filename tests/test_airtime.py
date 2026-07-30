"""The byte budget that keeps bulk transfers inside what the radio can carry.

The bug this exists to prevent: a config snapshot or a layout handed to the
radio faster than 57600 baud can drain it, the serial write timing out, and the
dropped frame being a fragment of a document nobody re-requests — a settings
page that stays blank for the rest of the match.
"""

import pytest

from robot.comms.airtime import BITS_PER_BYTE, MIN_BAUD, Airtime
from robot.comms.xbee_link import XBeeLink


def test_rate_is_the_line_rate_not_a_guess():
    """57600 baud is 5760 bytes a second, of which bulk may claim its share."""
    air = Airtime(57600, share=1.0)
    assert air.rate == pytest.approx(57600 / BITS_PER_BYTE)
    assert Airtime(57600, share=0.5).rate == pytest.approx(2880)


def test_a_silly_baud_does_not_wedge_every_transfer():
    """A typo in a config file should slow things down, not stop them."""
    assert Airtime(300).rate == pytest.approx(MIN_BAUD / BITS_PER_BYTE * 0.6)


def test_the_first_frame_goes_immediately():
    """Nobody should wait for a bucket to fill before the first fragment."""
    air = Airtime(57600, share=1.0, burst=512, now=0.0)
    assert air.take(400, now=0.0) is True


def test_a_burst_is_refused_rather_than_dropped():
    """The whole point: 'no' means 'keep it', and it must actually be said."""
    air = Airtime(57600, share=1.0, burst=512, now=0.0)
    assert air.take(400, now=0.0) is True
    assert air.take(400, now=0.0) is False  # only 112 bytes of credit left


def test_credit_refills_at_the_line_rate():
    air = Airtime(57600, share=1.0, burst=512, now=0.0)
    air.take(500, now=0.0)
    # 5760 B/s, so 400 bytes is ~70 ms of airtime and nothing sooner.
    assert air.take(400, now=0.03) is False
    assert air.take(400, now=0.08) is True


def test_credit_does_not_pile_up_while_idle():
    """A minute of quiet must not buy the right to burst a whole layout."""
    air = Airtime(57600, share=1.0, burst=512, now=0.0)
    assert air.take(512, now=60.0) is True
    assert air.take(1, now=60.0) is False


def test_realtime_traffic_is_charged_and_bulk_gives_way():
    """Telemetry never asks permission, but it does occupy the same line."""
    air = Airtime(57600, share=1.0, burst=512, now=0.0)
    air.debit(2000, now=0.0)  # a burst of telemetry the loop sent regardless
    assert air.take(1, now=0.0) is False
    assert air.take(1, now=0.1) is False  # still paying it back
    assert air.take(400, now=0.4) is True


def test_a_clock_that_went_backwards_mints_nothing():
    air = Airtime(57600, share=1.0, burst=512, now=100.0)
    air.take(512, now=100.0)
    assert air.take(1, now=50.0) is False


# --- the link itself --------------------------------------------------------

class FakeSerial:
    """Just enough serial port to see what reached the wire."""

    is_open = True

    def __init__(self):
        self.written = []

    def write(self, data):
        self.written.append(data)
        return len(data)


def _link(baud=57600):
    link = XBeeLink("/dev/null", baud, lambda msg: None)
    link._serial = FakeSerial()
    return link


def test_bulk_frames_stop_when_the_radio_is_full():
    link = _link()
    frame = {"type": "layout", "part": "x" * 400}
    assert link.send_bulk(frame) is True
    # Second one in the same instant would overrun the line: refused, and
    # crucially NOT written, so the caller still has it to send later.
    assert link.send_bulk(frame) is False
    assert len(link._serial.written) == 1


def test_realtime_frames_never_wait_for_airtime():
    """An e-stop must not queue behind a layout transfer."""
    link = _link()
    link.airtime.debit(100_000)  # bulk is now hours behind
    assert link.send({"type": "estop"}) is True
    assert link.send_bulk({"type": "layout", "part": "x"}) is False


def test_a_dead_port_drops_frames_instead_of_hoarding_them():
    """send_bulk answers "are you done with this?", not "did it go out?".

    A shut port never will take the frame, so True is the honest answer and the
    queue drains. Only congestion — which passes — earns a retry.
    """
    link = XBeeLink("/dev/null", 57600, lambda msg: None)
    assert link.send({"type": "telemetry"}) is False
    assert link.send_bulk({"type": "layout"}) is True
