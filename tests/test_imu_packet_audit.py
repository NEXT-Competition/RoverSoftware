"""The bus audit in tools/imu_monitor.py: is a bad packet corrupted or misframed?

Both surface identically in the journal — the driver looks up a report id it has
never heard of and a Python KeyError prints as a bare number — and they have
completely different causes. One flipped bit in an otherwise legal packet is a
physical-layer problem (cable, clock, pull-ups). A packet that no single bit
repairs is a reader parsing at the wrong offset, or damage on a different scale.

The load-bearing property is EXACTNESS: `_parses` demands that the reports tile
the payload and the last one end precisely on its declared length. Without that,
"flip a bit and see if it parses" would find a hit in almost anything and the
tool would confidently misdiagnose a healthy bus.

The first test is the packet that started this — captured off a real rover.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from imu_monitor import PacketAudit, _parses  # noqa: E402

# Straight off the rover's journal. A base timestamp reference (0xFB + a 4-byte
# delta) followed by a calibrated gyro report — except the first byte arrived as
# 0x7B, which is 0xFB with bit 7 knocked out.
OBSERVED = bytes([0x7B, 0x17, 0x00, 0x00, 0x00,
                  0x02, 0x92, 0x03, 0x00, 0x01, 0x00, 0xFE, 0xFF, 0x01, 0x00])
REPAIRED = bytes([0xFB]) + OBSERVED[1:]

# What the rover subscribes to: a timestamp and a rotation vector, 5 + 14.
ROTATION_BATCH = bytes([0xFB, 0x0A, 0x00, 0x00, 0x00]
                       + [0x05, 0x11, 0x03, 0x00] + [0x00] * 10)


# --- what counts as a legal payload ------------------------------------------

def test_the_repaired_packet_parses_exactly():
    assert _parses(REPAIRED) is True
    assert len(REPAIRED) == 15          # 5 (timestamp) + 10 (gyro), to the byte


def test_the_packet_as_it_arrived_does_not():
    assert _parses(OBSERVED) is False


def test_a_rotation_vector_batch_parses():
    assert _parses(ROTATION_BATCH) is True


def test_a_report_that_overruns_the_payload_is_refused():
    """The exactness that makes a single-bit hit meaningful: a batch whose last
    report is cut short is not a legal packet, however good its first bytes."""
    assert _parses(REPAIRED[:-1]) is False


def test_trailing_bytes_are_refused():
    assert _parses(REPAIRED + b"\x00") is False


def test_an_empty_payload_is_refused():
    assert _parses(b"") is False


# --- the diagnosis -----------------------------------------------------------

def test_the_observed_packet_is_one_flipped_bit():
    """The finding this whole feature exists to make reproducible."""
    index, bit, direction = PacketAudit.diagnose(OBSERVED)
    assert (index, bit) == (0, 7)
    assert direction == "1->0"          # the sensor sent a 1; the wire gave a 0
    assert OBSERVED[index] ^ (1 << bit) == 0xFB


def test_a_healthy_packet_is_not_diagnosed_as_anything():
    assert PacketAudit.diagnose(REPAIRED) is None


def test_multi_byte_damage_is_not_blamed_on_one_bit():
    """The other half of the verdict. If no single bit repairs it, the tool must
    say so rather than reach for the nearest tidy explanation."""
    wrecked = bytes([0x7B, 0x17, 0x00, 0x00, 0x00,
                     0x42, 0x92, 0x03, 0x00, 0x01, 0x00, 0xFE, 0xFF, 0x01, 0x00])
    assert PacketAudit.diagnose(wrecked) is None


def test_random_bytes_are_not_explained_away():
    """The false-positive guard. If flipping one bit could rescue arbitrary
    noise, every diagnosis would be worthless."""
    noise = bytes(range(0x30, 0x30 + 15))
    assert PacketAudit.diagnose(noise) is None


def test_a_zero_read_as_one_is_reported_as_such():
    """Direction matters: a 0 is actively driven and hard to corrupt, so this
    one points at injected noise rather than rise time."""
    corrupt = bytearray(REPAIRED)
    corrupt[5] |= 0x80                  # 0x02 -> 0x82, a 0 flipped up
    index, bit, direction = PacketAudit.diagnose(bytes(corrupt))
    assert (index, bit, direction) == (5, 7, "0->1")


# --- counting ----------------------------------------------------------------

class FakePacket:
    def __init__(self, payload):
        self.data = payload
        self.header = type("H", (), {"data_length": len(payload)})()


def test_it_counts_and_classifies():
    audit = PacketAudit()
    for _ in range(10):
        audit.note_ok()
    audit.note_bad(FakePacket(OBSERVED), KeyError(123))
    summary = audit.summary()
    assert audit.ok == 10 and audit.bad == 1 and audit.single_bit == 1
    # Not installed in this test, so summary() reports that rather than a rate;
    # the counts are what the live display reads.
    assert "Packet auditing was unavailable" in summary


def test_the_summary_reports_a_rate_and_a_verdict():
    audit = PacketAudit()
    audit._installed = True             # as install() would have set it
    for _ in range(999):
        audit.note_ok()
    audit.note_bad(FakePacket(OBSERVED), KeyError(123))
    summary = audit.summary()
    assert "1 in 1000" in summary
    assert "ONE flipped bit" in summary
    assert "1->0" in summary
    assert "RISE TIME" in summary       # the direction picked the advice


def test_a_clean_bus_says_so():
    audit = PacketAudit()
    audit._installed = True
    for _ in range(100):
        audit.note_ok()
    assert "a clean bus" in audit.summary()


def test_silence_is_reported_as_silence():
    """No packets at all is a different fault from bad packets, and the advice
    for it is different too."""
    audit = PacketAudit()
    audit._installed = True
    assert "No packets at all" in audit.summary()


def test_unexplained_damage_gets_its_own_advice():
    audit = PacketAudit()
    audit._installed = True
    audit.note_ok()
    audit.note_bad(FakePacket(bytes(range(0x30, 0x3F))), KeyError(48))
    summary = audit.summary()
    assert "none explained by a single flipped bit" in summary
    assert "misframed" in summary


def test_only_the_first_few_bad_packets_are_described(capsys):
    """A bus in a bad mood produces hundreds of these, and a screen full of hex
    is not more informative than five plus a count."""
    audit = PacketAudit()
    for _ in range(20):
        audit.note_bad(FakePacket(OBSERVED), KeyError(123))
    assert capsys.readouterr().out.count("ONE flipped bit") == 5
    assert audit.bad == 20


def test_a_packet_whose_bytes_cannot_be_read_is_still_counted():
    """Driver versions differ. Losing the diagnosis is acceptable; losing the
    count would make the tool lie about the error rate."""
    class Opaque:
        pass

    audit = PacketAudit()
    audit.note_bad(Opaque(), KeyError(123))
    assert audit.bad == 1 and audit.single_bit == 0


def test_install_declines_a_library_it_does_not_recognise(monkeypatch):
    import imu_monitor
    monkeypatch.setattr(imu_monitor, "adafruit_bno08x",
                        type("Stub", (), {})())
    audit = PacketAudit()
    assert audit.install() is False
    assert audit.installed is False
