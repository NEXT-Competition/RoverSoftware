#!/usr/bin/env python3
"""Live BNO085 heading / yaw-rate / calibration monitor — IMU bring-up.

Use this to confirm the IMU is wired and reading before trusting it for
navigation, and to run the first-power-up magnetometer calibration: move the
rover in a few figure-8s until the calibration level reaches 3.

    python tools/imu_monitor.py
    python tools/imu_monitor.py --address 0x4b --offset 90 --invert

The calibration level is 0-3 (3 = fully calibrated). heading() only reports a
value once it reaches the driver's min_calib; below that it prints
"(uncalibrated -> GPS fallback)", which is exactly how the rover behaves.

Once the level reaches 3 the driver saves the calibration to the BNO08x's own
on-chip flash (there is no file — the chip remembers it across power cycles), so
the robot boots pre-calibrated. Use --no-save for a monitor-only dry run.

It also AUDITS THE I2C BUS while it runs, which is the other reason to reach for
it. Every packet the driver receives is counted, and every one it chokes on is
diagnosed — see `PacketAudit`. That turns "I keep seeing a weird IMU error" into
a number, which is the only way to tell whether shortening a cable or dropping
the bus clock actually helped:

    python tools/imu_monitor.py --seconds 60
    ...
    packets 5417 ok, 3 bad (1 in 1806, 0.06%)
    all 3 explained by ONE flipped bit, every one a 1 read as 0

Off-hardware (no adafruit-circuitpython-bno08x / Blinka): prints a clear note
and exits, so this is safe to run on a laptop.
"""

import argparse
import os
import sys
import threading
from collections import Counter
from time import monotonic, sleep

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/imu_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import IMUConfig
from robot.sensors.bno085 import IMU, adafruit_bno08x
from robot.sensors.bno085_rvc import RVCIMU
from robot.sensors.imu import build_imu

# Length in bytes of each SH-2 report we might see, INCLUDING its id byte. This
# is not the full SH-2 catalogue — it is what a BNO085 configured the way this
# robot configures it actually emits (rotation vector + calibrated gyro), plus
# the timestamp reports that lead every batch, plus a few neighbours so that a
# bit flip landing on one of them is still recognised as a legal report rather
# than counted as a second fault.
#
# A report id NOT in here is exactly what makes the driver raise: it looks up
# the id, finds nothing, and a Python KeyError prints as the bare number you see
# in the journal.
_REPORT_LENGTHS = {
    0xFB: 5,    # base timestamp reference: id + int32 delta
    0xFA: 5,    # timestamp rebase
    0x01: 10,   # accelerometer          | the standard sensor-report shape:
    0x02: 10,   # gyroscope, calibrated  | id, sequence, status, delay,
    0x03: 10,   # magnetic field         | then three int16 axes
    0x04: 10,   # linear acceleration    |
    0x06: 10,   # gravity                |
    0x05: 14,   # rotation vector: the same prefix, four int16 + accuracy
    0x08: 12,   # game rotation vector: the same, without accuracy
}

# How many bad packets to describe in full before the summary carries the rest.
_VERBOSE_BAD = 5


def _parses(payload: bytes) -> bool:
    """Does this payload decode as a batch of whole SH-2 reports, exactly?

    Walks report by report and demands the last one end precisely on the
    payload's length. That exactness is what makes the single-bit test below
    meaningful rather than a coincidence: a random byte sequence almost never
    tiles into legal reports that sum to the declared length.
    """
    index = 0
    if not payload:
        return False
    while index < len(payload):
        length = _REPORT_LENGTHS.get(payload[index])
        if length is None or index + length > len(payload):
            return False
        index += length
    return index == len(payload)


class PacketAudit:
    """Counts packets, and works out what was wrong with the bad ones.

    The specific question it answers: is a failed packet CORRUPTED or
    MISFRAMED? They look identical in the journal — both surface as a report id
    the driver does not recognise — and they have completely different causes.

        one flipped bit   The payload is a legal batch of reports apart from a
                          single wrong bit. The bus is delivering the right
                          bytes almost all of the time and occasionally getting
                          one wrong: a physical-layer problem (cable length, bus
                          clock, pull-ups). The DIRECTION narrows it further —
                          a 1 read as 0 is a line that did not rise in time,
                          since a 0 is actively driven and hard to corrupt.
        anything else     The reader is parsing at the wrong offset, or several
                          bytes are wrong at once. A different problem, and
                          worth knowing it is a different problem.

    Runs on the driver's own reader thread, so the counters are locked.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.ok = 0
        self.bad = 0
        self.single_bit = 0
        self.flips = Counter()      # (byte index, bit) -> times seen
        self.directions = Counter() # "1->0" / "0->1"
        self.errors = Counter()     # str(exception) -> times seen
        self._described = 0
        self._installed = False

    # --- installation -------------------------------------------------------

    def install(self) -> bool:
        """Wrap the driver's packet handler. False if this version differs.

        Patched on the CLASS before the sensor is opened, so no packet escapes
        the count — an instance-level hook would race the reader thread, which
        opens the device itself.
        """
        base = getattr(adafruit_bno08x, "BNO08X", None)
        handler = getattr(base, "_handle_packet", None)
        if base is None or handler is None:
            return False
        audit = self

        def counting(sensor, packet):
            try:
                result = handler(sensor, packet)
            except Exception as error:
                audit.note_bad(packet, error)
                raise
            audit.note_ok()
            return result

        base._handle_packet = counting
        self._installed = True
        return True

    @property
    def installed(self) -> bool:
        return self._installed

    # --- counting -----------------------------------------------------------

    def note_ok(self) -> None:
        with self._lock:
            self.ok += 1

    def note_bad(self, packet, error) -> None:
        payload = _payload_of(packet)
        finding = self.diagnose(payload) if payload else None
        with self._lock:
            self.bad += 1
            self.errors[f"{type(error).__name__}: {error}"] += 1
            if finding is not None:
                self.single_bit += 1
                self.flips[(finding[0], finding[1])] += 1
                self.directions[finding[2]] += 1
            describe = self._described < _VERBOSE_BAD
            if describe:
                self._described += 1
        if describe:
            print(f"\n  ^ {self._explain(payload, finding, error)}")

    @staticmethod
    def diagnose(payload: bytes):
        """(byte index, bit, direction) if ONE flipped bit explains it, else None.

        Tries every single-bit correction and asks whether the result is a legal
        report batch. 15 bytes is 120 candidates, which is nothing, and the
        exactness demanded by `_parses` is what keeps a hit meaningful.
        """
        if _parses(payload):
            return None                      # not a framing fault at all
        for index in range(len(payload)):
            for bit in range(8):
                candidate = bytearray(payload)
                candidate[index] ^= 1 << bit
                if _parses(bytes(candidate)):
                    # The wire delivered `payload`; the sensor sent `candidate`.
                    sent_high = bool(candidate[index] & (1 << bit))
                    return (index, bit, "1->0" if sent_high else "0->1")
        return None

    @staticmethod
    def _explain(payload, finding, error) -> str:
        if not payload:
            return f"bad packet ({error}) — could not read its bytes"
        if finding is None:
            return (f"bad packet ({error}): NOT a single flipped bit — either "
                    f"several bytes are wrong or the reader is parsing at the "
                    f"wrong offset. Bytes: {payload.hex(' ')}")
        index, bit, direction = finding
        return (f"bad packet: ONE flipped bit — byte {index}, bit {bit} "
                f"(0x{payload[index]:02X} should be "
                f"0x{payload[index] ^ (1 << bit):02X}), a {direction} on the "
                f"wire. Everything else in the packet is a legal report.")

    # --- the verdict --------------------------------------------------------

    def summary(self) -> str:
        with self._lock:
            ok, bad, single = self.ok, self.bad, self.single_bit
            flips, directions = dict(self.flips), dict(self.directions)
            errors = dict(self.errors)
        total = ok + bad
        if not self._installed:
            return ("Packet auditing was unavailable: this adafruit_bno08x does "
                    "not have the internals this tool hooks.")
        if total == 0:
            return ("No packets at all. The sensor is not answering — check the "
                    "address (i2cdetect -y 1) and the wiring.")
        lines = [f"packets {ok} ok, {bad} bad"
                 + (f" (1 in {total // bad}, {100.0 * bad / total:.2f}%)"
                    if bad else " — a clean bus")]
        if not bad:
            return lines[0]

        if single == bad:
            lines.append(f"all {bad} explained by ONE flipped bit"
                         + (f", every one a {next(iter(directions))}"
                            if len(directions) == 1 else ""))
        elif single:
            lines.append(f"{single} of {bad} explained by one flipped bit; the "
                         f"other {bad - single} were not")
        else:
            lines.append("none explained by a single flipped bit")

        if flips:
            worst = sorted(flips.items(), key=lambda kv: -kv[1])[:4]
            lines.append("  where: " + ", ".join(
                f"byte {i} bit {b} x{n}" for (i, b), n in worst))
        for message, count in sorted(errors.items(), key=lambda kv: -kv[1])[:3]:
            lines.append(f"  driver said: {message} (x{count})")
        lines.append("")
        lines.append(_advice(single, bad, directions))
        return "\n".join(lines)


def _advice(single: int, bad: int, directions: dict) -> str:
    """What the numbers mean, in the imperative."""
    if single == 0:
        return ("Multi-byte damage or a misframed stream. Check that nothing "
                "else is talking to the sensor (stop the robot service before "
                "running this), then suspect the bus itself.")
    lines = ["Single-bit errors mean the bus is delivering the right bytes and "
             "occasionally\ngetting one wrong — a physical-layer problem, not a "
             "driver one."]
    if directions.get("1->0", 0) >= directions.get("0->1", 0):
        lines.append(
            "A 1 read as 0 points at RISE TIME: a 0 is actively driven low and "
            "hard to\ncorrupt, while a 1 is only a released line being pulled up "
            "through a resistor.\nToo much capacitance, too weak a pull-up, or "
            "too fast a clock for both.")
    else:
        lines.append(
            "A 0 read as 1 is unusual — a 0 is actively driven, so this points "
            "at injected\nnoise rather than rise time. Look for motor or ESC "
            "wiring running alongside\nSDA/SCL.")
    lines.append(
        "In order: shorten the I2C leads and route them away from motor wiring; "
        "check\n`grep i2c /boot/firmware/config.txt` and drop the clock to "
        "100000 if it is\nhigher; add a 2.2k pull-up pair at the sensor end. "
        "Re-run this for the same\nnumber of seconds after each change — that is "
        "what makes it an experiment.")
    if bad and single == bad:
        lines.append(
            "\nWorth knowing: the robot already survives this. A corrupted "
            "packet costs one\nsample, and the heading is only dropped after "
            "imu.sample_timeout without a valid\none. A rate of a few per "
            "thousand is not worth chasing.")
    return "\n".join(lines)


def _payload_of(packet) -> bytes:
    """The packet's data bytes, or b"" if this driver version hides them."""
    try:
        return bytes(bytearray(packet.data[:packet.header.data_length]))
    except Exception:
        return b""


def main():
    p = argparse.ArgumentParser(description="BNO085 IMU live monitor / calibration")
    p.add_argument("--mode", default=IMUConfig.mode, choices=["i2c", "uart_rvc"],
                   help="how the sensor is read — follow the board's PS0/PS1 "
                        "strapping (default: this build's configured mode)")
    p.add_argument("--port", default=IMUConfig.port,
                   help="serial port for uart_rvc (not the GPS's /dev/ttyAMA0)")
    p.add_argument("--address", type=lambda x: int(x, 0), default=0x4A,
                   help="I2C address (default 0x4a; 0x4b if DI/AD0 high)")
    p.add_argument("--offset", type=float, default=0.0,
                   help="heading offset (deg) to align yaw with North")
    p.add_argument("--invert", action="store_true", help="flip yaw sign to CW-positive")
    p.add_argument("--min-calib", type=int, default=1,
                   help="min calibration level (0-3) before heading is reported")
    p.add_argument("--rate", type=float, default=5.0, help="print rate (Hz)")
    p.add_argument("--no-save", action="store_true",
                   help="don't persist calibration to the chip's flash (monitor only)")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="stop after this long and print the bus audit. 0 = run "
                        "until Ctrl-C. Use the same value before and after a "
                        "wiring change, or the two error rates are not comparable")
    p.add_argument("--timeout", type=float, default=0.0,
                   help="seconds before a stale reading stops being reported "
                        "(0 = never, which is what you want while watching a "
                        "bus that drops packets)")
    args = p.parse_args()

    rvc = args.mode == "uart_rvc"
    if not rvc and adafruit_bno08x is None:
        print("adafruit_bno08x not found — install on the Pi: "
              "pip install adafruit-circuitpython-bno08x\n"
              "(nothing to monitor on a dev laptop without the sensor libraries)")
        return

    # Hooked BEFORE the sensor is built: the reader thread opens the device
    # itself, so anything installed afterwards would miss the first packets —
    # and on a bus that only goes wrong occasionally, missed packets are the
    # difference between a rate and an anecdote.
    #
    # I2C only. The RVC transport has its own integrity check built into every
    # frame, which is the entire reason to be on it, so the equivalent number
    # comes off the reader itself rather than out of a hook.
    audit = PacketAudit()
    if not rvc and not audit.install():
        print("note: this adafruit_bno08x does not expose the internals the "
              "packet audit hooks, so only heading/calibration are shown.\n")

    imu = build_imu(IMUConfig(
        mode=args.mode, port=args.port, i2c_address=args.address,
        heading_offset_deg=args.offset, invert=args.invert,
        min_calib=args.min_calib, persist_calibration=not args.no_save,
        # Staleness is the robot's safety rule, not a diagnostic one. Here it
        # would blank the readout during exactly the dropouts you are trying to
        # watch, so it is off unless asked for.
        sample_timeout=args.timeout))
    imu.start()
    dest = "off" if args.no_save else "BNO085 flash"
    if rvc:
        print("UART-RVC reports no calibration level, so there is nothing to "
              "watch converge\nhere — calibrate over I2C once (--mode i2c) and "
              "the chip keeps it in flash.")
        print("What to check instead: point the rover at a landmark, note the "
              "heading, drive\nit around, come back, and see whether the "
              "heading returns to the same number.\nIf it has drifted tens of "
              "degrees, this mode's yaw is not a compass heading and\nthis "
              "build should stay on heading_source=gps.")
    else:
        print(f"Move the rover in figure-8s until the calibration level reaches "
              f"3 (auto-save -> {dest}). Ctrl-C to stop.")
    if audit.installed:
        print("Counting packets as they arrive; the driver prints any it cannot "
              "parse and\nthis tool adds a diagnosis underneath. Summary on the "
              "way out.")
    print()

    period = 1.0 / args.rate if args.rate > 0 else 0.2
    deadline = monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while deadline is None or monotonic() < deadline:
            heading = imu.heading()
            rate = imu.yaw_rate()
            calib = imu.calibration()
            h = f"{heading:6.1f}°" if heading is not None else "  --   (no heading -> GPS fallback)"
            r = f"{rate:+6.1f}°/s" if rate is not None else "  --  "
            line = f"heading={h}  yaw_rate={r}"
            # None means the transport has no such number, which is a different
            # statement from 0 and is worth not flattening into one.
            line += f"  calib={calib}/3" if calib is not None else "  calib=n/a"
            if audit.installed:
                line += f"  pkt {audit.ok}/{audit.ok + audit.bad}"
            elif isinstance(imu, RVCIMU):
                seen, bad = imu.frame_counts()
                line += f"  frames {seen - bad}/{seen}"
            print(line)
            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
    print(f"\n{_verdict(imu, audit)}")


def _verdict(imu, audit: PacketAudit) -> str:
    """The bus health summary, in whichever terms this transport has.

    Same question either way — how much of what the sensor sent actually
    arrived — but the two transports answer it differently. I2C has no
    per-packet integrity check, so the audit has to infer corruption from what
    the driver choked on; an RVC frame carries a checksum, so the reader simply
    counts the ones that failed it.
    """
    if not isinstance(imu, RVCIMU):
        return audit.summary()
    seen, bad = imu.frame_counts()
    if seen == 0:
        return ("No frames at all. Nothing is arriving on that port: check it "
                "is the right\none (`ls /dev/ttyAMA*`; the GPS owns ttyAMA0), "
                "that the sensor's TX reaches the\nPi's RX, and that the board "
                "is strapped for UART-RVC rather than I2C.")
    if not bad:
        return (f"frames {seen} ok, 0 rejected — a clean line.\n"
                f"Every one of those carried a checksum, which is what this "
                f"transport buys:\nthe corruption that reached the heading over "
                f"I2C cannot get past this.")
    return (f"frames {seen - bad} ok, {bad} rejected "
            f"(1 in {seen // bad}, {100.0 * bad / seen:.2f}%)\n"
            f"Rejected means the checksum failed, so those never became a "
            f"heading — which is\nthe point. A few per thousand is a working "
            f"line; percent-level means the wiring\nor the baud rate is wrong, "
            f"and the heading is being updated that much less often.")


if __name__ == "__main__":
    main()
