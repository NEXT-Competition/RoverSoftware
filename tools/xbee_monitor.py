#!/usr/bin/env python3
"""XBee link monitor + injector — verify the radio is working end to end.

Run on the robot's Pi (or any node with an XBee) to WATCH every frame the base
station sends and to SEND frames yourself so you can confirm the other end
receives them. Frames that don't parse are shown as raw bytes — that's how you
spot a baud-rate or wiring problem (it arrives as garbage instead of JSON).

    python tools/xbee_monitor.py --port /dev/serial0 --baud 9600

While it runs, type a line and press Enter to send:
    <blank>        send a test telemetry frame (from = --id)
    d 0.5 -0.2     drive: throttle=0.5 steer=-0.2  (arcade)
    dt 0.4 0.6     drive: left=0.4 right=0.6        (tank)
    m waypoint     mode switch (teleop | color_align | waypoint)
    e   /   c      estop  /  clear_estop
    to rover2      address later sends to rover2 (default: no address = broadcast)
    to -           clear the address
    {"type":"..."} send raw JSON verbatim
    q              quit

Only want to watch (no typing/sending)?  add  --listen
Want a periodic test ping?               add  --heartbeat 1.0
"""

import argparse
import os
import sys
import threading
import time

# Make the repo root (parent of tools/) importable so `import robot` works even
# when run as `python tools/xbee_monitor.py`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.comms import protocol  # noqa: E402
from robot.config import RobotConfig  # noqa: E402

try:
    import serial
except Exception:
    serial = None

stats = {"rx": 0, "bad": 0, "tx": 0}
_write_lock = threading.Lock()


def now():
    return time.strftime("%H:%M:%S")


def describe(msg):
    t = msg.get("type", "?")
    rest = " ".join(f"{k}={v}" for k, v in msg.items() if k != "type")
    return f"{t:<12} {rest}".rstrip()


def reader_loop(ser, running):
    """Print every inbound frame; show raw bytes for anything unparseable."""
    buf = bytearray()
    while running.is_set():
        try:
            chunk = ser.readline()
        except Exception as e:
            print(f"[{now()}] read error: {e}")
            time.sleep(0.2)
            continue
        if not chunk:
            continue
        buf.extend(chunk)
        if not chunk.endswith(b"\n"):
            continue  # partial line (read timeout); keep accumulating
        line = bytes(buf).strip()
        buf.clear()
        if not line:
            continue
        msg = protocol.decode(line)
        if msg is None:
            stats["bad"] += 1
            print(f"[{now()}] RX  <unparsed> {line!r}  (check baud / wiring)")
        else:
            stats["rx"] += 1
            print(f"[{now()}] RX  {describe(msg)}")


def build_message(text, robot_id):
    """Turn a typed shorthand line into a protocol dict (or None if unknown)."""
    text = text.strip()
    if not text:
        return {"type": "telemetry", "from": robot_id, "mode": "teleop", "left": 0.0, "right": 0.0}
    if text.startswith("{"):
        return protocol.decode(text.encode())  # raw JSON (None if malformed)

    parts = text.split()
    cmd = parts[0]
    try:
        if cmd == "d" and len(parts) >= 3:
            return {"type": "drive", "throttle": float(parts[1]), "steer": float(parts[2])}
        if cmd == "dt" and len(parts) >= 3:
            return {"type": "drive", "left": float(parts[1]), "right": float(parts[2])}
        if cmd == "m" and len(parts) >= 2:
            return {"type": "mode", "mode": parts[1]}
        if cmd == "e":
            return {"type": "estop"}
        if cmd == "c":
            return {"type": "clear_estop"}
    except ValueError:
        return None
    return None


def make_sender(ser):
    def send(msg):
        if msg is None:
            print("  ! could not build a message from that input")
            return
        with _write_lock:
            ser.write(protocol.encode(msg))
        stats["tx"] += 1
        print(f"[{now()}] TX  {describe(msg)}")
    return send


def main():
    cfg = RobotConfig()
    p = argparse.ArgumentParser(description="Watch and inject XBee traffic")
    p.add_argument("--port", default=cfg.comms.port, help="serial port")
    p.add_argument("--baud", type=int, default=cfg.comms.baud)
    p.add_argument("--id", default=cfg.robot_id, help="'from' id used in test telemetry")
    p.add_argument("--to", default=None, help="default 'to' address for sent commands")
    p.add_argument("--listen", action="store_true", help="only watch; do not read stdin or send")
    p.add_argument("--heartbeat", type=float, default=0.0,
                   help="also send a test telemetry frame every N seconds")
    args = p.parse_args()

    if serial is None:
        sys.exit("pyserial not installed; run: pip install pyserial")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        sys.exit(f"could not open {args.port} @ {args.baud}: {e}")

    print(f"[xbee-monitor] {args.port} @ {args.baud} baud — watching. Ctrl-C to quit.")
    running = threading.Event()
    running.set()
    threading.Thread(target=reader_loop, args=(ser, running), daemon=True).start()

    send = make_sender(ser)
    to = args.to

    if args.heartbeat > 0:
        def heartbeat():
            while running.is_set():
                send({"type": "telemetry", "from": args.id, "mode": "teleop", "left": 0.0, "right": 0.0})
                time.sleep(args.heartbeat)
        threading.Thread(target=heartbeat, daemon=True).start()

    try:
        if args.listen:
            while True:
                time.sleep(1)
        else:
            for line in sys.stdin:
                line = line.strip()
                if line in ("q", "quit", "exit"):
                    break
                if line.startswith("to "):
                    arg = line[3:].strip()
                    to = None if arg in ("", "-") else arg
                    print(f"  default 'to' = {to!r}")
                    continue
                msg = build_message(line, args.id)
                # Address commands (not telemetry) when a 'to' is set.
                if msg and to and msg.get("type") != "telemetry":
                    msg = {**msg, "to": to}
                send(msg)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        print(f"\n[xbee-monitor] totals: rx={stats['rx']} unparsed={stats['bad']} tx={stats['tx']}")
        ser.close()


if __name__ == "__main__":
    main()
