#!/usr/bin/env python3
"""Base-station teleop: read a PS4 controller and stream drive commands.

Cross-platform (macOS / Linux / Windows) via pygame. Sends newline-delimited
JSON over the XBee serial link - the same protocol the robot speaks. This is
your end-to-end teleop test today, and the seed of the full base station later.

    # send over the XBee radio:
    python -m basestation.teleop_sender --port /dev/tty.usbserial-XXXX

    # no radio handy? print frames to the terminal to sanity-check:
    python -m basestation.teleop_sender

Controls: R2 = forward, L2 = reverse, right stick X = steer. The robot does the
arcade->tank mixing. Buttons map to mode switches / e-stop (see below).

Note: gamepad axis/button indices vary by OS and driver. If throttle/steer feel
wrong, print `js.get_numaxes()` and the per-axis values and adjust AXIS_*.
"""

import argparse
import json
import sys
import time

# Shares the axis map and the trigger arming latch with the base station so the
# two teleop paths can't drift apart. Importing it also pins SDL to the dummy
# video/audio drivers, which is what this headless sender wants anyway.
from basestation.controller_input import (
    AXIS_L2,
    AXIS_R2,
    AXIS_STEER,
    BTN_ALIGN,
    BTN_CLEAR,
    BTN_ESTOP,
    BTN_TELEOP,
    Trigger,
)

try:
    import serial
except Exception:
    serial = None

try:
    import pygame
except Exception:
    pygame = None


def send(ser, msg):
    line = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
    if ser is not None:
        ser.write(line)
    else:
        sys.stdout.write(line.decode())
        sys.stdout.flush()


def deadzone(v, dz=0.08):
    return 0.0 if abs(v) < dz else v


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None, help="XBee serial port (omit to print to stdout)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--hz", type=float, default=20.0)
    args = p.parse_args()

    if pygame is None:
        sys.exit("pygame required: pip install pygame")

    ser = None
    if args.port:
        if serial is None:
            sys.exit("pyserial required: pip install pyserial")
        ser = serial.Serial(args.port, args.baud, timeout=0.1)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No controller detected. Pair a PS4 controller and retry.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}  ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")
    print("R2 = forward, L2 = reverse, R-stick X = steer. Ctrl-C to quit.")

    prev_buttons = {}
    l2, r2 = Trigger(), Trigger()

    def edge(idx):
        """True on the frame a button transitions from up to down."""
        cur = js.get_button(idx) if idx < js.get_numbuttons() else 0
        was = prev_buttons.get(idx, 0)
        prev_buttons[idx] = cur
        return cur and not was

    period = 1.0 / args.hz
    try:
        while True:
            pygame.event.pump()

            if edge(BTN_ESTOP):
                send(ser, {"type": "estop"})
            if edge(BTN_CLEAR):
                send(ser, {"type": "clear_estop"})
            if edge(BTN_TELEOP):
                send(ser, {"type": "mode", "mode": "teleop"})
            if edge(BTN_ALIGN):
                send(ser, {"type": "mode", "mode": "object_align"})

            naxes = js.get_numaxes()
            fwd = r2.value(js.get_axis(AXIS_R2)) if naxes > AXIS_R2 else 0.0
            rev = l2.value(js.get_axis(AXIS_L2)) if naxes > AXIS_L2 else 0.0
            throttle = deadzone(fwd - rev)  # R2 forward, L2 reverse
            steer = deadzone(js.get_axis(AXIS_STEER)) if naxes > AXIS_STEER else 0.0
            send(ser, {"type": "drive", "throttle": round(throttle, 3), "steer": round(steer, 3)})

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        send(ser, {"type": "drive", "throttle": 0, "steer": 0})
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
