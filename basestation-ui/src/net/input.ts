// Unified input loop. One requestAnimationFrame pump merges the on-screen
// joystick and a physical gamepad, feeds the rate-limited drive sender, and
// turns gamepad button edges into estop/clear/mode actions — matching the
// server-side reader (basestation/controller_input.py).
//
// Priority: while the touch pad is engaged (thumb down) it wins; otherwise a
// connected gamepad drives. Buttons are always processed so a controller can
// e-stop even while steering by touch.

import { signal } from "@preact/signals";
import { selected, send } from "./ws.ts";
import { makeDriveSender } from "./drive.ts";
import {
  BTN_CLEAR,
  BTN_ALIGN,
  BTN_ESTOP,
  BTN_TELEOP,
  pollGamepad,
} from "./gamepad.ts";

/** Set by the DrivePad while a thumb is down; null when released. */
export const padInput = signal<{ throttle: number; steer: number } | null>(null);

const sender = makeDriveSender();
let running = false;
let prevButtons: boolean[] = [];

function edge(buttons: boolean[], idx: number): boolean {
  const cur = buttons[idx] ?? false;
  const was = prevButtons[idx] ?? false;
  return cur && !was;
}

function tick(): void {
  if (!running) return;
  const gp = pollGamepad();
  const rid = selected.value;

  // Button edges -> discrete actions (need a selected robot).
  if (gp && rid) {
    const b = gp.buttons;
    if (edge(b, BTN_ESTOP)) send({ action: "estop", robot_id: rid });
    if (edge(b, BTN_CLEAR)) send({ action: "clear_estop", robot_id: rid });
    if (edge(b, BTN_TELEOP)) send({ action: "mode", robot_id: rid, mode: "teleop" });
    if (edge(b, BTN_ALIGN)) send({ action: "mode", robot_id: rid, mode: "object_align" });
    prevButtons = b;
  } else if (gp) {
    prevButtons = gp.buttons;
  }

  // Drive vector: touch pad wins while engaged, else the gamepad triggers/stick.
  const pad = padInput.value;
  if (pad) {
    sender.update(pad.throttle, pad.steer);
  } else if (gp) {
    sender.update(gp.throttle, gp.steer);
  } else {
    sender.update(0, 0);
  }

  requestAnimationFrame(tick);
}

export function startInputLoop(): void {
  if (running) return;
  running = true;
  requestAnimationFrame(tick);
}

/** Stop everything and command a hard zero (page hidden / unmount). */
export function releaseDrive(): void {
  padInput.value = null;
  sender.release();
}
