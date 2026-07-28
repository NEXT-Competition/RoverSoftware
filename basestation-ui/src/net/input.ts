// Unified input loop. One requestAnimationFrame pump merges the on-screen
// joystick and a physical gamepad, feeds the rate-limited drive sender, and
// turns gamepad button edges into actions — using the same bindings as the
// server-side reader (basestation/controller_input.py), which the settings
// page can edit.
//
// Priority: while the touch pad is engaged (thumb down) it wins; otherwise a
// connected gamepad drives. Buttons are always processed so a controller can
// e-stop even while steering by touch.
//
// Conditioning (dead zone, gains, inversion) happens per source, not here:
// gamepad.ts applies the saved mapping to a pad sample, and the touch pad is
// deadzoned below. A single downstream dead zone would silently swallow a
// lowered throttle authority — see gamepad.ts::pollGamepad.

import { signal } from "@preact/signals";
import { selected, send } from "./ws.ts";
import { deadzone, makeDriveSender } from "./drive.ts";
import { mapped, type MappingKey, pollGamepad } from "./gamepad.ts";
import type { Action, Mode } from "./types.ts";

/** Set by the DrivePad while a thumb is down; null when released. */
export const padInput = signal<{ throttle: number; steer: number } | null>(null);

const sender = makeDriveSender();
let running = false;
let prevButtons: boolean[] = [];

function mode(rid: string, m: Mode): Action {
  return { action: "mode", robot_id: rid, mode: m };
}

/** Bindable button -> the action it sends. Mirrors ControllerMapping.actions(). */
const BINDINGS: { key: MappingKey; make: (rid: string) => Action }[] = [
  { key: "btn_estop", make: (rid) => ({ action: "estop", robot_id: rid }) },
  { key: "btn_clear", make: (rid) => ({ action: "clear_estop", robot_id: rid }) },
  { key: "btn_teleop", make: (rid) => mode(rid, "teleop") },
  { key: "btn_object_align", make: (rid) => mode(rid, "object_align") },
  { key: "btn_shooter_align", make: (rid) => mode(rid, "shooter_align") },
  { key: "btn_waypoint", make: (rid) => mode(rid, "waypoint") },
  { key: "btn_arm_shooter", make: (rid) => ({ action: "arm_shooter", robot_id: rid }) },
  { key: "btn_fire", make: (rid) => ({ action: "fire", robot_id: rid }) },
];

function edge(buttons: boolean[], idx: number): boolean {
  if (idx < 0) return false; // unbound
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
    for (const binding of BINDINGS) {
      if (edge(gp.buttons, mapped(binding.key) as number)) send(binding.make(rid));
    }
    prevButtons = gp.buttons;
  } else if (gp) {
    prevButtons = gp.buttons;
  }

  // Drive vector: touch pad wins while engaged, else the gamepad triggers/stick.
  const pad = padInput.value;
  if (pad) {
    sender.update(deadzone(pad.throttle), deadzone(pad.steer));
  } else if (gp) {
    sender.update(gp.throttle, gp.steer); // already conditioned by the mapping
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
