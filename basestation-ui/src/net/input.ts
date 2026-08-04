// The on-screen joysticks' input loop. One requestAnimationFrame pump feeds the
// rate-limited drive sender from the combined throttle and steering pads.
//
// --- Why there is no gamepad here ---
// A physical controller is read on the base station itself, by pygame
// (basestation/controller_input.py), and its commands go straight out over the
// XBee from app.py::on_drive. The browser does not participate. It used to poll
// the Gamepad API and forward drive frames over the WebSocket, which put the
// browser, the Deno front door and a socket reconnect in the path between a
// trigger pull and a rover moving — and made two readers race to command the
// same robot whenever both saw the pad.
//
// What the browser still owns is the touch pad, because that input has nowhere
// else to come from. Bindings are unchanged and still edited from the settings
// page: they live in the base station's saved ControllerMapping, which the
// pygame reader applies directly.
//
// Conditioning (dead zone, gains, inversion) happens at the source, as before —
// for a gamepad that is controller_input.py applying the saved mapping, and for
// the touch pad it is `deadzone()` here. A second dead zone downstream would
// silently swallow a lowered throttle authority.
//
// --- An idle pad says NOTHING, and that is load-bearing ---
// While no thumb is down this loop sends no frames at all. It must not "just
// keep commanding zero": the drive sender re-transmits every 250 ms to keep the
// robot's command_timeout alive, so an idle browser doing that would interleave
// throttle=0 at 4 Hz with the base station's held-trigger throttle=1 — both go
// out through the same app.py::dispatch — and the rover lurches. Silence is the
// only way for a browser with nothing to say to stay out of the gamepad's way.
//
// Silence is also safe, which is what makes it available: the robot stops on its
// own when commands stop arriving (command_timeout), so releasing the pad does
// not depend on a stop frame being delivered. We send one anyway, on the falling
// edge, so the stop is immediate rather than a timeout away. The keepalive still
// applies while a thumb IS down — a held stick must not look like a dead link.

import { signal } from "@preact/signals";
import { deadzone, makeDriveSender } from "./drive.ts";

/** Set by the DrivePad while either thumb is down; null when both are released. */
export const padInput = signal<{ throttle: number; steer: number } | null>(null);

const sender = makeDriveSender();
let running = false;
// Was a thumb down on the last tick? Drives the falling edge below.
let engaged = false;

function tick(): void {
  if (!running) return;
  const pad = padInput.value;
  if (pad) {
    engaged = true;
    sender.update(deadzone(pad.throttle), deadzone(pad.steer));
  } else if (engaged) {
    // Thumb just lifted: one hard zero, then silence.
    engaged = false;
    sender.release();
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
  engaged = false;
  sender.release();
}
