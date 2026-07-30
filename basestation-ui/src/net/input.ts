// The on-screen joystick's input loop. One requestAnimationFrame pump feeds the
// rate-limited drive sender from whatever the touch pad is reporting.
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

import { signal } from "@preact/signals";
import { deadzone, makeDriveSender } from "./drive.ts";

/** Set by the DrivePad while a thumb is down; null when released. */
export const padInput = signal<{ throttle: number; steer: number } | null>(null);

const sender = makeDriveSender();
let running = false;

function tick(): void {
  if (!running) return;
  const pad = padInput.value;
  if (pad) {
    sender.update(deadzone(pad.throttle), deadzone(pad.steer));
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
