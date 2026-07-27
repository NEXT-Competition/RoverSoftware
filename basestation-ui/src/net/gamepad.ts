// Physical gamepad via the browser Gamepad API — a best-effort companion to the
// on-screen joystick. Controls match the server reader
// (basestation/controller_input.py): R2 = forward, L2 = reverse, right stick =
// steer.
//   axes[2] = right stick X (steer)
//   btn0 = cross (clear)   btn1 = circle (e-stop)   btn4 = L1 (teleop)   btn5 = R1 (object_align)
//   btn6 = L2 (reverse)    btn7 = R2 (forward)
//
// The triggers are the one place the two readers can't share an index: the
// browser "standard" mapping exposes L2/R2 as *buttons* 6/7 with an analog
// .value already scaled 0..1, while SDL/pygame exposes them as axes 4/5 scaled
// -1..1. Same control, same feel, different lookup — hence no shared constant.
//
// Note WebView Gamepad support is uneven (WebView2 good, macOS WKWebView weak),
// which is exactly why the reliable physical-controller path stays server-side
// (pygame). This just makes a controller work when the browser exposes it.

import { signal } from "@preact/signals";

export const AXIS_STEER = 2;
export const BTN_ESTOP = 1;
export const BTN_CLEAR = 0;
export const BTN_TELEOP = 4;
export const BTN_ALIGN = 5;
export const BTN_L2 = 6;
export const BTN_R2 = 7;

export interface GamepadSample {
  name: string;
  throttle: number; // R2 - L2, so +forward / -reverse
  steer: number;
  buttons: boolean[];
}

/** Analog pull of a trigger button, 0..1. Falls back to its digital state. */
function triggerValue(pad: Gamepad, idx: number): number {
  const b = pad.buttons[idx];
  if (!b) return 0;
  const v = typeof b.value === "number" ? b.value : (b.pressed ? 1 : 0);
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

/** Reactive: is a browser-visible gamepad present, and its name. */
export const browserGamepad = signal<{ connected: boolean; name: string | null }>({
  connected: false,
  name: null,
});

export function pollGamepad(): GamepadSample | null {
  const pads = typeof navigator !== "undefined" && navigator.getGamepads
    ? navigator.getGamepads()
    : [];
  const pad = Array.from(pads).find((p): p is Gamepad => p != null);
  if (!pad) {
    if (browserGamepad.value.connected) {
      browserGamepad.value = { connected: false, name: null };
    }
    return null;
  }
  if (
    !browserGamepad.value.connected ||
    browserGamepad.value.name !== pad.id
  ) {
    browserGamepad.value = { connected: true, name: pad.id };
  }
  return {
    name: pad.id,
    throttle: triggerValue(pad, BTN_R2) - triggerValue(pad, BTN_L2),
    steer: pad.axes[AXIS_STEER] ?? 0,
    buttons: pad.buttons.map((b) => b.pressed),
  };
}
