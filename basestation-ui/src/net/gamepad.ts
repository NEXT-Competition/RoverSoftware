// Physical gamepad via the browser Gamepad API — a best-effort companion to the
// on-screen joystick. The browser "standard" mapping lines up with the DS4
// indices the server reader uses (basestation/controller_input.py):
//   axes[1] = left stick Y (negate -> throttle)   axes[2] = right stick X (steer)
//   btn0 = cross (clear)   btn1 = circle (e-stop)   btn4 = L1 (teleop)   btn5 = R1 (object_align)
//
// Note WebView Gamepad support is uneven (WebView2 good, macOS WKWebView weak),
// which is exactly why the reliable physical-controller path stays server-side
// (pygame). This just makes a controller work when the browser exposes it.

import { signal } from "@preact/signals";

export const AXIS_THROTTLE = 1;
export const AXIS_STEER = 2;
export const BTN_ESTOP = 1;
export const BTN_CLEAR = 0;
export const BTN_TELEOP = 4;
export const BTN_ALIGN = 5;

export interface GamepadSample {
  name: string;
  throttle: number; // already negated (up = forward)
  steer: number;
  buttons: boolean[];
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
    throttle: -(pad.axes[AXIS_THROTTLE] ?? 0),
    steer: pad.axes[AXIS_STEER] ?? 0,
    buttons: pad.buttons.map((b) => b.pressed),
  };
}
