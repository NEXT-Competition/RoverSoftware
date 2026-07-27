// Physical gamepad via the browser Gamepad API — a best-effort companion to the
// on-screen joystick. Controls match the server reader
// (basestation/controller_input.py): R2 = forward, L2 = reverse, right stick =
// steer.
//
// The layout is NOT hardcoded here. It comes from the base station's saved
// mapping (basestation/settings.py::ControllerMapping), pushed to every client
// on the /ws settings channel, so re-binding a pad in the settings page moves
// both readers at once. Two constants would drift apart the first time someone
// used a different controller.
//
// The triggers are the one place the two readers genuinely can't share an
// index: the browser "standard" mapping exposes L2/R2 as *buttons* 6/7 with an
// analog .value already scaled 0..1, while SDL/pygame exposes them as axes 4/5
// scaled -1..1. Same control, same feel, different lookup — so the mapping's
// trigger *axis* fields are server-side only, and the browser keeps its own
// button indices below. Everything else (steer axis, buttons, dead zone,
// gains, inversion) is shared.
//
// Note WebView Gamepad support is uneven (WebView2 good, macOS WKWebView weak),
// which is exactly why the reliable physical-controller path stays server-side
// (pygame). This just makes a controller work when the browser exposes it.

import { signal } from "@preact/signals";
import { baseSettings } from "./ws.ts";

// Browser "standard mapping" trigger buttons — see the note above.
export const BTN_L2 = 6;
export const BTN_R2 = 7;

/** Built-in fallbacks, used until the first settings frame lands. They match
 *  ControllerMapping's own defaults (a DualShock 4). */
const DEFAULTS = {
  axis_steer: 2,
  deadzone: 0.08,
  steer_gain: 1,
  throttle_gain: 1,
  invert_steer: false,
  btn_estop: 1,
  btn_clear: 0,
  btn_teleop: 4,
  btn_object_align: 5,
  btn_shooter_align: -1,
  btn_waypoint: -1,
  btn_arm_shooter: -1,
  btn_fire: -1,
} as const;

export type MappingKey = keyof typeof DEFAULTS;

/** One mapping value, from the server's copy or the built-in default. */
export function mapped<K extends MappingKey>(key: K): typeof DEFAULTS[K] {
  const v = baseSettings.value[`controller.${key}`];
  return (v === undefined ? DEFAULTS[key] : v) as typeof DEFAULTS[K];
}

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

function clamp1(v: number): number {
  return v < -1 ? -1 : v > 1 ? 1 : v;
}

/** Reactive: is a browser-visible gamepad present, and its name. */
export const browserGamepad = signal<{ connected: boolean; name: string | null }>({
  connected: false,
  name: null,
});

/**
 * Sample the pad and apply the mapping — dead zone, gains and inversion
 * included, exactly as controller_input.py does on the server.
 *
 * Returning an already-conditioned vector (rather than a raw one for the drive
 * sender to clean up) is what keeps a lowered throttle authority from being
 * swallowed by a second, larger dead zone downstream.
 */
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

  const dz = mapped("deadzone");
  const cut = (v: number) => (Math.abs(v) < dz ? 0 : v);
  const steerRaw = pad.axes[mapped("axis_steer")] ?? 0;
  const throttleRaw = triggerValue(pad, BTN_R2) - triggerValue(pad, BTN_L2);

  return {
    name: pad.id,
    throttle: clamp1(cut(throttleRaw) * mapped("throttle_gain")),
    steer: clamp1(
      cut(steerRaw) * mapped("steer_gain") * (mapped("invert_steer") ? -1 : 1),
    ),
    buttons: pad.buttons.map((b) => b.pressed),
  };
}
