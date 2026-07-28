// Editing state for the settings page.
//
// A field's displayed value comes from one of two places: the server's copy
// (the truth), or a local edit that hasn't been acknowledged yet. That second
// bucket exists because the round trip is real — a robot's config crosses a
// 57600-baud radio — and a slider that snapped back to the old value for 200 ms
// after every drag would be unusable.
//
// An edit is "staged" while you're still moving the control and "sent" once it
// has gone out. Only sent edits are cleared when the server answers, so an
// unrelated settings frame (someone else changed something, a robot answered a
// different request) can't wipe a value you're halfway through typing.
//
// The server ALWAYS wins on arrival: it clamps out-of-range numbers, so what
// comes back may not be what was asked for, and showing the asked-for value
// would be a quiet lie about what the robot is doing.

import { batch, computed, signal } from "@preact/signals";
import type { SettingValue } from "../net/types.ts";
import { baseSettings, robotConfigs, selected, send } from "../net/ws.ts";

interface Edit {
  value: SettingValue;
  sent: boolean;
}

const edits = signal<Record<string, Edit>>({});

/** Paths whose value the server confirmed recently, for a brief "saved" pulse. */
export const confirmed = signal<Record<string, number>>({});
const CONFIRM_MS = 1400;

/** Which settings tab is showing. */
export type Tab = "robot" | "hardware" | "routines" | "controller" | "base";
export const tab = signal<Tab>("robot");

/** The robot the settings page is editing. Defaults to the driving selection,
 *  but is separate: you may want to tune rover2 while still driving rover1. */
export const configTarget = signal<string | null>(null);

export const targetRobot = computed(() => configTarget.value ?? selected.value);

/** Server-side value for a path — base settings and robot config in one lookup,
 *  since the schema's dotted paths never collide ("base.*"/"controller.*" vs
 *  the robot's own tree). */
export function serverValue(path: string): SettingValue | undefined {
  if (path.startsWith("base.") || path.startsWith("controller.")) {
    return baseSettings.value[path];
  }
  const rid = targetRobot.value;
  if (!rid) return undefined;
  return robotConfigs.value[rid]?.config?.[path];
}

/** What the control should show: a local edit if there is one, else the server. */
export function fieldValue(path: string): SettingValue | undefined {
  const edit = edits.value[path];
  return edit ? edit.value : serverValue(path);
}

export function isDirty(path: string): boolean {
  return edits.value[path] != null;
}

export function isConfirmed(path: string): boolean {
  const at = confirmed.value[path];
  return at != null && Date.now() - at < CONFIRM_MS;
}

/** Record an edit locally without sending it (a slider mid-drag). */
export function stage(path: string, value: SettingValue): void {
  edits.value = { ...edits.value, [path]: { value, sent: false } };
}

/** Send an edit to wherever it belongs. Staged first, so the control keeps
 *  showing the new value across the round trip. */
export function commit(path: string, value: SettingValue): void {
  edits.value = { ...edits.value, [path]: { value, sent: true } };
  if (path.startsWith("base.") || path.startsWith("controller.")) {
    send({ action: "set_settings", settings: { [path]: value } });
    return;
  }
  const rid = targetRobot.value;
  if (!rid) return;
  send({ action: "set_config", robot_id: rid, config: { [path]: value } });
}

/** Commit whatever is currently staged for `path` (a slider's pointer-up). */
export function commitStaged(path: string): void {
  const edit = edits.value[path];
  if (edit && !edit.sent) commit(path, edit.value);
}

/** Drop a staged edit and fall back to the server's value (Escape / Reset). */
export function revert(path: string): void {
  const { [path]: _dropped, ...rest } = edits.value;
  edits.value = rest;
}

/**
 * Clear sent edits now the server has answered, and pulse them as confirmed.
 * Called from the settings-frame handler wiring in main.tsx.
 *
 * Reads its own state with `.peek()`, never `.value`. This runs inside an
 * effect that subscribes to whatever it reads, and subscribing to `edits`
 * would make committing an edit immediately acknowledge it — the field would
 * snap back to the server's old value the instant you released the slider,
 * for the whole round trip.
 */
export function acknowledge(): void {
  const now = Date.now();
  const current = edits.peek();
  const remaining: Record<string, Edit> = {};
  const pulses: Record<string, number> = { ...confirmed.peek() };
  let changed = false;
  for (const path of Object.keys(current)) {
    if (current[path].sent) {
      pulses[path] = now;
      changed = true;
      // Drop the pulse again on a timer, so the "saved" highlight actually
      // fades. Nothing else re-renders this page on a schedule — settings
      // frames arrive on change — so without this the green bar would sit
      // there until the next unrelated edit.
      setTimeout(() => {
        const { [path]: _expired, ...rest } = confirmed.peek();
        confirmed.value = rest;
      }, CONFIRM_MS);
    } else {
      remaining[path] = current[path];
    }
  }
  if (!changed) return;
  batch(() => {
    edits.value = remaining;
    confirmed.value = pulses;
  });
}

/** Ask the target robot for its config. Explicit — it costs radio airtime. */
export function refreshRobotConfig(): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "get_config", robot_id: rid });
}

/** Ask for the descriptors of the fields this file cannot know about, because
 *  the operator declared the actuators they belong to. Same explicit-fetch
 *  rule as the config, and usually a fraction of the size. */
export function refreshRobotFields(): void {
  const rid = targetRobot.value;
  if (rid) send({ action: "get_fields", robot_id: rid });
}

/** Subscribe/unsubscribe this socket to raw gamepad frames. */
export function watchGamepad(on: boolean): void {
  send({ action: "watch_gamepad", on });
}
