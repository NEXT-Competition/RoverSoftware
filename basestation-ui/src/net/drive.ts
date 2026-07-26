// Client-side drive throttling.
//
// IMPORTANT: the Python bridge does NOT rate-limit browser {action:"drive"}
// frames — its throttling in app.py::on_drive only guards the server-side
// pygame gamepad. So an un-capped on-screen joystick would flood the XBee and
// latency would grow without bound. We apply the same policy here:
//
//   DRIVE_EPS          0.01   send only on a meaningful change...
//   drive rate         server ...at most this often, plus...
//   DRIVE_KEEPALIVE    0.25s  ...a periodic keepalive so the robot's teleop
//                             command_timeout failsafe doesn't trip while the
//                             stick is held steady.
//
// The rate is NOT hardcoded: it comes from the bridge's --drive-hz via the
// fleet snapshot (ws.ts::driveHz). Radio airtime is one shared budget across
// the touch joystick, the server gamepad and telemetry, so the server owns the
// number. A local copy is how a lowered --drive-hz silently failed to reach the
// touch UI and left the link oversubscribed.
//
// Deadzone (0.08) and the throttle = -Y / steer = X convention mirror the
// server gamepad reader (basestation/controller_input.py:27-36,90-91) so touch
// and physical gamepad feel identical.

import { driveHz, selected, send } from "./ws.ts";

export const DRIVE_EPS = 0.01;
/** Used only until the first snapshot lands; matches run_basestation.py. */
export const DRIVE_HZ_FALLBACK = 15;
export const DRIVE_KEEPALIVE_MS = 250;
export const DEADZONE = 0.08;

/** Minimum gap between drive frames, from the server's budget. */
export function driveMinIntervalMs(): number {
  return 1000 / (driveHz.value ?? DRIVE_HZ_FALLBACK);
}

export function deadzone(v: number, dz = DEADZONE): number {
  return Math.abs(v) < dz ? 0 : v;
}

function clamp1(v: number): number {
  return v < -1 ? -1 : v > 1 ? 1 : v;
}

interface DriveState {
  throttle: number;
  steer: number;
  t: number;
}

/**
 * Create a drive sender. `update(throttle, steer)` is meant to be called every
 * animation frame with the current input vector; it emits at most ~30 Hz with a
 * 250 ms keepalive. `release()` commands a hard zero immediately (thumb lifted).
 */
export function makeDriveSender() {
  let last: DriveState = { throttle: 0, steer: 0, t: -1e9 };

  function push(throttle: number, steer: number, force: boolean): void {
    const rid = selected.value;
    if (!rid) return;
    const now = performance.now();
    const dt = now - last.t;
    const changed = Math.abs(throttle - last.throttle) > DRIVE_EPS ||
      Math.abs(steer - last.steer) > DRIVE_EPS;
    if (force || (changed && dt >= driveMinIntervalMs()) || dt >= DRIVE_KEEPALIVE_MS) {
      last = { throttle, steer, t: now };
      send({
        action: "drive",
        robot_id: rid,
        throttle: round3(throttle),
        steer: round3(steer),
      });
    }
  }

  return {
    /** Feed the current stick vector (raw; deadzone + clamp applied here). */
    update(throttle: number, steer: number): void {
      push(clamp1(deadzone(throttle)), clamp1(deadzone(steer)), false);
    },
    /** Command an immediate stop and reset the keepalive clock. */
    release(): void {
      last = { throttle: 0, steer: 0, t: -1e9 };
      push(0, 0, true);
    },
  };
}

function round3(v: number): number {
  return Math.round(v * 1000) / 1000;
}
