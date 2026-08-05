// Client-side drive throttling for the on-screen joystick — the only drive
// input the browser still sends. A physical controller never reaches this
// sender: it is read on the base station and goes straight to the radio (see
// net/input.ts).
//
// IMPORTANT: the Python bridge does NOT rate-limit browser {action:"drive"}
// frames — its throttling in app.py::on_drive guards the pygame gamepad only.
// So an un-capped on-screen joystick would flood the XBee and latency would
// grow without bound. We apply the same policy here:
//
//   DRIVE_EPS          0.01   send only on a meaningful change...
//   drive rate         server ...at most this often, plus...
//   DRIVE_KEEPALIVE    0.25s  ...a periodic keepalive so the robot's teleop
//                             command_timeout failsafe doesn't trip while the
//                             stick is held steady.
//
// The rate is NOT hardcoded: it comes from the bridge's --drive-hz via the
// fleet snapshot (ws.ts::driveHz). Radio airtime is one shared budget across
// the touch joystick, the base station's gamepad and telemetry, so the server
// owns the number. A local copy is how a lowered --drive-hz silently failed to
// reach the touch UI and left the link oversubscribed.
//
// Deadzone (0.08) and the signed throttle / steer convention mirror the gamepad
// reader (basestation/controller_input.py) so touch and a physical pad feel
// identical, even though they now reach the radio by different routes. The touch
// pad hands this sender a throttle already in -1..1, from its Y offset.

import { driveHz, selected, selectedRobot, send } from "./ws.ts";

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
    // The bridge streams drive frames to the selected rover and only while it
    // is in teleop, and drops anything else (app.py::send_drive). Not sending
    // what would be dropped keeps the socket quiet — but the check is
    // PERMISSIVE: a rover we have no telemetry for yet still gets its frames,
    // because the bridge is the authority and a UI guessing wrong here would
    // cost the operator control rather than a few bytes.
    const robot = selectedRobot.value;
    if (robot && robot.mode !== "teleop") return;
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
    /**
     * Feed the current stick vector, already conditioned by its source.
     *
     * Only clamped here, NOT deadzoned: the touch pad applies its own dead zone
     * via `deadzone()` above, and a second one at this layer would quietly
     * swallow small commands from an input running reduced authority.
     */
    update(throttle: number, steer: number): void {
      push(clamp1(throttle), clamp1(steer), false);
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
