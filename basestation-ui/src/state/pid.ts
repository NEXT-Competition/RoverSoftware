// A short rolling history of every closed loop the fleet is reporting.
//
// The robot sends only the LATEST step of each loop (robot/control/pid.py), not
// a log — a 50 Hz loop's history would be kilobytes a second on a radio shared
// with driving. So the history is kept here, built one frame at a time as
// telemetry arrives. That has a consequence worth being honest about on screen:
// this is the loop sampled at the TELEMETRY rate, typically 5 Hz, not the
// control rate. It shows drift, bias, a term that does nothing, a loop pinned
// at its limit and whether the error is converging. It cannot show an
// oscillation faster than half the telemetry rate — that one aliases, and the
// cure is to raise the telemetry rate while you look.

import { signal } from "@preact/signals";
import type { PidTrace, Robot } from "../net/types.ts";

/** One step, with the moment it arrived. The robot does not timestamp traces —
 *  it would cost bytes to tell us something we already know — so this is the
 *  arrival time, which is what the x-axis actually means. */
export interface PidSample extends PidTrace {
  t: number;
}

/** How much history to keep per loop. At the default 5 Hz telemetry this is
 *  about a minute — long enough to watch a leg of a route, short enough that
 *  the whole thing stays in a few hundred kilobytes for a fleet of three. */
export const HISTORY = 300;

/** robot_id -> loop path -> samples, oldest first. */
export const pidHistory = signal<Record<string, Record<string, PidSample[]>>>({});

/**
 * Fold one fleet frame into the history.
 *
 * Called from the socket rather than computed from `robots`, because a computed
 * cannot accumulate — and accumulation is the entire point: the robot's frame
 * carries one step, and a graph needs the ones before it.
 *
 * A robot that stops reporting a loop (mode changed, tracing switched off)
 * keeps whatever history it had. Dropping it on the first silent frame would
 * erase the trace of the manoeuvre you just watched, at the exact moment you
 * want to look at it — and the loop coming back appends rather than resumes, so
 * nothing is stitched across a gap it did not measure.
 */
export function recordPidTraces(robots: Robot[], now: number): void {
  let changed = false;
  const next = { ...pidHistory.value };
  for (const robot of robots) {
    const traces = robot.pid;
    if (!traces) continue;
    const loops = { ...(next[robot.robot_id] ?? {}) };
    for (const [loop, trace] of Object.entries(traces)) {
      const previous = loops[loop] ?? [];
      const sample: PidSample = { ...trace, t: now };
      loops[loop] = previous.length >= HISTORY
        ? [...previous.slice(previous.length - HISTORY + 1), sample]
        : [...previous, sample];
      changed = true;
    }
    next[robot.robot_id] = loops;
  }
  if (changed) pidHistory.value = next;
}

/** Throw away a robot's traces — for the button that says so. Tuning is an
 *  iterative act: you change a gain, and what you want on screen is what the
 *  loop did AFTER you changed it, not a curve dominated by the old one. */
export function clearPidHistory(robotId: string): void {
  const next = { ...pidHistory.value };
  delete next[robotId];
  pidHistory.value = next;
}

export function samplesFor(robotId: string | null, loop: string): PidSample[] {
  return robotId ? pidHistory.value[robotId]?.[loop] ?? [] : [];
}

/** The measurement this step, whether the robot sent one or not.
 *
 *  A loop whose setpoint is a constant zero — alignment, where "aligned" means
 *  centred — has no measurement distinct from its error, so the robot does not
 *  spend bytes repeating it. The chart still wants a line to draw. */
export function measured(sample: PidTrace): number {
  return sample.m ?? sample.sp + sample.e;
}
