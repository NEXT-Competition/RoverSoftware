// Breadcrumb trails, assembled here rather than resent by the bridge.
//
// A trail is hundreds of points that gains one per telemetry frame. Sending the
// whole thing at ui_hz made it ~94% of every fleet snapshot and scaled with the
// fleet — three rovers came to 37 KB thirty times a second, per open browser,
// on the same Wi-Fi the FPV video is using. So the bridge sends each trail once
// (`trails`) and thereafter only the points it has just added (`trail_add`).
//
// The one thing a delta scheme has to get right is noticing when it has missed
// something. Every frame carries `trail_seq` — the total number of points ever
// appended to that robot's trail — so the arithmetic is checkable: if we hold
// `n` points' worth of history and the frame says the count is now `seq`, then
// `seq - held` is how many we need and `trail_add` should contain at least that
// many. When it doesn't, we have dropped frames, and no amount of appending
// will repair it: ask for the trails frame again and start over.

import { signal } from "@preact/signals";
import type { LatLon, Robot, TrailsMessage } from "../net/types.ts";

/** Points held per robot, and the `trail_seq` they are current as of. */
interface Trail {
  points: LatLon[];
  seq: number;
}

/**
 * The assembled trails. A signal so the map redraws when they move, keyed by
 * robot_id. Replaced rather than mutated on change — the map diffs against its
 * own last-drawn copy, and an in-place push would be invisible to it.
 */
export const trails = signal<Record<string, Trail>>({});

/** The breadcrumb to draw for one robot, or an empty array if we have none. */
export function trailOf(robotId: string): LatLon[] {
  return trails.value[robotId]?.points ?? [];
}

/** Absorb a full `trails` frame, discarding whatever we had. */
export function resetTrails(msg: TrailsMessage): void {
  const next: Record<string, Trail> = {};
  for (const [id, entry] of Object.entries(msg.trails ?? {})) {
    next[id] = { points: entry.trail ?? [], seq: entry.seq ?? 0 };
  }
  trails.value = next;
}

/**
 * Fold one fleet frame's deltas in. Returns true if we are missing points and
 * need the bridge to resend — the caller owns the socket, not us.
 */
export function applyTrailDeltas(robots: Robot[], trailMax?: number): boolean {
  let gap = false;
  let changed = false;
  const next = { ...trails.value };

  for (const r of robots) {
    if (r.trail_seq == null) continue; // a bridge not sending deltas at all
    const held = next[r.robot_id] ?? { points: [], seq: 0 };
    const need = r.trail_seq - held.seq;
    if (need <= 0) continue; // nothing new, or we are somehow ahead

    const add = r.trail_add ?? [];
    if (need > add.length) {
      // A genuine hole. Leave what we have on screen — a stale trail beats a
      // blank map — and let the caller fetch a fresh copy.
      gap = true;
      continue;
    }
    // The last `need` of them: the bridge counts from ITS cursor, which may be
    // behind ours if a `trails` frame landed between two hot frames.
    const points = held.points.concat(add.slice(add.length - need));
    // Trim to the same cap the bridge keeps, so we drop the same oldest points
    // rather than growing a breadcrumb it has already forgotten.
    if (trailMax != null && trailMax >= 0 && points.length > trailMax) {
      points.splice(0, points.length - trailMax);
    }
    next[r.robot_id] = { points, seq: r.trail_seq };
    changed = true;
  }

  // Forget robots that have dropped out of the fleet, so their trails don't sit
  // in memory for the rest of the match.
  const live = new Set(robots.map((r) => r.robot_id));
  for (const id of Object.keys(next)) {
    if (!live.has(id)) {
      delete next[id];
      changed = true;
    }
  }

  if (changed) trails.value = next;
  return gap;
}
