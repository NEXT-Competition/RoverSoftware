// Starter routines: something to open instead of an empty canvas.
//
// A competitor's first contact with this tab used to be a blank grid and a
// two-state skeleton that drives nowhere. That teaches the mechanics of the
// editor and nothing about what a routine is FOR, and the gap between "I can
// drag a box" and "I can express an autonomous run" is where teams give up.
//
// Each template is a real, complete, runnable routine that demonstrates one
// idea end to end. They open as an editable draft and are never run behind the
// operator's back. The route one ships with no waypoints on purpose —
// coordinates are the one thing that cannot be guessed for someone else's
// field, so it says what it needs rather than inventing a location.
//
// --- Why the data lives in a .json next door ---
// Every verb used here has to exist in the robot's own BUILDERS tables and
// validate under the DEFAULT RoutineConfig — in particular nothing arms a
// launcher, because `arm` is refused unless the robot has been told to permit
// it, and a starter routine that fails to save is worse than no starter at all.
// The only way to know that is to run the real Python schema over them, so the
// specs live in templates.json and BOTH sides read that one file:
// tests/test_routine_templates.py parses every template with the real
// validator, and a template the robot would refuse fails the build instead of
// the operator's afternoon.

import type { RoutineSpec } from "../net/types.ts";
import TEMPLATE_DATA from "./templates.json" with { type: "json" };

export interface Template {
  key: string;
  title: string;
  blurb: string;
  /** What the operator must supply before it will do anything useful. */
  needs?: string;
  spec: RoutineSpec;
}

export const TEMPLATES: Template[] = TEMPLATE_DATA as Template[];
