// Which top-level view is showing.
//
// A signal rather than a router: there are three views, the console runs
// offline in a kiosk with no address bar, and URL state would only add a way
// for a reload to land somewhere other than the map.

import { signal } from "@preact/signals";
import { releaseDrive } from "../net/input.ts";

export type View = "ops" | "command" | "settings";

export const view = signal<View>("ops");

/**
 * Switch views, commanding a hard stop on the way out of the driving view.
 *
 * The drive pad unmounts when the settings sheet opens, so a thumb that was
 * down never gets its pointerup — without this, the last stick vector would
 * keep being re-sent by the keepalive while the operator reads about PID gains.
 *
 * A *physical* gamepad is unaffected, and that is now structural rather than a
 * decision this function makes: it is read on the base station and its commands
 * never pass through the browser at all. Which is what you want — a pad that
 * silently went dead behind a settings page is how you end up unable to stop a
 * rover with the stick in your hand.
 */
export function showView(next: View): void {
  if (next !== "ops") releaseDrive();
  view.value = next;
}

/**
 * The same switch, for a view name that came off the wire.
 *
 * A spoken "open settings" arrives as a string the base station planned (see
 * `show_view` in basestation/command/intents.py). It is validated there, and
 * validated again here, because this is where server data becomes navigation
 * and the check costs nothing.
 */
export function showViewByName(name: string): void {
  if (name === "ops" || name === "command" || name === "settings") showView(name);
}
