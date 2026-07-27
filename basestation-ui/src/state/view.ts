// Which top-level view is showing.
//
// A signal rather than a router: there are two views, the console runs offline
// in a kiosk with no address bar, and URL state would only add a way for a
// reload to land somewhere other than the map.

import { signal } from "@preact/signals";
import { releaseDrive } from "../net/input.ts";

export type View = "ops" | "settings";

export const view = signal<View>("ops");

/**
 * Switch views, commanding a hard stop on the way out of the driving view.
 *
 * The drive pad unmounts when the settings sheet opens, so a thumb that was
 * down never gets its pointerup — without this, the last stick vector would
 * keep being re-sent by the keepalive while the operator reads about PID gains.
 *
 * A *physical* gamepad deliberately keeps driving: it is a real control
 * surface the operator is still holding, the server-side reader never stopped
 * anyway, and a pad that silently went dead behind a settings page is how you
 * end up unable to stop a rover with the stick in your hand.
 */
export function showView(next: View): void {
  if (next !== "ops") releaseDrive();
  view.value = next;
}
