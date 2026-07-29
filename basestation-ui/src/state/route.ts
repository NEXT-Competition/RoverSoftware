// Waypoint-route drawing state, shared between the map (which appends points on
// tap) and the route controls (toggle / send / clear). Ports the pending-route
// logic from the old static/app.js.

import { signal } from "@preact/signals";
import type { LatLon } from "../net/types.ts";
import { selected, send } from "../net/ws.ts";
import { placeMode } from "./places.ts";

export const routeMode = signal(false);
export const routePts = signal<LatLon[]>([]);

export function toggleRouteMode(): void {
  routeMode.value = !routeMode.value;
  // One tap on the map can only mean one thing. Both modes consume the same
  // gesture, so turning this on turns place-dropping off rather than leaving
  // the map to guess (MapView resolves a tie in favour of the route).
  if (routeMode.value) placeMode.value = false;
}

export function addWaypoint(lat: number, lon: number): void {
  routePts.value = [...routePts.value, [lat, lon]];
}

export function clearRoute(): void {
  routePts.value = [];
}

/** Send the drawn route and switch the robot into waypoint mode. */
export function sendRoute(): void {
  const rid = selected.value;
  const pts = routePts.value;
  if (!rid || pts.length === 0) return;
  send({ action: "route", robot_id: rid, waypoints: pts });
  send({ action: "mode", robot_id: rid, mode: "waypoint" });
  routeMode.value = false;
}
