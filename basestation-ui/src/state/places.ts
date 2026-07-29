// Named field positions: capture, edit, and turn into routes.
//
// The workflow this exists for, in order:
//
//   1. drive a rover up to a bucket in teleop, eyes on the rover, not the screen
//   2. press "Save rover position" — the place is its GPS fix, named for you
//   3. every other rover can now be sent there, by route or by routine
//
// Step 2 is why capture reads the fleet frame rather than asking for
// coordinates. The operator is standing in a field holding a controller; the
// one thing they cannot do is type a latitude.
//
// The list is owned and persisted by the bridge (basestation/places.py) and
// mirrored here through the cold channel. Edits are applied locally AND sent:
// the local half keeps a rapid sequence of taps from lagging a round trip, and
// the echo is authoritative — the bridge is what renumbers a duplicate id, so
// whatever it says wins a moment later.

import { computed, signal } from "@preact/signals";
import type { LatLon, Place, PlaceKind } from "../net/types.ts";
import { places, selectedRobot, send } from "../net/ws.ts";

/** Tapping the map drops a place instead of a route waypoint. Mutually
 *  exclusive with route drawing — see state/route.ts. */
export const placeMode = signal(false);

/** Which place the panel has open for editing, if any. */
export const editingPlace = signal<string | null>(null);

/** Ordered for display: buckets first (they are what a match is about), then
 *  by name, so the list doesn't reshuffle as places are added. */
export const placeList = computed<Place[]>(() => {
  const order: Record<PlaceKind, number> = { bucket: 0, start: 1, marker: 2, hazard: 3 };
  return [...places.value].sort((a, b) =>
    (order[a.kind] ?? 9) - (order[b.kind] ?? 9) || a.name.localeCompare(b.name)
  );
});

export const placeById = computed<Record<string, Place>>(() =>
  Object.fromEntries(places.value.map((p) => [p.id, p]))
);

/** Mirrors basestation/places.py::slugify, so an optimistic id matches the one
 *  the bridge is about to hand back and the row doesn't flicker. The bridge is
 *  still the authority; this only has to agree with it in the common case. */
function slugify(name: string, taken: string[]): string {
  let base = name.trim().toLowerCase().replace(/[^a-z0-9]+/g, "_").replace(/^_+|_+$/g, "");
  if (!base || !/^[a-z]/.test(base)) base = base ? `p_${base}` : "place";
  base = base.slice(0, 32);
  if (!taken.includes(base)) return base;
  for (let n = 2; n < 1000; n++) {
    const suffix = `_${n}`;
    const candidate = base.slice(0, 32 - suffix.length) + suffix;
    if (!taken.includes(candidate)) return candidate;
  }
  return base;
}

const KIND_LABEL: Record<PlaceKind, string> = {
  bucket: "Bucket",
  marker: "Marker",
  start: "Start",
  hazard: "Hazard",
};

/** "Bucket 3" — the next unused number for this kind.
 *
 *  Auto-named rather than prompting: a modal between "I am next to the bucket"
 *  and "the position is saved" is a modal that gets dismissed, and a place you
 *  can rename later is strictly better than a fix you didn't take. */
function autoName(kind: PlaceKind, existing: Place[]): string {
  const label = KIND_LABEL[kind] ?? "Place";
  const used = new Set(existing.map((p) => p.name));
  for (let n = 1; n < 200; n++) {
    if (!used.has(`${label} ${n}`)) return `${label} ${n}`;
  }
  return label;
}

function commit(next: Place[]): void {
  places.value = next;             // optimistic; the echo confirms
  send({ action: "set_places", places: next });
}

/** Add a place at an explicit position (a map tap). Returns its id. */
export function addPlaceAt(
  lat: number,
  lon: number,
  kind: PlaceKind = "bucket",
  name?: string,
): string {
  const existing = places.value;
  const label = name?.trim() || autoName(kind, existing);
  const id = slugify(label, existing.map((p) => p.id));
  commit([...existing, { id, name: label, lat, lon, kind }]);
  return id;
}

/** Where the selected rover is right now, or null if it has no fix. */
export const capturablePosition = computed<LatLon | null>(() => {
  const robot = selectedRobot.value;
  if (!robot || robot.lat == null || robot.lon == null) return null;
  return [robot.lat, robot.lon];
});

/** Save the selected rover's current position as a place.
 *
 *  The headline feature: this is what "drive up to the bucket and save it"
 *  means. Returns the new place's id, or null when the rover has no GPS fix —
 *  which the button already reflects by being disabled, so null is the race
 *  where the fix dropped between render and press.
 */
export function captureRoverPosition(kind: PlaceKind = "bucket", name?: string): string | null {
  const at = capturablePosition.value;
  if (!at) return null;
  return addPlaceAt(at[0], at[1], kind, name);
}

export function renamePlace(id: string, name: string): void {
  const trimmed = name.slice(0, 40);
  commit(places.value.map((p) => (p.id === id ? { ...p, name: trimmed } : p)));
}

export function setPlaceKind(id: string, kind: PlaceKind): void {
  commit(places.value.map((p) => (p.id === id ? { ...p, kind } : p)));
}

/** Re-take a place's position from where the rover is now. A bucket that moved
 *  between matches is the common case, and deleting and re-adding it would
 *  break every routine that already points at it — the id is the whole value. */
export function recapturePlace(id: string): boolean {
  const at = capturablePosition.value;
  if (!at) return false;
  commit(places.value.map((p) => (p.id === id ? { ...p, lat: at[0], lon: at[1] } : p)));
  return true;
}

export function movePlaceTo(id: string, lat: number, lon: number): void {
  commit(places.value.map((p) => (p.id === id ? { ...p, lat, lon } : p)));
}

export function removePlace(id: string): void {
  if (editingPlace.value === id) editingPlace.value = null;
  commit(places.value.filter((p) => p.id !== id));
}

/** Place ids -> coordinates, dropping any that no longer exist.
 *
 *  Used when a routine is saved: the robot is handed plain lat/lon and never
 *  learns what a place is (see state/routines.ts::resolveRoutePlaces). */
export function resolvePlaces(ids: string[]): LatLon[] {
  const byId = placeById.value;
  const out: LatLon[] = [];
  for (const id of ids) {
    const place = byId[id];
    if (place) out.push([place.lat, place.lon]);
  }
  return out;
}

/** Ids that no longer name a place — what a routine editor has to warn about,
 *  since the saved routine would quietly drive a shorter route. */
export function missingPlaces(ids: string[]): string[] {
  const byId = placeById.value;
  return ids.filter((id) => !byId[id]);
}
