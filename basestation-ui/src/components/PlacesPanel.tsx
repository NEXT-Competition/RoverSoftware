// Saved field positions: capture them, fix them, and build a route out of them.
//
// The layout follows the order of the job rather than the shape of the data.
// Capturing comes first and is one button, because it happens with the operator
// looking at a rover parked against a bucket and not at this screen. Editing is
// folded away behind the row it belongs to — renaming is a calm-moment job, and
// putting a text field next to the capture button is how you rename a place
// when you meant to save one.

import { selected } from "../net/ws.ts";
import {
  capturablePosition,
  captureRoverPosition,
  editingPlace,
  placeList,
  placeMode,
  recapturePlace,
  removePlace,
  renamePlace,
  setPlaceKind,
} from "../state/places.ts";
import { addWaypoint, routeMode, routePts } from "../state/route.ts";
import type { Place, PlaceKind } from "../net/types.ts";

const KINDS: { value: PlaceKind; label: string; glyph: string }[] = [
  { value: "bucket", label: "Bucket", glyph: "◆" },
  { value: "marker", label: "Marker", glyph: "●" },
  { value: "start", label: "Start", glyph: "▲" },
  { value: "hazard", label: "Hazard", glyph: "✕" },
];

const GLYPH: Record<PlaceKind, string> = Object.fromEntries(
  KINDS.map((k) => [k.value, k.glyph]),
) as Record<PlaceKind, string>;

/** Six decimals is ~11 cm — past the point where more digits are GPS noise
 *  being presented as precision. Monospaced so a column of them scans. */
function coords(place: Place): string {
  return `${place.lat.toFixed(6)}, ${place.lon.toFixed(6)}`;
}

function PlaceRow({ place, inRoute }: { place: Place; inRoute: number }) {
  const open = editingPlace.value === place.id;
  const canRecapture = capturablePosition.value != null;

  return (
    <li class={`place-row${open ? " open" : ""}`}>
      <div class="place-head">
        <button
          class="place-id"
          title="Show on the map"
          onClick={() => (editingPlace.value = open ? null : place.id)}
        >
          <span class={`place-glyph k-${place.kind}`}>{GLYPH[place.kind]}</span>
          <span class="place-text">
            <span class="place-name">{place.name}</span>
            <span class="place-coords">{coords(place)}</span>
          </span>
        </button>
        <button
          class="btn small"
          title="Add to the pending route"
          onClick={() => {
            // Route drawing and place dropping share the map's tap handler, so
            // adding from here turns drawing ON — otherwise the operator adds
            // two places from the list, taps the map to add a third, and drops
            // a new place instead.
            routeMode.value = true;
            placeMode.value = false;
            addWaypoint(place.lat, place.lon);
          }}
        >
          {inRoute > 0 ? `→ ${inRoute}` : "→"}
        </button>
      </div>

      {open && (
        <div class="place-edit">
          <input
            class="field-input tiny"
            type="text"
            value={place.name}
            maxLength={40}
            aria-label="Place name"
            onInput={(e) => renamePlace(place.id, (e.target as HTMLInputElement).value)}
          />
          <div class="segmented">
            {KINDS.map((k) => (
              <button
                key={k.value}
                class={place.kind === k.value ? "on" : ""}
                onClick={() => setPlaceKind(place.id, k.value)}
              >
                {k.glyph}
              </button>
            ))}
          </div>
          <div class="btn-row">
            <button
              class="btn small"
              disabled={!canRecapture}
              title="Move this place to where the rover is now, keeping its name"
              onClick={() => recapturePlace(place.id)}
            >
              Re-take here
            </button>
            <button class="btn small danger" onClick={() => removePlace(place.id)}>
              Delete
            </button>
          </div>
        </div>
      )}
    </li>
  );
}

export function PlacesPanel() {
  const rid = selected.value;
  const list = placeList.value;
  const at = capturablePosition.value;
  const dropping = placeMode.value;
  const pending = routePts.value;

  // How many times each place is already in the pending route. Shown on the row
  // so "did I add that one?" is answerable without counting pins on the map.
  const counts = new Map<string, number>();
  for (const [lat, lon] of pending) {
    for (const place of list) {
      if (Math.abs(place.lat - lat) < 1e-9 && Math.abs(place.lon - lon) < 1e-9) {
        counts.set(place.id, (counts.get(place.id) ?? 0) + 1);
      }
    }
  }

  return (
    <div>
      <div class="section-title">
        <span class="eyebrow">Places</span>
        {list.length > 0 && <span class="eyebrow">{list.length}</span>}
      </div>

      <div class="btn-row" style="margin-top:8px">
        <button
          class="btn primary"
          disabled={!rid || !at}
          title={at
            ? "Save where this rover is standing"
            : "The selected rover has no GPS fix"}
          onClick={() => {
            const id = captureRoverPosition("bucket");
            if (id) editingPlace.value = id;
          }}
        >
          Save rover position
        </button>
        <button
          class={`btn ghost${dropping ? " active" : ""}`}
          onClick={() => {
            placeMode.value = !placeMode.value;
            if (placeMode.value) routeMode.value = false;
          }}
        >
          {dropping ? "Tapping…" : "Drop on map"}
        </button>
      </div>

      <p class="hint">
        {!rid
          ? "Select a rover to save its position."
          : at
          ? "Drive a rover to a bucket, then save where it is standing. Every rover can use it afterwards."
          : "No GPS fix on this rover yet — drop a place on the map instead."}
      </p>

      {list.length > 0 && (
        <ul class="place-list">
          {list.map((place) => (
            <PlaceRow key={place.id} place={place} inRoute={counts.get(place.id) ?? 0} />
          ))}
        </ul>
      )}
    </div>
  );
}
