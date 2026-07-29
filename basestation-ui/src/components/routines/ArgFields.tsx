// The arguments a chosen condition or action needs, rendered from its spec.
//
// One component rather than a case per verb: the vocabulary already says what
// each one takes (routines/vocab.ts), so adding a condition to the Python side
// and its entry there is enough to make it editable here.

import type { ArgSpec } from "../../routines/vocab.ts";
import { availableMechanisms } from "../../state/routines.ts";
import { placeById, placeList, resolvePlaces } from "../../state/places.ts";

/** A patch rather than one key, because a place writes two things at once: the
 *  id the editor reads back, and the plain lat/lon the robot actually drives.
 *  Setting them in two calls would lose the first — both derive from the same
 *  spec object, so the second overwrites it. */
export function ArgFields(
  { args, values, states, onChange }: {
    args: ArgSpec[];
    values: Record<string, unknown>;
    states: string[];
    onChange: (patch: Record<string, unknown>) => void;
  },
) {
  const mechanisms = availableMechanisms.value;
  const saved = placeList.value;
  const byId = placeById.value;

  return (
    <>
      {args.map((arg) => {
        const value = values[arg.key] ?? arg.fallback ?? "";

        if (arg.kind === "number") {
          return (
            <label key={arg.key} class="arg">
              <span>{arg.label}{arg.unit ? ` (${arg.unit})` : ""}</span>
              <input
                class="field-input tiny"
                type="number"
                value={value as number}
                min={arg.min}
                max={arg.max}
                step={arg.step ?? 0.1}
                onInput={(e) => {
                  const parsed = Number((e.target as HTMLInputElement).value);
                  if (!Number.isNaN(parsed)) onChange({ [arg.key]: parsed });
                }}
              />
            </label>
          );
        }

        if (arg.kind === "mech") {
          return (
            <label key={arg.key} class="arg">
              <span>{arg.label}</span>
              <select
                class="field-select tiny"
                value={String(value)}
                onChange={(e) => onChange({ [arg.key]: (e.target as HTMLSelectElement).value })}
              >
                <option value="">—</option>
                {mechanisms.map((m) => (
                  <option key={m.name} value={m.name}>{m.name}</option>
                ))}
              </select>
            </label>
          );
        }

        if (arg.kind === "preset" || arg.kind === "actuator") {
          // Scoped to whichever mechanism the sibling `mech` argument names, so
          // the menu can only offer presets that exist on it.
          const owner = mechanisms.find((m) => m.name === values.mech);
          const options = arg.kind === "preset"
            ? Object.keys(owner?.presets ?? {})
            : (owner?.actuators ?? []).map((a) => a.name);
          return (
            <label key={arg.key} class="arg">
              <span>{arg.label}</span>
              <select
                class="field-select tiny"
                value={String(value)}
                onChange={(e) => onChange({ [arg.key]: (e.target as HTMLSelectElement).value })}
              >
                <option value="">—</option>
                {options.map((o) => <option key={o} value={o}>{o}</option>)}
              </select>
            </label>
          );
        }

        if (arg.kind === "state") {
          return (
            <label key={arg.key} class="arg">
              <span>{arg.label}</span>
              <select
                class="field-select tiny"
                value={String(value)}
                onChange={(e) => onChange({ [arg.key]: (e.target as HTMLSelectElement).value })}
              >
                {states.map((s) => <option key={s} value={s}>{s}</option>)}
              </select>
            </label>
          );
        }

        if (arg.kind === "place") {
          // Writes the id AND the coordinates: the robot reads lat/lon and has
          // never heard of a place, while the id is what lets this menu still
          // show a name after the document has been round-tripped through it.
          const chosen = String(values.place ?? "");
          return (
            <label key={arg.key} class="arg">
              <span>{arg.label}</span>
              <select
                class="field-select tiny"
                value={chosen}
                onChange={(e) => {
                  const id = (e.target as HTMLSelectElement).value;
                  const place = byId[id];
                  onChange(place
                    ? { place: id, lat: place.lat, lon: place.lon }
                    : { place: "", lat: 0, lon: 0 });
                }}
              >
                <option value="">—</option>
                {chosen && !byId[chosen] && (
                  <option value={chosen}>{chosen} (deleted)</option>
                )}
                {saved.map((p) => <option key={p.id} value={p.id}>{p.name}</option>)}
              </select>
            </label>
          );
        }

        if (arg.kind === "route") {
          const ids = Array.isArray(values.places) ? (values.places as string[]) : [];
          // Every edit rewrites BOTH keys together, so the coordinates the
          // robot drives can never drift from the names shown here.
          const write = (next: string[]) =>
            onChange({ places: next, waypoints: resolvePlaces(next) });
          return (
            <div key={arg.key} class="arg arg-route">
              <span>{arg.label}</span>
              {ids.length > 0 && (
                <ol class="route-chips">
                  {ids.map((id, i) => (
                    <li key={`${id}-${i}`} class={`route-chip${byId[id] ? "" : " missing"}`}>
                      <span class="route-n">{i + 1}</span>
                      <span class="route-name">{byId[id]?.name ?? `${id} (deleted)`}</span>
                      {i > 0 && (
                        <button
                          type="button"
                          class="route-btn"
                          title="Move earlier"
                          onClick={() => {
                            const next = [...ids];
                            [next[i - 1], next[i]] = [next[i], next[i - 1]];
                            write(next);
                          }}
                        >
                          ↑
                        </button>
                      )}
                      <button
                        type="button"
                        class="route-btn"
                        title="Remove from the route"
                        onClick={() => write(ids.filter((_, j) => j !== i))}
                      >
                        ×
                      </button>
                    </li>
                  ))}
                </ol>
              )}
              <select
                class="field-select tiny"
                value=""
                onChange={(e) => {
                  const id = (e.target as HTMLSelectElement).value;
                  if (id) write([...ids, id]);
                  (e.target as HTMLSelectElement).value = "";
                }}
              >
                <option value="">
                  {saved.length ? "Add a place…" : "No places saved yet"}
                </option>
                {saved.map((p) => <option key={p.id} value={p.id}>{p.name}</option>)}
              </select>
            </div>
          );
        }

        return (
          <label key={arg.key} class="arg">
            <span>{arg.label}</span>
            <input
              class="field-input tiny"
              type="text"
              value={String(value)}
              onInput={(e) => onChange({ [arg.key]: (e.target as HTMLInputElement).value })}
            />
          </label>
        );
      })}
    </>
  );
}
