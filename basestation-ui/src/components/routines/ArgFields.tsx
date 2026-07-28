// The arguments a chosen condition or action needs, rendered from its spec.
//
// One component rather than a case per verb: the vocabulary already says what
// each one takes (routines/vocab.ts), so adding a condition to the Python side
// and its entry there is enough to make it editable here.

import type { ArgSpec } from "../../routines/vocab.ts";
import { availableMechanisms } from "../../state/routines.ts";

export function ArgFields(
  { args, values, states, onChange }: {
    args: ArgSpec[];
    values: Record<string, unknown>;
    states: string[];
    onChange: (key: string, value: unknown) => void;
  },
) {
  const mechanisms = availableMechanisms.value;

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
                  if (!Number.isNaN(parsed)) onChange(arg.key, parsed);
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
                onChange={(e) => onChange(arg.key, (e.target as HTMLSelectElement).value)}
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
                onChange={(e) => onChange(arg.key, (e.target as HTMLSelectElement).value)}
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
                onChange={(e) => onChange(arg.key, (e.target as HTMLSelectElement).value)}
              >
                {states.map((s) => <option key={s} value={s}>{s}</option>)}
              </select>
            </label>
          );
        }

        return (
          <label key={arg.key} class="arg">
            <span>{arg.label}</span>
            <input
              class="field-input tiny"
              type="text"
              value={String(value)}
              onInput={(e) => onChange(arg.key, (e.target as HTMLInputElement).value)}
            />
          </label>
        );
      })}
    </>
  );
}
