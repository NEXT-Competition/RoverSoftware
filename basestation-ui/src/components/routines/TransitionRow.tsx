// One transition: when this happens, go there.
//
// Transitions are evaluated top to bottom and the first match wins, so the
// order on the card is the order the robot checks them in. That is worth
// stating on screen rather than in a doc nobody reads.

import type { TransitionSpec } from "../../net/types.ts";
import { CONDITIONS, CONDITION_BY_KEY } from "../../routines/vocab.ts";
import { ArgFields } from "./ArgFields.tsx";

export function TransitionRow(
  { transition, states, index, onChange, onRemove }: {
    transition: TransitionSpec;
    states: string[];
    index: number;
    onChange: (next: TransitionSpec) => void;
    onRemove: () => void;
  },
) {
  const spec = CONDITION_BY_KEY[transition.when];
  const missing = !states.includes(transition.to);

  const pick = (when: string) => {
    // Keep only `to` and the hold time: the previous condition's arguments mean
    // nothing to the new one, and leaving them behind is how a document ends up
    // carrying fields the robot then reports as unexpected.
    const next: TransitionSpec = { when, to: transition.to };
    if (transition.for_seconds) next.for_seconds = transition.for_seconds;
    for (const arg of CONDITION_BY_KEY[when]?.args ?? []) {
      if (arg.fallback !== undefined) next[arg.key] = arg.fallback;
    }
    onChange(next);
  };

  return (
    <div class={`transition-row${missing ? " invalid" : ""}`}>
      <span class="transition-order" title="Checked in this order; first match wins">
        {index + 1}
      </span>

      <select
        class="field-select tiny"
        value={transition.when}
        onChange={(e) => pick((e.target as HTMLSelectElement).value)}
      >
        {CONDITIONS.map((c) => (
          <option key={c.key} value={c.key}>{c.label}</option>
        ))}
      </select>

      <ArgFields
        args={spec?.args ?? []}
        values={transition as Record<string, unknown>}
        states={states}
        onChange={(key, value) => onChange({ ...transition, [key]: value })}
      />

      <label class="arg">
        <span>held for (s)</span>
        <input
          class="field-input tiny"
          type="number"
          min={0}
          max={30}
          step={0.1}
          value={transition.for_seconds ?? 0}
          onInput={(e) => {
            const parsed = Number((e.target as HTMLInputElement).value);
            if (!Number.isNaN(parsed)) {
              onChange({ ...transition, for_seconds: parsed });
            }
          }}
          title="The condition must hold continuously for this long. One frame of 'aligned' is equally consistent with a false positive."
        />
      </label>

      {/* Arrow, target and delete travel together: wrapped apart, the arrow
          points at nothing and the row stops reading as a sentence. */}
      <span class="transition-target">
        <span class="transition-arrow" aria-hidden="true">→</span>
        <select
          class="field-select tiny"
          value={transition.to}
          onChange={(e) =>
            onChange({ ...transition, to: (e.target as HTMLSelectElement).value })}
        >
          {missing && <option value={transition.to}>{transition.to} (missing)</option>}
          {states.map((s) => <option key={s} value={s}>{s}</option>)}
        </select>
        <button type="button" class="btn ghost small danger" onClick={onRemove}>
          ×
        </button>
      </span>

      {spec?.help && <p class="doc-hint full">{spec.help}</p>}
    </div>
  );
}
