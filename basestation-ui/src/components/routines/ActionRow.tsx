// One action in one of a state's three slots.

import type { ActionSpec } from "../../net/types.ts";
import { ACTIONS, ACTION_BY_KEY } from "../../routines/vocab.ts";
import { ArgFields } from "./ArgFields.tsx";

export function ActionRow(
  { action, states, onChange, onRemove }: {
    action: ActionSpec;
    states: string[];
    onChange: (next: ActionSpec) => void;
    onRemove: () => void;
  },
) {
  const spec = ACTION_BY_KEY[action.do];

  const pick = (verb: string) => {
    const next: ActionSpec = { do: verb };
    for (const arg of ACTION_BY_KEY[verb]?.args ?? []) {
      if (arg.fallback !== undefined) next[arg.key] = arg.fallback;
    }
    onChange(next);
  };

  return (
    <div class="action-row">
      <select
        class="field-select tiny"
        value={action.do}
        onChange={(e) => pick((e.target as HTMLSelectElement).value)}
      >
        {ACTIONS.map((a) => <option key={a.key} value={a.key}>{a.label}</option>)}
      </select>

      <ArgFields
        args={spec?.args ?? []}
        values={action as Record<string, unknown>}
        states={states}
        onChange={(key, value) => onChange({ ...action, [key]: value })}
      />

      <button type="button" class="btn ghost small danger" onClick={onRemove}>
        ×
      </button>

      {spec?.help && <p class="doc-hint full">{spec.help}</p>}
    </div>
  );
}
