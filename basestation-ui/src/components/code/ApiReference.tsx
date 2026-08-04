// The API panel: every sensor and every actuator, beside the editor.
//
// This is not documentation that happens to be nearby — it is the reason the
// coding interface is usable at all. A rover's API is not guessable, the base
// station runs offline in a pit with no docs open, and the alternative to a
// panel is memorising which of `rover.vision.distance` and
// `rover.distance_ahead()` answers which question.
//
// Clicking an entry inserts it at the caret, so the panel is also the fastest
// way to type the call rather than only a way to look it up.

import { useState } from "preact/hooks";
import { API } from "../../scripts/api.ts";

interface Props {
  /** Insert a signature's call form at the caret. Absent = read-only panel. */
  onInsert?: (snippet: string) => void;
}

/**
 * The bit of a signature worth pasting.
 *
 * `rover.mech(name).power(value, actuator=None)` should land as
 * `rover.mech(name).power(value)` — the defaulted arguments are there to
 * document what the call accepts, not to be typed out every time. The return
 * annotation goes too: `-> bool` describes the call, it is not part of it.
 */
function snippetFor(sig: string): string {
  const call = sig.split(" -> ")[0].split("  /  ")[0].trim();
  return call.replace(/\(([^)]*)\)/g, (_, args: string) => {
    const kept = args
      .split(",")
      .map((a) => a.trim())
      .filter((a) => a && !a.includes("="));
    return `(${kept.join(", ")})`;
  });
}

export function ApiReference({ onInsert }: Props) {
  // Driving open, the rest closed. Six expanded groups is a wall of text that
  // has to be scrolled past to reach anything; one open says what the shape of
  // the panel is without being that wall.
  const [open, setOpen] = useState<Record<string, boolean>>({ drive: true });
  const [filter, setFilter] = useState("");
  const needle = filter.trim().toLowerCase();

  const groups = API.map((group) => ({
    ...group,
    entries: needle
      ? group.entries.filter((e) =>
        e.sig.toLowerCase().includes(needle) || e.help.toLowerCase().includes(needle)
      )
      : group.entries,
  })).filter((group) => group.entries.length > 0);

  return (
    <aside class="api-panel">
      <div class="api-head">
        <span class="eyebrow">The rover API</span>
        <input
          class="field-input tiny"
          type="search"
          placeholder="find a call…"
          value={filter}
          onInput={(e) => setFilter((e.target as HTMLInputElement).value)}
        />
      </div>

      {groups.length === 0 && (
        <p class="hint pad">
          Nothing matches “{filter}”. Everything about the robot hangs off{" "}
          <code>rover</code> — try “mech”, “vision” or “wait”.
        </p>
      )}

      {groups.map((group) => {
        // A search opens everything it matched: hiding results inside collapsed
        // groups is the one behaviour that makes a filter feel broken.
        const expanded = needle ? true : !!open[group.key];
        return (
          <section key={group.key} class={`api-group${expanded ? " open" : ""}`}>
            <button
              type="button"
              class="api-group-head"
              aria-expanded={expanded}
              onClick={() => setOpen((o) => ({ ...o, [group.key]: !expanded }))}
            >
              <span class="api-chevron">{expanded ? "▾" : "▸"}</span>
              {group.title}
              <span class="api-count">{group.entries.length}</span>
            </button>

            {expanded && (
              <div class="api-group-body">
                {group.note && !needle && <p class="hint api-note">{group.note}</p>}
                {group.entries.map((entry) => (
                  <div key={entry.sig} class="api-entry">
                    <button
                      type="button"
                      class="api-sig"
                      disabled={!onInsert}
                      title={onInsert ? "Insert at the caret" : undefined}
                      onClick={() => onInsert?.(snippetFor(entry.sig))}
                    >
                      <code>{entry.sig}</code>
                      {entry.blocks && (
                        <span
                          class="api-blocks"
                          title="Waits for the robot to actually do it before the next line runs"
                        >
                          waits
                        </span>
                      )}
                    </button>
                    <p class="api-help">{entry.help}</p>
                  </div>
                ))}
              </div>
            )}
          </section>
        );
      })}
    </aside>
  );
}
