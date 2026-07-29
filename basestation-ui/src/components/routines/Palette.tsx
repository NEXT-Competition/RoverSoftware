// "Add a step": the searchable menu of everything this robot can be asked to do.
//
// The canvas used to offer one button — "+ State" — which drops a legal box
// that does nothing. That box is where a first-time competitor stalls: nothing
// on screen says what a state *can* be, so the only way to find out is to add
// one and go hunting through the inspector's dropdowns.
//
// So the choice is inverted. You pick the capability first, by name or by
// searching for it, and the state is built around it already configured. The
// list is grouped rather than alphabetical, because the grouping is the only
// documentation of the robot's vocabulary a competitor ever reads.
//
// Every entry here is generated from vocab.ts, which mirrors the robot's own
// BUILDERS tables — so the palette cannot offer something the robot would
// refuse, and it cannot hide something the robot supports.

import { useMemo, useRef, useState } from "preact/hooks";
import { ACTIONS, DRIVE_MODES, type VerbGroup, type VerbSpec } from "../../routines/vocab.ts";
import { addState, addStateFrom } from "../../state/routines.ts";

/** Driving is chosen per state, not added as an action — but from a newcomer's
 *  point of view "make the robot follow a route" is the same kind of choice as
 *  "fire the launcher", so the palette lists both and builds the right shape. */
const DRIVE_VERBS: VerbSpec[] = DRIVE_MODES
  .filter((m) => m.value !== "stop")
  .map((m) => ({
    key: m.value,
    group: "navigation" as VerbGroup,
    label: m.label,
    // No per-item help: it would be the same sentence six times, which the
    // group blurb already says once. Repetition here is length, not clarity.
    args: [],
  }));

const GROUP_ORDER: { key: VerbGroup; title: string; blurb: string }[] = [
  { key: "navigation", title: "Driving", blurb: "How the robot moves while it is in this step." },
  { key: "mechanism", title: "Mechanisms", blurb: "Intakes, arms, launchers — anything the layout declares." },
  { key: "logic", title: "Counting", blurb: "Repeat a step a fixed number of times." },
  { key: "operator", title: "Notes", blurb: "Things for the person watching, not the robot." },
  { key: "timing", title: "Timing", blurb: "" },
  { key: "vision", title: "Vision", blurb: "" },
];

interface Entry {
  verb: VerbSpec;
  kind: "action" | "drive";
}

export function Palette({ at }: { at?: () => { x: number; y: number } }) {
  const [open, setOpen] = useState(false);
  const [query, setQuery] = useState("");
  const input = useRef<HTMLInputElement>(null);

  const entries: Entry[] = useMemo(() => [
    ...DRIVE_VERBS.map((verb) => ({ verb, kind: "drive" as const })),
    ...ACTIONS.map((verb) => ({ verb, kind: "action" as const })),
  ], []);

  // Match the words a person would actually type — the label they can see, not
  // the wire key they can't. The key is searched too so an operator who has
  // read the docs can type `set_route` and find it.
  const q = query.trim().toLowerCase();
  const hits = q
    ? entries.filter((e) =>
      e.verb.label.toLowerCase().includes(q) ||
      e.verb.key.toLowerCase().includes(q) ||
      (e.verb.help ?? "").toLowerCase().includes(q)
    )
    : entries;

  const grouped = GROUP_ORDER
    .map((g) => ({ ...g, items: hits.filter((e) => e.verb.group === g.key) }))
    .filter((g) => g.items.length > 0);

  function choose(entry: Entry) {
    addStateFrom(entry.verb, entry.kind, at?.());
    setOpen(false);
    setQuery("");
  }

  if (!open) {
    return (
      <button
        type="button"
        class="btn small primary"
        onClick={() => {
          setOpen(true);
          // Focus after paint so the field is live for a keyboard user without
          // stealing the on-screen keyboard from a kiosk operator who tapped.
          setTimeout(() => input.current?.focus(), 0);
        }}
      >
        + Add step
      </button>
    );
  }

  return (
    <div class="palette" role="dialog" aria-label="Add a step">
      <div class="palette-head">
        <input
          ref={input}
          class="palette-search"
          type="search"
          placeholder="Search what the robot can do…"
          value={query}
          onInput={(e) => setQuery((e.target as HTMLInputElement).value)}
        />
        <button
          type="button"
          class="btn small ghost"
          onClick={() => {
            setOpen(false);
            setQuery("");
          }}
        >
          Close
        </button>
      </div>

      <div class="palette-body">
        {grouped.map((group) => (
          <div key={group.key} class="palette-group">
            <div class="palette-group-head">
              <span class="eyebrow">{group.title}</span>
              {group.blurb && !q && <span class="palette-blurb">{group.blurb}</span>}
            </div>
            {group.items.map((entry) => (
              <button
                key={`${entry.kind}:${entry.verb.key}`}
                type="button"
                class="palette-item"
                onClick={() => choose(entry)}
              >
                <span class={`palette-icon g-${entry.verb.group}`} aria-hidden="true" />
                <span class="palette-text">
                  <span class="palette-label">{entry.verb.label}</span>
                  {entry.verb.help && <span class="palette-help">{entry.verb.help}</span>}
                </span>
              </button>
            ))}
          </div>
        ))}

        {grouped.length === 0 && (
          <p class="palette-empty">
            Nothing matches “{query}”. The list only offers what this robot
            actually supports, so if it isn't here the robot would refuse it.
          </p>
        )}

        {/* The old behaviour, kept and demoted: sometimes you do want an empty
            box to wire up first and fill in later. */}
        <button
          type="button"
          class="palette-item subtle"
          onClick={() => {
            addState(at?.());
            setOpen(false);
            setQuery("");
          }}
        >
          <span class="palette-icon g-blank" aria-hidden="true" />
          <span class="palette-text">
            <span class="palette-label">Empty step</span>
            <span class="palette-help">Start blank and configure it in the inspector.</span>
          </span>
        </button>
      </div>
    </div>
  );
}
