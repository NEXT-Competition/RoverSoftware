// What the script said: its printed output, and the values it asked to be
// watched.
//
// Two panes rather than one, because they answer different questions. `print`
// is a log — a history, read from the bottom — while `rover.watch` is a set of
// live numbers, and printing those in a 50 Hz loop produces a waterfall nobody
// can read. Giving the second one a table is what makes "is the offset actually
// converging" answerable at a glance.
//
// This is the ONLY thing on the Code tab that needs WiFi. Output rides the
// bulk link with the config snapshots and the documents; the radio carries
// driving, telemetry and the e-stop, and a script's prints are not any of
// those. So the panel says so rather than sitting mysteriously blank.

import { useEffect, useRef } from "preact/hooks";
import type { ScriptConsole, ScriptStatus } from "../../net/types.ts";

interface Props {
  console: ScriptConsole | null;
  status: ScriptStatus | null;
  onClear: () => void;
}

function value(v: unknown): string {
  if (v === null || v === undefined) return "—";
  if (typeof v === "number") {
    return Number.isInteger(v) ? String(v) : v.toFixed(3);
  }
  return String(v);
}

export function Console({ console: output, status, onClear }: Props) {
  const log = useRef<HTMLDivElement>(null);
  const pinned = useRef(true);
  const lines = output?.lines ?? [];
  const watch = Object.entries(output?.watch ?? {});

  // Follow the tail, but only while the operator is already at it. Yanking the
  // view back down while somebody is reading a traceback further up is the
  // thing every log pane gets wrong.
  useEffect(() => {
    const el = log.current;
    if (el && pinned.current) el.scrollTop = el.scrollHeight;
  }, [output?.rev]);

  function onScroll() {
    const el = log.current;
    if (el) {
      pinned.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
    }
  }

  return (
    <section class="script-console">
      <div class="console-head">
        <span class="eyebrow">Console</span>
        {status?.err && <span class="console-err">{status.err}</span>}
        <span class="console-count">
          {lines.length ? `${lines.length} line${lines.length === 1 ? "" : "s"}` : ""}
        </span>
        <button
          type="button"
          class="btn ghost small"
          disabled={!lines.length && !watch.length}
          onClick={onClear}
        >
          Clear
        </button>
      </div>

      {watch.length > 0 && (
        <div class="console-watch">
          {watch.map(([name, v]) => (
            <div key={name} class="watch-cell">
              <span class="watch-name">{name}</span>
              <span class="watch-value tape">{value(v)}</span>
            </div>
          ))}
        </div>
      )}

      <div class="console-log" ref={log} onScroll={onScroll}>
        {lines.length === 0
          ? (
            <p class="hint">
              Nothing printed yet. <code>print(...)</code> and{" "}
              <code>rover.log(...)</code> land here, and{" "}
              <code>rover.watch(name, value)</code> puts a live number in the
              row above.
              <br />
              Output travels over WiFi, not the radio — a rover out of WiFi
              range still runs its script, it just cannot tell you what it is
              saying.
            </p>
          )
          : lines.map((line, index) => (
            <div
              key={index}
              class={`console-line${
                /^(line \d+:|[A-Za-z]*Error|Traceback|\[script\])/.test(line)
                  ? " bad"
                  : ""
              }`}
            >
              {line || " "}
            </div>
          ))}
      </div>
    </section>
  );
}
