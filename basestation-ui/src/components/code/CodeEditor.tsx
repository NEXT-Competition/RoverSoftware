// The editor itself: a textarea with a highlighted layer behind it.
//
// The classic technique, and the right one here. A `<textarea>` keeps every
// piece of text editing the platform already does correctly — selection,
// undo, IME, autocorrect, and the on-screen keyboard a tablet in a pit puts up
// — while a `<pre>` underneath it, laid out identically and coloured, provides
// the only thing a textarea cannot do. The text on top is transparent; the
// caret is not.
//
// What the editor adds on top of that is the handful of things that make
// writing Python in a browser bearable rather than infuriating:
//
//   Tab / Shift+Tab   indent and dedent, over a selection as well as a line —
//                     without this, a language whose blocks ARE indentation is
//                     being edited with the one key that moves focus away.
//   Enter             carry the current indent, and add a level after a colon.
//   Backspace         delete a whole indent level when that is all there is
//                     to the left of the caret.
//   Cmd/Ctrl+Enter    run, without reaching for the mouse.
//
// The error marker comes from the ROBOT: it compiles every script when the
// document lands and answers with a line number, which is what puts a red bar
// on line 12 rather than a squiggle from a parser in here guessing.

import { useEffect, useLayoutEffect, useRef } from "preact/hooks";
import { API_NAMES } from "../../scripts/api.ts";
import { highlight } from "./highlight.ts";

const INDENT = "    ";

interface Props {
  code: string;
  onChange: (code: string) => void;
  onRun?: () => void;
  /** 1-based line the robot refused this script on, if it did. */
  errorLine?: number | null;
  readOnly?: boolean;
}

/** Where the line containing `at` starts. */
function lineStart(text: string, at: number): number {
  return text.lastIndexOf("\n", at - 1) + 1;
}

export function CodeEditor(
  { code, onChange, onRun, errorLine, readOnly }: Props,
) {
  const input = useRef<HTMLTextAreaElement>(null);
  const layer = useRef<HTMLPreElement>(null);
  const gutter = useRef<HTMLDivElement>(null);
  // Where to put the caret after a programmatic edit. Applied in a layout
  // effect rather than straight after setting `value`, because the value we
  // set is overwritten by the re-render that our own onChange triggers — so
  // restoring the selection before that lands puts it back in the wrong place.
  const caret = useRef<[number, number] | null>(null);

  useLayoutEffect(() => {
    const target = caret.current;
    if (target && input.current) {
      input.current.selectionStart = target[0];
      input.current.selectionEnd = target[1];
      caret.current = null;
    }
  }, [code]);

  // Keep the three layers aligned. The gutter and the highlight scroll with
  // the textarea rather than having their own scrollbars: three scrollbars for
  // one document is three chances to be out of step by a pixel.
  useEffect(() => {
    const el = input.current;
    if (!el) return;
    const sync = () => {
      if (layer.current) {
        layer.current.scrollTop = el.scrollTop;
        layer.current.scrollLeft = el.scrollLeft;
      }
      if (gutter.current) gutter.current.scrollTop = el.scrollTop;
    };
    el.addEventListener("scroll", sync);
    sync();
    return () => el.removeEventListener("scroll", sync);
  }, []);

  /** Replace a span of the text and say where the caret should end up. */
  function splice(from: number, to: number, insert: string,
                  select: [number, number]): void {
    caret.current = select;
    onChange(code.slice(0, from) + insert + code.slice(to));
  }

  function onKeyDown(e: KeyboardEvent) {
    const el = input.current;
    if (!el) return;
    const start = el.selectionStart;
    const end = el.selectionEnd;

    if ((e.metaKey || e.ctrlKey) && e.key === "Enter") {
      e.preventDefault();
      onRun?.();
      return;
    }
    if (readOnly) return;

    if (e.key === "Tab") {
      e.preventDefault();
      const from = lineStart(code, start);
      const multi = code.slice(start, end).includes("\n");

      if (!e.shiftKey && !multi) {
        // A plain tab inside a line indents to the next stop rather than
        // inserting a fixed four, so it lines up with the level above it.
        const width = INDENT.length - ((start - from) % INDENT.length);
        const pad = " ".repeat(width);
        splice(start, end, pad, [start + width, start + width]);
        return;
      }

      // Over a selection (or on shift), work line by line. Ending the
      // selection exactly at a line start would otherwise pull in the line
      // below, which is not the one anybody thinks they selected.
      const to = end > start && code[end - 1] === "\n" ? end - 1 : end;
      const block = code.slice(from, Math.max(from, lineEnd(code, to)));
      const lines = block.split("\n");
      let first = 0;
      const shifted = lines.map((line, index) => {
        if (e.shiftKey) {
          const cut = line.length - line.replace(/^ {1,4}/, "").length;
          if (index === 0) first = cut;
          return line.slice(cut);
        }
        if (index === 0) first = INDENT.length;
        return line.trim() ? INDENT + line : line;
      });
      const next = shifted.join("\n");
      splice(from, from + block.length, next, [
        Math.max(from, start + (e.shiftKey ? -first : first)),
        from + next.length,
      ]);
      return;
    }

    if (e.key === "Enter" && start === end) {
      e.preventDefault();
      const from = lineStart(code, start);
      const line = code.slice(from, start);
      const indent = (/^\s*/.exec(line)?.[0] ?? "");
      // A colon opens a block, so the next line belongs one level in. Getting
      // this wrong is the single most tiring thing about typing Python into a
      // plain textarea.
      const deeper = /:\s*$/.test(line) ? INDENT : "";
      const insert = "\n" + indent + deeper;
      splice(start, end, insert,
        [start + insert.length, start + insert.length]);
      return;
    }

    if (e.key === "Backspace" && start === end) {
      const from = lineStart(code, start);
      const before = code.slice(from, start);
      if (before.length >= INDENT.length && /^\s+$/.test(before)) {
        // Nothing but indentation to the left: eat a whole level, so getting
        // back out of a block is one key rather than four.
        e.preventDefault();
        const width = ((before.length - 1) % INDENT.length) + 1;
        splice(start - width, end, "", [start - width, start - width]);
      }
    }
  }

  const lines = code.split("\n");

  return (
    <div class="code-editor">
      <div class="code-gutter" ref={gutter} aria-hidden="true">
        {lines.map((_, index) => (
          <div
            key={index}
            class={`code-lineno${errorLine === index + 1 ? " bad" : ""}`}
          >
            {index + 1}
          </div>
        ))}
      </div>
      <div class="code-pane">
        <pre
          class="code-highlight"
          ref={layer}
          aria-hidden="true"
          dangerouslySetInnerHTML={{ __html: highlight(code, API_NAMES) }}
        />
        <textarea
          class="code-input"
          ref={input}
          value={code}
          readOnly={readOnly}
          spellcheck={false}
          autocapitalize="off"
          autocomplete="off"
          autocorrect="off"
          wrap="off"
          aria-label="Python source"
          onInput={(e) => onChange((e.target as HTMLTextAreaElement).value)}
          onKeyDown={onKeyDown}
        />
      </div>
    </div>
  );
}

/** Where the line containing `at` ends (exclusive of its newline). */
function lineEnd(text: string, at: number): number {
  const index = text.indexOf("\n", at);
  return index === -1 ? text.length : index;
}
