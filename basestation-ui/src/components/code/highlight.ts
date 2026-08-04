// Python syntax highlighting, in about eighty lines.
//
// Hand-written rather than pulled in, and that is a deliberate trade rather
// than a preference. The console runs offline in a kiosk and is bundled whole;
// a real editor component (CodeMirror, Monaco) is megabytes of it, needs a
// worker, and brings its own scrolling, its own theming and its own touch
// behaviour to fight with. What this file has to do is much smaller: colour
// five kinds of token in a language whose lexical structure fits in one regex.
//
// The one thing it is NOT is a parser. It cannot tell you that a line is wrong
// — the ROBOT does that, by compiling the script when it lands and answering
// with a line number (robot/script/schema.py). A browser-side Python parser
// would be a second, subtly different opinion about what compiles, and the two
// would disagree exactly when it mattered.

const KEYWORDS = new Set([
  "and", "as", "assert", "async", "await", "break", "class", "continue",
  "def", "del", "elif", "else", "except", "finally", "for", "from", "global",
  "if", "import", "in", "is", "lambda", "nonlocal", "not", "or", "pass",
  "raise", "return", "try", "while", "with", "yield",
]);

const CONSTANTS = new Set(["True", "False", "None", "self"]);

const BUILTINS = new Set([
  "abs", "all", "any", "bool", "dict", "enumerate", "filter", "float", "int",
  "len", "list", "map", "max", "min", "print", "range", "round", "set",
  "sorted", "str", "sum", "tuple", "zip", "isinstance", "reversed", "type",
]);

// One pass, longest-match-first. Triple-quoted strings before single ones, and
// strings before comments, so a `#` inside a string stays a string.
const TOKEN =
  /("""[\s\S]*?"""|'''[\s\S]*?'''|"(?:\\.|[^"\\\n])*"?|'(?:\\.|[^'\\\n])*'?)|(#[^\n]*)|(\b\d+\.?\d*(?:[eE][-+]?\d+)?\b)|(\b[A-Za-z_]\w*\b)/g;

function escape(text: string): string {
  return text.replace(/[&<>]/g, (c) => (c === "&" ? "&amp;" : c === "<" ? "&lt;" : "&gt;"));
}

/**
 * Highlighted HTML for one script.
 *
 * `apiNames` is the set the reference panel lists (scripts/api.ts), so a call
 * shown in the panel is a call that lights up in the editor — which is the
 * cheapest possible answer to "did I spell that right".
 *
 * The output ends with a newline on purpose: a `<pre>` whose last line is empty
 * collapses, and the highlighted layer would then sit one line short of the
 * textarea above it and everything would drift as you typed at the bottom.
 */
export function highlight(code: string, apiNames: ReadonlySet<string>): string {
  let out = "";
  let last = 0;
  TOKEN.lastIndex = 0;
  for (let m = TOKEN.exec(code); m; m = TOKEN.exec(code)) {
    out += escape(code.slice(last, m.index));
    last = m.index + m[0].length;
    const [, string_, comment, number, word] = m;
    if (string_ !== undefined) {
      out += `<span class="tok-str">${escape(string_)}</span>`;
    } else if (comment !== undefined) {
      out += `<span class="tok-com">${escape(comment)}</span>`;
    } else if (number !== undefined) {
      out += `<span class="tok-num">${number}</span>`;
    } else if (word !== undefined) {
      const cls = KEYWORDS.has(word)
        ? "tok-kw"
        : CONSTANTS.has(word)
        ? "tok-const"
        : word === "rover"
        ? "tok-rover"
        : apiNames.has(word)
        ? "tok-api"
        : BUILTINS.has(word)
        ? "tok-builtin"
        : "";
      out += cls ? `<span class="${cls}">${word}</span>` : word;
    }
  }
  out += escape(code.slice(last));
  return out + "\n";
}
