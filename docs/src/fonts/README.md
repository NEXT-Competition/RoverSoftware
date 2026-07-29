# Fonts

The two faces the base station itself uses, vendored so `docs/index.html`
renders identically offline — the handbook has the same "no CDN" constraint the
dashboard does, and a silently substituted fallback would misrepresent the
product it is documenting.

| File | Family | Licence |
|---|---|---|
| `archivo-latin-wght-normal.woff2` | Archivo Variable (wght 100–900) | SIL Open Font License 1.1 — © 2020 The Archivo Project Authors |
| `jetbrains-mono-latin-wght-normal.woff2` | JetBrains Mono Variable (wght 100–800) | SIL Open Font License 1.1 — © 2020 The JetBrains Mono Project Authors |

Both are copied from the `@fontsource-variable` packages already pinned in
`basestation-ui/package.json`; refresh them from there rather than downloading
new copies, so the docs and the app never drift onto different cuts.
