import { conn } from "../net/ws.ts";

const LABEL = {
  connecting: "connecting…",
  live: "live",
  reconnecting: "reconnecting…",
} as const;

export function ConnectionPill() {
  const state = conn.value;
  const cls = state === "live" ? "ok" : state === "reconnecting" ? "bad" : "warn";
  return (
    <span class={`pill ${cls}`}>
      <span class="led" />
      {LABEL[state]}
    </span>
  );
}
