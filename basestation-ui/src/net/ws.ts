// Typed WebSocket client + reactive store.
//
// Connects to the same-origin /ws (proxied to the Python bridge), decodes the
// fleet snapshots into @preact/signals, and exposes send() for outbound
// actions. Auto-reconnects on drop, mirroring the old app.js (1s backoff).

import { batch, computed, signal } from "@preact/signals";
import type {
  Action,
  ConnState,
  ControllerStatus,
  FleetMessage,
  LatLon,
  Robot,
  Site,
} from "./types.ts";

export const conn = signal<ConnState>("connecting");
export const robots = signal<Robot[]>([]);
export const controller = signal<ControllerStatus>({ connected: false, name: null });
export const tilesUrl = signal<string | null>(null);
export const tilesMaxZoom = signal<number | null>(null);
/** Credit line for the basemap (imagery licences require it on screen). */
export const tilesAttribution = signal<string | null>(null);
/** robot_ids currently streaming a live FPV feed. */
export const videoRobots = signal<string[]>([]);
/**
 * Max drive-command rate the bridge wants us to send at, in Hz. Comes from the
 * server's --drive-hz so the touch joystick and the server-side gamepad share
 * one radio budget; see net/drive.ts. Null until the first snapshot arrives.
 */
export const driveHz = signal<number | null>(null);
/** Active site's fence outline, or empty when it has none (basestation/sites.py). */
export const boundary = signal<LatLon[]>([]);
/** Every site the dashboard can switch to, keyed by id. */
export const sites = signal<Record<string, Site>>({});
/** Which key in `sites` the map + simulator are currently locked to. */
export const activeSiteId = signal<string | null>(null);

// Locally-owned selection so a tap feels instant; seeded from the server the
// first time it tells us who's selected (matches the old app.js behaviour).
export const selected = signal<string | null>(null);

/** The currently-selected robot object, or null. */
export const selectedRobot = computed<Robot | null>(() => {
  const id = selected.value;
  if (!id) return null;
  return robots.value.find((r) => r.robot_id === id) ?? null;
});

let ws: WebSocket | null = null;
let reconnectTimer: ReturnType<typeof setTimeout> | undefined;

function wsUrl(): string {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws`;
}

export function connect(): void {
  clearTimeout(reconnectTimer);
  ws = new WebSocket(wsUrl());

  ws.onopen = () => {
    conn.value = "live";
  };

  ws.onclose = () => {
    conn.value = "reconnecting";
    reconnectTimer = setTimeout(connect, 1000);
  };

  ws.onerror = () => {
    // onclose fires next and handles the retry.
    try {
      ws?.close();
    } catch { /* ignore */ }
  };

  ws.onmessage = (ev) => {
    let msg: FleetMessage;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (msg.type !== "fleet") return;
    batch(() => {
      robots.value = msg.robots ?? [];
      controller.value = msg.controller ?? { connected: false, name: null };
      tilesUrl.value = msg.tiles ?? null;
      tilesMaxZoom.value = msg.tiles_maxzoom ?? null;
      tilesAttribution.value = msg.tiles_attribution ?? null;
      videoRobots.value = msg.video ?? [];
      driveHz.value = typeof msg.drive_hz === "number" && msg.drive_hz > 0 ? msg.drive_hz : null;
      boundary.value = msg.boundary ?? [];
      if (msg.sites) sites.value = msg.sites;
      if (msg.active_site !== undefined) activeSiteId.value = msg.active_site;
      if (selected.value == null) selected.value = msg.selected;
    });
  };
}

/** Send an action to the bridge (no-op if the socket isn't open). */
export function send(action: Action): void {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(action));
  }
}

/** Select a robot: instant locally, and tell the bridge. */
export function selectRobot(id: string): void {
  selected.value = id;
  send({ action: "select", robot_id: id });
}

/** Move the map + (if running) the simulated fleet to a different site. */
export function selectSite(id: string): void {
  send({ action: "select_site", site_id: id });
}
