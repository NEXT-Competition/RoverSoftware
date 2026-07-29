// Typed WebSocket client + reactive store.
//
// Connects to the same-origin /ws (proxied to the Python bridge), decodes the
// fleet snapshots into @preact/signals, and exposes send() for outbound
// actions. Auto-reconnects on drop, mirroring the old app.js (1s backoff).

import { batch, computed, signal } from "@preact/signals";
import type {
  Action,
  CommandMessage,
  ConnState,
  ControllerStatus,
  FleetMessage,
  GamepadMessage,
  GamepadState,
  Place,
  PlacesResult,
  Robot,
  RobotConfigEntry,
  RobotDocuments,
  SettingsMessage,
  SettingValue,
} from "./types.ts";
import { applyCommandFrame } from "../state/command.ts";

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

// Locally-owned selection so a tap feels instant; seeded from the server the
// first time it tells us who's selected (matches the old app.js behaviour).
export const selected = signal<string | null>(null);

// ---- the cold channel (see basestation/app.py::settings_frame) ----
/** Base-station settings and gamepad mapping, by flat dotted path. */
export const baseSettings = signal<Record<string, SettingValue>>({});
/** Outcome of the last base-station settings change (clamped, refused, saved). */
export const settingsResult = signal<SettingsMessage["settings_result"]>(null);
/** Each robot's tunable config, as the bridge has it cached. */
export const robotConfigs = signal<Record<string, RobotConfigEntry>>({});
/** Each robot's hardware layout, routines and field descriptors. Same cold
 *  channel as the configs, and for the same reason: kilobytes that change when
 *  someone presses Save, not thirty times a second. */
export const robotDocuments = signal<Record<string, RobotDocuments>>({});
/** Named field positions, fleet-wide. The bridge owns the list and persists it
 *  (basestation/places.py); this is the mirror every map and picker reads. */
export const places = signal<Place[]>([]);
/** Which entries the bridge refused on the last save, if any. */
export const placesResult = signal<PlacesResult | null>(null);
/** Raw gamepad sample. Only streams while a client is watching. */
export const gamepad = signal<GamepadState | null>(null);
/** True once the first settings frame has landed (before that, a blank form
 *  would be indistinguishable from "every value is zero"). */
export const settingsReady = signal(false);

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
    let msg: FleetMessage | SettingsMessage | GamepadMessage | CommandMessage;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (msg.type === "command") {
      applyCommandFrame(msg);
      return;
    }
    if (msg.type === "settings") {
      batch(() => {
        baseSettings.value = msg.settings ?? {};
        settingsResult.value = msg.settings_result ?? null;
        robotConfigs.value = msg.configs ?? {};
        robotDocuments.value = msg.documents ?? {};
        places.value = msg.places ?? [];
        placesResult.value = msg.places_result ?? null;
        if (msg.gamepad) gamepad.value = msg.gamepad;
        settingsReady.value = true;
      });
      return;
    }
    if (msg.type === "gamepad") {
      gamepad.value = msg.gamepad;
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

/**
 * Send a chunk of microphone PCM (net/audio.ts) up the same socket.
 *
 * Binary rather than a second connection, so audio and the `mic` on/off actions
 * that bracket it stay ordered — a separate socket could deliver the release
 * before the last frame of speech.
 *
 * Dropped rather than buffered when the socket is busy: `bufferedAmount` grows
 * when the link can't keep up, and queued *old* audio is worse than a gap. The
 * threshold is roughly a second of PCM at 16 kHz.
 */
export function sendAudio(pcm: ArrayBuffer): void {
  if (ws && ws.readyState === WebSocket.OPEN && ws.bufferedAmount < 32000) {
    ws.send(pcm);
  }
}

/** Select a robot: instant locally, and tell the bridge. */
export function selectRobot(id: string): void {
  selected.value = id;
  send({ action: "select", robot_id: id });
}
