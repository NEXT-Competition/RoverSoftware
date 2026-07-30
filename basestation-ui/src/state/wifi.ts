// Putting a rover on a network from the dashboard.
//
// The password is held in a signal for exactly as long as it takes to type it
// and press Connect, then dropped. It is never persisted, never put in a
// document, and never read back from the robot — the robot strips it from its
// own reply (robot/comms/wifi.py). If you reload this page mid-connect, the
// password is gone, which is the correct amount of memory for it to have.

import { computed, effect, signal } from "@preact/signals";
import type { WifiNetwork, WifiState } from "../net/types.ts";
import { robotWifi, send } from "../net/ws.ts";
import { targetRobot } from "./settings.ts";

/** What the rover last told us. */
export const wifi = computed<WifiState | null>(() => {
  const rid = targetRobot.value;
  return rid ? robotWifi.value[rid] ?? null : null;
});

/**
 * The last list of networks the rover reported, kept across later answers.
 *
 * The rover's answers replace one another — a failed join is not a partial scan
 * — and that is right on the wire but wrong on screen: pressing Connect and
 * getting the password wrong made the list you picked from vanish, so the retry
 * meant scanning again. The list is the one part of the conversation that stays
 * true until the next scan.
 */
export const networks = signal<WifiNetwork[]>([]);

/** The network picked from the scan list, or typed in by hand. */
export const ssid = signal("");

/** Held only until Connect is pressed. See the note at the top of this file. */
export const psk = signal("");

/** A network that does not broadcast its name has to be typed AND declared —
 *  NetworkManager needs telling, or it will not find it. */
export const hidden = signal(false);

/** Two-letter regulatory domain, blank to leave it alone. */
export const country = signal("");

/** Which request is in flight, so the buttons can say what they are doing.
 *  The robot answers every request with a `wifi` frame, so an arriving frame is
 *  what clears this — not a timer. */
export const pending = signal<"scan" | "connect" | "forget" | "status" | null>(null);

/**
 * Stop showing "working" when the answer lands.
 *
 * Keyed on the revision, not on the arrival of any settings frame: those are
 * pushed whenever anything cold changes, so clearing on one would stop the
 * spinner before the rover had said a word. The rover answers every request
 * with a `wifi` frame, so a revision that moved IS the answer.
 *
 * No timeout. A rover that never answers leaves the button saying it is still
 * working, which is true — the request went out over a radio and nothing has
 * come back. Pretending otherwise after five seconds would just be a guess
 * rendered as a fact.
 */
let lastRev = -1;
let lastRobot: string | null = null;
effect(() => {
  const robot = targetRobot.value;
  const state = wifi.value;
  if (robot !== lastRobot) {
    // A different rover is a different set of answers. Keeping the list would
    // offer rover2 the networks rover1 can see — which may genuinely differ,
    // since they are metres apart with metal between them.
    lastRobot = robot;
    lastRev = -1;
    networks.value = [];
    pending.value = null;
  }
  const rev = state?.rev ?? -1;
  if (rev === lastRev) return;
  lastRev = rev;
  pending.value = null;
  // Only a scan carries a list; every other answer leaves the last one standing.
  if (state?.networks) networks.value = state.networks;
});

export function refreshWifi(): void {
  const rid = targetRobot.value;
  if (!rid) return;
  pending.value = "status";
  send({ action: "get_wifi", robot_id: rid });
}

export function scanWifi(): void {
  const rid = targetRobot.value;
  if (!rid) return;
  pending.value = "scan";
  send({ action: "scan_wifi", robot_id: rid });
}

/**
 * Join the named network.
 *
 * The password is cleared the moment it is handed over. There is nothing to be
 * gained by keeping it — the rover stores its own profile and rejoins by itself
 * — and a field that still holds a password ten minutes later is one somebody
 * else can read over your shoulder at a competition.
 */
export function connectWifi(): void {
  const rid = targetRobot.value;
  const name = ssid.value.trim();
  if (!rid || !name) return;
  pending.value = "connect";
  send({
    action: "set_wifi",
    robot_id: rid,
    ssid: name,
    psk: psk.value,
    hidden: hidden.value,
    ...(country.value.trim() ? { country: country.value.trim().toUpperCase() } : {}),
  });
  psk.value = "";
}

/** Drop a stored profile, so the rover stops rejoining last venue's network in
 *  preference to the one in front of it. */
export function forgetWifi(name: string): void {
  const rid = targetRobot.value;
  if (!rid || !name) return;
  pending.value = "forget";
  send({ action: "forget_wifi", robot_id: rid, ssid: name });
}
