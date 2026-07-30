// Network tab: put this rover on the WiFi in front of you.
//
// The job this does is a specific, recurring twenty minutes: you arrive at a
// venue, the rovers are on last week's network, and everything that rides WiFi —
// configuration, layouts, routines, the camera feed — is unavailable until each
// Pi is told about the new one. The alternative to this page is an HDMI cable
// and a keyboard, per rover.
//
// It works because the radio does not need a network. Driving, telemetry and the
// e-stop never left it, so there is always a way to talk to a rover that is on no
// WiFi at all — which is exactly the rover that needs this.
//
// Two things the page has to be honest about, both of them stated on screen
// rather than buried here:
//
//   * Joining a network is not instant and can fail. So every button reports,
//     and a failure shows NetworkManager's own words ("Secrets were required,
//     but not provided." is a wrong password) rather than a paraphrase.
//   * A password sent to a rover that is not yet on WiFi crosses the radio, and
//     the XBee ships unencrypted. That is a real exposure and the operator is
//     the only one who can judge it, so it is said plainly at the point of
//     typing — not hidden in a doc nobody reads at a competition.

import { conn, robots } from "../../net/ws.ts";
import { configTarget, targetRobot } from "../../state/settings.ts";
import {
  connectWifi,
  country,
  forgetWifi,
  hidden,
  networks as scanned,
  pending,
  psk,
  refreshWifi,
  scanWifi,
  ssid,
  wifi,
} from "../../state/wifi.ts";
import type { WifiNetwork } from "../../net/types.ts";

/** Signal strength as four bars. A percentage is a number you have to interpret;
 *  bars are the thing every phone has already taught everybody to read. */
function Bars({ signal }: { signal: number }) {
  const lit = signal >= 75 ? 4 : signal >= 50 ? 3 : signal >= 25 ? 2 : 1;
  return (
    <span class="wifi-bars" title={`${signal}%`} aria-label={`signal ${signal} percent`}>
      {[1, 2, 3, 4].map((n) => (
        <i key={n} class={n <= lit ? "on" : ""} style={`height:${2 + n * 2}px`} />
      ))}
    </span>
  );
}

function NetworkRow({ net }: { net: WifiNetwork }) {
  const chosen = ssid.value === net.ssid;
  return (
    <li>
      <button
        type="button"
        class={`wifi-row${chosen ? " chosen" : ""}`}
        onClick={() => {
          ssid.value = net.ssid;
          // An open network has no password to type, and leaving a stale one in
          // the field would send it as the PSK for a network that has none.
          if (!net.secure) psk.value = "";
          hidden.value = false;
        }}
      >
        <Bars signal={net.signal} />
        <span class="wifi-ssid">{net.ssid}</span>
        {net.secure
          ? <span class="wifi-lock" title="Password required">🔒</span>
          : <span class="wifi-open">open</span>}
      </button>
    </li>
  );
}

export function NetworkPage() {
  const rid = targetRobot.value;
  const state = wifi.value;
  const busy = pending.value;
  const live = conn.value === "live";

  if (!rid) {
    return <p class="hint pad">No robot selected — pick one on the driving view first.</p>;
  }

  const networks = scanned.value;
  const chosen = networks.find((n) => n.ssid === ssid.value);
  // Unknown until it has been picked from a scan: a hand-typed SSID might be
  // open or might not, and assuming it needs no password is the assumption that
  // silently sends an empty PSK and reports a confusing failure.
  const needsPassword = chosen ? chosen.secure : true;

  return (
    <>
      <div class="settings-bar">
        <div class="settings-bar-group">
          <span class="eyebrow">Robot</span>
          <select
            class="field-select"
            value={rid}
            onChange={(e) => configTarget.value = (e.target as HTMLSelectElement).value}
          >
            {robots.value.map((r) => (
              <option key={r.robot_id} value={r.robot_id}>{r.robot_id}</option>
            ))}
          </select>
        </div>
        <div class="settings-bar-group">
          <button
            type="button"
            class="btn ghost small"
            disabled={!live || !!busy}
            onClick={refreshWifi}
          >
            {busy === "status" ? "Asking…" : "Refresh"}
          </button>
          <button
            type="button"
            class="btn small primary"
            disabled={!live || !!busy}
            onClick={scanWifi}
          >
            {busy === "scan" ? "Scanning…" : "Scan"}
          </button>
        </div>
      </div>

      {/* Where it is now. First, because it is the question you came here with. */}
      <section class="wifi-now">
        <div class="section-title">
          <span class="eyebrow">This rover</span>
          {state?.device && <span class="eyebrow">{state.device}</span>}
        </div>
        {state == null
          ? (
            <p class="hint">
              Nothing asked yet. <strong>Refresh</strong> reads what network
              {" "}{rid} is on; <strong>Scan</strong> lists what it can see. Both
              go over the radio if it is not on WiFi, which is the point.
            </p>
          )
          : state.managed === false
          ? (
            <p class="banner warn">
              {state.error} — this Pi is on the older wpa_supplicant stack, so its
              WiFi cannot be set from here. Either flash a Bookworm-or-later image
              or edit <code>/etc/wpa_supplicant/wpa_supplicant.conf</code> on the
              card.
            </p>
          )
          : state.ssid
          ? (
            <dl class="wifi-facts">
              <dt>network</dt>
              <dd class="mono">
                {state.ssid}
                {typeof state.signal === "number" && <Bars signal={state.signal} />}
              </dd>
              <dt>address</dt>
              <dd class="mono">{state.ip ?? "waiting for a lease…"}</dd>
            </dl>
          )
          : <p class="hint">Not on any WiFi. Driving and the e-stop are unaffected — those never leave the radio.</p>}

        {state && !state.ok && state.error && state.managed !== false && (
          <p class="banner error">{state.error}</p>
        )}
        {state?.ok && state.forgot && (
          <p class="banner">Forgot “{state.forgot}”.</p>
        )}
        {state?.ssid && (
          <button
            type="button"
            class="btn ghost small danger"
            disabled={!live || !!busy}
            onClick={() => state.ssid && forgetWifi(state.ssid)}
            title="Delete the stored profile, so this rover stops rejoining it in preference to the network in front of it"
          >
            {busy === "forget" ? "Forgetting…" : `Forget ${state.ssid}`}
          </button>
        )}
      </section>

      {/* What it can see. */}
      {networks.length > 0 && (
        <section class="wifi-scan">
          <div class="section-title">
            <span class="eyebrow">In range</span>
            <span class="eyebrow">{networks.length}</span>
          </div>
          <ul class="wifi-list">
            {networks.map((net) => <NetworkRow key={net.ssid} net={net} />)}
          </ul>
        </section>
      )}

      {/* Joining one. */}
      <section class="wifi-join">
        <div class="section-title">
          <span class="eyebrow">Join a network</span>
        </div>
        <form
          class="wifi-form"
          onSubmit={(e) => {
            e.preventDefault();
            connectWifi();
          }}
        >
          <label class="arg">
            <span>network name</span>
            <input
              class="field-input"
              type="text"
              value={ssid.value}
              placeholder="pick one above, or type it"
              autocomplete="off"
              onInput={(e) => ssid.value = (e.target as HTMLInputElement).value}
            />
          </label>
          <label class="arg">
            <span>password</span>
            <input
              class="field-input"
              type="password"
              value={psk.value}
              placeholder={needsPassword ? "" : "not needed — open network"}
              disabled={!needsPassword}
              // Off deliberately: this is the rover's network credential, not
              // the operator's, and a base station laptop shared between drivers
              // should not be quietly collecting it.
              autocomplete="off"
              onInput={(e) => psk.value = (e.target as HTMLInputElement).value}
            />
          </label>
          <label class="arg">
            <span>country</span>
            <input
              class="field-input tiny"
              type="text"
              value={country.value}
              placeholder="GB"
              maxLength={2}
              autocomplete="off"
              onInput={(e) => country.value = (e.target as HTMLInputElement).value}
            />
          </label>
          <label class="arg wifi-hidden">
            <span>hidden</span>
            <input
              type="checkbox"
              checked={hidden.value}
              onChange={(e) => hidden.value = (e.target as HTMLInputElement).checked}
              title="The network does not broadcast its name, so NetworkManager has to be told to look for it"
            />
          </label>
          <button
            type="submit"
            class="btn primary"
            disabled={!live || !!busy || !ssid.value.trim()}
          >
            {busy === "connect" ? "Joining…" : "Connect"}
          </button>
        </form>

        <p class="hint">
          The password is sent, applied, and forgotten — it is never saved on the
          base station and never comes back in any status. The rover stores its
          own profile and rejoins this network by itself next power-up.
        </p>
        <p class="hint">
          <strong>Set the country once per country.</strong> With no regulatory
          domain a Pi has 5&nbsp;GHz soft-blocked, so a venue's 5&nbsp;GHz network
          is missing from the scan rather than refusing to connect.
        </p>
        {!state?.ssid && (
          <p class="banner warn">
            {rid} is not on WiFi, so this will go over the <strong>radio</strong>,
            and the XBee is unencrypted unless you have configured its AES key —
            a password sent this way is readable by anything listening on the
            channel. Fine on a bench or a home network; think twice with a
            credential you care about. A rover already on some WiFi is moved to
            another one over WiFi, and nothing goes on air.
          </p>
        )}
      </section>
    </>
  );
}
