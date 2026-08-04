// The settings view: a full-screen sheet over the map, with three tabs.
//
// Full-screen rather than a panel in the rail, because these are forms with
// help text and they lose to a 360 px column. It sits above the HUD but BELOW
// the e-stop dock — tuning a PID gain must never be a reason the stop button
// is out of reach.

import { conn } from "../../net/ws.ts";
import { tab } from "../../state/settings.ts";
import { showView } from "../../state/view.ts";
import { HardwarePage } from "../hardware/HardwarePage.tsx";
import { RoutinesPage } from "../routines/RoutinesPage.tsx";
import { CodePage } from "../code/CodePage.tsx";
import { NetworkPage } from "./NetworkPage.tsx";
import { BaseSettings } from "./BaseSettings.tsx";
import { ControllerSettings } from "./ControllerSettings.tsx";
import { RobotSettings } from "./RobotSettings.tsx";

// Hardware and Routines sit behind the gear with the rest of the configuration
// rather than as their own top-level view: making them one would duplicate the
// robot picker, the connection pill and the offline banner for no benefit.
const TABS = [
  { key: "robot", label: "Tuning" },
  { key: "hardware", label: "Hardware" },
  { key: "routines", label: "Routines" },
  // Beside Routines rather than replacing it. The two are siblings: a graph is
  // the better shape for a sequence of states, code is the better shape for
  // anything with arithmetic in it, and most teams end up with some of each.
  { key: "code", label: "Code" },
  // Network sits with the other per-robot tabs, not under Base station: it is
  // the rover's WiFi that changes when you travel, and it changes per rover.
  { key: "network", label: "Network" },
  { key: "controller", label: "Controller" },
  { key: "base", label: "Base station" },
] as const;

/**
 * What actually happens when you change something, per tab.
 *
 * This used to be one sentence printed under all six — "Changes apply
 * immediately and are saved" — which is true of the three that commit per
 * field and false of the three that do not. Hardware and Routines edit a whole
 * document locally and send it on Save, so telling an operator their unsaved
 * routine was already stored is the one sentence here that can cost a match.
 * Network sends a request over the radio that can fail, and deliberately saves
 * nothing; it explains itself on the page and gets no footnote at all.
 */
function Footnote({ active }: { active: string }) {
  if (active === "network") return null;
  if (active === "hardware" || active === "routines" || active === "code") {
    return (
      <p class="hint pad footnote">
        Edits here stay on this base station until you press{" "}
        <strong>Save to robot</strong> — the rover keeps running what it already
        has until then. Discard throws the draft away.
        {/* Worth its own sentence, because it is the one thing on this page
            that behaves better than an operator would expect: the rover
            compiles a script before it stores it, so a typo comes back as a
            line number here rather than as a Run that dies at the field. */}
        {active === "code" && (
          <>
            {" "}The rover compiles each script as it lands, so a syntax error
            is refused with its line number instead of being discovered when you
            press Run.
          </>
        )}
      </p>
    );
  }
  return (
    <p class="hint pad footnote">
      Changes apply immediately and are saved — on the robot for robot settings,
      on this base station for the rest. Fields badged “restart” are stored but
      only take effect when the service is restarted.
    </p>
  );
}

export function SettingsPage() {
  const active = tab.value;
  return (
    <div class="settings-sheet">
      <header class="settings-head panel">
        <button
          type="button"
          class="btn ghost back"
          onClick={() => showView("ops")}
        >
          ‹ Driving
        </button>
        <div class="tabs">
          {TABS.map((t) => (
            <button
              key={t.key}
              type="button"
              class={active === t.key ? "on" : ""}
              onClick={() => tab.value = t.key}
            >
              {t.label}
            </button>
          ))}
        </div>
        <span class="settings-conn">
          {conn.value === "live" ? "" : "offline — changes will not be sent"}
        </span>
      </header>

      <div class="settings-body">
        {/* The Routines tab is a canvas, not a form, so it opts out of the
            reading measure the other tabs want. */}
        <div
          class={`settings-col${
            active === "routines" || active === "code" ? " wide" : ""
          }`}
        >
          {active === "robot" && <RobotSettings />}
          {active === "hardware" && <HardwarePage />}
          {active === "routines" && <RoutinesPage />}
          {active === "code" && <CodePage />}
          {active === "network" && <NetworkPage />}
          {active === "controller" && <ControllerSettings />}
          {active === "base" && <BaseSettings />}
          <Footnote active={active} />
        </div>
      </div>
    </div>
  );
}
