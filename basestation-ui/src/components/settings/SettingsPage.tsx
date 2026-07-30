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
  // Network sits with the other per-robot tabs, not under Base station: it is
  // the rover's WiFi that changes when you travel, and it changes per rover.
  { key: "network", label: "Network" },
  { key: "controller", label: "Controller" },
  { key: "base", label: "Base station" },
] as const;

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
        <div class={`settings-col${active === "routines" ? " wide" : ""}`}>
          {active === "robot" && <RobotSettings />}
          {active === "hardware" && <HardwarePage />}
          {active === "routines" && <RoutinesPage />}
          {active === "network" && <NetworkPage />}
          {active === "controller" && <ControllerSettings />}
          {active === "base" && <BaseSettings />}
          <p class="hint pad footnote">
            Changes apply immediately and are saved — on the robot for robot
            settings, on this base station for the rest. Fields badged
            “restart” are stored but only take effect when the service is
            restarted.
          </p>
        </div>
      </div>
    </div>
  );
}
