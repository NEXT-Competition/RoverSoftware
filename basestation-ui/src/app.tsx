import { useState } from "preact/hooks";
import { robots, selected } from "./net/ws.ts";
import { showView, view } from "./state/view.ts";
import { MapView } from "./components/MapView.tsx";
import { ConnectionPill } from "./components/ConnectionPill.tsx";
import { ControllerStatus } from "./components/ControllerStatus.tsx";
import { FleetPanel } from "./components/FleetPanel.tsx";
import { FPV } from "./components/FPV.tsx";
import { ModeControls } from "./components/ModeControls.tsx";
import { RouteControls } from "./components/RouteControls.tsx";
import { ShooterControls } from "./components/ShooterControls.tsx";
import { Telemetry } from "./components/Telemetry.tsx";
import { DrivePad } from "./components/DrivePad.tsx";
import { EstopBar } from "./components/EstopBar.tsx";
import { SettingsPage } from "./components/settings/SettingsPage.tsx";

function GearIcon() {
  return (
    <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true">
      <path
        fill="currentColor"
        d="M12 8.4a3.6 3.6 0 1 0 0 7.2 3.6 3.6 0 0 0 0-7.2Zm0 5.6a2 2 0 1 1 0-4 2 2 0 0 1 0 4Z"
      />
      <path
        fill="currentColor"
        opacity=".6"
        d="M19.9 12c0-.4 0-.8-.1-1.2l1.7-1.3a.5.5 0 0 0 .1-.7l-1.6-2.7a.5.5 0 0 0-.6-.2l-2 .8a7.6 7.6 0 0 0-2-1.2l-.3-2.1a.5.5 0 0 0-.5-.4h-3.2a.5.5 0 0 0-.5.4l-.3 2.1c-.7.3-1.4.7-2 1.2l-2-.8a.5.5 0 0 0-.6.2L4.4 8.8a.5.5 0 0 0 .1.7l1.7 1.3a8 8 0 0 0 0 2.4l-1.7 1.3a.5.5 0 0 0-.1.7l1.6 2.7c.1.2.4.3.6.2l2-.8c.6.5 1.3.9 2 1.2l.3 2.1c0 .2.3.4.5.4h3.2c.2 0 .5-.2.5-.4l.3-2.1c.7-.3 1.4-.7 2-1.2l2 .8c.2.1.5 0 .6-.2l1.6-2.7a.5.5 0 0 0-.1-.7l-1.7-1.3c.1-.4.1-.8.1-1.2Z"
      />
    </svg>
  );
}

function TopBar() {
  const online = robots.value.filter((r) => r.online).length;
  return (
    <header class="topbar panel">
      <div class="brand">
        <img src="/icon.svg" alt="" />
        <span class="name">
          RoverSoftware
          <small>base station</small>
        </span>
      </div>
      {/* Fleet health at a glance, so the rail is for detail rather than for
          the one number an operator checks constantly. */}
      <span class="pill quiet" title="Robots currently reporting telemetry">
        <span class={`led ${online > 0 ? "ok" : "bad"}`} />
        {online}/{robots.value.length} live
      </span>
      <ConnectionPill />
      <button
        type="button"
        class="icon-btn"
        title="Settings"
        aria-label="Settings"
        onClick={() => showView("settings")}
      >
        <GearIcon />
      </button>
    </header>
  );
}

function ControlSection() {
  const sel = selected.value;
  return (
    <section class="rail-section">
      <div class="section-title" style="margin-bottom:10px">
        <span class="eyebrow">Selected</span>
        <span class="eyebrow" style="color:var(--accent)">{sel ?? "—"}</span>
      </div>
      <ModeControls />
      <div style="height:14px" />
      {/* Renders nothing unless the robot is in shooter_align (see the component). */}
      <ShooterControls />
      <RouteControls />
      <div style="height:12px" />
      <Telemetry />
    </section>
  );
}

export function App() {
  // Portrait bottom-sheet collapse (ignored by the landscape layout).
  const [collapsed, setCollapsed] = useState(false);
  const settings = view.value === "settings";

  return (
    <>
      {/* The map stays mounted under the settings sheet: Leaflet re-initialises
          slowly and loses its viewport, and returning to a re-centred map after
          changing a setting is disorienting. */}
      <MapView />
      {!settings && (
        <div class="hud">
          <TopBar />

          <aside class={`rail panel${collapsed ? " collapsed" : ""}`}>
            <div class="drawer-handle" onClick={() => setCollapsed((c) => !c)} />
            <div class="rail-body">
              <FleetPanel />
              <FPV />
              <ControlSection />
              <div class="rail-section" style="border:none">
                <ControllerStatus />
              </div>
            </div>
          </aside>

          <div class="dock">
            <DrivePad />
          </div>
        </div>
      )}

      {settings && <SettingsPage />}

      {/* Outside the view switch on purpose: the stop button stays reachable on
          every screen, including the one where drive limits are being changed. */}
      <EstopBar />
    </>
  );
}
