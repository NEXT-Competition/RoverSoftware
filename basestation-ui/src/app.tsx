import { useState } from "preact/hooks";
import { activeSiteId, selectSite, selected, sites } from "./net/ws.ts";
import { mapFlipped, toggleMapFlip } from "./state/mapMode.ts";
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

function TopBar() {
  const flipped = mapFlipped.value;
  const siteList = sites.value;
  const activeId = activeSiteId.value;
  return (
    <header class="topbar panel">
      <div class="brand">
        <img src="/icon.svg" alt="" />
        <span class="name">
          RoverSoftware
          <small>base station</small>
        </span>
      </div>
      {Object.keys(siteList).length > 0 && (
        <select
          class="btn ghost"
          value={activeId ?? ""}
          onChange={(e) => selectSite((e.target as HTMLSelectElement).value)}
          title="Switch test site — moves the map and (in sim) the fleet"
        >
          {Object.entries(siteList).map(([id, site]) => (
            <option key={id} value={id}>{site.name}</option>
          ))}
        </select>
      )}
      <button
        class={`btn ghost${flipped ? " active" : ""}`}
        onClick={toggleMapFlip}
        title="Flip the field view 180°"
      >
        ⟲ Flip view
      </button>
      <ConnectionPill />
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

  return (
    <>
      <MapView />
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

      <EstopBar />
    </>
  );
}
