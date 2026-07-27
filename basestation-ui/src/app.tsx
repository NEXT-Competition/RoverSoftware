import { useEffect, useState } from "preact/hooks";
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
import { EstopBar } from "./components/EstopBar.tsx";

function TopBar() {
  const flipped = mapFlipped.value;
  const siteList = sites.value;
  const activeId = activeSiteId.value;
  // Flip view only makes sense for a fixed, rotated frame matched against a
  // real boundary — free-pan sites (no boundary to hold the view against,
  // e.g. GMU plaza) already let the user look at it from any angle.
  const locked = (activeId ? siteList[activeId]?.locked : undefined) ?? true;
  // A free-pan site can't un-flip itself (the button below is disabled), so
  // don't let a flip carried over from a locked site strand the view rotated.
  useEffect(() => {
    if (!locked && mapFlipped.value) mapFlipped.value = false;
  }, [locked]);
  return (
    <header class="topbar panel">
      <div class="topbar-row">
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
      </div>
      <div class="topbar-row">
        <button
          class={`btn ghost${flipped ? " active" : ""}`}
          onClick={toggleMapFlip}
          disabled={!locked}
          title={locked ? "Flip the field view 180°" : "Free pan/zoom sites don't lock to one orientation"}
        >
          ⟲ Flip view
        </button>
        <ConnectionPill />
      </div>
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

function FleetRail() {
  // Portrait bottom-sheet collapse (ignored by the landscape layout).
  const [collapsed, setCollapsed] = useState(false);
  return (
    <aside class={`rail rail-left panel${collapsed ? " collapsed" : ""}`}>
      <div class="drawer-handle" onClick={() => setCollapsed((c) => !c)} />
      <div class="rail-body">
        <FleetPanel />
        <FPV />
      </div>
    </aside>
  );
}

function ControlRail() {
  const [collapsed, setCollapsed] = useState(false);
  return (
    <aside class={`rail rail-right panel${collapsed ? " collapsed" : ""}`}>
      <div class="drawer-handle" onClick={() => setCollapsed((c) => !c)} />
      <div class="rail-body">
        <ControlSection />
        <div class="rail-section" style="border:none">
          <ControllerStatus />
        </div>
      </div>
    </aside>
  );
}

export function App() {
  return (
    <>
      <MapView />
      <div class="hud">
        <TopBar />
        <FleetRail />
        <ControlRail />
        <div class="map-hole" />
      </div>

      <EstopBar />
    </>
  );
}
