// RoverSoftware base station dashboard.
// Streams fleet state over a WebSocket, renders robots on a Leaflet map, and
// sends control actions (select / mode / e-stop / route) back to the backend.

let map, tileLayer;
const markers = {};   // robot_id -> { marker, trail }
let selected = null;
let firstFit = true;

// --- pending route drawing state ---
let routeMode = false;
let routePts = [];         // [[lat, lon], ...]
let routeLine = null;
let routeMarkers = [];

// ---------- Map ----------
function initMap(tilesUrl, maxNativeZoom) {
  map = L.map("map", { zoomControl: true }).setView([37.7749, -122.4194], 17);
  const opts = { maxZoom: 20, attribution: "© OpenStreetMap" };
  // Offline caches only go so deep; upscale the deepest cached tiles beyond that
  // instead of showing blank tiles when the user zooms past the cache.
  if (maxNativeZoom) opts.maxNativeZoom = maxNativeZoom;
  tileLayer = L.tileLayer(tilesUrl || "https://tile.openstreetmap.org/{z}/{x}/{y}.png", opts).addTo(map);

  map.on("click", (e) => {
    if (!routeMode) return;
    routePts.push([e.latlng.lat, e.latlng.lng]);
    redrawRoute();
  });
}

function robotIcon(r, isSel) {
  const cls = "robot-icon" + (r.online ? "" : " offline") + (isSel ? " sel" : "");
  const heading = r.heading || 0;
  return L.divIcon({
    className: cls,
    html: `<div class="body"><div class="arrow" style="transform:rotate(${heading}deg)"></div></div>
           <div class="tag">${r.robot_id}</div>`,
    iconSize: [0, 0],
  });
}

function updateMap(robots) {
  const seen = new Set();
  const positioned = [];

  for (const r of robots) {
    if (r.lat == null || r.lon == null) continue;
    seen.add(r.robot_id);
    positioned.push([r.lat, r.lon]);
    const isSel = r.robot_id === selected;
    let entry = markers[r.robot_id];
    if (!entry) {
      entry = {
        marker: L.marker([r.lat, r.lon], { icon: robotIcon(r, isSel) }).addTo(map),
        trail: L.polyline([], { color: "#4c9eff", weight: 2, opacity: 0.6 }).addTo(map),
      };
      entry.marker.on("click", () => selectRobot(r.robot_id));
      markers[r.robot_id] = entry;
    }
    entry.marker.setLatLng([r.lat, r.lon]);
    entry.marker.setIcon(robotIcon(r, isSel));
    if (r.trail && r.trail.length) entry.trail.setLatLngs(r.trail);
  }

  // drop markers for robots that vanished
  for (const id of Object.keys(markers)) {
    if (!seen.has(id)) {
      map.removeLayer(markers[id].marker);
      map.removeLayer(markers[id].trail);
      delete markers[id];
    }
  }

  if (firstFit && positioned.length) {
    firstFit = false;
    if (positioned.length === 1) map.setView(positioned[0], 18);
    else map.fitBounds(positioned, { padding: [60, 60] });
  }
}

// ---------- Route ----------
function redrawRoute() {
  routeMarkers.forEach((m) => map.removeLayer(m));
  routeMarkers = [];
  if (routeLine) { map.removeLayer(routeLine); routeLine = null; }
  routePts.forEach((p, i) => {
    routeMarkers.push(
      L.circleMarker(p, { radius: 5, color: "#f0a53a", fillOpacity: 1 })
        .bindTooltip("" + (i + 1), { permanent: true, direction: "top", className: "wp-tip" })
        .addTo(map)
    );
  });
  if (routePts.length > 1) {
    routeLine = L.polyline(routePts, { color: "#f0a53a", weight: 2, dashArray: "6 6" }).addTo(map);
  }
}

function clearRoute() {
  routePts = [];
  redrawRoute();
}

// ---------- Sidebar ----------
function bar(v) {
  const pct = Math.min(Math.abs(v), 1) * 50;
  const side = v >= 0 ? `left:50%;width:${pct}%` : `left:${50 - pct}%;width:${pct}%`;
  const col = v >= 0 ? "#4c9eff" : "#f0a53a";
  return `<div class="bar-track"><div class="bar-fill" style="${side};background:${col}"></div></div>`;
}

function renderFleet(robots) {
  const list = document.getElementById("fleet-list");
  list.innerHTML = "";
  for (const r of robots) {
    const li = document.createElement("li");
    li.className = "robot" + (r.robot_id === selected ? " selected" : "");
    const batt = r.battery == null ? "—" : `${r.battery.toFixed(0)}%`;
    li.innerHTML = `
      <div class="top">
        <span class="name"><span class="dot ${r.online ? "online" : "offline"}"></span>${r.robot_id}</span>
        <span class="mode">${r.estop ? '<span class="estop-flag">E-STOP</span>' : r.mode}</span>
      </div>
      <div class="meta"><span>🔋 ${batt}</span><span>${r.online ? "live" : (r.age != null ? r.age + "s ago" : "no signal")}</span></div>
      <div class="bars">
        <div class="bar-wrap"><div class="bar-label">L ${r.left.toFixed(2)}</div>${bar(r.left)}</div>
        <div class="bar-wrap"><div class="bar-label">R ${r.right.toFixed(2)}</div>${bar(r.right)}</div>
      </div>`;
    li.onclick = () => selectRobot(r.robot_id);
    list.appendChild(li);
  }

  const sel = robots.find((r) => r.robot_id === selected);
  document.getElementById("sel-name").textContent = selected || "—";
  const tel = document.getElementById("telemetry");
  if (sel) {
    const fmt = (v, d = 5) => (v == null ? "—" : v.toFixed(d));
    tel.innerHTML = `
      <div class="tel-row"><span class="k">mode</span><span>${sel.mode}</span></div>
      <div class="tel-row"><span class="k">position</span><span>${fmt(sel.lat)}, ${fmt(sel.lon)}</span></div>
      <div class="tel-row"><span class="k">heading</span><span>${fmt(sel.heading, 1)}°</span></div>
      <div class="tel-row"><span class="k">battery</span><span>${sel.battery == null ? "—" : sel.battery.toFixed(1) + "%"}</span></div>
      <div class="tel-row"><span class="k">link</span><span>${sel.online ? "online" : "offline"}</span></div>`;
  } else {
    tel.innerHTML = "";
  }

  document.querySelectorAll(".modes button").forEach((b) => {
    b.classList.toggle("active", sel && b.dataset.mode === sel.mode);
  });
}

// ---------- Actions ----------
let ws;
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}
function selectRobot(id) {
  selected = id;
  send({ action: "select", robot_id: id });
}

function wireControls() {
  document.querySelectorAll(".modes button").forEach((b) => {
    b.onclick = () => selected && send({ action: "mode", robot_id: selected, mode: b.dataset.mode });
  });
  document.getElementById("estop").onclick = () => selected && send({ action: "estop", robot_id: selected });
  document.getElementById("clear-estop").onclick = () => selected && send({ action: "clear_estop", robot_id: selected });

  const toggle = document.getElementById("route-toggle");
  toggle.onclick = () => {
    routeMode = !routeMode;
    toggle.classList.toggle("active", routeMode);
    document.getElementById("route-hint").textContent = routeMode
      ? "Click the map to drop waypoints, then Send."
      : "Toggle “Add waypoints”, click the map, then Send.";
  };
  document.getElementById("route-send").onclick = () => {
    if (selected && routePts.length) {
      send({ action: "route", robot_id: selected, waypoints: routePts });
      send({ action: "mode", robot_id: selected, mode: "waypoint" });
    }
  };
  document.getElementById("route-clear").onclick = clearRoute;
}

// ---------- WebSocket ----------
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  const conn = document.getElementById("conn");

  ws.onopen = () => { conn.textContent = "live"; conn.className = "pill ok"; };
  ws.onclose = () => {
    conn.textContent = "reconnecting…"; conn.className = "pill bad";
    setTimeout(connect, 1000);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type !== "fleet") return;
    if (!map) initMap(msg.tiles, msg.tiles_maxzoom);
    if (selected == null) selected = msg.selected;

    renderFleet(msg.robots);
    updateMap(msg.robots);

    const cs = document.getElementById("controller-status");
    const c = msg.controller || {};
    cs.textContent = c.connected ? `🎮 ${c.name}` : "🎮 no controller";
  };
}

wireControls();
connect();
