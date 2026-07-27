import { useEffect, useRef } from "preact/hooks";
import L from "leaflet";
import {
  activeSiteId,
  boundary,
  robots,
  selected,
  selectRobot,
  sites,
  tilesAttribution,
  tilesMaxZoom,
  tilesUrl,
} from "../net/ws.ts";
import { addWaypoint, routeMode, routePts } from "../state/route.ts";
import { mapFlipped } from "../state/mapMode.ts";
import type { LatLon, Robot } from "../net/types.ts";

// Satellite imagery, not a street map: driving a rover is about the terrain you
// can see (grass, gravel, tree lines), and rural test sites are mostly blank on
// a road basemap. Esri World Imagery needs no API key. Used only until the
// bridge's first fleet message arrives with the configured `tiles` URL.
const DEFAULT_TILES =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const DEFAULT_ATTRIBUTION = "Imagery © Esri, Maxar, Earthstar Geographics";
// Global imagery bottoms out around z19, but this specific field has real
// z20 coverage (checked directly against the tile server — z21 comes back
// "not yet available"). Past the true max, upscale rather than requesting
// tiles that come back blank.
const IMAGERY_MAX_NATIVE_ZOOM = 20;
// 1x1 transparent PNG so uncached (204) tiles render blank, not broken.
const BLANK_TILE =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==";

// The dashboard is locked to whichever named site is active (basestation/sites.py)
// — not a general-purpose map, so its view is fixed rather than fit to whatever's
// on screen. These are just the bootstrap values used for the very first paint,
// before the bridge's first snapshot (with the real site list) arrives; the
// reactive "site view" effect below takes over from there.
const DEFAULT_CENTER: LatLon = [38.8331773, -77.3232135];
const DEFAULT_ROTATION_DEG = 33;
const DEFAULT_ZOOM = 19.5;
// How much bigger than the visible viewport the (square) Leaflet container is,
// so its corners still cover the screen once rotated and clipped. Generous on
// purpose — the cost is a few off-screen tiles, not correctness.
const OVERSIZE_PCT = 250;

function robotIcon(r: Robot, isSel: boolean, rotationDeg: number): L.DivIcon {
  const cls = "robot-icon" + (r.online ? "" : " offline") + (isSel ? " sel" : "");
  const heading = r.heading ?? 0;
  return L.divIcon({
    className: cls,
    html:
      `<div class="body"><div class="arrow" style="transform:rotate(${heading}deg)"></div></div>` +
      // Counter-rotate the label only: the map (and this icon with it) is
      // rotated by rotationDeg, so spinning the tag back by the same amount
      // keeps the text upright while the heading arrow above it still
      // correctly inherits the map's rotation.
      `<div class="tag" style="transform:translateX(-50%) rotate(${-rotationDeg}deg)">${r.robot_id}</div>`,
    iconSize: [0, 0],
  });
}

/**
 * The map div is rotated for readability, but Leaflet has no idea — it still
 * does its own pixel math in an unrotated frame. A raw click's e.latlng would
 * therefore land in the wrong spot. Since panning/zoom are locked (the only
 * thing that ever moves is what the user taps), the fix is just undoing a
 * fixed rotation around a fixed center: take the click in the *unrotated*
 * wrapper's coordinate space, rotate that point by -angle around the visible
 * center, then hand Leaflet a corrected container point.
 */
function correctedLatLng(
  ev: MouseEvent,
  map: L.Map,
  mapEl: HTMLElement,
  wrapEl: HTMLElement,
  angleDeg: number,
): L.LatLng {
  const wrapRect = wrapEl.getBoundingClientRect();
  // mapEl's own left/top (set in the rotation effect) is the actual pivot —
  // usually the visible panel gap's center, not the wrap's raw center.
  const cx = mapEl.offsetLeft;
  const cy = mapEl.offsetTop;
  const dx = ev.clientX - wrapRect.left - cx;
  const dy = ev.clientY - wrapRect.top - cy;
  const rad = (-angleDeg * Math.PI) / 180;
  const dxr = dx * Math.cos(rad) - dy * Math.sin(rad);
  const dyr = dx * Math.sin(rad) + dy * Math.cos(rad);
  const px = dxr + mapEl.offsetWidth / 2;
  const py = dyr + mapEl.offsetHeight / 2;
  return map.containerPointToLatLng(L.point(px, py));
}

interface MarkerEntry {
  marker: L.Marker;
  trail: L.Polyline;
  iconKey: string;
}

export function MapView() {
  const wrapRef = useRef<HTMLDivElement>(null);
  const elRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const tileRef = useRef<L.TileLayer | null>(null);
  const markersRef = useRef<Record<string, MarkerEntry>>({});
  const routeLayersRef = useRef<{ pts: L.CircleMarker[]; line: L.Polyline | null }>({
    pts: [],
    line: null,
  });
  const boundaryLayerRef = useRef<L.Polygon | null>(null);
  const routeModeRef = useRef(false);
  const rotationRef = useRef(DEFAULT_ROTATION_DEG);

  // Read signals so this component re-renders (and the sync effects run) on change.
  const robotList = robots.value;
  const sel = selected.value;
  const tUrl = tilesUrl.value;
  const tMax = tilesMaxZoom.value;
  const tAttr = tilesAttribution.value;
  const rMode = routeMode.value;
  const rPts = routePts.value;
  const bounds = boundary.value;
  const flipped = mapFlipped.value;
  const activeSite = activeSiteId.value ? sites.value[activeSiteId.value] : undefined;
  const siteCenter = activeSite?.origin ?? DEFAULT_CENTER;
  const siteZoom = activeSite?.zoom ?? DEFAULT_ZOOM;
  const rotationDeg = (activeSite?.rotation_deg ?? DEFAULT_ROTATION_DEG) + (flipped ? 180 : 0);
  routeModeRef.current = rMode;
  rotationRef.current = rotationDeg;

  // ---- init once: locked interactions (view itself is set by the site effect below) ----
  useEffect(() => {
    const mapEl = elRef.current!;
    const map = L.map(mapEl, {
      zoomControl: false,
      zoomSnap: 0.25, // lets a site's zoom sit at a half-step instead of a whole zoom level
      dragging: false,
      touchZoom: false,
      scrollWheelZoom: false,
      doubleClickZoom: false,
      boxZoom: false,
      keyboard: false,
      minZoom: 15,
      maxZoom: 20,
    }).setView(DEFAULT_CENTER, DEFAULT_ZOOM);
    mapRef.current = map;
    // Imagery licences want a visible credit, but a kiosk doesn't need the
    // "Leaflet" link — keep the control, drop the prefix.
    map.attributionControl.setPrefix(false);
    tileRef.current = L.tileLayer(DEFAULT_TILES, {
      maxZoom: IMAGERY_MAX_NATIVE_ZOOM,
      maxNativeZoom: IMAGERY_MAX_NATIVE_ZOOM,
      errorTileUrl: BLANK_TILE,
      attribution: DEFAULT_ATTRIBUTION,
    }).addTo(map);

    map.on("click", (e: L.LeafletMouseEvent) => {
      if (!routeModeRef.current) return;
      const ll = correctedLatLng(e.originalEvent, map, mapEl, wrapRef.current!, rotationRef.current);
      addWaypoint(ll.lat, ll.lng);
    });

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // ---- rotation: applied on mount, whenever the flip mode changes, and
  // whenever the visible gap between panels resizes ----
  useEffect(() => {
    const mapEl = elRef.current!;
    const wrapEl = wrapRef.current!;
    // The HUD grid has a top bar but no matching bottom bar, so the visible
    // "hole" between panels isn't centered on the viewport — it sits lower.
    // Anchor the map's rotation on that hole's actual center (measured via
    // the invisible .map-hole grid cell) instead of the viewport's 50/50,
    // so the field stays centered in the visible gap at any rotation angle.
    const holeEl = document.querySelector<HTMLElement>(".map-hole");
    const position = () => {
      const wrapRect = wrapEl.getBoundingClientRect();
      let cx = wrapRect.width / 2;
      let cy = wrapRect.height / 2;
      if (holeEl) {
        const holeRect = holeEl.getBoundingClientRect();
        cx = holeRect.left - wrapRect.left + holeRect.width / 2;
        cy = holeRect.top - wrapRect.top + holeRect.height / 2;
      }
      mapEl.style.left = `${cx}px`;
      mapEl.style.top = `${cy}px`;
      mapEl.style.transform = `translate(-50%, -50%) rotate(${rotationDeg}deg)`;
    };
    position();
    if (!holeEl) return;
    const ro = new ResizeObserver(position);
    ro.observe(holeEl);
    return () => ro.disconnect();
  }, [rotationDeg]);

  // ---- site view: re-center/re-zoom (and re-lock the zoom range) whenever
  // the active site changes. Keyed on the actual lat/lon/zoom values, not the
  // site object, since `sites` is replaced wholesale on every snapshot. ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    map.setMinZoom(siteZoom);
    map.setMaxZoom(siteZoom);
    map.setView(siteCenter, siteZoom);
  }, [siteCenter[0], siteCenter[1], siteZoom]);

  // ---- tile source (offline /tiles arrives with the first fleet message) ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const url = tUrl || DEFAULT_TILES;
    const opts: L.TileLayerOptions = {
      maxZoom: IMAGERY_MAX_NATIVE_ZOOM,
      // Cache depth if we know it, else the imagery source's global limit;
      // either way Leaflet upscales instead of asking for tiles that don't exist.
      maxNativeZoom: tMax ?? IMAGERY_MAX_NATIVE_ZOOM,
      errorTileUrl: BLANK_TILE,
      attribution: tAttr ?? (tUrl ? undefined : DEFAULT_ATTRIBUTION),
    };
    if (tileRef.current) map.removeLayer(tileRef.current);
    tileRef.current = L.tileLayer(url, opts).addTo(map);
  }, [tUrl, tMax, tAttr]);

  // ---- robots: markers + trails (runs each render) ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const markers = markersRef.current;
    const seen = new Set<string>();

    for (const r of robotList) {
      if (r.lat == null || r.lon == null) continue;
      seen.add(r.robot_id);
      const isSel = r.robot_id === sel;
      let entry = markers[r.robot_id];
      const iconKey = `${r.online}|${isSel}|${r.heading ?? 0}|${rotationDeg}`;
      if (!entry) {
        entry = {
          marker: L.marker([r.lat, r.lon], { icon: robotIcon(r, isSel, rotationDeg) }).addTo(map),
          trail: L.polyline([], { color: "#4c9eff", weight: 2, opacity: 0.55 }).addTo(map),
          iconKey,
        };
        entry.marker.on("click", () => selectRobot(r.robot_id));
        markers[r.robot_id] = entry;
      }
      entry.marker.setLatLng([r.lat, r.lon]);
      // setIcon tears down and rebuilds the marker's DOM node (and its click
      // listener) — at 30Hz telemetry that was happening ~27x/sec, so a real
      // click could land mid-swap and get silently dropped. Only rebuild when
      // something the rendered icon actually depends on has changed.
      if (entry.iconKey !== iconKey) {
        entry.marker.setIcon(robotIcon(r, isSel, rotationDeg));
        entry.iconKey = iconKey;
      }
      if (r.trail && r.trail.length) entry.trail.setLatLngs(r.trail as L.LatLngExpression[]);
    }

    for (const id of Object.keys(markers)) {
      if (!seen.has(id)) {
        map.removeLayer(markers[id].marker);
        map.removeLayer(markers[id].trail);
        delete markers[id];
      }
    }
  });

  // ---- field boundary outline — some sites (e.g. an open plaza) have none,
  // so an empty `bounds` means "remove whatever fence the last site had" ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    if (boundaryLayerRef.current) {
      map.removeLayer(boundaryLayerRef.current);
      boundaryLayerRef.current = null;
    }
    if (bounds.length) {
      boundaryLayerRef.current = L.polygon(bounds as L.LatLngExpression[], {
        color: "#ff2d2d",
        weight: 4,
        fill: false,
      }).addTo(map);
    }
  }, [bounds]);

  // ---- pending route overlay ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const layers = routeLayersRef.current;
    layers.pts.forEach((m) => map.removeLayer(m));
    layers.pts = [];
    if (layers.line) {
      map.removeLayer(layers.line);
      layers.line = null;
    }
    rPts.forEach((p, i) => {
      layers.pts.push(
        L.circleMarker(p as L.LatLngExpression, {
          radius: 6,
          color: "#f0a53a",
          fillColor: "#f0a53a",
          fillOpacity: 1,
          weight: 2,
        })
          .bindTooltip(
            // Counter-rotate the number too, same reasoning as the robot tag.
            `<span style="display:inline-block;transform:rotate(${-rotationDeg}deg)">${i + 1}</span>`,
            { permanent: true, direction: "top", className: "wp-tip" },
          )
          .addTo(map),
      );
    });
    if (rPts.length > 1) {
      layers.line = L.polyline(rPts as L.LatLngExpression[], {
        color: "#f0a53a",
        weight: 2,
        dashArray: "6 6",
      }).addTo(map);
    }
  }, [rPts, rotationDeg]);

  return (
    <div ref={wrapRef} class="map-wrap">
      <div
        ref={elRef}
        class="map"
        style={{ width: `${OVERSIZE_PCT}%`, height: `${OVERSIZE_PCT}%` }}
      />
    </div>
  );
}
