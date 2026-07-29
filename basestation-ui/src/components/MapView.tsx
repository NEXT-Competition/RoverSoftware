import { useEffect, useRef } from "preact/hooks";
import L from "leaflet";
import {
  robots,
  selected,
  selectRobot,
  tilesAttribution,
  tilesMaxZoom,
  tilesUrl,
} from "../net/ws.ts";
import { addWaypoint, routeMode, routePts } from "../state/route.ts";
import {
  addPlaceAt,
  editingPlace,
  movePlaceTo,
  placeMode,
  placeList,
} from "../state/places.ts";
import type { LatLon, Place, Robot } from "../net/types.ts";

// Satellite imagery, not a street map: driving a rover is about the terrain you
// can see (grass, gravel, tree lines), and rural test sites are mostly blank on
// a road basemap. Esri World Imagery needs no API key. Used only until the
// bridge's first fleet message arrives with the configured `tiles` URL.
const DEFAULT_TILES =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";
const DEFAULT_ATTRIBUTION = "Imagery © Esri, Maxar, Earthstar Geographics";
// Global imagery bottoms out around z19. Past that, upscale the deepest tile we
// can actually get rather than requesting tiles that come back blank.
const IMAGERY_MAX_NATIVE_ZOOM = 19;
// 1x1 transparent PNG so uncached (204) tiles render blank, not broken.
const BLANK_TILE =
  "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==";

function robotIcon(r: Robot, isSel: boolean): L.DivIcon {
  const cls = "robot-icon" + (r.online ? "" : " offline") + (isSel ? " sel" : "");
  const heading = r.heading ?? 0;
  return L.divIcon({
    className: cls,
    html:
      `<div class="body"><div class="arrow" style="transform:rotate(${heading}deg)"></div></div>` +
      `<div class="tag">${r.robot_id}</div>`,
    iconSize: [0, 0],
  });
}

interface MarkerEntry {
  marker: L.Marker;
  trail: L.Polyline;
}

// A saved place reads as permanent, a pending route waypoint as transient, so
// they must not share a colour: the whole point of the workflow is knowing at a
// glance which marks survive the match and which are this run's plan.
const KIND_GLYPH: Record<Place["kind"], string> = {
  bucket: "◆",
  marker: "●",
  start: "▲",
  hazard: "✕",
};

/** A place name is operator-typed and lands in a divIcon's innerHTML. */
function escapeHtml(text: string): string {
  return text.replace(
    /[&<>"']/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]!),
  );
}

function placeIcon(place: Place, isSel: boolean): L.DivIcon {
  return L.divIcon({
    className: `place-icon k-${place.kind}` + (isSel ? " sel" : ""),
    html: `<div class="pin">${KIND_GLYPH[place.kind] ?? "●"}</div>` +
      `<div class="tag">${escapeHtml(place.name)}</div>`,
    iconSize: [0, 0],
  });
}

export function MapView() {
  const elRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const tileRef = useRef<L.TileLayer | null>(null);
  const markersRef = useRef<Record<string, MarkerEntry>>({});
  const routeLayersRef = useRef<{ pts: L.CircleMarker[]; line: L.Polyline | null }>({
    pts: [],
    line: null,
  });
  const placeLayersRef = useRef<Record<string, L.Marker>>({});
  const firstFitRef = useRef(true);
  const routeModeRef = useRef(false);
  const placeModeRef = useRef(false);

  // Read signals so this component re-renders (and the sync effects run) on change.
  const robotList = robots.value;
  const sel = selected.value;
  const tUrl = tilesUrl.value;
  const tMax = tilesMaxZoom.value;
  const tAttr = tilesAttribution.value;
  const rMode = routeMode.value;
  const rPts = routePts.value;
  const pMode = placeMode.value;
  const savedPlaces = placeList.value;
  const editing = editingPlace.value;
  routeModeRef.current = rMode;
  placeModeRef.current = pMode;

  // ---- init once ----
  useEffect(() => {
    // Touch-first: no +/- control (it collided with the top bar); pinch and
    // scroll-wheel zoom stay enabled by default.
    const map = L.map(elRef.current!, { zoomControl: false })
      .setView([37.7749, -122.4194], 17);
    mapRef.current = map;
    // Imagery licences want a visible credit, but a kiosk doesn't need the
    // "Leaflet" link — keep the control, drop the prefix.
    map.attributionControl.setPrefix(false);
    tileRef.current = L.tileLayer(DEFAULT_TILES, {
      maxZoom: 20,
      maxNativeZoom: IMAGERY_MAX_NATIVE_ZOOM,
      errorTileUrl: BLANK_TILE,
      attribution: DEFAULT_ATTRIBUTION,
    }).addTo(map);

    map.on("click", (e: L.LeafletMouseEvent) => {
      // Route drawing wins if both are somehow on; the controls keep them
      // mutually exclusive (state/route.ts), so this is belt and braces.
      if (routeModeRef.current) addWaypoint(e.latlng.lat, e.latlng.lng);
      else if (placeModeRef.current) {
        editingPlace.value = addPlaceAt(e.latlng.lat, e.latlng.lng);
      }
    });

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  // ---- tile source (offline /tiles arrives with the first fleet message) ----
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const url = tUrl || DEFAULT_TILES;
    const opts: L.TileLayerOptions = {
      maxZoom: 20,
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
    const positioned: LatLon[] = [];

    for (const r of robotList) {
      if (r.lat == null || r.lon == null) continue;
      seen.add(r.robot_id);
      positioned.push([r.lat, r.lon]);
      const isSel = r.robot_id === sel;
      let entry = markers[r.robot_id];
      if (!entry) {
        entry = {
          marker: L.marker([r.lat, r.lon], { icon: robotIcon(r, isSel) }).addTo(map),
          trail: L.polyline([], { color: "#4c9eff", weight: 2, opacity: 0.55 }).addTo(map),
        };
        entry.marker.on("click", () => selectRobot(r.robot_id));
        markers[r.robot_id] = entry;
      }
      entry.marker.setLatLng([r.lat, r.lon]);
      entry.marker.setIcon(robotIcon(r, isSel));
      if (r.trail && r.trail.length) entry.trail.setLatLngs(r.trail as L.LatLngExpression[]);
    }

    for (const id of Object.keys(markers)) {
      if (!seen.has(id)) {
        map.removeLayer(markers[id].marker);
        map.removeLayer(markers[id].trail);
        delete markers[id];
      }
    }

    if (firstFitRef.current && positioned.length) {
      firstFitRef.current = false;
      if (positioned.length === 1) map.setView(positioned[0], 18);
      else map.fitBounds(positioned as L.LatLngBoundsExpression, { padding: [70, 70] });
    }
  });

  // ---- saved places ----
  // Draggable, because a bucket recorded from a rover parked beside it is a
  // metre or two off by construction, and nudging the pin is how that gets
  // fixed without driving back out to it.
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    const layers = placeLayersRef.current;
    const seen = new Set<string>();

    for (const place of savedPlaces) {
      seen.add(place.id);
      let marker = layers[place.id];
      if (!marker) {
        marker = L.marker([place.lat, place.lon], {
          icon: placeIcon(place, place.id === editing),
          draggable: true,
        }).addTo(map);
        marker.on("click", () => {
          editingPlace.value = editingPlace.value === place.id ? null : place.id;
        });
        marker.on("dragend", (e) => {
          const at = (e.target as L.Marker).getLatLng();
          movePlaceTo(place.id, at.lat, at.lng);
        });
        layers[place.id] = marker;
      }
      marker.setLatLng([place.lat, place.lon]);
      marker.setIcon(placeIcon(place, place.id === editing));
    }

    for (const id of Object.keys(layers)) {
      if (!seen.has(id)) {
        map.removeLayer(layers[id]);
        delete layers[id];
      }
    }
  }, [savedPlaces, editing]);

  // Drop mode changes what a tap on the map MEANS, which is exactly the kind of
  // mode an operator forgets they are in. The cursor is the standing reminder.
  useEffect(() => {
    const el = elRef.current;
    if (el) el.classList.toggle("dropping", rMode || pMode);
  }, [rMode, pMode]);

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
          .bindTooltip(String(i + 1), {
            permanent: true,
            direction: "top",
            className: "wp-tip",
          })
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
  }, [rPts]);

  return <div ref={elRef} class="map" />;
}
