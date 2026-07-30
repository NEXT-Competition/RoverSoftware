// The state machine as a node graph: boxes you drag, wires you draw.
//
// Hand-rolled SVG, no graph library. That is not stubbornness — the dashboard
// ships as an offline kiosk with no CDN, the interactions here are drag, tap and
// pinch, and a library would bring a megabyte and a mouse-shaped API for the
// four gestures below.
//
// Pointer events throughout rather than mouse events, because the primary target
// is an iPad in a field. One consequence worth knowing: `setPointerCapture` is
// what makes a drag survive the pointer leaving the node, so a fast flick does
// not abandon the box halfway.
//
// Coordinates: nodes live in canvas space, the view is a {x, y, k} transform, and
// screen→canvas conversion goes through the SVG's own CTM so it stays correct
// under any zoom, scroll position or device pixel ratio.

import { useEffect, useRef, useState } from "preact/hooks";
import type { RoutineSpec } from "../../net/types.ts";
import { describeCondition } from "../../routines/vocab.ts";
import { Palette } from "./Palette.tsx";
import { groupForState, type VerbGroup } from "../../routines/vocab.ts";

/** The per-kind mark on a node, matching Palette.tsx's icons exactly. Drawn in
 *  ink rather than a signal colour: this is a category, not a condition. */
function CategoryMark({ group }: { group: VerbGroup | null }) {
  if (group === null) return null;
  const cx = 18, cy = 27;
  if (group === "mechanism") {
    return <circle class="rc-kind" cx={cx} cy={cy} r={5} />;
  }
  if (group === "logic") {
    return (
      <rect
        class="rc-kind hollow"
        x={cx - 5}
        y={cy - 5}
        width={10}
        height={10}
        rx={3}
      />
    );
  }
  return (
    <rect
      class={`rc-kind${group === "navigation" ? "" : " quiet"}`}
      x={cx - 5}
      y={cy - 5}
      width={10}
      height={10}
      rx={3}
    />
  );
}
import {
  connect,
  NODE_H,
  NODE_W,
  placed,
  relayout,
  selectedEdge,
  selectedState,
  setStatePosition,
} from "../../state/routines.ts";

const MIN_K = 0.35;
const MAX_K = 2.2;
const GRID = 20;

interface View {
  x: number;
  y: number;
  k: number;
}

interface Point {
  x: number;
  y: number;
}

/** The point half way ALONG a cubic bezier, not half way between its ends.
 *
 *  Labels hung on the chord midpoint drift off a bowed wire and land on top of
 *  whatever node the bow was routed around — which is exactly the case a bow
 *  exists to handle. This is the curve's own middle, so the label rides it. */
function curveMidpoint(p0: Point, p1: Point, p2: Point, p3: Point): Point {
  return {
    x: (p0.x + 3 * p1.x + 3 * p2.x + p3.x) / 8,
    y: (p0.y + 3 * p1.y + 3 * p2.y + p3.y) / 8,
  };
}

function curve(p0: Point, p1: Point, p2: Point, p3: Point) {
  return {
    d: `M ${p0.x} ${p0.y} C ${p1.x} ${p1.y}, ${p2.x} ${p2.y}, ${p3.x} ${p3.y}`,
    mid: curveMidpoint(p0, p1, p2, p3),
  };
}

/** A wire between two nodes, leaving the right edge and entering the left. */
function wire(from: Point, to: Point) {
  const start = { x: from.x + NODE_W, y: from.y + NODE_H / 2 };
  const end = { x: to.x, y: to.y + NODE_H / 2 };

  // A wire to a node that is level with or behind the source has to leave and
  // re-enter from the same side, or it draws straight through both boxes.
  if (end.x < start.x + 24) {
    const bow = Math.max(70, Math.abs(end.y - start.y) * 0.7 + 60);
    return curve(
      start,
      { x: start.x + bow, y: start.y },
      { x: end.x - bow, y: end.y },
      end,
    );
  }
  const reach = Math.max(48, (end.x - start.x) * 0.55);
  return curve(
    start,
    { x: start.x + reach, y: start.y },
    { x: end.x - reach, y: end.y },
    end,
  );
}

/** A self-transition: a loop out of the right side and back into the left. */
function selfWire(at: Point) {
  const y = at.y + NODE_H / 2 - 8;
  return curve(
    { x: at.x + NODE_W, y },
    { x: at.x + NODE_W + 64, y: y - 38 },
    { x: at.x - 64, y: y - 38 },
    { x: at.x, y },
  );
}

export function RoutineCanvas(
  { routine, live, problemStates }: {
    routine: RoutineSpec;
    live: string | null;
    problemStates: Set<string>;
  },
) {
  const svg = useRef<SVGSVGElement>(null);
  const [view, setView] = useState<View>({ x: 40, y: 40, k: 1 });
  /** An in-progress wire: which node it started from, and where the finger is. */
  const [linking, setLinking] = useState<
    { from: string; at: { x: number; y: number } } | null
  >(null);

  // Live during a gesture, deliberately outside state: a drag updates on every
  // pointer sample, and re-rendering the whole graph through a signal per sample
  // is how a node editor starts feeling like treacle on a tablet.
  const drag = useRef<
    | { kind: "node"; id: string; dx: number; dy: number }
    | { kind: "pan"; x: number; y: number; ox: number; oy: number }
    | null
  >(null);
  const pinch = useRef<{ dist: number; k: number } | null>(null);

  const nodes = placed.value;
  const byId = new Map(nodes.map((n) => [n.state.id, n]));

  /** Screen point -> canvas point, via the SVG's own transform. */
  const toCanvas = (event: PointerEvent | WheelEvent) => {
    const element = svg.current;
    if (!element) return { x: 0, y: 0 };
    const ctm = element.getScreenCTM();
    if (!ctm) return { x: 0, y: 0 };
    const point = element.createSVGPoint();
    point.x = event.clientX;
    point.y = event.clientY;
    const local = point.matrixTransform(ctm.inverse());
    return { x: (local.x - view.x) / view.k, y: (local.y - view.y) / view.k };
  };

  // --- gestures -------------------------------------------------------------

  const onNodeDown = (event: PointerEvent, id: string) => {
    event.stopPropagation();
    (event.currentTarget as Element).setPointerCapture(event.pointerId);
    const node = byId.get(id);
    if (!node) return;
    const at = toCanvas(event);
    drag.current = { kind: "node", id, dx: at.x - node.x, dy: at.y - node.y };
    selectedState.value = id;
    selectedEdge.value = null;
  };

  const onHandleDown = (event: PointerEvent, id: string) => {
    event.stopPropagation();
    (event.currentTarget as Element).setPointerCapture(event.pointerId);
    // Select the source as the wire is started, not just when it lands: the
    // condition you are about to set lives on THIS state, so the inspector has
    // to be showing it for `connect`'s highlight to land anywhere visible.
    selectedState.value = id;
    setLinking({ from: id, at: toCanvas(event) });
  };

  const onBackgroundDown = (event: PointerEvent) => {
    (event.currentTarget as Element).setPointerCapture(event.pointerId);
    drag.current = {
      kind: "pan",
      x: event.clientX,
      y: event.clientY,
      ox: view.x,
      oy: view.y,
    };
    selectedEdge.value = null;
  };

  const onMove = (event: PointerEvent) => {
    if (linking) {
      setLinking({ from: linking.from, at: toCanvas(event) });
      return;
    }
    const active = drag.current;
    if (!active) return;
    if (active.kind === "node") {
      const at = toCanvas(event);
      setStatePosition(active.id, at.x - active.dx, at.y - active.dy);
      return;
    }
    setView((v) => ({
      ...v,
      x: active.ox + (event.clientX - active.x),
      y: active.oy + (event.clientY - active.y),
    }));
  };

  const onUp = (event: PointerEvent) => {
    if (linking) {
      // Drop onto a node to wire them together. The hit test is done here rather
      // than with pointer events on the target, because the wire being dragged
      // sits over everything and would swallow them.
      const at = toCanvas(event);
      const target = nodes.find((n) =>
        at.x >= n.x && at.x <= n.x + NODE_W && at.y >= n.y && at.y <= n.y + NODE_H
      );
      if (target && target.state.id !== linking.from) {
        connect(linking.from, target.state.id);
      }
      setLinking(null);
    }
    drag.current = null;
    pinch.current = null;
  };

  const onWheel = (event: WheelEvent) => {
    event.preventDefault();
    const at = toCanvas(event);
    const k = Math.min(MAX_K, Math.max(MIN_K, view.k * (event.deltaY < 0 ? 1.12 : 1 / 1.12)));
    // Zoom about the cursor, not the corner: zooming about the origin makes the
    // thing you were looking at leave the screen.
    setView({ k, x: view.x + at.x * (view.k - k), y: view.y + at.y * (view.k - k) });
  };

  // Two-finger pinch. Tracked on the container rather than per-node so it works
  // wherever the fingers land.
  const onTouch = (event: TouchEvent) => {
    if (event.touches.length !== 2) {
      pinch.current = null;
      return;
    }
    event.preventDefault();
    const [a, b] = [event.touches[0], event.touches[1]];
    const dist = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
    if (!pinch.current) {
      pinch.current = { dist, k: view.k };
      return;
    }
    const k = Math.min(MAX_K, Math.max(MIN_K, pinch.current.k * (dist / pinch.current.dist)));
    setView((v) => ({ ...v, k }));
  };

  // Wheel has to be a non-passive listener to be preventable, which the JSX
  // attribute does not guarantee.
  useEffect(() => {
    const element = svg.current;
    if (!element) return;
    const handler = (e: WheelEvent) => onWheel(e);
    element.addEventListener("wheel", handler, { passive: false });
    return () => element.removeEventListener("wheel", handler);
  }, [view.k, view.x, view.y]);

  const zoom = (factor: number) =>
    setView((v) => ({ ...v, k: Math.min(MAX_K, Math.max(MIN_K, v.k * factor)) }));

  /** Frame everything, so a graph dragged off-screen is always recoverable.
   *
   *  Also run automatically when the routine changes — the layered layout uses
   *  a coordinate space wider than the pane as soon as there are three columns,
   *  and a node you cannot see is a node whose output handle you cannot drag.
   *  Panning to find it is not a reasonable thing to ask on first open. */
  const fit = () => {
    if (nodes.length === 0) return;
    const box = svg.current?.getBoundingClientRect();
    if (!box || box.width === 0) return;
    const minX = Math.min(...nodes.map((n) => n.x));
    const minY = Math.min(...nodes.map((n) => n.y));
    const maxX = Math.max(...nodes.map((n) => n.x + NODE_W));
    const maxY = Math.max(...nodes.map((n) => n.y + NODE_H));
    const pad = 40;
    const k = Math.min(
      MAX_K,
      Math.max(MIN_K, Math.min(
        (box.width - pad * 2) / Math.max(1, maxX - minX),
        (box.height - pad * 2) / Math.max(1, maxY - minY),
      )),
    );
    setView({
      k,
      x: pad - minX * k + (box.width - pad * 2 - (maxX - minX) * k) / 2,
      y: pad - minY * k + (box.height - pad * 2 - (maxY - minY) * k) / 2,
    });
  };

  // Frame the graph when the routine changes — but never afterwards, or it would
  // fight the pan and zoom the operator just set. A timeout of 0 so the SVG has
  // been laid out and `fit` has a real box to measure.
  useEffect(() => {
    const timer = setTimeout(fit, 0);
    return () => clearTimeout(timer);
  }, [routine.id]);

  /** Canvas coordinates of the middle of what is currently on screen. */
  const viewCentre = () => {
    const box = svg.current?.getBoundingClientRect();
    if (!box) return { x: 0, y: 0 };
    return {
      x: (box.width / 2 - view.x) / view.k - NODE_W / 2,
      y: (box.height / 2 - view.y) / view.k - NODE_H / 2,
    };
  };

  // --- edges ----------------------------------------------------------------

  interface Edge {
    key: string;
    from: string;
    index: number;
    label: string;
    hold: number;
    geometry: { d: string; mid: { x: number; y: number } } | null;
    dangling: boolean;
    to: string;
  }

  const edges: Edge[] = [];
  for (const node of nodes) {
    const transitions = node.state.transitions ?? [];
    transitions.forEach((transition, index) => {
      const target = byId.get(transition.to);
      const geometry = target
        ? (target.state.id === node.state.id ? selfWire(node) : wire(node, target))
        : null;
      edges.push({
        key: `${node.state.id}:${index}`,
        from: node.state.id,
        index,
        label: describeCondition(transition.when, transition as Record<string, unknown>),
        hold: transition.for_seconds ?? 0,
        geometry,
        dangling: !target,
        to: transition.to,
      });
    });
  }

  const chosen = selectedEdge.value;

  return (
    <div class="canvas-wrap">
      {/* Chrome lives in a strip ABOVE the canvas, not floating over it. A
          toolbar overlaying a canvas creates invisible dead zones exactly where
          the auto-layout likes to put nodes, and "my click did nothing" is an
          unfair puzzle to hand someone. */}
      <div class="canvas-bar">
        {/* Pick the capability first, get a state already configured for it —
            see Palette.tsx for why a blank box was the wrong default. */}
        <Palette at={viewCentre} />
        <span class="canvas-hint">
          Drag a box to move it · drag from its right edge onto another to wire
          them · tap a wire to set its condition
        </span>
        <div class="canvas-tools">
          <button type="button" class="btn ghost small" onClick={fit} title="Fit to view">
            ⤢
          </button>
          <button type="button" class="btn ghost small" onClick={() => zoom(1.2)} title="Zoom in">
            +
          </button>
          <button type="button" class="btn ghost small" onClick={() => zoom(1 / 1.2)} title="Zoom out">
            −
          </button>
          <button
            type="button"
            class="btn ghost small"
            onClick={relayout}
            title="Lay the graph out again from the start state"
          >
            Tidy
          </button>
        </div>
      </div>

      <svg
        ref={svg}
        class="routine-canvas"
        onPointerDown={onBackgroundDown}
        onPointerMove={onMove}
        onPointerUp={onUp}
        onPointerCancel={onUp}
        onTouchMove={onTouch}
        onTouchEnd={() => pinch.current = null}
        role="application"
        aria-label="State machine graph"
      >
        <defs>
          <marker
            id="rc-arrow"
            viewBox="0 0 10 10"
            refX="9"
            refY="5"
            markerWidth="7"
            markerHeight="7"
            orient="auto-start-reverse"
          >
            <path d="M0,0 L10,5 L0,10 z" fill="currentColor" />
          </marker>
          <pattern id="rc-grid" width={GRID} height={GRID} patternUnits="userSpaceOnUse">
            <circle cx={0.5} cy={0.5} r={0.5} />
          </pattern>
        </defs>

        {/* The grid lives in canvas space so it pans and zooms with the nodes,
            which is what makes dragging feel like moving a box on a surface
            rather than sliding a picture around. */}
        <g transform={`translate(${view.x} ${view.y}) scale(${view.k})`}>
          <rect
            class="rc-grid"
            x={-4000}
            y={-4000}
            width={8000}
            height={8000}
            fill="url(#rc-grid)"
          />

          {edges.map((edge) => {
            if (!edge.geometry) return null;
            const on = chosen != null && chosen.state === edge.from &&
              chosen.index === edge.index;
            return (
              <g
                key={edge.key}
                class={`rc-edge${on ? " on" : ""}${edge.dangling ? " dangling" : ""}`}
                onPointerDown={(e) => {
                  e.stopPropagation();
                  selectedEdge.value = { state: edge.from, index: edge.index };
                  selectedState.value = edge.from;
                }}
              >
                {/* A fat invisible copy underneath: a 2 px wire is impossible to
                    hit with a fingertip. */}
                <path class="rc-hit" d={edge.geometry.d} />
                <path class="rc-line" d={edge.geometry.d} marker-end="url(#rc-arrow)" />
                <g transform={`translate(${edge.geometry.mid.x} ${edge.geometry.mid.y})`}>
                  <rect
                    class="rc-label-bg"
                    x={-Math.max(22, edge.label.length * 3.6) - 6}
                    y={-9}
                    width={Math.max(44, edge.label.length * 7.2) + 12}
                    height={18}
                    rx={9}
                  />
                  <text class="rc-label" text-anchor="middle" y={4}>
                    {edge.label}
                    {edge.hold > 0 ? ` · ${edge.hold}s` : ""}
                  </text>
                </g>
              </g>
            );
          })}

          {linking && byId.has(linking.from) && (
            <path
              class="rc-linking"
              d={wire(byId.get(linking.from)!, {
                x: linking.at.x,
                y: linking.at.y - NODE_H / 2,
              }).d}
              marker-end="url(#rc-arrow)"
            />
          )}

          {nodes.map(({ state, x, y }) => {
            const isLive = state.id === live;
            const classes = [
              "rc-node",
              state.id === routine.start ? "start" : "",
              state.terminal ? "terminal" : "",
              isLive ? "live" : "",
              state.id === selectedState.value ? "selected" : "",
              problemStates.has(state.id) ? "problem" : "",
            ].filter(Boolean).join(" ");
            const mode = state.drive?.mode ?? "stop";
            const actions = (state.on_enter?.length ?? 0) +
              (state.on_tick?.length ?? 0) + (state.on_exit?.length ?? 0);
            return (
              <g key={state.id} class={classes} transform={`translate(${x} ${y})`}>
                <rect
                  class="rc-body"
                  width={NODE_W}
                  height={NODE_H}
                  rx={10}
                  onPointerDown={(e) => onNodeDown(e, state.id)}
                />
                {/* Category mark, in the same shapes the palette uses — a
                    circle for a mechanism, a filled square for driving, an
                    outline for counting. It says what KIND of step this is;
                    the stroke and fill still say what STATE it is in, and the
                    two never share a channel. */}
                <CategoryMark group={groupForState(state)} />
                <text class="rc-id" x={30} y={23}>{state.id}</text>
                <text class="rc-sub" x={30} y={41}>
                  {mode === "stop" || mode === "hold" ? mode : mode.replace(/_/g, " ")}
                  {/* What it aims at, in place of the action count when it has
                      one. On a graph, "align · bucket" is the difference between
                      reading what a routine does and having to open every box
                      that aims to find out. */}
                  {state.drive?.target
                    ? ` · ${state.drive.target}`
                    : actions > 0
                    ? ` · ${actions} action${actions === 1 ? "" : "s"}`
                    : ""}
                </text>
                {state.id === routine.start && (
                  <circle class="rc-badge start" cx={NODE_W - 13} cy={13} r={4} />
                )}
                {state.terminal && (
                  <rect class="rc-badge terminal" x={NODE_W - 17} y={9} width={8} height={8} rx={1} />
                )}
                {/* The output handle. Terminal states have no outgoing wires, so
                    they do not get one — the shape of the node says so. */}
                {!state.terminal && (
                  <circle
                    class="rc-handle"
                    cx={NODE_W}
                    cy={NODE_H / 2}
                    r={9}
                    onPointerDown={(e) => onHandleDown(e, state.id)}
                  />
                )}
              </g>
            );
          })}
        </g>
      </svg>

    </div>
  );
}
