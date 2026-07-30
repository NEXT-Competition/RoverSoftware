// What a PID loop is actually doing, above the gains that shape it.
//
// A PID is the one part of this robot you cannot tune by watching it. "It
// wobbles" does not say whether kp is too high or kd is doing nothing, and the
// loop runs fifty times a second with every intermediate value thrown away. So:
// two plots, and they are two rather than one on purpose.
//
//   TRACKING, in the loop's own units — where it is aiming, where it is
//   pointing, and the gap between them. Degrees for heading; a fraction of the
//   lens for alignment. The same units the gains are expressed in.
//
//   OUTPUT, in command units — the steering command, split into the P, I and D
//   contributions that add up to it.
//
// Putting those on one chart would need two y-scales, and a dual-axis chart is
// a picture whose crossings mean nothing: you can make any two lines touch by
// choosing the scales. Two plots, one scale each, stacked so time lines up.
//
// The colour work: P/I/D carry the only identity-by-colour in the panel, on the
// three leading slots of the validated categorical ramp (all-pairs safe for
// colour-vision deficiency in both renditions). Everything else is structural —
// the setpoint is a dashed reference, the measurement and the output are ink,
// and the error is the shaded gap rather than a fourth hue.

import { useState } from "preact/hooks";
import {
  clearPidHistory,
  measured,
  pidHistory,
  type PidSample,
  samplesFor,
} from "../../state/pid.ts";
import { fieldValue } from "../../state/settings.ts";

/** Plot box, in user units. Rendered with width:100% and a fixed aspect, so
 *  strokes scale evenly rather than smearing the way preserveAspectRatio:none
 *  does to a 2px line. */
const W = 620;
const H = 96;
const PAD_L = 44;
const PAD_R = 8;
const PAD_Y = 10;

interface Series {
  key: string;
  label: string;
  /** A CSS custom property, so both renditions come from the stylesheet. */
  stroke: string;
  value: (s: PidSample) => number;
  dash?: string;
  width?: number;
  /** Legend-only: this one is the shaded band, not a stroke. */
  band?: boolean;
}

const TRACKING: Series[] = [
  { key: "m", label: "measured", stroke: "var(--ink)", value: measured, width: 2 },
  {
    key: "sp",
    label: "setpoint",
    stroke: "var(--muted)",
    value: (s) => s.sp,
    dash: "5 4",
    width: 1.5,
  },
];

/** The error earns a legend row even though it is drawn as the shaded gap
 *  rather than a line — it is the number people read, and the swatch says which
 *  mark on the plot it refers to. */
const TRACKING_LEGEND: Series[] = [
  ...TRACKING,
  { key: "e", label: "error", stroke: "var(--muted)", value: (s) => s.e, band: true },
];

const OUTPUT: Series[] = [
  { key: "o", label: "output", stroke: "var(--ink)", value: (s) => s.o, width: 2 },
  { key: "p", label: "P", stroke: "var(--chart-1)", value: (s) => s.p, width: 2 },
  { key: "i", label: "I", stroke: "var(--chart-2)", value: (s) => s.i, width: 2 },
  { key: "d", label: "D", stroke: "var(--chart-3)", value: (s) => s.d, width: 2 },
];

/**
 * A y-range that contains the data, never collapses, and does not jitter.
 *
 * Padded by a tenth and snapped outward to a round step: a range fitted exactly
 * to the data puts the extremes on the frame, and one recomputed to the last
 * decimal every frame makes a still loop look like it is breathing.
 *
 * `includeZero` is per chart, not a global habit. The output chart must show
 * zero, because the sign of a steering command is which way the robot turns and
 * the P/I/D terms straddle it. The tracking chart must NOT: a rover holding a
 * bearing of 34° with zero forced into view spends six-sevenths of the plot on
 * empty degrees nobody is asking about, which is exactly the range you cannot
 * see the tracking error in.
 */
function extent(
  values: number[],
  floor: number,
  includeZero: boolean,
): [number, number] {
  let lo = Math.min(...values);
  let hi = Math.max(...values);
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [-floor, floor];
  if (includeZero) {
    lo = Math.min(lo, 0);
    hi = Math.max(hi, 0);
  }
  // A loop sitting still has no span of its own; give it one rather than
  // magnifying float noise into a mountain range.
  if (hi - lo < floor) {
    const mid = (hi + lo) / 2;
    lo = mid - floor / 2;
    hi = mid + floor / 2;
  }
  const span = hi - lo;
  const pad = span * 0.1;
  const step = Math.pow(10, Math.floor(Math.log10(span))) / 2;
  return [
    Math.floor((lo - pad) / step) * step,
    Math.ceil((hi + pad) / step) * step,
  ];
}

/** Two decimals below ten, one above — enough to see a gain move without the
 *  legend changing width as the value crosses a power of ten — and significant
 *  figures rather than decimals once a gain is too small for either. */
function format(value: number): string {
  const abs = Math.abs(value);
  if (abs >= 10) return value.toFixed(1);
  if (abs >= 0.01 || abs === 0) return value.toFixed(2);
  // Below a hundredth, two decimals is not a rounded number — it is "0.00" for
  // every value, which is the one thing a readout beside a curve must not be.
  // Loops whose error is in real units have gains this small BY CONSTRUCTION:
  // the heading loops see degrees and the wheel-speed loop sees RPM, so their
  // useful gains live two or three decimals down. Show enough digits to be a
  // digit, and drop the zeros that padding adds.
  return value.toPrecision(2).replace(/0+$/, "").replace(/\.$/, "");
}

/** An axis label with enough precision to be a DIFFERENT number from its
 *  neighbour. Precision from the span, not the magnitude: a rover holding 241°
 *  inside a degree of range labelled by magnitude prints "241" at both ends of
 *  the axis, which is not a scale — it is two smudges. */
function formatTick(value: number, span: number): string {
  const decimals = span >= 20 ? 0 : span >= 2 ? 1 : span >= 0.2 ? 2 : 3;
  return value.toFixed(decimals);
}

function Plot(
  { samples, series, unit, floor, band, zero, cursor, onCursor }: {
    samples: PidSample[];
    series: Series[];
    unit: string;
    /** Smallest y-span worth showing, so a loop sitting at zero does not get
     *  magnified into noise. */
    floor: number;
    /** Shade between the setpoint and the measurement — the error, as the gap
     *  it actually is, rather than as a fourth line. */
    band?: boolean;
    /** Keep zero in view. True where the sign is the point (which way the robot
     *  turns); false where the data lives far from it (a bearing of 34°). */
    zero?: boolean;
    cursor: number | null;
    onCursor: (index: number | null) => void;
  },
) {
  const n = samples.length;
  const x = (index: number) =>
    PAD_L + (n <= 1 ? W - PAD_L - PAD_R : ((W - PAD_L - PAD_R) * index) / (n - 1));

  const all = series.flatMap((s) => samples.map(s.value));
  const [lo, hi] = extent(all, floor, !!zero);
  const y = (value: number) =>
    PAD_Y + (H - 2 * PAD_Y) * (1 - (value - lo) / (hi - lo || 1));

  // The ends always; zero only when it is in view and not sitting on top of an
  // end label. Two numbers 4px apart are not two numbers, they are a smudge.
  const ticks = [hi, lo].concat(
    lo < 0 && hi > 0 && Math.min(Math.abs(y(0) - y(hi)), Math.abs(y(0) - y(lo))) > 12
      ? [0]
      : [],
  );

  const path = (of: (s: PidSample) => number) =>
    samples.map((s, index) => `${index ? "L" : "M"}${x(index).toFixed(1)} ${y(of(s)).toFixed(1)}`)
      .join(" ");

  return (
    <svg
      class="pid-plot"
      viewBox={`0 0 ${W} ${H}`}
      role="img"
      aria-label={`${series.map((s) => s.label).join(", ")} over the last ${n} samples, in ${unit}`}
      onPointerMove={(e) => {
        // A single sample has no span to interpolate across; dividing by n-1
        // there yields NaN, which silently reads as "no cursor" rather than as
        // the first sample the pointer is actually over.
        if (n <= 1) return onCursor(n - 1);
        const box = (e.currentTarget as SVGSVGElement).getBoundingClientRect();
        const at = ((e.clientX - box.left) / box.width) * W;
        const index = Math.round(((at - PAD_L) / (W - PAD_L - PAD_R)) * (n - 1));
        onCursor(Math.max(0, Math.min(n - 1, index)));
      }}
      onPointerLeave={() => onCursor(null)}
    >
      {/* Axis: the two ends and zero, no more. A grid dense enough to read
          values off is a grid that competes with the data for attention. */}
      {ticks.map((tick) => (
        <g key={tick}>
          <line class="pid-grid" x1={PAD_L} x2={W - PAD_R} y1={y(tick)} y2={y(tick)} />
          <text class="pid-tick" x={PAD_L - 6} y={y(tick) + 3} text-anchor="end">
            {formatTick(tick, hi - lo)}
          </text>
        </g>
      ))}

      {/* The error, as the gap it is: out along the measurement, back along the
          setpoint, closed. Reading "how far off is it" off the distance between
          two lines is what everybody does anyway; shading it says so. */}
      {band && n > 1 && (
        <path
          class="pid-band"
          d={path(measured) + " " +
            samples.map((_, index) => {
              const back = n - 1 - index;
              return `L${x(back).toFixed(1)} ${y(samples[back].sp).toFixed(1)}`;
            }).join(" ") + " Z"}
        />
      )}

      {n > 1 && series.map((s) => (
        <path
          key={s.key}
          d={path(s.value)}
          fill="none"
          stroke={s.stroke}
          stroke-width={s.width ?? 2}
          stroke-dasharray={s.dash}
          stroke-linejoin="round"
          stroke-linecap="round"
        />
      ))}

      {cursor != null && cursor < n && (
        <>
          <line class="pid-cursor" x1={x(cursor)} x2={x(cursor)} y1={PAD_Y} y2={H - PAD_Y} />
          {series.map((s) => (
            <circle
              key={s.key}
              cx={x(cursor)}
              cy={y(s.value(samples[cursor]))}
              r={3.5}
              fill={s.stroke}
              class="pid-dot"
            />
          ))}
        </>
      )}
    </svg>
  );
}

function Legend(
  { series, sample, unit }: { series: Series[]; sample: PidSample | null; unit: string },
) {
  // Values live in the legend rather than on the marks: a number on every point
  // is unreadable, and one number per series — the value under the cursor, or
  // the latest — is what anybody actually reads off. It doubles as the visible
  // label that keeps identity off colour alone.
  return (
    <ul class="pid-legend">
      {series.map((s) => (
        <li key={s.key}>
          <span
            class={`pid-swatch${s.dash ? " dashed" : ""}${s.band ? " band" : ""}`}
            style={`--swatch:${s.stroke}`}
          />
          <span class="pid-legend-label">{s.label}</span>
          <span class="pid-legend-value">
            {sample ? format(s.value(sample)) : "—"}
            <span class="pid-unit">{unit}</span>
          </span>
        </li>
      ))}
    </ul>
  );
}

export function PidGraph({ robotId, loop, unit }: {
  robotId: string;
  /** The loop's tuning path — "align.pid", "nav.heading_pid". It is also the
   *  prefix of its gain fields, which is how the graph and the knobs below it
   *  are guaranteed to be talking about the same loop. */
  loop: string;
  /** What the setpoint and error are measured in. */
  unit: string;
}) {
  const [cursor, setCursor] = useState<number | null>(null);
  // Read the signal so this re-renders as frames land.
  pidHistory.value;
  const samples = samplesFor(robotId, loop);
  const at = cursor != null && cursor < samples.length
    ? samples[cursor]
    : samples[samples.length - 1] ?? null;

  // Read straight through, not memoised: `fieldValue` shows a gain you have
  // typed but not yet committed, and the whole point of putting the numbers
  // beside the curve is that they are the numbers in play right now.
  const gains = ["kp", "ki", "kd"].map((k) => ({
    key: k,
    value: fieldValue(`${loop}.${k}`),
  }));

  if (samples.length === 0) {
    return (
      <p class="hint pid-empty">
        No trace yet. Switch on <strong>Graph the loops</strong> under Control
        loop, then put the robot in the mode that runs this one — a loop nobody
        is running has nothing to say about how it behaves.
      </p>
    );
  }

  const seconds = samples.length > 1
    ? Math.round((samples[samples.length - 1].t - samples[0].t) / 1000)
    : 0;

  return (
    <figure class="pid-graph">
      <figcaption class="pid-head">
        <span class="pid-gains">
          {gains.map((g) => (
            <span key={g.key} class="pid-gain">
              <b>{g.key}</b>
              {typeof g.value === "number" ? format(g.value) : "—"}
            </span>
          ))}
        </span>
        <span class="pid-meta">
          {at?.sat && (
            <span class="pid-sat" title="The output is pinned at its limit, so more gain changes nothing.">
              at limit
            </span>
          )}
          <span>{seconds}s</span>
          <button
            type="button"
            class="btn ghost small"
            title="Throw away the history, so the next curve is the one your new gains produced"
            onClick={() => clearPidHistory(robotId)}
          >
            Clear
          </button>
        </span>
      </figcaption>

      <div class="pid-pane">
        <span class="pid-axis-label">tracking · {unit}</span>
        <Plot
          samples={samples}
          series={TRACKING}
          unit={unit}
          floor={1e-3}
          band
          cursor={cursor}
          onCursor={setCursor}
        />
        <Legend series={TRACKING_LEGEND} sample={at} unit={unit} />
      </div>

      <div class="pid-pane">
        <span class="pid-axis-label">output · P + I + D</span>
        <Plot
          samples={samples}
          series={OUTPUT}
          unit=""
          floor={0.05}
          zero
          cursor={cursor}
          onCursor={setCursor}
        />
        <Legend series={OUTPUT} sample={at} unit="" />
      </div>

      <p class="hint pid-note">
        Sampled at the telemetry rate, not the {" "}
        control rate — enough to see drift, bias, a term doing nothing, or a
        loop stuck at its limit. A wobble faster than half the telemetry rate
        aliases; raise <strong>Telemetry rate</strong> while you look at it.
      </p>
    </figure>
  );
}
