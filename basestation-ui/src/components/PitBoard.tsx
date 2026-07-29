import { selectedRobot } from "../net/ws.ts";

/**
 * The three numbers an operator reads at distance, at poster scale.
 *
 * A pit board is held over a wall so a driver going past can read it in one
 * look. This is the same job: the values you glance at from across a bench
 * while your hands are busy, set far larger than a document would justify.
 *
 * Throttle, heading and link, because those three exist on EVERY robot — the
 * simulator and real hardware both report `left`/`right`, `heading` and a frame
 * age. GPS speed and satellites were the obvious picks and the wrong ones: the
 * simulator sends no `gps` object at all, and a headline number that is blank
 * on the demo path is a headline number nobody trusts. Battery has the mirror
 * problem — the sim reports it, real rovers don't emit it yet.
 *
 * Deliberately three. DESIGN.md caps the screen at three display numerals: a
 * pit board with ten numbers is a spreadsheet, and nobody reads a spreadsheet
 * at speed.
 */
function Cell(
  { k, value, unit }: { k: string; value: string | null; unit?: string },
) {
  // A value the robot has never reported is shown, but visibly not real — and
  // NOT as an em-dash at display scale, which reads as a redaction bar rather
  // than as "no reading". It drops to label scale instead.
  const unknown = value == null;
  return (
    <div class={`pb-cell${unknown ? " unknown" : ""}`}>
      <span class="pb-key">
        {k}
        {unit && <span class="pb-unit">{unit}</span>}
      </span>
      <span class="pb-val">
        {unknown ? <span class="pb-none">no data</span> : value}
      </span>
    </div>
  );
}

export function PitBoard() {
  const r = selectedRobot.value;
  if (!r) return null;
  // What the drivetrain actually applied, averaged across the two tracks. The
  // robot reports it back, so it is a measurement, not the command we sent.
  const throttle = r.left == null || r.right == null
    ? null
    : (r.left + r.right) / 2;
  return (
    <div class="pitboard">
      {/* Three glyphs is the budget: a rail column is ~104px and a numeral that
          overruns it collides with its neighbour. Percent, whole degrees and
          one decimal of age all fit; two decimals of throttle did not. */}
      <Cell
        k="thr"
        unit="%"
        value={throttle == null ? null : Math.round(throttle * 100).toString()}
      />
      <Cell
        k="hdg"
        unit="°"
        value={r.heading == null ? null : Math.round(r.heading).toString()}
      />
      <Cell
        k="link"
        unit="s"
        value={!r.online ? null : r.age == null ? "0.0" : r.age.toFixed(1)}
      />
    </div>
  );
}
