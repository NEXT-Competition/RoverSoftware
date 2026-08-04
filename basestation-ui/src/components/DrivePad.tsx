import { useRef, useState } from "preact/hooks";
import { selected } from "../net/ws.ts";
import { padInput, releaseDrive } from "../net/input.ts";

type Axis = "throttle" | "steer";
type Nub = { x: number; y: number };

interface AxisPadProps {
  axis: Axis;
  disabled: boolean;
  onChange: (axis: Axis, value: number, active: boolean) => void;
}

// One half of the two-stick touch control. Throttle is deliberately locked to
// the vertical axis and steering to the horizontal axis, so diagonal thumb
// drift can never change the other command.
function AxisPad({ axis, disabled, onChange }: AxisPadProps) {
  const ref = useRef<HTMLDivElement>(null);
  const active = useRef(false);
  const [nub, setNub] = useState<Nub>({ x: 0, y: 0 });

  function compute(clientX: number, clientY: number) {
    const el = ref.current;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const maxR = rect.width * 0.34;
    const raw = axis === "throttle"
      ? -(clientY - (rect.top + rect.height / 2)) / maxR
      : (clientX - (rect.left + rect.width / 2)) / maxR;
    const value = Math.max(-1, Math.min(1, raw));
    setNub(axis === "throttle"
      ? { x: 0, y: -value * maxR }
      : { x: value * maxR, y: 0 });
    onChange(axis, value, true);
  }

  function onDown(e: PointerEvent) {
    if (disabled) return;
    (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
    active.current = true;
    navigator.vibrate?.(10);
    compute(e.clientX, e.clientY);
  }

  function onMove(e: PointerEvent) {
    if (active.current) compute(e.clientX, e.clientY);
  }

  function onUp() {
    if (!active.current) return;
    active.current = false;
    setNub({ x: 0, y: 0 });
    onChange(axis, 0, false);
  }

  return (
    <div
      ref={ref}
      class={`drivepad drivepad-${axis}${active.current ? " active" : ""}${disabled ? " disabled" : ""}`}
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
      onPointerCancel={onUp}
      onLostPointerCapture={onUp}
    >
      <div class="crosshair" />
      <div class="ring" />
      <div
        class="nub"
        style={`transform:translate(${nub.x}px, ${nub.y}px)`}
      />
      <div class="pad-label">
        {disabled ? "select a robot" : axis}
      </div>
    </div>
  );
}

// Two independent pointers feed one throttle/steer command. Keeping the active
// flags here matters: lifting the steering thumb must center steering without
// stopping throttle that is still held on the other pad (and vice versa).
export function DrivePad() {
  const values = useRef({ throttle: 0, steer: 0 });
  const held = useRef({ throttle: false, steer: false });
  const disabled = selected.value == null;

  function onChange(axis: Axis, value: number, active: boolean) {
    values.current[axis] = value;
    held.current[axis] = active;
    if (held.current.throttle || held.current.steer) {
      padInput.value = { ...values.current };
    } else {
      releaseDrive();
    }
  }

  return (
    <>
      <AxisPad axis="throttle" disabled={disabled} onChange={onChange} />
      <AxisPad axis="steer" disabled={disabled} onChange={onChange} />
    </>
  );
}
