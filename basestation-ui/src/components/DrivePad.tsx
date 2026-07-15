import { useRef, useState } from "preact/hooks";
import { selected } from "../net/ws.ts";
import { padInput, releaseDrive } from "../net/input.ts";

// On-screen joystick. Reports a unit vector where up = +throttle and
// right = +steer, matching the gamepad convention the server uses. Deadzone
// and rate-limiting live downstream in drive.ts; this just tracks the thumb.
export function DrivePad() {
  const ref = useRef<HTMLDivElement>(null);
  const [nub, setNub] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [active, setActive] = useState(false);
  const disabled = selected.value == null;

  function compute(clientX: number, clientY: number) {
    const el = ref.current;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const maxR = rect.width * 0.34; // travel radius for the nub
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > maxR) {
      dx = (dx / dist) * maxR;
      dy = (dy / dist) * maxR;
    }
    setNub({ x: dx, y: dy });
    // Normalize to [-1, 1]; invert Y so pushing up drives forward.
    padInput.value = { throttle: -(dy / maxR), steer: dx / maxR };
  }

  function onDown(e: PointerEvent) {
    if (disabled) return;
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
    setActive(true);
    navigator.vibrate?.(10); // Android only; silent no-op on iOS
    compute(e.clientX, e.clientY);
  }
  function onMove(e: PointerEvent) {
    if (!active) return;
    compute(e.clientX, e.clientY);
  }
  function onUp() {
    if (!active) return;
    setActive(false);
    setNub({ x: 0, y: 0 });
    releaseDrive(); // hard zero immediately
  }

  return (
    <div
      ref={ref}
      class={`drivepad${active ? " active" : ""}${disabled ? " disabled" : ""}`}
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
      <div class="pad-label">{disabled ? "select a robot" : "drive"}</div>
    </div>
  );
}
