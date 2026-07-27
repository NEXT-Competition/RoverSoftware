import { signal } from "@preact/signals";

/** Field view: normal orientation, or rotated another 180° (see MapView.tsx). */
export const mapFlipped = signal(false);

export function toggleMapFlip(): void {
  mapFlipped.value = !mapFlipped.value;
}
