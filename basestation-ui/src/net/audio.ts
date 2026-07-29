// Push-to-talk microphone capture, as raw PCM the base station can feed
// straight to Whisper.
//
// The whole design goal is that the *server* needs no audio libraries. So this
// does the format conversion here, where the browser already has a resampler:
// an AudioContext opened at 16 kHz resamples the microphone for free, and a
// worklet converts float samples to signed 16-bit. What goes up the WebSocket
// is bare PCM — no container, no codec, nothing to demux on a Raspberry Pi.
// (basestation/command/stt.py is the other half of this contract.)
//
// --- Why push-to-talk ---
// No voice-activity detection anywhere. The operator holds a key or a button.
// In a pit with other teams shouting, VAD invents commands nobody gave, and the
// failure mode of an invented command is a moving robot. Holding a button is
// also how every handheld radio in the building works.
//
// --- Why the permission is requested up front ---
// getUserMedia shows a browser prompt. A prompt that appears the first time
// somebody holds the mic button eats that utterance and, on a kiosk with no
// keyboard, may not be dismissable at all. So the stream is opened once when
// the command view mounts, and held.

import { signal } from "@preact/signals";

/** Whisper's native rate — see basestation/command/stt.py::SAMPLE_RATE. */
const SAMPLE_RATE = 16000;

/** Samples per message. 1024 at 16 kHz is 64 ms — small enough that releasing
 *  the button doesn't strand a long tail of un-sent audio, large enough that we
 *  aren't posting a WebSocket frame every animation frame. */
const FRAME_SAMPLES = 1024;

/** Mic state, for the view to render. `denied` is separate from `error`: it is
 *  the one failure the operator can actually fix, and it needs different words. */
export type MicState = "idle" | "starting" | "ready" | "recording" | "denied" | "error";

export const micState = signal<MicState>("idle");
export const micError = signal<string | null>(null);
/** Rolling input level, 0..1. Purely so the operator can see the meter move —
 *  a mic that is muted at the OS level looks identical to a quiet room, and
 *  this is the only way to tell them apart before speaking a command. */
export const micLevel = signal(0);

// The worklet, inlined and loaded from a blob URL. Kept here rather than in a
// separate file under public/ because it must stay in lockstep with the format
// the server expects, and a copy that can drift is a copy that will.
const WORKLET_SOURCE = `
class PcmTap extends AudioWorkletProcessor {
  constructor() {
    super();
    this._buf = new Int16Array(${FRAME_SAMPLES});
    this._n = 0;
  }
  process(inputs) {
    const ch = inputs[0] && inputs[0][0];
    if (!ch) return true;
    let peak = 0;
    for (let i = 0; i < ch.length; i++) {
      const s = ch[i];
      const a = s < 0 ? -s : s;
      if (a > peak) peak = a;
      // Clamp before scaling: a sample slightly over 1.0 (which resampling can
      // produce) would otherwise wrap to a loud negative click.
      const c = s > 1 ? 1 : s < -1 ? -1 : s;
      this._buf[this._n++] = c < 0 ? c * 0x8000 : c * 0x7fff;
      if (this._n === ${FRAME_SAMPLES}) {
        // Copy, because the port transfers and we keep reusing the buffer.
        this.port.postMessage(this._buf.slice(0));
        this._n = 0;
      }
    }
    this.port.postMessage({ peak });
    return true;
  }
}
registerProcessor("pcm-tap", PcmTap);
`;

let context: AudioContext | null = null;
let stream: MediaStream | null = null;
let node: AudioWorkletNode | null = null;
let source: MediaStreamAudioSourceNode | null = null;
let sink: ((pcm: ArrayBuffer) => void) | null = null;
let recording = false;

/** Open the microphone and keep it. Safe to call repeatedly. */
export async function startMic(): Promise<boolean> {
  if (micState.value === "ready" || micState.value === "recording") return true;
  if (micState.value === "starting") return false;
  micState.value = "starting";
  micError.value = null;

  try {
    if (!navigator.mediaDevices?.getUserMedia) {
      throw new Error("this browser has no microphone API (needs HTTPS or localhost)");
    }
    stream = await navigator.mediaDevices.getUserMedia({
      audio: {
        // Let the browser clean up the signal. A base station sits next to
        // whining motors and a laptop fan, and Whisper is markedly better on
        // audio that has already had the steady part removed.
        echoCancellation: true,
        noiseSuppression: true,
        autoGainControl: true,
        channelCount: 1,
      },
    });

    // Asking for 16 kHz here is what removes the need for any resampling on
    // the base station. Browsers that refuse the rate still work — they hand
    // back their native rate and resample into the graph — so this is an
    // optimisation, not a requirement.
    context = new AudioContext({ sampleRate: SAMPLE_RATE });
    const url = URL.createObjectURL(
      new Blob([WORKLET_SOURCE], { type: "application/javascript" }),
    );
    try {
      await context.audioWorklet.addModule(url);
    } finally {
      URL.revokeObjectURL(url);
    }

    source = context.createMediaStreamSource(stream);
    node = new AudioWorkletNode(context, "pcm-tap");
    node.port.onmessage = (ev) => {
      const data = ev.data;
      if (data instanceof Int16Array) {
        // Only forwarded while the button is held. The graph keeps running
        // between utterances so the level meter stays live — that is how an
        // operator knows the mic works *before* giving an order.
        if (recording && sink) sink(data.buffer as ArrayBuffer);
        return;
      }
      if (data && typeof data.peak === "number") {
        // Decay slowly, rise instantly: a meter that tracks the peak exactly
        // reads as noise rather than as speech.
        micLevel.value = Math.max(data.peak, micLevel.value * 0.82);
      }
    };
    source.connect(node);
    // Not connected to the destination: routing the microphone to the speakers
    // on a base station next to the operator is a feedback loop.

    micState.value = "ready";
    return true;
  } catch (e) {
    stopMic();
    const err = e as DOMException;
    if (err?.name === "NotAllowedError" || err?.name === "SecurityError") {
      micState.value = "denied";
      micError.value = "Microphone permission was refused — allow it in the browser, then press Retry.";
    } else if (err?.name === "NotFoundError") {
      micState.value = "error";
      micError.value = "No microphone is attached to this machine.";
    } else {
      micState.value = "error";
      micError.value = `${err?.message || e}`;
    }
    return false;
  }
}

export function stopMic(): void {
  recording = false;
  try {
    node?.port.close();
    node?.disconnect();
    source?.disconnect();
    stream?.getTracks().forEach((t) => t.stop());
    void context?.close();
  } catch { /* tearing down a half-built graph is not an error */ }
  node = source = null;
  stream = null;
  context = null;
  micLevel.value = 0;
  if (micState.value !== "denied" && micState.value !== "error") micState.value = "idle";
}

/** Where captured PCM goes. Set once by the command view (to ws.sendAudio). */
export function setAudioSink(fn: ((pcm: ArrayBuffer) => void) | null): void {
  sink = fn;
}

/** Button down. Returns false when the mic isn't open, so the caller can say so. */
export function beginUtterance(): boolean {
  if (micState.value !== "ready") return false;
  // Chrome suspends an AudioContext created before any user gesture. Resuming
  // on the press is the reliable moment — it *is* the gesture.
  void context?.resume();
  recording = true;
  micState.value = "recording";
  return true;
}

/** Button up. */
export function endUtterance(): void {
  if (!recording) return;
  recording = false;
  micState.value = "ready";
  micLevel.value = 0;
}

export const isRecording = () => recording;
