"""Speech to text, on this machine, with no internet.

faster-whisper (CTranslate2) rather than the browser's SpeechRecognition API,
and the reason is the same one that put an .mbtiles cache in tiles.py: Chrome
implements SpeechRecognition by streaming audio to Google, so it stops working
at exactly the moment a base station is most needed — a field with no signal.
A voice interface that silently dies when the hotspot drops is worse than no
voice interface, because somebody will rely on it.

--- The audio contract ---
The browser sends **16 kHz mono signed 16-bit little-endian PCM**, raw, with no
container (see basestation-ui/src/net/audio.ts). That is deliberate: WebM/Opus
would need a demuxer and an Opus decoder on the base station, which on a
Raspberry Pi means dragging in PyAV or shelling out to ffmpeg for every
utterance. The browser's AudioContext already resamples to any rate we ask for,
so asking for Whisper's native 16 kHz costs nothing and skips the entire
decoding problem.

--- Push to talk ---
There is no voice-activity detection here. The operator holds a key or a button,
and the utterance ends when they let go. In a pit with three teams shouting and
a generator running, VAD produces commands nobody gave — and the failure mode of
an accidental command is a moving robot. Holding a button is also how every
other radio in the building works, so it needs no explaining.

--- Absence is normal ---
faster-whisper is an optional dependency (see requirements.txt). Everything here
degrades to a clear "speech recognition isn't installed" rather than an import
error at startup: typing into the command view still works, MCP still works, and
the fast path still stops rovers.
"""

from __future__ import annotations

import threading
from typing import Any, List, Optional, Tuple

SAMPLE_RATE = 16000
"""Whisper's native rate. Anything else costs a resample for no benefit."""

# An utterance longer than this is not a command, it is a conversation happening
# near the microphone. Capped so a stuck push-to-talk key can't grow a buffer
# without bound or hand Whisper a two-minute clip to chew on.
MAX_SECONDS = 20.0
MAX_SAMPLES = int(SAMPLE_RATE * MAX_SECONDS)

# Below this, the operator tapped the key rather than said something. Whisper
# hallucinates confidently on very short clips — "you", "thank you", and
# "Thanks for watching!" are its classic outputs on silence — so we refuse
# before asking rather than filtering its answer afterwards.
MIN_SECONDS = 0.25
MIN_SAMPLES = int(SAMPLE_RATE * MIN_SECONDS)

# Whisper's known hallucinations on silence or noise. Matched after
# transcription and discarded: these reach the intent layer as garbage that the
# model then has to be trusted to reject, and it is much safer to never send them.
_HALLUCINATIONS = {
    "you", "thank you", "thanks for watching", "thanks for watching!",
    "thank you for watching", "bye", "bye.", "so", ".", "...", "[blank_audio]",
    "subs by www.zeoranger.co.uk", "subtitles by the amara.org community",
    "please subscribe", "thank you.", "you.",
}

DEFAULT_MODEL = "base.en"
"""Good enough for a 20-word vocabulary and roughly 4x faster than `small` on a
Pi 5. `tiny.en` is faster still but starts confusing rover numbers, which is the
one thing this has to get right. Override with RS_STT_MODEL."""


class STTUnavailable(RuntimeError):
    """faster-whisper isn't installed, or the model could not be loaded."""


class Transcriber:
    """Wraps one loaded Whisper model. Thread-safe; blocking.

    Loading is lazy and happens off the request path (see `preload`) because the
    first call pulls ~150 MB from disk and, on a fresh machine, downloads it —
    which must not be what happens the first time somebody holds the mic key
    with a rover on the field.
    """

    def __init__(self, model_name: str = DEFAULT_MODEL, device: str = "auto",
                 compute_type: str = "int8"):
        self.model_name = model_name
        self.device = device
        # int8 rather than float16: this runs on a Pi or a laptop CPU, where
        # int8 is 2-3x faster and the accuracy difference on short commands is
        # not measurable.
        self.compute_type = compute_type
        self._model: Optional[Any] = None
        self._lock = threading.Lock()
        self._error: Optional[str] = None

    # --- availability ------------------------------------------------------

    @property
    def loaded(self) -> bool:
        return self._model is not None

    def status(self) -> dict:
        return {"available": self._model is not None,
                "model": self.model_name,
                "error": self._error,
                "sample_rate": SAMPLE_RATE}

    def preload(self) -> Optional[str]:
        """Load the model now. Returns an error string, or None on success.

        Called from a background thread at startup so the base station comes up
        immediately and the command view can report "loading speech model…"
        rather than appearing to hang.
        """
        try:
            self._ensure()
            return None
        except STTUnavailable as e:
            return str(e)

    def _ensure(self) -> Any:
        if self._model is not None:
            return self._model
        with self._lock:
            if self._model is not None:
                return self._model
            try:
                from faster_whisper import WhisperModel  # type: ignore
            except ImportError:
                self._error = ("faster-whisper is not installed — "
                               "`pip install faster-whisper` for voice input "
                               "(typing still works)")
                raise STTUnavailable(self._error)
            try:
                self._model = WhisperModel(self.model_name, device=self.device,
                                           compute_type=self.compute_type)
            except Exception as e:
                self._error = f"could not load Whisper model {self.model_name!r}: {e}"
                raise STTUnavailable(self._error)
            self._error = None
            return self._model

    # --- the one call that matters ----------------------------------------

    def transcribe(self, pcm: bytes, vocabulary: Optional[List[str]] = None
                   ) -> Tuple[str, float]:
        """Raw 16 kHz mono s16le PCM -> (text, mean confidence 0..1).

        `vocabulary` is fed to Whisper as an initial prompt. It is worth more
        than it looks: proper nouns like "rover2" and "bucket A" are exactly what
        a general model gets wrong, and biasing it with the words that are
        currently legal turns "Rover to" into "rover2" without any post-hoc
        correction.
        """
        model = self._ensure()

        try:
            import numpy as np  # type: ignore
        except ImportError:
            raise STTUnavailable("numpy is not installed — required for voice input")

        if len(pcm) < MIN_SAMPLES * 2:
            return "", 0.0

        audio = np.frombuffer(pcm[: MAX_SAMPLES * 2], dtype="<i2")
        # Whisper wants float32 in [-1, 1]; 32768 rather than 32767 so a full-
        # scale negative sample doesn't clip to slightly over -1.
        audio = audio.astype(np.float32) / 32768.0

        prompt = None
        if vocabulary:
            # Phrased as a sentence rather than a bare list — Whisper's initial
            # prompt is conditioning text, and a comma-separated list biases it
            # toward transcribing commas.
            prompt = ("Rover commands: " + ", ".join(vocabulary[:40]) + ".")

        segments, _info = model.transcribe(
            audio,
            language="en",
            # Greedy. This is a 20-word vocabulary, and beam search costs 2-3x
            # for no measurable gain on utterances this short.
            beam_size=1,
            # No context carried between utterances: each command is independent,
            # and a previous command in the context window is precisely what
            # makes Whisper repeat it when the next one is quiet.
            condition_on_previous_text=False,
            initial_prompt=prompt,
            vad_filter=False,  # push-to-talk already bounds the utterance
        )

        parts: List[str] = []
        logprobs: List[float] = []
        for seg in segments:
            parts.append(seg.text)
            if getattr(seg, "avg_logprob", None) is not None:
                logprobs.append(seg.avg_logprob)

        text = " ".join(p.strip() for p in parts).strip()
        if _is_hallucination(text):
            return "", 0.0

        # avg_logprob is roughly (-inf, 0]; -1.0 is already poor for clean
        # speech. Mapped to 0..1 only so the UI can show a bar — it is a hint
        # for the operator, never a threshold anything is gated on.
        import math
        conf = 0.0
        if logprobs:
            conf = max(0.0, min(1.0, math.exp(sum(logprobs) / len(logprobs))))
        return text, conf


def _is_hallucination(text: str) -> bool:
    stripped = (text or "").strip().lower().rstrip(".!?")
    if not stripped:
        return True
    return stripped in {h.rstrip(".!?") for h in _HALLUCINATIONS}


class Utterance:
    """Accumulates PCM chunks for one push-to-talk press.

    One per WebSocket connection, so two tablets can hold their mic buttons at
    the same time without interleaving audio into one another's command.
    """

    def __init__(self):
        self._chunks: List[bytes] = []
        self._bytes = 0
        self.active = False

    def begin(self) -> None:
        self._chunks.clear()
        self._bytes = 0
        self.active = True

    def feed(self, chunk: bytes) -> None:
        if not self.active or not chunk:
            return
        # Hard cap. A tablet whose pointerup event was lost to a dropped socket
        # would otherwise stream until the process runs out of memory.
        if self._bytes >= MAX_SAMPLES * 2:
            return
        self._chunks.append(chunk)
        self._bytes += len(chunk)

    def end(self) -> bytes:
        self.active = False
        pcm = b"".join(self._chunks)
        self._chunks.clear()
        self._bytes = 0
        return pcm

    @property
    def seconds(self) -> float:
        return self._bytes / 2.0 / SAMPLE_RATE
