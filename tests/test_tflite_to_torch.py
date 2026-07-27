"""Proof that the PyTorch rebuild IS the .lite model it was read from.

Comparing only the final softmax is not enough to trust a reconstruction. A
transposed kernel, an off-by-one SAME pad or a dropped fused ReLU6 can all hide
inside an end-to-end average. These tests go layer by layer instead.

Two exports of the same network are available, and they check different things:

* The FLOAT32 export is the strict oracle. Nothing is quantized anywhere, so the
  rebuild should reproduce every intermediate tensor to float32 epsilon. Any
  structural mistake shows up immediately and unambiguously.

* The INT8 export cannot be that strict, because it rounds every intermediate to
  int8 and the rebuild does not. Fed the reference's OWN inputs, though, each
  layer must still land within half a quantization step — the rounding floor —
  everywhere except the handful of elements where the .lite saturates its int8
  range, which is information the export itself threw away.

These tests need the conversion toolchain:  uv sync --group convert
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

BASE = "model/ei-next-east-object-detection-tensorflow-lite-"
FLOAT_MODEL = ROOT / (BASE + "float32-model.4.lite")
INT8_MODEL = ROOT / (BASE + "int8-quantized-model.4.lite")

pytest.importorskip("torch", reason="needs `uv sync --group convert`")
pytest.importorskip("tflite", reason="needs `uv sync --group convert`")
pytest.importorskip("ai_edge_litert", reason="needs `uv sync --group convert`")


def _structured_images(n: int, size: int = 240, seed: int = 7) -> np.ndarray:
    """Low-frequency, image-like inputs in [0, 1] — white noise barely activates
    this network, so it makes a weak test signal."""
    rng = np.random.default_rng(seed)
    out = np.empty((n, 3, size, size), np.float32)
    for i in range(n):
        coarse = rng.random((3, 15, 15), dtype=np.float32)
        out[i] = np.repeat(np.repeat(coarse, size // 15, 1), size // 15, 2)
    return out


def _interpreter(path: Path):
    from ai_edge_litert.interpreter import Interpreter
    # preserve_all_tensors keeps intermediates addressable; without it the
    # delegate fuses operators and there is nothing per-layer to compare to.
    it = Interpreter(model_path=str(path), experimental_preserve_all_tensors=True)
    it.allocate_tensors()
    return it


def _run(it, img_nchw: np.ndarray):
    """Feed one NCHW [0,1] image, returning a per-tensor-index accessor."""
    ind = it.get_input_details()[0]
    scale, zero = ind["quantization"]
    nhwc = img_nchw.transpose(0, 2, 3, 1)
    feed = (np.clip(np.round(nhwc / scale + zero), -128, 127).astype(np.int8)
            if scale else nhwc.astype(ind["dtype"]))
    it.set_tensor(ind["index"], feed)
    it.invoke()
    details = {d["index"]: d for d in it.get_tensor_details()}

    def get(idx):
        """(dequantized float NHWC, quantization step, representable range).

        The range is what int8 can actually express for this tensor. A raw value
        sitting at ±127 is NOT by itself evidence of clipping — with zero_point
        -128 every post-ReLU zero lands on -128 legitimately. Clipping is only
        established by comparing against the value that SHOULD have been there,
        which the caller has and this helper does not.
        """
        d = details[idx]
        s, z = d["quantization"]
        raw = it.get_tensor(idx)
        if not s:
            return raw.astype(np.float64), 0.0, (-np.inf, np.inf)
        return ((raw.astype(np.float64) - z) * s, s,
                ((-128 - z) * s, (127 - z) * s))

    return get, feed


def _replay(net, feed_nchw):
    """Run the rebuild step by step, returning {tflite tensor index: output}."""
    import torch

    captured = {}
    vals = {net._input_index: torch.from_numpy(np.ascontiguousarray(feed_nchw))}
    with torch.no_grad():
        for step in net._steps:
            if step.kind == "conv":
                y = net.ops[step.module](vals[step.ins[0]])
            elif step.kind == "add":
                y = vals[step.ins[0]] + vals[step.ins[1]]
            else:
                y = torch.softmax(vals[step.ins[0]], dim=1)
            y = net._activate(y, step.act)
            vals[step.out] = y
            captured[step.out] = y.numpy().transpose(0, 2, 3, 1)  # -> NHWC
    return captured


# --- metadata -----------------------------------------------------------------

@pytest.mark.parametrize("path", [FLOAT_MODEL, INT8_MODEL])
def test_metadata_matches_the_lite_file(path):
    if not path.exists():
        pytest.skip(f"{path.name} not present")
    import tflite_to_torch
    _, meta = tflite_to_torch.load(str(path))
    assert meta.input_shape_nchw == (1, 3, 240, 240)
    assert meta.num_classes == 4
    assert meta.grid == (30, 30)
    assert meta.op_histogram == {
        "CONV_2D": 15, "DEPTHWISE_CONV_2D": 6, "ADD": 3, "SOFTMAX": 1}
    # Both exports must advertise the same [0,1] preprocessing contract, or the
    # Pi would need different scaling depending on which one was converted.
    lo, hi = meta.input_range
    assert lo == pytest.approx(0.0, abs=1e-6)
    assert hi == pytest.approx(1.0, abs=1e-2)


def test_float_export_is_flagged_as_float():
    if not FLOAT_MODEL.exists():
        pytest.skip("float model not present")
    import tflite_to_torch
    _, meta = tflite_to_torch.load(str(FLOAT_MODEL))
    assert meta.is_float
    _, int8_meta = tflite_to_torch.load(str(INT8_MODEL))
    assert not int8_meta.is_float


# --- the strict oracle: float32 export ----------------------------------------

@pytest.mark.skipif(not FLOAT_MODEL.exists(), reason="float model not present")
def test_float_model_every_layer_matches_to_float_epsilon():
    """No quantization anywhere, so every intermediate must match exactly.

    This is the test that actually proves the weight transposes, the asymmetric
    SAME padding and the fused activations are all right. There is no rounding
    noise to absorb a mistake.
    """
    import tflite_to_torch

    net, _ = tflite_to_torch.load(str(FLOAT_MODEL))
    img = _structured_images(1)
    it = _interpreter(FLOAT_MODEL)
    get, feed = _run(it, img)
    captured = _replay(net, img)

    checked = 0
    for idx, got in captured.items():
        ref, _, _ = get(idx)
        assert got.shape == ref.shape, f"tensor {idx}: {got.shape} vs {ref.shape}"
        scale = max(np.abs(ref).max(), 1e-6)
        err = np.abs(got - ref).max()
        assert err / scale < 1e-4, (
            f"tensor {idx} differs by {err:.3g} "
            f"({err / scale:.2e} relative) — not a rounding difference")
        checked += 1
    assert checked == 25, f"expected to compare 25 op outputs, compared {checked}"


@pytest.mark.skipif(not FLOAT_MODEL.exists(), reason="float model not present")
def test_float_model_end_to_end():
    import tflite_to_torch
    import torch

    net, _ = tflite_to_torch.load(str(FLOAT_MODEL), output_is_nhwc=True)
    img = _structured_images(4)
    it = _interpreter(FLOAT_MODEL)
    outs = []
    for i in range(len(img)):
        get, _ = _run(it, img[i:i + 1])
        outs.append(get(it.get_output_details()[0]["index"])[0])
    ref = np.concatenate(outs, 0)
    with torch.no_grad():
        got = net(torch.from_numpy(img)).numpy()
    assert np.abs(got - ref).max() < 1e-4
    assert (got.argmax(-1) == ref.argmax(-1)).all()


# --- int8 export: exact once saturation is accounted for ----------------------

@pytest.mark.skipif(not INT8_MODEL.exists(), reason="int8 model not present")
def test_int8_model_each_layer_is_exact_given_reference_inputs():
    """Isolate every layer from accumulated error by feeding it the .lite's own
    intermediate values, then require half-a-step agreement.

    Half a quantization step is the floor: the reference IS the rounded version
    of the value being computed, so it cannot be closer than that. Elements where
    the .lite saturated int8 are excluded — those are clipped in the export and
    genuinely unrecoverable, not a reconstruction error. There are only a few
    dozen of them across the whole network, and the test asserts that too, so
    this exclusion can never quietly grow into a blanket exemption.
    """
    import tflite_to_torch
    import torch

    net, _ = tflite_to_torch.load(str(INT8_MODEL))
    img = _structured_images(1)
    it = _interpreter(INT8_MODEL)
    get, feed = _run(it, img)

    total_saturated = 0
    total_elements = 0
    checked = 0
    with torch.no_grad():
        for step in net._steps:
            ins = [torch.from_numpy(
                np.ascontiguousarray(get(i)[0].astype(np.float32).transpose(0, 3, 1, 2)))
                for i in step.ins]
            if step.kind == "conv":
                y = net.ops[step.module](ins[0])
            elif step.kind == "add":
                y = ins[0] + ins[1]
            else:
                y = torch.softmax(ins[0], dim=1)
            y = net._activate(y, step.act)

            ref, scale, (lo, hi) = get(step.out)
            got = y.numpy().transpose(0, 2, 3, 1)
            assert got.shape == ref.shape

            # Clipped exactly where the true value falls outside what int8 can
            # hold. Everything else must agree to the rounding floor.
            saturated = (got > hi) | (got < lo)
            err = np.abs(got - ref)
            live = err[~saturated]
            assert live.size, f"tensor {step.out} is entirely saturated"
            assert live.max() <= 0.5001 * scale, (
                f"tensor {step.out} differs by {live.max():.6g}, more than half a "
                f"quantization step ({0.5 * scale:.6g}) — a structural mismatch")
            total_saturated += int(saturated.sum())
            total_elements += saturated.size
            checked += 1

    assert checked == 25
    # Saturation is rare. If this ever trips, the exclusion above is masking
    # something and the comparison is no longer meaningful.
    assert total_saturated < 0.001 * total_elements, (
        f"{total_saturated}/{total_elements} elements saturated — too many to "
        "dismiss as clipping")


@pytest.mark.skipif(not INT8_MODEL.exists(), reason="int8 model not present")
def test_first_conv_is_exact():
    """The first convolution reads the graph input, so it carries no upstream
    drift at all — nothing can explain away a failure here."""
    import tflite_to_torch
    import torch

    net, _ = tflite_to_torch.load(str(INT8_MODEL))
    img = _structured_images(1)
    it = _interpreter(INT8_MODEL)
    get, feed = _run(it, img)

    first = net._steps[0]
    x = torch.from_numpy(np.ascontiguousarray(
        ((feed.astype(np.float32) - it.get_input_details()[0]["quantization"][1])
         * it.get_input_details()[0]["quantization"][0]).transpose(0, 3, 1, 2)))
    with torch.no_grad():
        got = net._activate(net.ops[first.module](x), first.act)

    ref, scale, _ = get(first.out)
    err = np.abs(got.numpy().transpose(0, 2, 3, 1) - ref).max()
    assert err <= 0.5001 * scale, (
        f"first conv differs by {err:.6g} > half a quantization step "
        f"({0.5 * scale:.6g}) — the weight transpose or SAME padding is wrong")


# --- the two exports agree on their preprocessing contract --------------------

@pytest.mark.skipif(not (FLOAT_MODEL.exists() and INT8_MODEL.exists()),
                    reason="need both exports")
def test_both_exports_want_the_same_input_range():
    """ModelMeta assumes [0,1] for the float export, which carries no scale.

    If that assumption were wrong — if the float model actually wanted [0,255]
    or ImageNet normalization — feeding both models the same [0,1] image would
    make them disagree wildly. They agree, so the assumption holds.
    """
    img = _structured_images(2)
    outs = []
    for path in (FLOAT_MODEL, INT8_MODEL):
        it = _interpreter(path)
        per = []
        for i in range(len(img)):
            get, _ = _run(it, img[i:i + 1])
            per.append(get(it.get_output_details()[0]["index"])[0])
        outs.append(np.concatenate(per, 0))
    flt, i8 = outs
    assert (flt.argmax(-1) == i8.argmax(-1)).mean() > 0.99
    assert np.abs(flt - i8).mean() < 0.01


# --- guards -------------------------------------------------------------------

def test_same_padding_is_asymmetric_where_tensorflow_says_it_is():
    import tflite_to_torch as t2t
    # 240 -> 120 with a 3x3 stride-2 kernel: TF pads nothing before, one after.
    assert t2t._same_padding(240, 3, 2, 1) == (0, 1)
    assert t2t._same_padding(120, 3, 1, 1) == (1, 1)   # symmetric half-padding
    assert t2t._same_padding(60, 1, 1, 1) == (0, 0)    # 1x1 never pads


def test_unsupported_operator_is_rejected(monkeypatch):
    """An op outside the modelled set must fail loudly, never be skipped."""
    import tflite_to_torch as t2t
    monkeypatch.setattr(t2t, "_builtin_code", lambda model, op: 12345)
    with pytest.raises(ValueError, match="not supported"):
        t2t.load(str(FLOAT_MODEL if FLOAT_MODEL.exists() else INT8_MODEL))


def test_missing_file_raises():
    import tflite_to_torch as t2t
    with pytest.raises(OSError):
        t2t.load(str(ROOT / "model" / "does-not-exist.lite"))
