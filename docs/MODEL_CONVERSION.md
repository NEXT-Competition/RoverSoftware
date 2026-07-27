# Converting an Edge Impulse model for the Sony IMX500

Turns the Edge Impulse `.lite` export in `model/` into a network the Raspberry Pi
AI Camera runs **on the sensor**, so the Pi spends no CPU on inference.

```
uv sync --group convert
uv run tools/imx500_convert.py --images path/to/frames/
```

Everything below explains what that does, what was verified, and the one thing
that still has to happen before `object_align` can use the result.

---

## Why it is not a one-liner

The IMX500 toolchain does not accept `.tflite`. `imxconv-pt` takes an ONNX file
that **MCT** — Sony's Model Compression Toolkit — has already quantized against
the IMX500 target platform, and MCT wants a *float* model to quantize. So the
`.lite` has to be walked backwards into a framework model first:

```
.lite  ->  float PyTorch  ->  MCT PTQ (IMX500 TPC)  ->  ONNX+MCTQ  ->  packerOut.zip  ->  network.rpk
        ^                  ^                          ^              ^                  ^
   tflite_to_torch.py   imx500_convert.py         (exporter)     imxconv-pt      imx500-package
                                                                                   (on the Pi)
```

The conventional first hop is `tf2onnx` + `onnx2torch`, which pins a TensorFlow
version and leaves a NHWC↔NCHW transpose soup for MCT to fold. This network is
small and plain enough — `CONV_2D` ×15, `DEPTHWISE_CONV_2D` ×6, `ADD` ×3,
`SOFTMAX` ×1, nothing else — that `tools/tflite_to_torch.py` reads the flatbuffer
directly and emits real `nn.Conv2d`s instead. No TensorFlow anywhere, and a
cleaner graph for the converter.

## Which export to convert

Two Edge Impulse exports of the same network are in `model/`. **Use the float32
one**, which is the default:

| | float32 | int8 quantized |
|---|---|---|
| rebuild fidelity | exact to float32 epsilon (3.5e-08 mean) | exact to ½ a quantization step |
| weights | as trained | Edge Impulse's rounding baked in |
| activations | intact | **saturates at 4 layers** — clipped, unrecoverable |
| quantizations applied | 1 (IMX500's) | 2 (Edge Impulse's, then IMX500's) |

Converting the int8 export works and is verified, but it stacks a second
rounding on a model that has already lost information. The int8 export clips a
few dozen activations to the ends of its int8 range in `block_1_project`,
`block_2_project`, `block_3_project` and `block_4_add`; dequantization returns
the clipped values, not the originals.

## Calibration data matters more than anything else here

MCT sets activation ranges by running real inputs through the float model. Pass
`--images` a directory of frames the rover actually sees:

```
uv run tools/imx500_convert.py --images ~/rover-frames/ --num-samples 128
```

Without it the script falls back to synthetic low-frequency images and says so,
in the console and in `conversion_report.json`. That fallback is deliberately not
white noise — natural images are dominated by low spatial frequencies, and i.i.d.
noise drives the first 3×3 conv far harder than any real frame, inflating the
calibrated range and wasting int8 codes downstream. It is still a guess. This
network is unusually forgiving because every conv is fused with RELU6 and clamped
into [0, 6], but the head convs before the softmax are not bounded.

## What comes out

```
build/imx500/
  model.onnx                   quantized, MCTQ quantizer nodes, mct_quantizers domain
  conversion_report.json       inputs used + every fidelity number below
  converted/
    packerOut.zip              <- the portable artifact; copy this to the Pi
    model_MemoryReport.json    sensor memory budget
    dnnParams.xml              final tensor layouts/scales
```

Sensor budget for this network: **1.34 MB of 8 MB, 17% utilization,
`"Fit In Chip": true`.**

## Finishing on the Pi

`imx500-package` ships in the Pi's `imx500-tools` apt package, not on PyPI, so it
cannot run on a dev laptop. Copy `packerOut.zip` over and:

```
sudo apt install imx500-tools
imx500-package -i packerOut.zip -o network/
# -> network/network.rpk
```

Point `RS_VISION_IMX500_MODEL` at the `.rpk` and set `RS_VISION_BACKEND=imx500`.

---

## ⚠️ The decoder does not understand this network yet

**The conversion is complete and verified, but `object_align` cannot use the
`.rpk` as-is.** This is a real gap, not a caveat.

`robot/sensors/imx500.py::Decoder.parse()` handles exactly two output layouts,
both from picamera2's demo:

1. `postprocess == "nanodet"` → NMS on the Pi
2. everything else → `outputs[0], outputs[1], outputs[2]` as parallel
   **boxes, scores, classes** tensors, i.e. an SSD/YOLO-style `_pp` network whose
   post-processing is baked into the `.rpk`

This network is **FOMO**, and emits neither. It has a *single* output: a
`30×30×4` softmax heatmap — a per-cell class probability grid at input stride 8.
`outputs[1]` and `outputs[2]` do not exist, so `parse()` will fail rather than
mis-decode, which at least fails loudly.

Making it usable needs a FOMO decode path: threshold the heatmap, take connected
peaks per class as centroids, and emit fixed cell-sized boxes. Two things worth
knowing before doing that:

- `to_detection()` currently asserts size is always available, justified by "the
  IMX500 model zoo is real bounding-box detectors". That comment stops being
  true for a FOMO network — centroids have no extent, so `size` would have to go
  back to `None` and the FOMO degradation path in `detector.py` applies again.
- `requirements.txt` already advises exporting a **YOLO-style
  (`object_detection`) model, not FOMO**, precisely because FOMO gives no object
  size and so `object_align` can only turn to face a target, never approach it.
  Re-exporting from Edge Impulse as a bounding-box detector would sidestep the
  decoder work entirely and give better autonomy — and this same converter
  handles it, as long as it stays within the supported operator set.

---

## Verification

`tests/test_tflite_to_torch.py` (11 tests) proves the rebuild is the same network
rather than merely a similar-scoring one. Comparing only the final softmax would
hide a transposed kernel or an off-by-one pad inside an end-to-end average, so it
goes layer by layer:

- **float32 export, every intermediate tensor** — matches to float32 epsilon
  (< 1e-4 relative). Nothing is quantized, so a structural error cannot hide.
- **int8 export, every layer fed the reference's own inputs** — matches to
  ≤ ½ a quantization step, the rounding floor, excluding only elements where the
  `.lite` itself saturated. The test also asserts those are < 0.1% of all
  elements, so the exclusion can't quietly widen into a blanket exemption.
- **first conv exact** — it reads the graph input, so no upstream drift can
  explain a failure away.
- **both exports agree on [0, 1] input** — the float export carries no input
  scale, so the preprocessing contract is an assumption; feeding both models the
  same [0, 1] image and getting the same detections is what justifies it.
- SAME-padding asymmetry (240→120 stride-2 pads `(0, 1)`, not `(1, 1)`), and
  unsupported operators raising rather than being skipped.

End-to-end fidelity of the shipped network, on synthetic calibration:

| comparison | mean abs err | argmax agreement |
|---|---|---|
| float rebuild vs float `.lite` | 3.5e-08 | 100% |
| IMX500-quantized vs float `.lite` | 0.0018 | 99.89% |
| IMX500-quantized vs deployed int8 `.lite` | 0.0021 | 99.57% |
| float `.lite` vs deployed int8 `.lite` | 0.0028 | 99.60% |

The last row is the useful control: the IMX500 network is as close to the
currently-deployed int8 model as the float source itself is, so essentially all
the remaining difference is Edge Impulse's original quantization, not ours.

## The torch pin is load-bearing

`pyproject.toml` pins `torch>=2.4,<2.9` in the `convert` group. Do not relax it.

`mct_quantizers` emits its MCTQ quantizer nodes from `symbolic()` methods on
`torch.autograd.Function` — a hook only the **legacy TorchScript** ONNX exporter
calls. torch 2.9 flipped `torch.onnx.export` to `dynamo=True` by default, and the
dynamo exporter never consults those symbolics, so the export either errors out
or silently produces an ONNX file with no quantization nodes in it, which
`imxconv-pt` then rejects.

`torch` is in the `convert` group and not in `[project.dependencies]` on purpose:
it is a build-machine-only dependency, nothing on the robot imports it, and the
Pi just loads the finished `.rpk` through picamera2. Plain `uv sync` stays lean.

## Options

```
--model PATH        source .lite (default: the float32 export)
--images DIR        calibration frames — strongly recommended
--num-samples N     how many to use (default 64)
--tpc-version V     IMX500 target platform: 1.0 | 1.0_lut | 4.0 | 4.1 | 5.0 (default 4.0)
--nhwc-output       append a permute so output is NHWC like the .lite, not ONNX-native NCHW
--skip-converter    stop after the ONNX export
```
