# Running our own model on the Sony IMX500

The Raspberry Pi AI Camera runs its network **inside the sensor**, so the Pi
spends no CPU on inference. This is how our trained model gets in there.

```
uv sync --group convert          # once, on this machine
just model-bootstrap             # once per robot: AI Camera stack on the Pi
just model-deploy                # export here, build the .rpk on the Pi, install it
just model-selftest              # does the sensor actually see anything?
```

Then `just config` and set:

```
RS_VISION_BACKEND=imx500
RS_VISION_IMX500_MODEL=/var/lib/roversoftware/network.rpk
RS_VISION_IMX500_LABELS=/var/lib/roversoftware/labels.txt
RS_VISION_HFOV=66                               # the AI Camera's real FOV
```

### Why it takes two machines

**There is no `.rpk` on your laptop, and there cannot be.** `imx500-package` ships
only in the Pi's `imx500-tools` apt package — it is not on PyPI and not available
for macOS. `packerOut.zip` is the portable artifact; the `.rpk` is built from it
on the Pi.

```
this machine                         the Pi
────────────                         ──────
best.pt                              Ultralytics + MCT + imxconv-pt
  -> packerOut.zip   ──scp──>        imx500-package -> network.rpk
```

`just model-deploy` does both halves. The individual steps, if you want them:

| recipe | where | does |
|---|---|---|
| `just model-bootstrap` | Pi | installs `imx500-tools`, `imx500-all`, `python3-picamera2` |
| `just model-export` | here | stages the dataset, converts `best.pt` -> `packerOut.zip` |
| `just model-validate` | here | measures what quantization cost |
| `just model-install` | both | scp's the zip, packages the `.rpk` on the Pi, installs it |
| `just model-selftest` | Pi | runs the rover's own decode against the live sensor |
| `just model-status` | Pi | what is currently installed |

All of them honour `ROBOT_HOST` / `just host=rover2.local ...` like the rest of
the Justfile. Or do it by hand:

```
uv run tools/prepare_yolo_dataset.py --src model/data --out build/dataset
uv run tools/imx500_export_yolo.py  --model model/best.pt --data build/dataset/data.yaml
uv run tools/imx500_validate.py
scp build/imx500-yolo/packerOut.zip pi@rover1.local:/tmp/
ssh pi@rover1.local 'imx500-package -i /tmp/packerOut.zip -o /tmp/net'
```

---

## Two converters, and which one you want

| script | input | use it for |
|---|---|---|
| `tools/prepare_yolo_dataset.py` | Label Studio export | staging `model/data` into something Ultralytics reads |
| **`tools/imx500_export_yolo.py`** | Ultralytics `.pt` (`best.pt`) | **the live path** — our YOLO11n detector |
| `tools/imx500_validate.py` | the exported ONNX | measuring what quantization cost |
| `tools/imx500_convert.py` | Edge Impulse `.lite` | an Edge Impulse export, if we ever go back to one |

The `.lite` converter came first, for an Edge Impulse export that turned out to
be a FOMO classification grid rather than a detector. It works and is verified
(see the bottom of this file), but it is not the path the rover is on.

## What the YOLO export actually does

`YOLO.export(format="imx")` does the hard part, and `imx500_export_yolo.py`
wraps it rather than reimplementing it:

1. Rebinds the `Detect` head so the DFL box decode happens **inside** the graph
   and sets `xyxy=True`, so boxes come out as corners rather than centre/size.
2. Quantizes with MCT against the IMX500 target platform (TPC 4.0, INT8).
3. Wraps the result in edge-mdt-cl's `multiclass_nms_with_indices`, so **NMS runs
   on the sensor**. The Pi receives finished detections, not raw anchors.
4. Runs `imxconv-pt` to produce `packerOut.zip`, plus `labels.txt`.

Reproducing that by hand would mean re-deriving head surgery that changes between
YOLO versions — not worth it.

### The two patches, and why they're safe

Ultralytics `assert LINUX` before exporting. That is a support policy, not a
technical limit: everything behind it (`imxconv-pt`, its bundled JVM `sdspconv`
backend, MCT) is pure Python plus a platform-independent jar, and `torch2imx`
even has an explicit Windows branch for the binary name. The whole toolchain was
verified end to end on macOS/arm64 before the wrapper existed. Set
`RS_IMX_REQUIRE_LINUX=1` to restore the assert.

The second patch stubs `check_font`, which downloads a font for plotting labels
and blows up on recent macOS because matplotlib's `_get_macos_fonts()` hits
`KeyError('_items')`. Nothing to do with the export. Both patches are no-ops on
Linux.

### Calibration is the thing that decides accuracy

`--data` takes the dataset YAML. MCT sets every activation range by running those
images through the float model, so this is the single biggest lever on how well
the quantized network performs.

Without it the script falls back to `coco8.yaml` — eight generic COCO images —
prints a loud warning, and records `"calibration_is_placeholder": true` in
`export_report.json`. That is enough to prove the pipeline runs and **not** enough
to trust the accuracy of the result. Calibrating on the real 648 images takes
~10 minutes instead of ~2.5; that is the whole cost.

### Staging the dataset

`model/data` is a Label Studio export, which looks like an Ultralytics dataset
and is not one. It has no `data.yaml`, and — the part that bites — the image and
label filenames do not pair:

```
images/00BCF467-...-CDA3A7094BF5.jpg.6tfj6d12.ingestion-5759d4ffb8-8lvsf.jpg
labels/00BCF467-...-CDA3A7094BF5.txt
```

Ultralytics finds a label by swapping `/images/` for `/labels/` and changing the
extension, so the stems must match exactly. Left alone, **every image is silently
treated as having no objects** — the export still succeeds and nothing warns.
`tools/prepare_yolo_dataset.py` stages a directory of symlinks whose stems match
and writes the yaml. It refuses to stage an export where the stem rule does not
fit, rather than producing a quietly-empty dataset.

The generated yaml points *both* splits at every image, because the purpose is
calibration and more data is strictly better. That makes it wrong for scoring, so
the file says so in a comment; `--val-split 0.2` gives a real disjoint split.

## What the sensor sends back

Four tensors, because NMS already ran on-sensor:

| tensor | shape | on-wire | meaning |
|---|---|---|---|
| boxes | `(max_det, 4)` | int16 × 1/32 | `(x_min, y_min, x_max, y_max)` in **network input pixels**, not normalized |
| scores | `(max_det,)` | uint8 × 1/256 | descending |
| labels | `(max_det,)` | int16 × 1 | class index |
| n_valid | `(1,)` | int16 × 1 | real detections; everything after is zero padding |

`robot/sensors/imx500.py::unpack_edgemdt_nms()` decodes this, and
`Decoder.parse()` tries it **first** — it is recognized from the tensors
themselves, whereas the two model-zoo layouts are selected by intrinsics that a
custom `.rpk` doesn't carry.

Two details in there are load-bearing:

- **`n_valid` is honoured.** Ignore it and every frame decodes 300 zero-padded
  boxes at the origin; `select="centermost"` then locks onto the padding forever.
- **Tensors are identified by shape, not position.** The ONNX graph declares them
  `boxes, scores, labels, n_valid`; the converter's own `dnnParams.xml` lists them
  in the *opposite* order. Which one picamera2 follows is only observable on a Pi.
  Shape settles the two unambiguous ones — boxes is the only 2-D tensor, `n_valid`
  the only single-element one — and where those landed reveals whether the whole
  list is forward or reversed, which fixes scores vs labels too. Both orders are
  covered by tests.

If no boxes appear on the rover, `Decoder.describe_layout(metadata)` reports which
decode path a real frame actually took.

## What quantization actually cost

`tools/imx500_validate.py` runs the **real exported graph** — MCTQ quantizer
nodes, on-sensor NMS and all — under onnxruntime and compares it against the
float checkpoint on the same images. Calibrated on the 648-image dataset, over 40
images at `conf=0.25`:

| | |
|---|---|
| float detections | 176 |
| quantized detections | 175 |
| matched at IoU ≥ 0.5 (one-to-one) | 151 — **85.8%** of float |
| mean IoU of matches | **0.910** |
| label agreement | **100%** |

Read that carefully: detection *counts* agree to within one, boxes that match do
so tightly, and no match ever changes class. The ~14% unmatched are detections
sitting near the 0.25 confidence threshold that quantization nudged across it,
not boxes landing in the wrong place — which is what a mean IoU of 0.91 tells you.

Worth re-running after every export, because the failure it catches is silent: a
model calibrated on the wrong images still exports, still packs, still loads, and
simply sees less well. The tool also re-checks the output contract the decoder
depends on and **fails the run** if a future Ultralytics release reorders or
reformats those tensors — better here than on the rover.

## object_align gets its size back

The Edge Impulse FOMO caveat in `requirements.txt` — centroids only, so the rover
can turn to face a target but never approach it — **does not apply here**. This is
a real bounding-box detector, so `to_detection()` reports a true `size` and
approach/standoff work. Calibrate `RS_VISION_STANDOFF` with
`tools/detector_selftest.py` rather than guessing.

Set `RS_VISION_HFOV=66`. The 50° default is the *post-crop* figure for the Edge
Impulse backend; the IMX500 path maps boxes back to the full frame, so it needs
the camera's real FOV or the steering derivative is scaled wrong.

## Metric range (optional, telemetry only)

Standoff above stops on the raw `size` ratio and needs no calibration. If you
also want **metres** in the telemetry (`dist`, shown on the base station), that
needs a tape measure and a target of known height:

```bash
# 1. CALIBRATE — target centred, tape-measured, NOT touching the frame edge
python tools/detector_selftest.py --backend imx500 \
    --imx500-model /path/to/network/network.rpk \
    --imx500-labels /path/to/labels.txt \
    --label bucket --target-height 0.29 --distance 3.00
#    -> prints RS_VISION_TARGET_HEIGHT / RS_VISION_FOCAL_FRAC to paste

# 2. VERIFY at a distance you did NOT calibrate at
python tools/detector_selftest.py --backend imx500 ... \
    --target-height 0.29 --focal-frac <the number>
#    -> `range=` per frame; a few percent off is right
```

`--imx500-labels` is **required for a custom export** — the `.rpk` this pipeline
produces carries no embedded labels, so without it every box comes back `"0"`
and `--label` never matches.

The constant is specific to *this* network at *this* `imgsz`. It does not carry
over from the Edge Impulse backend (which normalizes against a ~50° crop, a ~28%
difference), and re-exporting at a different `--imgsz` is a new calibration.
Both env vars default to `0`, which reports range as absent rather than as a
plausible wrong number.

## Sensor memory is tight at 640

Our YOLO11n at `imgsz=640` uses **7.12 MB of the 8 MB on-chip budget (90%)**. It
fits, but there is not much room. If a future model doesn't fit, the first lever
is `--imgsz 480` or `320`; the exporter prints the memory report and says
`Fit In Chip` on every run.

For comparison, the FOMO network was 1.34 MB (17%).

---

## Verifying the `.lite` converter (`tools/imx500_convert.py`)

Kept for reference; this is the Edge Impulse path, not the live one.

The IMX500 toolchain does not accept `.tflite` — `imxconv-pt` wants ONNX that MCT
has already quantized, and MCT wants a *float* model. The usual bridge is
`tf2onnx` + `onnx2torch`, which pins a TensorFlow version and leaves a NHWC/NCHW
transpose soup. The Edge Impulse graph was small enough (`CONV_2D` ×15,
`DEPTHWISE_CONV_2D` ×6, `ADD` ×3, `SOFTMAX` ×1) that `tools/tflite_to_torch.py`
reads the flatbuffer directly and emits real `nn.Conv2d`s. No TensorFlow anywhere.

`tests/test_tflite_to_torch.py` verifies it layer by layer rather than end to end,
because comparing only the final output hides a transposed kernel or an off-by-one
SAME pad inside an average:

- **float32 export, every intermediate tensor** — matches to float32 epsilon
  (3.5e-08 mean). Nothing is quantized, so nothing structural can hide.
- **int8 export, each layer fed the reference's own inputs** — matches to ≤ ½ a
  quantization step, the rounding floor, excluding only the few dozen elements
  where the `.lite` itself saturated int8. The test bounds those at 0.1% so the
  exclusion can't quietly widen.
- **first conv exact** — it reads the graph input, so no upstream drift can
  explain a failure away.

## The torch pin is load-bearing

`pyproject.toml` pins `torch>=2.4,<2.9` in the `convert` group. Do not relax it.

`mct_quantizers` emits its MCTQ quantizer nodes from `symbolic()` methods on
`torch.autograd.Function` — a hook only the **legacy TorchScript** ONNX exporter
calls. torch 2.9 flipped `torch.onnx.export` to `dynamo=True` by default, and the
dynamo exporter never consults those symbolics, so the export either errors out or
silently produces an ONNX file with no quantization nodes, which `imxconv-pt` then
rejects.

`torch` and `ultralytics` live in the `convert` group, not
`[project.dependencies]`: they are build-machine-only, nothing on the robot
imports them, and the Pi just loads the finished `.rpk` through picamera2. Plain
`uv sync` stays lean.

**Licensing note:** Ultralytics is AGPL-3.0, and a model trained from
`yolo11n.pt` inherits that. It is a build-time tool here — no Ultralytics code
ships to the robot or runs on it — but if this rover is ever distributed, that is
worth a deliberate decision rather than a default.
