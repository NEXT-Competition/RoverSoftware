#!/usr/bin/env python3
"""Convert an Ultralytics YOLO checkpoint (best.pt) into an IMX500 network.

    uv sync --group convert
    uv run tools/imx500_export_yolo.py --model model/best.pt --data path/to/data.yaml

Produces `packerOut.zip` and `labels.txt`. The final `.rpk` is built on the Pi —
`imx500-package` ships in the `imx500-tools` apt package, not on PyPI. See
docs/MODEL_CONVERSION.md.

--- Why this wraps Ultralytics instead of reimplementing it ---
`YOLO.export(format="imx")` already does the hard part correctly: it rebinds the
Detect head so the DFL box decode happens inside the graph, quantizes with MCT
against the IMX500 platform, and wraps the result in edge-mdt-cl's
`multiclass_nms_with_indices` so NMS runs ON THE SENSOR. Reproducing that by hand
would mean re-deriving head surgery that changes between YOLO versions. What this
script adds is the two workarounds below, calibration wiring, and a report.

--- The two patches, and why they are safe ---
1. `assert LINUX`. Ultralytics refuses to export on anything but Linux. That
   assert is a support policy, not a technical limit: everything it guards —
   `imxconv-pt`, its bundled JVM `sdspconv` backend, MCT — is pure Python plus a
   platform-independent jar, and `torch2imx` itself even has an explicit Windows
   branch for the binary name. This repo verified the whole toolchain end to end
   on macOS/arm64 before this script existed. The export is byte-identical work;
   only the support statement differs. Set RS_IMX_REQUIRE_LINUX=1 to restore it.
2. `check_font`. Ultralytics downloads a font to draw labels on plots. On recent
   macOS, matplotlib's `_get_macos_fonts()` raises KeyError('_items') because the
   `system_profiler` output changed shape. It has nothing to do with the export,
   so it is stubbed out rather than worked around.

Both are no-ops on Linux, where neither patch is reached.

--- Calibration ---
`--data` takes the dataset YAML the model was trained on, and it matters: MCT
sets every activation range by running those images through the float model. Point
it at the real dataset. Without it this falls back to `coco8.yaml` (eight generic
COCO images), which is enough to prove the pipeline runs and not enough to trust
the accuracy of what comes out — so it says so, loudly, and records the choice in
the report.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import List, Optional

DEFAULT_MODEL = "model/best.pt"
FALLBACK_DATA = "coco8.yaml"


def _patch_for_non_linux() -> List[str]:
    """Apply the two workarounds above. Returns what was patched, for the report."""
    applied = []
    import ultralytics.engine.exporter as exporter
    import ultralytics.data.utils as data_utils
    import ultralytics.utils.checks as checks

    if not getattr(exporter, "LINUX", False):
        if os.environ.get("RS_IMX_REQUIRE_LINUX") == "1":
            raise SystemExit(
                "RS_IMX_REQUIRE_LINUX=1 and this is not Linux — refusing to patch "
                "Ultralytics' platform assert. Run on a Pi/Linux box, or unset it.")
        exporter.LINUX = True
        applied.append("exporter.LINUX")

    try:  # only actually broken on some macOS versions; patch only if it is
        from matplotlib import font_manager
        font_manager.findSystemFonts()
    except Exception:
        data_utils.check_font = lambda *a, **k: None
        checks.check_font = lambda *a, **k: None
        applied.append("check_font")
    return applied


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Export an Ultralytics YOLO .pt to a Sony IMX500 network.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="Ultralytics .pt checkpoint")
    ap.add_argument("--data", default=None,
                    help="dataset YAML for INT8 calibration — STRONGLY recommended; "
                         f"falls back to {FALLBACK_DATA}")
    ap.add_argument("--out", default="build/imx500-yolo", help="where to copy the artifacts")
    ap.add_argument("--imgsz", type=int, default=640, help="network input size")
    ap.add_argument("--conf", type=float, default=0.001,
                    help="score threshold baked into the on-sensor NMS; keep it low "
                         "and filter with RS_VISION_MIN_CONFIDENCE at runtime")
    ap.add_argument("--iou", type=float, default=0.7, help="on-sensor NMS IoU threshold")
    ap.add_argument("--max-det", type=int, default=300, help="max detections per frame")
    ap.add_argument("--fraction", type=float, default=1.0,
                    help="fraction of the calibration set to use")
    args = ap.parse_args(argv)

    if not os.path.exists(args.model):
        raise SystemExit(f"model not found: {args.model}")

    patched = _patch_for_non_linux()
    if patched:
        print(f"[patch] applied: {', '.join(patched)} (see this file's docstring)")

    from ultralytics import YOLO

    data = args.data or FALLBACK_DATA
    if not args.data:
        print(f"\n*** No --data given. Calibrating on {FALLBACK_DATA}, eight generic\n"
              f"*** COCO images. That proves the pipeline but NOT the accuracy —\n"
              f"*** re-run with --data pointing at the real dataset before deploying.\n")

    model = YOLO(args.model)
    names = list(model.names.values())
    print(f"[model] task={model.task} classes={names}")

    out = model.export(format="imx", data=data, imgsz=args.imgsz, conf=args.conf,
                       iou=args.iou, max_det=args.max_det, fraction=args.fraction)
    export_dir = Path(str(out))
    print(f"[export] {export_dir}")

    dest = Path(args.out)
    dest.mkdir(parents=True, exist_ok=True)
    copied = []
    for name in ("packerOut.zip", "labels.txt", "dnnParams.xml"):
        src = export_dir / name
        if src.exists():
            shutil.copy2(src, dest / name)
            copied.append(name)

    mem = {}
    for report in export_dir.glob("*MemoryReport.json"):
        mem = json.loads(report.read_text()).get("Memory Report", {})
        shutil.copy2(report, dest / report.name)
        copied.append(report.name)

    report = {
        "source_model": args.model,
        "task": model.task,
        "classes": names,
        "imgsz": args.imgsz,
        "calibration_data": data,
        "calibration_is_placeholder": args.data is None,
        "nms": {"on_sensor": True, "conf": args.conf, "iou": args.iou,
                "max_det": args.max_det},
        "output_layout": "edge-mdt NMS: boxes(max_det,4) xyxy in input pixels, "
                         "scores, labels, n_valid",
        "memory": {k: mem.get(k) for k in
                   ("Memory Usage", "Total Memory Available On Chip",
                    "Memory Utilization", "Fit In Chip")},
        "artifacts": copied,
        "patches_applied": patched,
    }
    (dest / "export_report.json").write_text(json.dumps(report, indent=2) + "\n")

    print(f"\n[memory] {mem.get('Memory Usage')} of {mem.get('Total Memory Available On Chip')}"
          f" ({mem.get('Memory Utilization')}), fits={mem.get('Fit In Chip')}")
    if mem.get("Fit In Chip") is False:
        print("*** Does NOT fit on the sensor — lower --imgsz (e.g. 480 or 320).")
    print(f"[out] {dest}  ({', '.join(copied)})")
    print(f"\nNext, on the Raspberry Pi (needs `sudo apt install imx500-tools`):\n"
          f"    imx500-package -i {dest/'packerOut.zip'} -o network/\n"
          f"    # -> network/network.rpk\n"
          f"Then point the rover at it:\n"
          f"    RS_VISION_BACKEND=imx500\n"
          f"    RS_VISION_IMX500_MODEL=/path/to/network/network.rpk\n"
          f"    RS_VISION_IMX500_LABELS=/path/to/labels.txt\n"
          f"    RS_VISION_HFOV=66        # the AI Camera's real FOV, not a cropped one\n"
          f"Then calibrate against THIS network — a focal_frac from another model\n"
          f"or the edge_impulse backend does not carry over:\n"
          f"    tools/detector_selftest.py --backend imx500 \\\n"
          f"        --imx500-model /path/to/network/network.rpk \\\n"
          f"        --imx500-labels /path/to/labels.txt \\\n"
          f"        --target-height <target height, m> --distance <tape measure, m>\n"
          f"    # -> prints RS_VISION_TARGET_HEIGHT / RS_VISION_FOCAL_FRAC")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
