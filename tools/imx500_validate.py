#!/usr/bin/env python3
"""Measure what INT8 quantization cost the exported IMX500 model.

    uv run tools/imx500_validate.py --onnx model/best_imx_model/model_imx.onnx \
                                    --model model/best.pt --images model/data/images

Runs the ACTUAL exported graph — MCTQ quantizer nodes, on-sensor NMS and all —
under onnxruntime, and compares its detections against the float checkpoint on
the same images. This is the closest thing to on-rover truth that a laptop can
produce: the only pieces not exercised are the sensor's own fixed-point
arithmetic and picamera2's coordinate mapping.

Worth running after every re-export, because the failure it catches is silent.
A model calibrated on the wrong images still exports, still packs, still loads on
the sensor, and simply detects less well — there is no error anywhere, just a
rover that stops seeing the blue bucket at the range it used to.

It also re-checks the output contract the decoder depends on (boxes xyxy in input
pixels, scores descending, n_valid honest), so a future Ultralytics release that
reorders or reformats those tensors trips this rather than the rover.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
from typing import List, Optional

import numpy as np

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def _iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Compare the exported IMX500 graph against the float checkpoint.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--onnx", default="model/best_imx_model/model_imx.onnx",
                    help="the quantized ONNX the exporter produced")
    ap.add_argument("--model", default="model/best.pt", help="float checkpoint")
    ap.add_argument("--images", default="model/data/images", help="directory of test images")
    ap.add_argument("--limit", type=int, default=40, help="images to test")
    ap.add_argument("--conf", type=float, default=0.25, help="confidence threshold for both")
    ap.add_argument("--iou", type=float, default=0.5, help="IoU to count a detection matched")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--report", default=None, help="write JSON here")
    args = ap.parse_args(argv)

    for path in (args.onnx, args.model):
        if not os.path.exists(path):
            raise SystemExit(f"not found: {path}")

    import onnxruntime as ort
    from onnxruntime_extensions import get_library_path
    import edgemdt_cl.pytorch.nms.nms_ort  # noqa: F401 - registers the NMS custom op
    import mct_quantizers                  # noqa: F401 - registers the MCTQ ops
    from PIL import Image
    from ultralytics import YOLO

    so = ort.SessionOptions()
    so.register_custom_ops_library(get_library_path())
    sess = ort.InferenceSession(args.onnx, so, providers=["CPUExecutionProvider"])
    float_model = YOLO(args.model)

    paths = sorted(p for p in glob.glob(os.path.join(args.images, "*"))
                   if p.lower().endswith(IMAGE_SUFFIXES))[:args.limit]
    if not paths:
        raise SystemExit(f"no images under {args.images}")

    ious, matched, n_float, n_quant, label_ok = [], 0, 0, 0, 0
    contract = {"boxes_are_xyxy": True, "scores_descending": True,
                "boxes_in_pixel_space": True, "n_valid_within_bounds": True}

    for p in paths:
        # Feed both models the identical array: resizing to a square imgsz makes
        # Ultralytics' letterbox a no-op, so no preprocessing difference can be
        # mistaken for quantization loss.
        im = Image.open(p).convert("RGB").resize((args.imgsz, args.imgsz), Image.BILINEAR)
        arr = np.asarray(im, np.float32) / 255.0

        r = float_model.predict(im, imgsz=args.imgsz, conf=args.conf, verbose=False)[0].boxes
        fb, fl = r.xyxy.cpu().numpy(), r.cls.cpu().numpy().astype(int)

        out = sess.run(None, {sess.get_inputs()[0].name: arr.transpose(2, 0, 1)[None]})
        boxes, scores, labels, n_valid = out
        n = int(np.asarray(n_valid).reshape(-1)[0])
        if not 0 <= n <= boxes.shape[1]:
            contract["n_valid_within_bounds"] = False
            n = max(0, min(n, boxes.shape[1]))
        b_all, s_all = boxes[0][:n], scores[0][:n]
        if n:
            if not np.all(b_all[:, 2] >= b_all[:, 0]) or not np.all(b_all[:, 3] >= b_all[:, 1]):
                contract["boxes_are_xyxy"] = False
            if not np.all(np.diff(s_all) <= 1e-6):
                contract["scores_descending"] = False
            if b_all.max() <= 1.5:  # normalized would never exceed ~1
                contract["boxes_in_pixel_space"] = False

        keep = s_all >= args.conf
        qb, ql = b_all[keep], labels[0][:n][keep].astype(int)
        n_float += len(fb)
        n_quant += len(qb)

        used = set()
        for i, box in enumerate(fb):
            if not len(qb):
                break
            cand = [(_iou(box, q), j) for j, q in enumerate(qb) if j not in used]
            if not cand:
                break
            v, j = max(cand)
            if v >= args.iou:
                used.add(j)
                ious.append(v)
                matched += 1
                label_ok += int(fl[i] == ql[j])

    report = {
        "onnx": args.onnx,
        "float_model": args.model,
        "images": len(paths),
        "conf": args.conf,
        "match_iou": args.iou,
        "float_detections": n_float,
        "quantized_detections": n_quant,
        "matched": matched,
        "recall_of_float": round(matched / max(n_float, 1), 4),
        "mean_iou_of_matches": round(float(np.mean(ious)), 4) if ious else None,
        "label_agreement": round(label_ok / max(matched, 1), 4),
        "output_contract": contract,
    }
    print(json.dumps(report, indent=2))

    if not all(contract.values()):
        print("\n*** OUTPUT CONTRACT VIOLATED — robot/sensors/imx500.py:"
              "unpack_edgemdt_nms() assumes what this just disproved. Do not deploy.")
        return 1
    if report["recall_of_float"] < 0.85:
        print(f"\n*** Only {report['recall_of_float']:.1%} of float detections survived "
              "quantization. Check that --data pointed at the real dataset.")
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
