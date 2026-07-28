#!/usr/bin/env python3
"""Convert the Edge Impulse int8 `.lite` model into a Sony IMX500 network.

    uv sync --group convert
    uv run tools/imx500_convert.py --images path/to/frames/

Pipeline
--------
    .lite (int8 TFLite)
      -> float PyTorch          (tools/tflite_to_torch.py, exact dequantization)
      -> MCT post-training quantization against the IMX500 target platform
      -> ONNX with MCTQ quantizer nodes
      -> imxconv-pt             -> packerOut.zip
      -> imx500-package (ON THE PI, from the imx500-tools apt package)
      -> network.rpk            <- what robot/sensors/imx500.py loads

The last hop is deliberately not run here: `imx500-package` ships in the Pi's
`imx500-tools` apt package, not on PyPI, so it does not exist on a dev laptop.
`packerOut.zip` is the portable artifact; copy it over and run one command there.
See docs/MODEL_CONVERSION.md.

--- On representative data ---
MCT calibrates activation ranges by running real inputs through the float model.
Point `--images` at a directory of frames the rover actually sees and the ranges
will be right. Without it this falls back to synthetic images, and the fallback
is honest-but-inferior: it reports the agreement it achieved so you can decide
whether it is good enough. This particular network is unusually forgiving —
every conv is fused with RELU6, so most activations are clamped into [0, 6] no
matter what you feed it — but "forgiving" is not "free", and the final conv
stack before the softmax is not bounded.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterator, List, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

# The float32 export is the right source. Recovering weights from the int8
# export works (and is verified to be exact), but those weights already carry
# Edge Impulse's rounding, and its int8 activations SATURATE at four layers —
# clipped values that dequantization cannot bring back. Starting from float
# means the IMX500 quantization is the only one applied.
DEFAULT_MODEL = ("model/"
                 "ei-next-east-object-detection-tensorflow-lite-float32-model.4.lite")
INT8_MODEL = ("model/"
              "ei-next-east-object-detection-tensorflow-lite-int8-quantized-model.4.lite")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# --- representative data -----------------------------------------------------

def _load_images(directory: str, size: int, limit: int) -> np.ndarray:
    """Real frames -> NCHW float32 in [0, 1], the range the .lite input maps to."""
    from PIL import Image

    paths = sorted(p for p in Path(directory).rglob("*")
                   if p.suffix.lower() in IMAGE_SUFFIXES)
    if not paths:
        raise SystemExit(f"no images found under {directory!r} "
                         f"(looked for {sorted(IMAGE_SUFFIXES)})")
    paths = paths[:limit]
    out = np.empty((len(paths), 3, size, size), np.float32)
    for i, p in enumerate(paths):
        im = Image.open(p).convert("RGB").resize((size, size), Image.BILINEAR)
        out[i] = np.asarray(im, np.float32).transpose(2, 0, 1) / 255.0
    print(f"[rep] loaded {len(paths)} calibration images from {directory}")
    return out


def _synthetic(size: int, count: int, seed: int = 0) -> np.ndarray:
    """Stand-in calibration data with roughly image-like statistics.

    NOT white noise. Natural images are dominated by low spatial frequencies and
    per-channel colour bias; i.i.d. uniform noise has neither, and it drives the
    first 3x3 conv far harder than any real frame would, which inflates the
    calibrated activation range and wastes int8 codes everywhere downstream.
    Low-frequency blobs plus a little fine grain is closer, and cheap.
    """
    rng = np.random.default_rng(seed)
    out = np.empty((count, 3, size, size), np.float32)
    for i in range(count):
        acc = np.zeros((3, size, size), np.float32)
        for octave, weight in ((4, 1.0), (8, 0.5), (16, 0.25), (32, 0.125)):
            coarse = rng.random((3, octave, octave), dtype=np.float32)
            reps = size // octave + 1
            up = np.repeat(np.repeat(coarse, reps, 1), reps, 2)[:, :size, :size]
            acc += weight * up
        acc += 0.05 * rng.random((3, size, size), dtype=np.float32)
        acc -= acc.min()
        peak = acc.max()
        out[i] = acc / peak if peak > 0 else acc
    print(f"[rep] no --images given; using {count} synthetic calibration frames")
    return out


def make_rep_gen(data: np.ndarray, batch: int):
    """MCT wants a zero-arg callable yielding a LIST of inputs per step."""
    import torch

    def gen() -> Iterator[List["torch.Tensor"]]:
        for i in range(0, len(data), batch):
            yield [torch.from_numpy(data[i:i + batch])]

    return gen


# --- evaluation ---------------------------------------------------------------

def _tflite_reference(model_path: str, data: np.ndarray) -> np.ndarray:
    """Original .lite outputs for `data` as float NHWC.

    Handles both exports: an int8 model needs its input quantized and its output
    dequantized, a float32 model takes and returns floats directly (its
    "quantization" tuple is the (0.0, 0) placeholder).
    """
    from ai_edge_litert.interpreter import Interpreter

    it = Interpreter(model_path=model_path)
    it.allocate_tensors()
    ind, outd = it.get_input_details()[0], it.get_output_details()[0]
    s_in, z_in = ind["quantization"]
    s_out, z_out = outd["quantization"]

    outs = []
    for x in data:  # NCHW float -> NHWC
        nhwc = x.transpose(1, 2, 0)[None]
        if s_in:
            feed = np.clip(np.round(nhwc / s_in + z_in), -128, 127).astype(np.int8)
        else:
            feed = nhwc.astype(ind["dtype"])
        it.set_tensor(ind["index"], feed)
        it.invoke()
        raw = it.get_tensor(outd["index"]).astype(np.float32)
        outs.append((raw - z_out) * s_out if s_out else raw)
    return np.concatenate(outs, 0)


def _agreement(a: np.ndarray, b: np.ndarray) -> dict:
    """How close two [N, H, W, C] softmax stacks are, in terms that matter.

    Mean absolute error is the headline, but for a detector what actually
    changes behaviour is whether each cell still picks the same class, so the
    argmax agreement is reported alongside it.
    """
    return {
        "mean_abs_err": float(np.abs(a - b).mean()),
        "max_abs_err": float(np.abs(a - b).max()),
        "argmax_agreement": float((a.argmax(-1) == b.argmax(-1)).mean()),
    }


# --- main ---------------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert an Edge Impulse int8 .lite model to a Sony IMX500 network.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="input .lite file")
    ap.add_argument("--out", default="build/imx500", help="output directory")
    ap.add_argument("--images", default=None,
                    help="directory of real frames for calibration (strongly recommended)")
    ap.add_argument("--num-samples", type=int, default=64,
                    help="calibration images to use")
    ap.add_argument("--batch", type=int, default=8, help="calibration batch size")
    ap.add_argument("--tpc-version", default="4.0",
                    help="IMX500 target-platform version (1.0 | 1.0_lut | 4.0 | 4.1 | 5.0)")
    ap.add_argument("--nhwc-output", action="store_true",
                    help="append a permute so the network's output is NHWC like the "
                         "original .lite, instead of the ONNX-native NCHW")
    ap.add_argument("--opset", type=int, default=17, help="ONNX opset for the export")
    ap.add_argument("--skip-converter", action="store_true",
                    help="stop after the ONNX export, do not run imxconv-pt")
    args = ap.parse_args(argv)

    if not os.path.exists(args.model):
        raise SystemExit(f"model not found: {args.model}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    import torch
    import model_compression_toolkit as mct
    from edgemdt_tpc import get_target_platform_capabilities
    import tflite_to_torch

    # 1. TFLite -> float PyTorch --------------------------------------------
    print(f"[1/5] reading {args.model}")
    net, meta = tflite_to_torch.load(args.model, output_is_nhwc=args.nhwc_output)
    size = meta.input_shape_nchw[2]
    print(f"      source is {'float32' if meta.is_float else 'int8-quantized'}")
    print(f"      ops={meta.op_histogram}")
    print(f"      input NCHW{meta.input_shape_nchw} float range "
          f"[{meta.input_range[0]:.3f}, {meta.input_range[1]:.3f}]")
    print(f"      output {meta.grid[0]}x{meta.grid[1]} grid, "
          f"{meta.num_classes} classes (FOMO heatmap)")

    # 2. calibration data ----------------------------------------------------
    print("[2/5] preparing representative dataset")
    if args.images:
        data = _load_images(args.images, size, args.num_samples)
    else:
        data = _synthetic(size, args.num_samples)
    rep_gen = make_rep_gen(data, args.batch)

    # Sanity-check the float rebuild against the .lite it came from BEFORE
    # quantizing, so a reconstruction bug can never be mistaken for
    # quantization loss further down.
    print("      checking float rebuild against the original .lite")
    check = data[: min(8, len(data))]
    ref = _tflite_reference(args.model, check)
    with torch.no_grad():
        got = net(torch.from_numpy(check)).numpy()
    if not args.nhwc_output:
        got = got.transpose(0, 2, 3, 1)
    rebuild = _agreement(got, ref)
    print(f"      rebuild vs .lite: {json.dumps(rebuild)}")
    if rebuild["argmax_agreement"] < 0.99:
        raise SystemExit("float rebuild does not match the original model — "
                         "refusing to continue")

    # 3. MCT post-training quantization -------------------------------------
    print(f"[3/5] MCT PTQ against IMX500 TPC v{args.tpc_version}")
    tpc = get_target_platform_capabilities(args.tpc_version, "imx500")
    q_model, quant_info = mct.ptq.pytorch_post_training_quantization(
        in_module=net,
        representative_data_gen=rep_gen,
        target_platform_capabilities=tpc,
    )

    with torch.no_grad():
        q_out = q_model(torch.from_numpy(check)).cpu().numpy()
    if not args.nhwc_output:
        q_out = q_out.transpose(0, 2, 3, 1)
    quantized = _agreement(q_out, ref)
    print(f"      quantized vs source .lite: {json.dumps(quantized)}")

    # Also compare against the int8 export, when it is sitting alongside. That
    # is the model the rover runs on the Edge Impulse backend TODAY, so it — not
    # the float source — is the behaviour anyone will notice a change against.
    vs_int8 = None
    if os.path.exists(INT8_MODEL) and os.path.abspath(INT8_MODEL) != os.path.abspath(args.model):
        int8_ref = _tflite_reference(INT8_MODEL, check)
        vs_int8 = {
            "float_source_vs_int8": _agreement(ref, int8_ref),
            "imx500_vs_int8": _agreement(q_out, int8_ref),
        }
        print(f"      imx500 vs deployed int8: {json.dumps(vs_int8['imx500_vs_int8'])}")

    # 4. export ONNX ---------------------------------------------------------
    onnx_path = out_dir / "model.onnx"
    print(f"[4/5] exporting {onnx_path}")
    mct.exporter.pytorch_export_model(
        model=q_model,
        save_model_path=str(onnx_path),
        repr_dataset=rep_gen,
        serialization_format=mct.exporter.PytorchExportSerializationFormat.ONNX,
        quantization_format=mct.exporter.QuantizationFormat.MCTQ,
        onnx_opset_version=args.opset,
    )

    report = {
        "source_model": args.model,
        "source_dtype": "float32" if meta.is_float else "int8",
        "input_shape_nchw": list(meta.input_shape_nchw),
        "input_float_range": list(meta.input_range),
        "output_layout": "NHWC" if args.nhwc_output else "NCHW",
        "output_grid": list(meta.grid),
        "num_classes": meta.num_classes,
        "tpc_version": args.tpc_version,
        "calibration": {
            "source": args.images or "synthetic",
            "num_samples": int(len(data)),
        },
        "float_rebuild_vs_tflite": rebuild,
        "quantized_vs_tflite": quantized,
    }
    if vs_int8:
        report["vs_deployed_int8_model"] = vs_int8

    # 5. imxconv-pt ----------------------------------------------------------
    if args.skip_converter:
        print("[5/5] skipped (--skip-converter)")
    else:
        conv_dir = out_dir / "converted"
        exe = shutil.which("imxconv-pt")
        if exe is None:
            raise SystemExit("imxconv-pt not on PATH — run this under "
                             "`uv run` so the project venv's bin/ is visible")
        cmd = [exe, "-i", str(onnx_path), "-o", str(conv_dir), "--overwrite-output"]
        print(f"[5/5] {' '.join(cmd)}")
        proc = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        if proc.returncode != 0:
            raise SystemExit(f"imxconv-pt failed with exit code {proc.returncode}")
        report["converted_dir"] = str(conv_dir)
        packer = conv_dir / "packerOut.zip"
        if packer.exists():
            report["packer_out"] = str(packer)

    (out_dir / "conversion_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out_dir/'conversion_report.json'}")
    if "packer_out" in report:
        print(f"\nNext, on the Raspberry Pi (needs `sudo apt install imx500-tools`):\n"
              f"    imx500-package -i {report['packer_out']} -o network/\n"
              f"    # -> network/network.rpk, the file RS_VISION_IMX500_MODEL points at")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
