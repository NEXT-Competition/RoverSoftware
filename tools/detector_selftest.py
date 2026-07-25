#!/usr/bin/env python3
"""Standalone detector self-test — verify vision in isolation, either backend.

Runs a one-shot DIAGNOSTIC of the camera + model by itself, independent of the
rest of the robot stack, and prints a PASS/FAIL summary. Run it first when
bringing up vision or chasing a "the model doesn't work on the rover" problem.

    python tools/detector_selftest.py                      # auto-detect backend
    python tools/detector_selftest.py --backend imx500     # AI Camera, on-sensor
    python tools/detector_selftest.py --model /var/lib/roversoftware/model.eim
    python tools/detector_selftest.py --label cone --frames 50
    python tools/detector_selftest.py --save /tmp/frame.jpg

Which backend it tests follows the same rule the rover uses
(`sensors/imx500.resolve_backend`), so this cannot drift from what actually
runs. Checks, in both cases:

  1. The runtime is installed (edge_impulse_linux / picamera2 + IMX500).
  2. The model file exists (and, for the .eim, is EXECUTABLE — it's a binary EI
     runs; the .rpk is data the sensor loads, so it needs no chmod).
  3. The model loads; prints labels and input size — warning loudly on Edge
     Impulse FOMO, which cannot report object size (so no approach/standoff).
  4. The camera opens; prints which backend won.
  5. N live frames: per-frame ms, achieved fps, and every detection as the
     controller would see it (label, conf, error_x, size).

--- This is also the standoff calibration procedure ---
`VisionConfig.standoff_size` is "how big the box looks when we're close enough",
which is not a number anyone can guess. Park the rover at the distance you want
it to stop, run this, and read off the printed `size`. That's your
RS_VISION_STANDOFF. No tape measure, no camera intrinsics. (Recalibrate it if
you switch backends — the two normalize `size` against different rectangles.)

--- And it's how you verify framing and colour order ---
`--save` writes a frame. On Edge Impulse it's the model's own CROPPED input, and
two things need checking by eye: the colours must look right (if red and blue
are swapped, the frames are RGB where Edge Impulse expects BGR — the model will
still return confident garbage, so nothing else will tell you), and your target
must be inside the crop (EI discards ~25% of the width at 640x480). On the
IMX500 it's the full frame with the sensor's boxes drawn on it — there is no
crop, so what you're checking is that the boxes land on the objects.

Off-hardware (neither runtime installed) it prints a clear note and exits 0, so
it's safe to run on a dev laptop.
"""

import argparse
import os
import sys
import time

# Make the repo root (parent of tools/) importable, matching the other tools.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.config import RobotConfig  # noqa: E402
from robot.sensors.camera import describe, draw_boxes, encode_jpeg, open_source  # noqa: E402
from robot.sensors.imx500 import (Decoder, resolve_backend, select_box,  # noqa: E402
                                  to_detection)

try:
    from edge_impulse_linux.image import ImageImpulseRunner
    _IMPORT_ERROR = None
except Exception as _e:
    # edge_impulse_linux itself is pure Python, but its image module imports cv2
    # (OpenCV) and numpy at load time. Keep the real error so a missing cv2 on
    # the Pi doesn't masquerade as "edge_impulse_linux not installed".
    ImageImpulseRunner = None
    _IMPORT_ERROR = _e

_FOMO_TYPE = "constrained_object_detection"


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def imx500_selftest(cfg, args) -> int:
    """The AI Camera path: same five checks, no CPU inference anywhere.

    Drives `sensors/imx500.Decoder` — the exact decode the rover runs — so a PASS
    here means the rover will see what this prints, coordinates included.
    """
    print("=== IMX500 (Raspberry Pi AI Camera) detector self-test ===\n")

    # 1. Runtime present?
    try:
        from picamera2 import Picamera2  # noqa: F401
        from picamera2.devices import IMX500  # noqa: F401
    except Exception as e:
        print(f"picamera2 with IMX500 support not importable ({e}) — nothing to test here.")
        print("This is expected on a dev laptop. On the Pi:")
        print("  sudo apt install python3-picamera2 imx500-all")
        return 0
    print("[1/5] PASS  picamera2 + IMX500 device support installed")

    # 2. Network file present? No chmod check: unlike the .eim this is data the
    #    sensor loads, not a binary the Pi executes.
    path = os.path.expanduser(cfg.vision.imx500_model)
    if not os.path.exists(path):
        print(f"[2/5] FAIL  no network at {path}")
        print("      Install the Sony model zoo:  sudo apt install imx500-all")
        print("      (they land in /usr/share/imx500-models/)")
        return 1
    print(f"[2/5] PASS  network {path} ({os.path.getsize(path) / 1e6:.1f} MB)")

    # 3. Is the camera actually attached? Checked before opening so the failure
    #    is one clear line instead of a libcamera stack trace.
    from robot.sensors.camera import imx500_present
    if not imx500_present():
        print("[3/5] FAIL  no IMX500 camera found on the CSI bus")
        print("      Check the ribbon (contacts toward the board), then:  rpicam-hello --list-cameras")
        return 1
    print("[3/5] PASS  AI Camera detected")

    # 4. Open it — this is where the network crosses to the sensor.
    print("[4/5] opening the camera and uploading the network to the sensor")
    print("      (cold start takes tens of seconds — this is normal)...")
    t0 = time.monotonic()
    source = open_source(cfg.camera, cfg.vision)
    if source is None or getattr(source, "imx500", None) is None:
        print(f"[4/5] FAIL  could not open the AI Camera with {os.path.basename(path)}")
        if source is not None:
            source.close()
        return 1
    decoder = Decoder(source, cfg.vision)
    print(f"[4/5] PASS  {describe(source)} in {time.monotonic() - t0:.1f}s")
    print(f"            {decoder.describe()}")
    task = getattr(decoder.intrinsics, "task", "?")
    if task != "object detection":
        print(f"            WARNING: task is {task!r}, not object detection — this "
              "network will not produce boxes.")
    if args.label and decoder.labels and args.label not in decoder.labels:
        print(f"            WARNING: --label {args.label!r} is not in this network's "
              f"labels — nothing will ever match. First few: {decoder.labels[:8]}")

    # 5. Live frames. No inference timing to report — the sensor did that before
    #    the frame arrived; what's measured here is the Pi-side decode.
    print(f"\n[5/5] reading {args.frames} frames...\n")
    times, seen, last = [], 0, None
    try:
        for i in range(args.frames):
            frame, metadata = source.read_with_metadata()
            if frame is None:
                print(f"  frame {i:3d}  (no frame)")
                continue
            h, w = frame.shape[0], frame.shape[1]
            t1 = time.monotonic()
            boxes = decoder.parse(metadata, w, h)
            ms = (time.monotonic() - t1) * 1e3
            times.append(ms)
            last = (frame, boxes)
            if not boxes:
                print(f"  frame {i:3d}  {ms:6.2f} ms   (no target)")
                continue
            seen += 1
            best = select_box(boxes, cfg.vision.select, w)
            for b in boxes:
                d = to_detection(b, w, h, 0.0)
                mark = "*" if b is best else " "
                print(f"  frame {i:3d}  {ms:6.2f} ms {mark} {d.label:<12} "
                      f"conf={d.confidence:.2f}  error_x={d.error_x:+.3f}  "
                      f"size={d.size:.3f}")
    finally:
        source.close()

    print()
    if times:
        avg = sum(times) / len(times)
        print(f"  Pi-side decode: avg {avg:.2f} ms (inference itself ran on the sensor)")
    print(f"  frames with a target: {seen}/{len(times)}")
    print("  '*' marks the box object_align would steer on "
          f"(select={cfg.vision.select}).")

    if args.save and last is not None:
        frame, boxes = last
        jpeg = encode_jpeg(draw_boxes(frame, [(*b[:4], b[4], b[5], True) for b in boxes]), 85)
        if jpeg:
            with open(args.save, "wb") as f:
                f.write(jpeg)
            print(f"\n  wrote the last frame + boxes to {args.save}")
            print("  Check by eye: do the boxes land on the objects, and are the")
            print("  colours right (red/blue not swapped)?")
        else:
            print(f"\n  could not encode {args.save} (install python3-opencv or Pillow)")

    print()
    if seen:
        print("PASS — on-sensor detection is working. To calibrate standoff: park at")
        print("your stop distance and put the printed `size` in RS_VISION_STANDOFF.")
        print("Also set RS_VISION_HFOV to the camera's REAL horizontal FOV (~66 for")
        print("the stock lens) — there is no Edge Impulse crop on this path.")
        return 0
    print("No targets detected. If something should be in frame, check: the label")
    print("filter, --conf, and whether this network was trained on that object")
    print(f"(labels: {decoder.labels[:12]}).")
    return 1


def main() -> int:
    cfg = RobotConfig()
    p = argparse.ArgumentParser(description="Object detector self-test")
    p.add_argument("--backend", default=os.environ.get("RS_VISION_BACKEND", cfg.vision.backend),
                   choices=["auto", "edge_impulse", "imx500"],
                   help="which detection backend to test (default: the rover's own rule)")
    p.add_argument("--model", default=os.environ.get("RS_VISION_MODEL", cfg.vision.model_path))
    p.add_argument("--imx500-model", dest="imx500_model",
                   default=os.environ.get("RS_VISION_IMX500_MODEL", cfg.vision.imx500_model),
                   help="the .rpk network uploaded to the AI Camera's sensor")
    p.add_argument("--label", default=os.environ.get("RS_VISION_LABEL", cfg.vision.target_label),
                   help="only report this label ('' = all)")
    p.add_argument("--device", default=os.environ.get("RS_CAMERA_DEVICE", cfg.camera.device))
    p.add_argument("--conf", type=float, default=cfg.vision.min_confidence)
    p.add_argument("--frames", type=int, default=20, help="how many frames to classify")
    p.add_argument("--save", default=None, metavar="OUT.jpg",
                   help="write a frame here (check colour + framing)")
    args = p.parse_args()

    cfg.vision.backend = args.backend
    cfg.vision.model_path = args.model
    cfg.vision.imx500_model = args.imx500_model
    cfg.vision.target_label = args.label
    cfg.vision.min_confidence = args.conf
    cfg.camera.device = args.device
    # Same decision the rover makes — and it points cfg.camera.device at the
    # matching capture backend, so the test can't accidentally pass on one
    # camera while the rover opens another.
    backend = resolve_backend(cfg.vision, cfg.camera)
    if backend == "imx500":
        return imx500_selftest(cfg, args)

    print("=== Edge Impulse detector self-test ===\n")

    # 1. Library present?
    if ImageImpulseRunner is None:
        missing = getattr(_IMPORT_ERROR, "name", None)
        if isinstance(_IMPORT_ERROR, ModuleNotFoundError) and missing and missing != "edge_impulse_linux":
            # edge_impulse_linux is installed, but a dependency of it isn't —
            # usually OpenCV, which the Pi lacks by default. This is a real FAIL
            # on the Pi, not the "nothing to test on a laptop" case below.
            hint = ("sudo apt install python3-opencv" if missing == "cv2"
                    else f"pip install {missing}")
            print(f"[1/5] FAIL  edge_impulse_linux is installed, but its dependency "
                  f"'{missing}' is missing.")
            print(f"            Install it:  {hint}")
            return 1
        print(f"edge_impulse_linux not importable ({_IMPORT_ERROR}) — nothing to test here.")
        print("This is expected on a dev laptop (it's Linux-only and needs a")
        print("compiled .eim). On the Pi:  pip install edge_impulse_linux")
        return 0
    print("[1/5] PASS  edge_impulse_linux installed")

    # 2. Model file present and executable?
    path = os.path.expanduser(args.model)
    if not os.path.exists(path):
        print(f"[2/5] FAIL  no model at {path}")
        print("      Download one:  edge-impulse-linux-runner --download model.eim")
        return 1
    if not os.access(path, os.X_OK):
        print(f"[2/5] FAIL  {path} is not executable")
        print(f"      The .eim is a binary Edge Impulse runs.  Fix:  chmod +x {path}")
        return 1
    size_mb = os.path.getsize(path) / 1e6
    print(f"[2/5] PASS  model {path} ({size_mb:.1f} MB, executable)")

    # 3. Model init.
    runner = ImageImpulseRunner(path)
    try:
        info = runner.init()
    except Exception as e:
        print(f"[3/5] FAIL  could not init the model: {e}")
        return 1
    mp = info.get("model_parameters", {}) or {}
    labels = list(mp.get("labels", []) or [])
    mw = int(mp.get("image_input_width", 0) or 0)
    mh = int(mp.get("image_input_height", 0) or 0)
    mtype = mp.get("model_type", "?")
    fomo = mtype == _FOMO_TYPE
    proj = info.get("project", {})
    print(f"[3/5] PASS  '{proj.get('owner','?')}/{proj.get('name','?')}' "
          f"input={mw}x{mh} type={mtype}")
    print(f"            labels: {labels}")
    if args.label and args.label not in labels:
        print(f"            WARNING: --label {args.label!r} is not in this model's "
              f"labels — nothing will ever match.")
    if fomo:
        print()
        print("            *** FOMO MODEL — object size is NOT available. ***")
        print("            FOMO reports centroids with fixed cell-sized boxes, so")
        print("            object_align can turn to face the target but can NOT")
        print("            approach or stop at a standoff. Export a YOLO-style")
        print("            (object_detection) model if you need approach.")
        print()

    # 4. Camera.
    cfg.camera.device = args.device
    source = open_source(cfg.camera)
    if source is None:
        print("[4/5] FAIL  no camera available")
        print("      Install one:  sudo apt install python3-picamera2   (Pi Camera)")
        print("                    pip install opencv-python            (USB webcam)")
        runner.stop()
        return 1
    print(f"[4/5] PASS  camera open via {describe(source)} "
          f"({cfg.camera.width}x{cfg.camera.height})")

    # 5. Live inference.
    print(f"[5/5] classifying {args.frames} frames...\n")
    times, seen, last_cropped = [], 0, None
    try:
        for i in range(args.frames):
            frame = source.read()
            if frame is None:
                print(f"  frame {i:3d}  (no frame)")
                continue
            t0 = time.monotonic()
            features, cropped = runner.get_features_from_image(frame)
            res = runner.classify(features)
            ms = (time.monotonic() - t0) * 1e3
            times.append(ms)
            last_cropped = cropped

            boxes = [b for b in (res.get("result", {}) or {}).get("bounding_boxes", []) or []
                     if b.get("value", 0.0) >= args.conf
                     and (not args.label or b.get("label") == args.label)]
            if not boxes:
                print(f"  frame {i:3d}  {ms:6.1f} ms   (no target)")
                continue
            seen += 1
            for b in boxes:
                cx = b["x"] + b["width"] / 2.0
                ex = _clamp(2.0 * cx / mw - 1.0, -1.0, 1.0)
                sz = "n/a (FOMO)" if fomo else f"{b['height'] / mh:.3f}"
                print(f"  frame {i:3d}  {ms:6.1f} ms   {b['label']:<12} "
                      f"conf={b['value']:.2f}  error_x={ex:+.3f}  size={sz}")
    finally:
        source.close()
        runner.stop()

    print()
    if times:
        avg = sum(times) / len(times)
        print(f"  inference: avg {avg:.1f} ms  ->  {1000.0 / avg:.1f} fps "
              f"(min {min(times):.0f} / max {max(times):.0f})")
    print(f"  frames with a target: {seen}/{len(times)}")

    if args.save and last_cropped is not None:
        try:
            import cv2
            cv2.imwrite(args.save, last_cropped)
            print(f"\n  wrote the model's cropped input to {args.save}")
            print("  Check it by eye: are the colours right (red/blue not swapped),")
            print("  and is your target inside the crop?")
        except Exception as e:
            print(f"\n  could not write {args.save}: {e}")

    print()
    if seen:
        sz_hint = ("size is unavailable on FOMO — approach/standoff is off"
                   if fomo else
                   "park at your stop distance and put the printed size in RS_VISION_STANDOFF")
        print(f"PASS — detector is working. To calibrate standoff: {sz_hint}.")
        return 0
    print("No targets detected. If something should be in frame, check: the label")
    print("filter, --conf, the colour order (--save), and whether the target is")
    print("inside Edge Impulse's center-crop.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
