#!/usr/bin/env python3
"""Make a Label Studio YOLO export usable by Ultralytics.

    uv run tools/prepare_yolo_dataset.py --src model/data --out build/dataset

Label Studio exports `images/`, `labels/` and `classes.txt`, which looks like an
Ultralytics dataset and is not one, for two reasons:

1. **No `data.yaml`.** Ultralytics needs one naming the splits and classes.
2. **The filenames do not pair.** Ultralytics finds a label by swapping
   `/images/` for `/labels/` in the image path and changing the extension, so
   the two stems must match exactly. Label Studio appends an ingestion suffix to
   the image and not to the label:

       images/00BCF467-...-CDA3A7094BF5.jpg.6tfj6d12.ingestion-5759d4ffb8-8lvsf.jpg
       labels/00BCF467-...-CDA3A7094BF5.txt

   Left alone, every image is silently treated as having no objects. For PTQ
   calibration that is survivable — MCT only pushes pixels through the network
   and never looks at a label — but it makes the same dataset useless for `yolo
   val`, and a silent "0 labels found" is exactly the kind of thing that gets
   discovered three debugging sessions later.

So this stages a directory of symlinks whose stems match, and writes the yaml.
Symlinks, not copies: the images here are 4032x3024 phone photos and copying
them would burn a gigabyte to rename them.

--- On the train/val split ---
The generated yaml points BOTH splits at every image, because its purpose is
quantization calibration and more calibration data is strictly better. That
makes it wrong for measuring accuracy — `yolo val` against it would score the
model on its own training data. The yaml says so in a comment, and `--val-split`
produces a real disjoint split if you want one.
"""

from __future__ import annotations

import argparse
import os
import random
import re
import sys
from pathlib import Path
from typing import List, Optional

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
# Matches the FIRST image extension that is followed by more filename — the
# boundary Label Studio's ingestion suffix begins at.
_INGESTION = re.compile(r"\.(jpg|jpeg|png|bmp|webp)(?=\.)", re.I)


def label_stem(filename: str) -> str:
    """Image filename -> the stem its label file uses."""
    m = _INGESTION.search(filename)
    return filename[:m.start()] if m else os.path.splitext(filename)[0]


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Stage a Label Studio YOLO export as an Ultralytics dataset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--src", default="model/data",
                    help="export directory containing images/, labels/, classes.txt")
    ap.add_argument("--out", default="build/dataset", help="where to stage it")
    ap.add_argument("--val-split", type=float, default=0.0,
                    help="fraction held out as a DISJOINT val set. 0 (default) puts "
                         "every image in both splits, which is right for calibration "
                         "and wrong for measuring accuracy")
    ap.add_argument("--seed", type=int, default=0, help="split shuffle seed")
    args = ap.parse_args(argv)

    src = Path(args.src)
    img_dir, lab_dir = src / "images", src / "labels"
    if not img_dir.is_dir() or not lab_dir.is_dir():
        raise SystemExit(f"{src} does not look like a Label Studio YOLO export "
                         "(need images/ and labels/)")

    classes_file = src / "classes.txt"
    if not classes_file.exists():
        raise SystemExit(f"missing {classes_file}")
    names = [ln.strip() for ln in classes_file.read_text().splitlines() if ln.strip()]
    if not names:
        raise SystemExit(f"{classes_file} is empty")

    images = sorted(p for p in img_dir.iterdir()
                    if p.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise SystemExit(f"no images under {img_dir}")

    # Pair up, and refuse to stage a dataset that is quietly missing labels —
    # a calibration set is allowed to be imperfect, but not silently so.
    pairs, unlabelled, collisions = [], [], []
    seen = {}
    for img in images:
        stem = label_stem(img.name)
        lab = lab_dir / f"{stem}.txt"
        if stem in seen:
            collisions.append((img.name, seen[stem]))
            continue
        seen[stem] = img.name
        if lab.exists():
            pairs.append((img, lab, stem))
        else:
            unlabelled.append(img.name)

    print(f"[scan] {len(images)} images, {len(pairs)} paired with labels")
    if collisions:
        print(f"[warn] {len(collisions)} images collapsed to a stem already used "
              f"— skipped, e.g. {collisions[0]}")
    if unlabelled:
        print(f"[warn] {len(unlabelled)} images have no label file and are being "
              f"staged anyway (they count as background), e.g. {unlabelled[0]}")
    if not pairs:
        raise SystemExit("nothing paired — the stem rule does not fit this export")

    out = Path(args.out).resolve()
    for split in ("train", "val"):
        for kind in ("images", "labels"):
            d = out / split / kind
            d.mkdir(parents=True, exist_ok=True)
            for old in d.iterdir():          # idempotent re-runs
                old.unlink()

    idx = list(range(len(pairs)))
    if args.val_split > 0:
        random.Random(args.seed).shuffle(idx)
        cut = max(1, int(len(idx) * args.val_split))
        split_map = {"val": idx[:cut], "train": idx[cut:]}
    else:
        split_map = {"val": idx, "train": idx}

    for split, members in split_map.items():
        for i in members:
            img, lab, stem = pairs[i]
            (out / split / "images" / f"{stem}{img.suffix.lower()}").symlink_to(img.resolve())
            (out / split / "labels" / f"{stem}.txt").symlink_to(lab.resolve())
        print(f"[{split}] {len(members)} images")

    overlapping = args.val_split <= 0
    yaml = out / "data.yaml"
    yaml.write_text(
        "# Generated by tools/prepare_yolo_dataset.py — do not edit by hand.\n"
        + ("# WARNING: train and val are THE SAME IMAGES. This dataset exists to\n"
           "# calibrate INT8 quantization, where more data is strictly better and\n"
           "# labels are never read. Do NOT use it to measure accuracy — regenerate\n"
           "# with --val-split 0.2 for that.\n" if overlapping else
           "# train and val are disjoint.\n")
        + f"path: {out}\n"
          f"train: train/images\n"
          f"val: val/images\n"
          f"nc: {len(names)}\n"
          "names:\n"
        + "".join(f"  {i}: {n}\n" for i, n in enumerate(names))
    )
    print(f"\n[classes] {names}")
    print(f"[out] {yaml}")
    print(f"\nNow export:\n"
          f"    uv run tools/imx500_export_yolo.py --model model/best.pt --data {yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
