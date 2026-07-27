"""Tests for the Label Studio -> Ultralytics dataset staging.

The rule under test looks trivial and is the exact thing that fails silently.
Label Studio names the image and its label differently:

    images/00BCF467-...-CDA3A7094BF5.jpg.6tfj6d12.ingestion-5759d4ffb8-8lvsf.jpg
    labels/00BCF467-...-CDA3A7094BF5.txt

Ultralytics pairs them by identical stem. Get the rule wrong and nothing raises —
every image is quietly treated as having no objects, the export still succeeds,
and the only symptom is a model calibrated on images the tooling thought were
empty. That is worth a handful of assertions.

These run with no extra dependencies; the staging tool only needs the stdlib.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from prepare_yolo_dataset import label_stem  # noqa: E402


# --- the stem rule -----------------------------------------------------------

def test_strips_the_label_studio_ingestion_suffix():
    name = "00BCF467-08AB-4275-9B70-CDA3A7094BF5.jpg.6tfj6d12.ingestion-5759d4ffb8-8lvsf.jpg"
    assert label_stem(name) == "00BCF467-08AB-4275-9B70-CDA3A7094BF5"


def test_handles_a_jpeg_original_renamed_to_jpg():
    """Real case from this dataset: the source was .jpeg, the ingested copy .jpg."""
    name = "048B1F57-5083-4200-AEF5-A0D603B5187F_1_105_c.jpeg.6tfj7i8q.ingestion-x.jpg"
    assert label_stem(name) == "048B1F57-5083-4200-AEF5-A0D603B5187F_1_105_c"


def test_a_plain_filename_is_just_its_stem():
    """No ingestion suffix -> ordinary splitext, so a hand-made dataset works too."""
    assert label_stem("frame_0001.jpg") == "frame_0001"
    assert label_stem("frame_0001.png") == "frame_0001"


def test_only_an_extension_followed_by_more_name_counts():
    """The trailing extension must NOT be stripped as if it were a suffix — doing
    so would collapse every file onto a shorter stem and mispair the lot."""
    assert label_stem("a.jpg") == "a"
    assert label_stem("a.jpg.b.jpg") == "a"


def test_is_case_insensitive():
    assert label_stem("IMG_1234.JPG.abc.jpg") == "IMG_1234"


def test_dots_in_the_name_survive():
    """A stem with dots that are not image extensions must be left alone."""
    assert label_stem("run.2026.07.27.jpg") == "run.2026.07.27"


# --- end to end on a fake export ---------------------------------------------

def _make_export(tmp_path: Path, names=("ball", "blue bucket")):
    src = tmp_path / "data"
    (src / "images").mkdir(parents=True)
    (src / "labels").mkdir(parents=True)
    (src / "classes.txt").write_text("\n".join(names) + "\n")
    return src


def test_staging_pairs_every_image(tmp_path, capsys):
    import prepare_yolo_dataset as prep

    src = _make_export(tmp_path)
    for i in range(4):
        stem = f"UUID-{i}"
        (src / "images" / f"{stem}.jpg.ingest-{i}.jpg").write_bytes(b"x")
        (src / "labels" / f"{stem}.txt").write_text("0 0.5 0.5 0.1 0.1\n")

    out = tmp_path / "staged"
    assert prep.main(["--src", str(src), "--out", str(out)]) == 0

    imgs = sorted((out / "val" / "images").iterdir())
    labs = sorted((out / "val" / "labels").iterdir())
    assert len(imgs) == 4 and len(labs) == 4
    # The whole point: stems must match across the two directories.
    assert [p.stem for p in imgs] == [p.stem for p in labs]
    assert (out / "data.yaml").exists()


def test_generated_yaml_lists_classes_in_index_order(tmp_path):
    import prepare_yolo_dataset as prep

    src = _make_export(tmp_path, names=("ball", "blue bucket", "orange bucket"))
    (src / "images" / "a.jpg.x.jpg").write_bytes(b"x")
    (src / "labels" / "a.txt").write_text("2 0.5 0.5 0.1 0.1\n")
    out = tmp_path / "staged"
    prep.main(["--src", str(src), "--out", str(out)])

    text = (out / "data.yaml").read_text()
    assert "nc: 3" in text
    # Index order is load-bearing — the sensor returns an index, not a name.
    assert "  0: ball\n" in text
    assert "  1: blue bucket\n" in text
    assert "  2: orange bucket\n" in text


def test_calibration_yaml_warns_that_the_splits_overlap(tmp_path):
    """Default staging puts every image in both splits. That is right for
    quantization calibration and wrong for scoring, so it must say so."""
    import prepare_yolo_dataset as prep

    src = _make_export(tmp_path)
    (src / "images" / "a.jpg.x.jpg").write_bytes(b"x")
    (src / "labels" / "a.txt").write_text("0 0.5 0.5 0.1 0.1\n")
    out = tmp_path / "staged"
    prep.main(["--src", str(src), "--out", str(out)])
    assert "THE SAME IMAGES" in (out / "data.yaml").read_text()


def test_val_split_produces_disjoint_sets(tmp_path):
    import prepare_yolo_dataset as prep

    src = _make_export(tmp_path)
    for i in range(10):
        (src / "images" / f"U{i}.jpg.x.jpg").write_bytes(b"x")
        (src / "labels" / f"U{i}.txt").write_text("0 0.5 0.5 0.1 0.1\n")
    out = tmp_path / "staged"
    prep.main(["--src", str(src), "--out", str(out), "--val-split", "0.3"])

    train = {p.stem for p in (out / "train" / "images").iterdir()}
    val = {p.stem for p in (out / "val" / "images").iterdir()}
    assert train and val
    assert not (train & val)
    assert len(train) + len(val) == 10
    assert "THE SAME IMAGES" not in (out / "data.yaml").read_text()


def test_missing_classes_file_is_an_error(tmp_path):
    import prepare_yolo_dataset as prep

    src = tmp_path / "data"
    (src / "images").mkdir(parents=True)
    (src / "labels").mkdir(parents=True)
    (src / "images" / "a.jpg").write_bytes(b"x")
    with pytest.raises(SystemExit):
        prep.main(["--src", str(src), "--out", str(tmp_path / "staged")])


def test_a_directory_that_is_not_an_export_is_an_error(tmp_path):
    import prepare_yolo_dataset as prep

    with pytest.raises(SystemExit):
        prep.main(["--src", str(tmp_path), "--out", str(tmp_path / "staged")])


def test_restaging_does_not_accumulate_stale_links(tmp_path):
    """Re-running after images are removed from the export must not leave the
    old symlinks behind — they would point at files that no longer exist and
    Ultralytics would fail on a dataset that looks fine in git."""
    import prepare_yolo_dataset as prep

    src = _make_export(tmp_path)
    for i in range(3):
        (src / "images" / f"U{i}.jpg.x.jpg").write_bytes(b"x")
        (src / "labels" / f"U{i}.txt").write_text("0 0.5 0.5 0.1 0.1\n")
    out = tmp_path / "staged"
    prep.main(["--src", str(src), "--out", str(out)])
    assert len(list((out / "val" / "images").iterdir())) == 3

    (src / "images" / "U2.jpg.x.jpg").unlink()
    prep.main(["--src", str(src), "--out", str(out)])
    assert len(list((out / "val" / "images").iterdir())) == 2
