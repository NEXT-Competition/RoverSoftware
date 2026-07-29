"""The base station's named field positions (basestation/places.py).

The rules worth pinning down are the ones that protect an operator standing in
a field: a place must survive a restart, a corrupt file must not stop the base
station launching, and a coordinate that isn't one must be refused rather than
stored and drawn somewhere wrong on the map.
"""

import json

import pytest

from basestation.places import MAX_PLACES, PlaceStore, clean, slugify


@pytest.fixture
def store(tmp_path):
    return PlaceStore(path=str(tmp_path / "places.json"))


# --- ids ---------------------------------------------------------------------

def test_slugify_makes_a_legal_id_from_anything_typed():
    assert slugify("Bucket A (far)") == "bucket_a_far"
    assert slugify("  ") == "place"
    # Must start with a letter, like every other id in the system.
    assert slugify("2nd bucket").startswith("p_")


def test_slugify_never_collides():
    assert slugify("Bucket A", ("bucket_a",)) == "bucket_a_2"
    assert slugify("Bucket A", ("bucket_a", "bucket_a_2")) == "bucket_a_3"


def test_a_duplicate_id_is_re_derived_from_the_name_not_dropped(store):
    # Losing the second place would lose a position somebody drove out to take.
    store.replace([
        {"id": "bucket_a", "name": "Bucket A", "lat": 1.0, "lon": 2.0},
        {"id": "bucket_a", "name": "Bucket A again", "lat": 3.0, "lon": 4.0},
    ])
    ids = [p["id"] for p in store.snapshot()]
    assert ids == ["bucket_a", "bucket_a_again"]


def test_two_places_named_the_same_get_numbered(store):
    store.replace([
        {"name": "Bucket A", "lat": 1.0, "lon": 2.0},
        {"name": "Bucket A", "lat": 3.0, "lon": 4.0},
    ])
    assert [p["id"] for p in store.snapshot()] == ["bucket_a", "bucket_a_2"]


# --- coercion ----------------------------------------------------------------

@pytest.mark.parametrize("bad", [
    {"name": "no position"},
    {"name": "off the planet", "lat": 91.0, "lon": 0.0},
    {"name": "not a number", "lat": "over there", "lon": 0.0},
    {"name": "nan", "lat": float("nan"), "lon": 0.0},
    "a string",
])
def test_a_place_that_is_not_a_position_is_refused(bad):
    place, reason = clean(bad)
    assert place is None and reason


def test_an_unknown_kind_falls_back_rather_than_refusing():
    # The position is the valuable half. Losing a saved bucket over a typo in a
    # label the operator can fix in one tap would be the wrong trade.
    place, _ = clean({"name": "B", "lat": 1.0, "lon": 2.0, "kind": "wharrgarbl"})
    assert place is not None and place["kind"] == "marker"


def test_coordinates_are_rounded_to_roughly_a_centimetre():
    place, _ = clean({"name": "B", "lat": 1.123456789, "lon": 2.0})
    assert place["lat"] == 1.1234568


def test_the_list_is_capped(store):
    result = store.replace([
        {"name": f"P{n}", "lat": 1.0, "lon": float(n)} for n in range(MAX_PLACES + 5)
    ])
    assert len(store.snapshot()) == MAX_PLACES
    assert result["rejected"]


# --- persistence -------------------------------------------------------------

def test_places_survive_a_restart(tmp_path):
    path = str(tmp_path / "places.json")
    PlaceStore(path=path).replace(
        [{"name": "Bucket A", "lat": 37.1, "lon": -122.2, "kind": "bucket"}])

    reopened = PlaceStore(path=path).snapshot()
    assert len(reopened) == 1
    assert reopened[0]["name"] == "Bucket A"
    assert reopened[0]["kind"] == "bucket"


def test_a_corrupt_file_leaves_the_base_station_launchable(tmp_path):
    path = tmp_path / "places.json"
    path.write_text("{ this is not json", encoding="utf-8")
    assert PlaceStore(path=str(path)).snapshot() == []


def test_a_file_of_the_wrong_shape_is_ignored(tmp_path):
    path = tmp_path / "places.json"
    path.write_text(json.dumps({"places": "nope"}), encoding="utf-8")
    assert PlaceStore(path=str(path)).snapshot() == []


def test_an_unwritable_path_reports_rather_than_raises(tmp_path):
    # A read-only home directory must not undo an edit that already took effect
    # in memory — the same rule settings.py follows.
    store = PlaceStore(path=str(tmp_path / "nope") + "\0/places.json", load=False)
    result = store.replace([{"name": "B", "lat": 1.0, "lon": 2.0}])
    assert result["save_error"]
    assert len(store.snapshot()) == 1


def test_replace_is_a_whole_list_swap(store):
    store.replace([{"name": "A", "lat": 1.0, "lon": 1.0}])
    store.replace([{"name": "B", "lat": 2.0, "lon": 2.0}])
    assert [p["name"] for p in store.snapshot()] == ["B"]


def test_snapshot_is_a_copy(store):
    store.replace([{"name": "A", "lat": 1.0, "lon": 1.0}])
    snap = store.snapshot()
    snap[0]["name"] = "mutated"
    assert store.snapshot()[0]["name"] == "A"
