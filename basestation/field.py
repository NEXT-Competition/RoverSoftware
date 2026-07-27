"""Geodetic boundary of the field this rover fleet operates on.

Corners for "Field #3" at 4609 Rapidan River Rd, Fairfax, VA 22030 — traced to
the painted touchlines/goal lines, not the turf run-off (the turf extends
another few meters past this on every side). Found the same way as the turf
edge before it: pixel-located the (faint, low-contrast) white lines in a
rendered map screenshot, then converted those pixels to lat/lon through the
live map's own projection. Rectangle, 100m x 64.8m — a standard FIFA pitch
size, which cross-checks that this reading is right.
"""

FIELD_BOUNDARY = [
    (38.83332617755978, -77.32379862315594),  # top-left in the dashboard's default (unflipped) view
    (38.83364321026917, -77.32317191504019),  # top-right
    (38.832889862250944, -77.32254387145103),  # bottom-right
    (38.832572826185995, -77.32317057956675),  # bottom-left
]


def point_in_polygon(lat: float, lon: float, polygon=FIELD_BOUNDARY) -> bool:
    """PNPOLY ray-casting test; lat/lon treated as planar (fine at field scale)."""
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        lat_i, lon_i = polygon[i]
        lat_j, lon_j = polygon[j]
        if (lon_i > lon) != (lon_j > lon) and (
            lat < (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i) + lat_i
        ):
            inside = not inside
        j = i
    return inside
