"""Named test sites the base station can point the map + simulator at.

Each site is a real, GPS-locatable place the team drives/tests robots at.
Only the soccer field has a hard boundary (real painted lines measured
against); GMU's engineering plaza is open pavement, so it has none — the
simulator skips the fence check entirely when a site's boundary is None (see
SimulatedFleet.set_site).
"""

from .field import FIELD_BOUNDARY

SITES = {
    "soccer_field": {
        "name": "Fairfax Soccer Field",
        "origin": (38.8331773, -77.3232135),
        "boundary": FIELD_BOUNDARY,
        "rotation_deg": 33,
        "zoom": 19.5,
    },
    "gmu_plaza": {
        "name": "GMU Engineering Plaza",
        "origin": (38.82815, -77.30527),
        "boundary": None,
        "rotation_deg": 0,
        "zoom": 19.5,
    },
}

DEFAULT_SITE = "soccer_field"
