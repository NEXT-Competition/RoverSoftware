"""u-blox NEO-6M (GY-GPS6MV2) GPS reader for waypoint autonomy.

The module speaks plain NMEA at 9600 baud out of the box. We read sentences on a
background thread and cache the latest valid fix, so the 50 Hz control loop can
poll `pose()` without ever blocking on the serial port.

    gps = GPS("/dev/ttyAMA0", 9600)
    gps.start()
    ...
    fix = gps.pose()          # -> (lat, lon, heading_deg) or None
    gps.stop()

This satisfies the WaypointController's `pose_provider` contract:
    pose() -> (lat, lon, heading_deg) or None
where heading is 0 = North, clockwise-positive.

--- Heading caveat (read this) ---
The NEO-6M has NO compass. The only heading it can give is *course over ground*
(the direction you're actually travelling), and that's only meaningful while
moving. So:
  * Before the rover moves, heading is unknown; we report the last known course
    (0 = North until the first one arrives). The waypoint PID will still start
    driving and self-correct as soon as motion produces a real course.
  * Below `min_move_mps` the reported course is GPS noise, so we ignore it and
    hold the last good heading.
For heading that's valid at a standstill you'd add a magnetometer/IMU and fuse
it here; the rest of the stack wouldn't change.

Graceful degradation: if pyserial/pynmea2 aren't installed, or the device can't
be opened, `start()` logs why and the reader stays inert — `pose()` returns None
and waypoint mode simply holds position (never crashes on a dev laptop).
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

try:  # pragma: no cover - hardware/deps optional on a dev laptop
    import serial
except Exception:
    serial = None

try:  # pragma: no cover
    import pynmea2
except Exception:
    pynmea2 = None

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)

_KNOTS_TO_MPS = 0.514444


class GPS:
    def __init__(
        self,
        port: str,
        baud: int = 9600,
        fix_timeout: float = 5.0,
        min_move_mps: float = 0.5,
    ):
        self.port = port
        self.baud = baud
        self.fix_timeout = fix_timeout      # a fix older than this is treated as lost
        self.min_move_mps = min_move_mps    # below this, course-over-ground is noise

        self._serial = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        # Latest cached fix (guarded by _lock).
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._heading = 0.0            # degrees, 0 = North, CW positive
        self._have_heading = False    # True once a real (moving) course arrived
        self._speed_mps = 0.0
        self._last_fix = 0.0          # monotonic timestamp of the last valid position

    def start(self) -> None:
        """Open the port and begin reading NMEA on a background thread."""
        if serial is None or pynmea2 is None:
            print("[GPS] pyserial/pynmea2 not installed — GPS disabled "
                  "(waypoint mode will hold position). Install: pip install pyserial pynmea2")
            return
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
        except Exception as e:
            print(f"[GPS] could not open {self.port} @ {self.baud}: {e} — GPS disabled")
            return
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="gps-rx", daemon=True)
        self._thread.start()
        print(f"[GPS] reading NMEA on {self.port} @ {self.baud} baud")

    def _read_loop(self) -> None:
        while self._running:
            try:
                raw = self._serial.readline()  # up to '\n' or the 1s read timeout
            except Exception as e:  # keep the reader alive across transient errors
                print(f"[GPS] read error: {e}")
                time.sleep(0.2)
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line.startswith("$"):
                continue  # partial line or noise
            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue  # garbage frame (often a baud/wiring problem)
            self._consume(msg)

    def _consume(self, msg) -> None:
        """Fold one parsed NMEA sentence into the cached fix.

        We use GGA (position + fix quality) and RMC (position + course + speed);
        everything else (GSV/GSA/VTG/GLL) is ignored.
        """
        stype = getattr(msg, "sentence_type", "")

        # Only trust position from a sentence that reports a valid fix.
        if stype == "GGA":
            try:
                if int(msg.gps_qual) == 0:
                    return  # 0 = no fix yet (acquiring satellites)
            except (TypeError, ValueError):
                return
        elif stype == "RMC":
            if getattr(msg, "status", "V") != "A":
                return  # 'V' = navigation receiver warning (no valid fix)
        else:
            return

        lat = getattr(msg, "latitude", None)
        lon = getattr(msg, "longitude", None)
        if lat is None or lon is None:
            return
        if lat == 0.0 and lon == 0.0:
            return  # null-island sentinel => not a real fix

        with self._lock:
            self._lat = lat
            self._lon = lon
            self._last_fix = time.monotonic()

            # Heading + speed come only from RMC's course-over-ground.
            if stype == "RMC":
                spd = getattr(msg, "spd_over_grnd", None)
                self._speed_mps = (
                    float(spd) * _KNOTS_TO_MPS if spd not in (None, "") else 0.0
                )
                course = getattr(msg, "true_course", None)
                # Course is only meaningful while actually moving.
                if course not in (None, "") and self._speed_mps >= self.min_move_mps:
                    self._heading = float(course)
                    self._have_heading = True

    def pose(self) -> Optional[Pose]:
        """Latest (lat, lon, heading_deg), or None if there's no fresh fix.

        heading is None until a real course-over-ground has been observed (the
        NEO-6M has no compass, so heading only exists once we're moving). A
        caller must not treat a missing heading as "pointing North" — that's what
        made the rover spin in place. See WaypointController for how it's handled.
        """
        with self._lock:
            if self._lat is None or self._lon is None:
                return None
            if (time.monotonic() - self._last_fix) > self.fix_timeout:
                return None  # stale: satellites lost / antenna unplugged
            heading = self._heading if self._have_heading else None
            return (self._lat, self._lon, heading)

    def has_heading(self) -> bool:
        """True once a real course-over-ground (from motion) has been observed."""
        with self._lock:
            return self._have_heading

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
