"""Adafruit Ultimate GPS reader for waypoint autonomy.

Driven with Adafruit's own `adafruit_gps` library (the MTK3339/PA1616D breakout,
FeatherWing, or HAT) over a plain serial port, so the module is configured the
way Adafruit's examples do it — PMTK commands for which sentences to emit and how
fast — instead of us hand-parsing NMEA. We read on a background thread and cache
the latest valid fix, so the control loop can poll `pose()` without ever blocking
on the serial port.

    gps = GPS("/dev/ttyAMA0", 9600)
    gps.start()
    ...
    fix = gps.pose()          # -> (lat, lon, heading_deg) or None
    gps.stop()

This satisfies the WaypointController's `pose_provider` contract:
    pose() -> (lat, lon, heading_deg) or None
where heading is 0 = North, clockwise-positive.

--- Heading: the GPS track angle ---
The module reports a **track angle** (course over ground, from RMC/VTG), which IS
a true-North heading in degrees — no IMU, no magnetometer, no calibration dance,
and no magnetic declination to correct for. `pose()` returns it directly, so a
rover with no IMU at all still navigates waypoints on real headings.

Read this before you rip the IMU out, though: a track angle is the direction the
antenna is *travelling*, not the direction the chassis is *pointing*. So:
  * It is meaningless at a standstill — the receiver keeps emitting a number, but
    it's noise. Below `min_move_mps` we ignore it and hold the last good value.
  * It can't see a pivot in place: spin the rover on the spot and the track angle
    doesn't move, because the antenna isn't going anywhere.
  * Driving in reverse reads as a heading 180 degrees off.
That's why `PoseEstimator` still prefers the IMU when one is present and
calibrated (an absolute heading, valid at rest) and falls back to this. With
`heading_source="gps"` — or simply no IMU — the track angle carries heading
alone, and the waypoint controller's "drive straight to establish a course"
fallback covers the standstill case.

We ask the module for GGA + RMC + VTG: GGA carries fix quality, satellite count
and HDOP (what you actually want when a fix looks wrong in the field), RMC and
VTG carry the track angle and ground speed.

Graceful degradation: if pyserial/adafruit_gps aren't installed, or the device
can't be opened, `start()` logs why and the reader stays inert — `pose()` returns
None and waypoint mode simply holds position (never crashes on a dev laptop).

Not Adafruit hardware? This still works. `adafruit_gps` parses standard NMEA, so
a u-blox NEO-6M (what this rover used to carry) reads fine; it just ignores the
MTK-proprietary PMTK setup commands and stays at whatever it was configured for.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

try:  # pragma: no cover - hardware/deps optional on a dev laptop
    import serial
except Exception:
    serial = None

try:  # pragma: no cover - Pi-only (pulls in Blinka); see requirements.txt
    import adafruit_gps
except Exception:
    adafruit_gps = None

Pose = Tuple[float, float, Optional[float]]  # (lat, lon, heading_deg | None)

_KNOTS_TO_MPS = 0.514444

# PMTK_314_SET_NMEA_OUTPUT. Field order is GLL,RMC,VTG,GGA,GSA,GSV,... so this is
# "RMC + VTG + GGA, nothing else": position and fix quality (GGA) plus the track
# angle and ground speed (RMC/VTG), and none of the GSV satellite-detail spam
# that would eat the 9600-baud link. See the PMTK command reference:
# https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
_PMTK_OUTPUT_GGA_RMC_VTG = b"PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
_PMTK_UPDATE_RATE = b"PMTK220,%d"  # milliseconds between fixes

# Serial read timeout. adafruit_gps only calls readline() once a sentence-worth
# of bytes is buffered, so this is just a backstop against a truncated line
# wedging the reader thread.
_SERIAL_TIMEOUT = 0.5

# Sentences to drain per pass before yielding. A fix burst is 3 sentences (GGA,
# RMC, VTG); the cap keeps stop() responsive if the port ever floods.
_MAX_SENTENCES_PER_PASS = 16

_POLL_BUSY = 0.01   # the cap was hit, so more is waiting — come straight back
_POLL_IDLE = 0.05   # buffer drained; 20 Hz is ample for a 1-10 Hz module
_ERROR_LOG_INTERVAL = 5.0  # throttle repeated read/parse errors to one line per


class GPS:
    def __init__(
        self,
        port: str,
        baud: int = 9600,
        fix_timeout: float = 5.0,
        min_move_mps: float = 0.5,
        update_rate_ms: int = 1000,
    ):
        self.port = port
        self.baud = baud
        self.fix_timeout = fix_timeout      # a fix older than this is treated as lost
        self.min_move_mps = min_move_mps    # below this, the track angle is noise
        self.update_rate_ms = update_rate_ms

        self._serial = None
        self._gps = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._last_error_log = 0.0

        # Latest cached fix (guarded by _lock).
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._heading = 0.0            # degrees, 0 = North, CW positive
        self._have_heading = False    # True once a real (moving) track angle arrived
        self._heading_at = 0.0        # monotonic timestamp of that track angle
        self._speed_mps = 0.0
        self._last_fix = 0.0          # monotonic timestamp of the last valid position
        # Fix diagnostics, straight off GGA — what you look at when a fix is bad.
        self._fix_quality = 0
        self._satellites: Optional[int] = None
        self._altitude_m: Optional[float] = None
        self._hdop: Optional[float] = None

    def start(self) -> None:
        """Open the port, configure the module, and read on a background thread."""
        if serial is None or adafruit_gps is None:
            print("[GPS] pyserial/adafruit_gps not installed — GPS disabled "
                  "(waypoint mode will hold position). Install: "
                  "pip install pyserial adafruit-circuitpython-gps")
            return
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=_SERIAL_TIMEOUT)
        except Exception as e:
            print(f"[GPS] could not open {self.port} @ {self.baud}: {e} — GPS disabled")
            return

        self._gps = adafruit_gps.GPS(self._serial, debug=False)
        self._configure()

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, name="gps-rx", daemon=True)
        self._thread.start()
        print(f"[GPS] reading NMEA on {self.port} @ {self.baud} baud "
              f"({self.update_rate_ms} ms fixes, track angle from RMC/VTG)")

    def _configure(self) -> None:
        """Ask for GGA+RMC+VTG at `update_rate_ms`. Harmless on a non-MTK module."""
        # A faster rate needs a wider pipe: GGA+RMC+VTG is ~150-200 bytes per fix
        # and 9600 baud carries under 1 KB/s. Past ~5 Hz the sentences arrive
        # truncated and every checksum fails, which looks like "no fix" rather
        # than like a baud problem — so say it out loud instead.
        if self.update_rate_ms < 200 and self.baud <= 9600:
            print(f"[GPS] warning: {self.update_rate_ms} ms fixes will not fit in "
                  f"{self.baud} baud — raise the module baud (PMTK251) or the rate")
        try:
            self._gps.send_command(_PMTK_OUTPUT_GGA_RMC_VTG)
            self._gps.send_command(_PMTK_UPDATE_RATE % int(self.update_rate_ms))
        except Exception as e:  # a write failure shouldn't stop us from reading
            print(f"[GPS] could not send PMTK setup commands: {e}")

    def _read_loop(self) -> None:
        while self._running:
            self._read_loop_once()

    def _read_loop_once(self) -> None:
        """Drain the sentences that have arrived, cache the result, yield.

        Split out of the loop so tests can step the reader one pass at a time.
        """
        drained = 0
        try:
            while (self._running and drained < _MAX_SENTENCES_PER_PASS
                   and self._gps.update()):
                drained += 1
        except Exception as e:
            # adafruit_gps does string arithmetic on whatever arrives, so a noisy
            # line (wrong baud, marginal wiring) can raise out of update(). Keep
            # the reader alive, and log at most one line per _ERROR_LOG_INTERVAL
            # so a bad cable can't flood the journal.
            now = time.monotonic()
            if now - self._last_error_log >= _ERROR_LOG_INTERVAL:
                self._last_error_log = now
                print(f"[GPS] read/parse error: {e}")
            time.sleep(0.2)
            return
        if drained:
            self._snapshot()
        time.sleep(_POLL_BUSY if drained >= _MAX_SENTENCES_PER_PASS else _POLL_IDLE)

    def _snapshot(self) -> None:
        """Copy the library's parsed state into our locked cache.

        Called after draining sentences. `adafruit_gps` writes latitude/longitude
        even for a sentence that reports NO fix (it zeroes fix_quality but keeps
        the coordinates), so gating on `has_fix` here is what stops the rover
        from navigating to a garbage position.
        """
        g = self._gps
        if not g.has_fix:
            return  # acquiring satellites; whatever lat/lon holds is not a fix
        lat, lon = g.latitude, g.longitude
        if lat is None or lon is None:
            return
        if lat == 0.0 and lon == 0.0:
            return  # null-island sentinel => not a real fix

        speed = _speed_mps(g)
        track = g.track_angle_deg
        now = time.monotonic()

        with self._lock:
            self._lat = float(lat)
            self._lon = float(lon)
            self._last_fix = now
            self._speed_mps = speed

            # Heading = track angle (course over ground), only trustworthy while
            # actually moving — see the module docstring.
            if track is not None and speed >= self.min_move_mps:
                self._heading = float(track) % 360.0
                self._have_heading = True
                self._heading_at = now

            self._fix_quality = g.fix_quality or 0
            self._satellites = g.satellites
            self._altitude_m = g.altitude_m
            self._hdop = g.horizontal_dilution

    def pose(self) -> Optional[Pose]:
        """Latest (lat, lon, heading_deg), or None if there's no fresh fix.

        heading is the GPS track angle, and is None until a real one has been
        observed (it only exists once we're moving). A caller must not treat a
        missing heading as "pointing North" — that's what made the rover spin in
        place. See WaypointController for how it's handled.
        """
        with self._lock:
            if self._lat is None or self._lon is None:
                return None
            if (time.monotonic() - self._last_fix) > self.fix_timeout:
                return None  # stale: satellites lost / antenna unplugged
            heading = self._heading if self._have_heading else None
            return (self._lat, self._lon, heading)

    def is_running(self) -> bool:
        """True when the reader thread is up, i.e. start() actually opened the port."""
        return self._running

    def has_heading(self) -> bool:
        """True once a real track angle (from motion) has been observed."""
        with self._lock:
            return self._have_heading

    def track_angle(self) -> Optional[float]:
        """Last trusted course over ground in degrees (0 = North, CW+), or None."""
        with self._lock:
            return self._heading if self._have_heading else None

    def speed_mps(self) -> float:
        """Ground speed in m/s as of the last fix (0.0 when unknown)."""
        with self._lock:
            return self._speed_mps

    def telemetry(self) -> dict:
        """Compact fix-health summary for the base station.

        Satellite count and HDOP are the two numbers that explain a bad fix, and
        you can't read them off a rover sitting in a field — so they go on the
        radio. Kept small and rounded: the XBee link is 57600 baud and shared.
        """
        with self._lock:
            fresh = (
                self._lat is not None
                and (time.monotonic() - self._last_fix) <= self.fix_timeout
            )
            t = {
                "fix": int(self._fix_quality) if fresh else 0,
                "sats": self._satellites,
                "speed": round(self._speed_mps, 2),
            }
            if self._hdop is not None:
                t["hdop"] = round(float(self._hdop), 1)
            if self._altitude_m is not None:
                t["alt"] = round(float(self._altitude_m), 1)
            if self._have_heading:
                t["track"] = round(self._heading, 1)
                # How long ago that track angle was measured. A heading held from
                # a minute ago is still reported, and this is how you know.
                t["track_age"] = round(time.monotonic() - self._heading_at, 1)
            return t

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass


def _speed_mps(g) -> float:
    """Ground speed in m/s from whichever speed field the module gave us.

    RMC carries knots, VTG carries both knots and km/h. Unknown speed reads as
    0.0, which makes `_snapshot` distrust the track angle — the safe direction: a
    bogus heading steers the rover, a missing one only delays it.
    """
    knots = getattr(g, "speed_knots", None)
    if knots is not None:
        return float(knots) * _KNOTS_TO_MPS
    kmh = getattr(g, "speed_kmh", None)
    if kmh is not None:
        return float(kmh) / 3.6
    return 0.0
