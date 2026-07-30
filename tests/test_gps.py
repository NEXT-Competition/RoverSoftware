"""GPS reader contract: what the rover is allowed to believe about a fix.

The rules that matter here are the ones that steer a robot into a fence if they
break:

  * a position is only a position when the receiver says it HAS a fix —
    adafruit_gps keeps writing latitude/longitude through a no-fix sentence,
  * the track angle is only a heading while we're actually MOVING,
  * a fix that stopped updating is not a fix,
  * and a receiver spitting garbage must not take the reader thread down.

No hardware here — adafruit_gps is a Pi-only dependency (it pulls in Blinka), so
we substitute a fake for the parsed-state object it hands us.
"""

import types
from typing import Optional

import pytest

from robot.sensors import gps as gps_mod
from robot.sensors.gps import GPS
from robot.sensors.pose import PoseEstimator


class FakeChip:
    """Stands in for adafruit_gps.GPS: the parsed state, plus a sentence queue.

    `sentences` is what update() walks through — each entry is a dict of
    attributes to apply (or an Exception to raise, for the garbage-input test).
    """

    def __init__(self, sentences=None):
        self.sentences = list(sentences or [])
        self.commands = []
        # Same starting values the real library uses.
        self.fix_quality: Optional[int] = 0
        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.track_angle_deg: Optional[float] = None
        self.speed_knots: Optional[float] = None
        self.speed_kmh: Optional[float] = None
        self.satellites: Optional[int] = None
        self.altitude_m: Optional[float] = None
        self.horizontal_dilution: Optional[float] = None

    @property
    def has_fix(self):
        return self.fix_quality is not None and self.fix_quality >= 1

    def send_command(self, command, add_checksum=True):
        self.commands.append(command)

    def update(self):
        if not self.sentences:
            return False
        item = self.sentences.pop(0)
        if isinstance(item, Exception):
            raise item
        for k, v in item.items():
            setattr(self, k, v)
        return True

    def write(self, *a, **k):
        pass


def make_gps(chip, **kwargs):
    """A GPS wired to a fake chip, as if start() had opened the port."""
    g = GPS("/dev/null", 9600, **kwargs)
    g._gps = chip
    return g


# A moving fix: 1.0 knot ~= 0.51 m/s, which clears the default min_move_mps.
MOVING = {
    "fix_quality": 1, "latitude": 37.7749, "longitude": -122.4194,
    "track_angle_deg": 123.4, "speed_knots": 4.0, "satellites": 9,
    "altitude_m": 12.3, "horizontal_dilution": 0.9,
}
# The same fix parked: the receiver still reports a course, but it's noise.
STOPPED = {**MOVING, "track_angle_deg": 271.0, "speed_knots": 0.2}


def test_no_libs_start_is_inert(monkeypatch, capsys):
    """A dev laptop with no pyserial/adafruit_gps must degrade, not crash."""
    monkeypatch.setattr(gps_mod, "adafruit_gps", None)
    g = GPS("/dev/null")
    g.start()
    assert not g.is_running()
    assert g.pose() is None
    assert "adafruit_gps" in capsys.readouterr().out


def test_no_fix_yields_no_position():
    """adafruit_gps writes lat/lon even for a no-fix sentence — ignore it."""
    chip = FakeChip()
    chip.fix_quality = 0
    chip.latitude, chip.longitude = 37.7749, -122.4194  # stale, not a fix
    g = make_gps(chip)
    g._snapshot()
    assert g.pose() is None


def test_null_island_is_not_a_fix():
    chip = FakeChip()
    chip.fix_quality = 1
    chip.latitude, chip.longitude = 0.0, 0.0
    g = make_gps(chip)
    g._snapshot()
    assert g.pose() is None


def test_track_angle_becomes_heading_while_moving():
    chip = FakeChip()
    for k, v in MOVING.items():
        setattr(chip, k, v)
    g = make_gps(chip)
    g._snapshot()

    pose = g.pose()
    assert pose is not None
    lat, lon, heading = pose
    assert (lat, lon) == (37.7749, -122.4194)
    assert heading == pytest.approx(123.4)   # the GPS track angle, used directly
    assert g.has_heading()
    assert g.track_angle() == pytest.approx(123.4)
    assert g.speed_mps() == pytest.approx(4.0 * 0.514444)


def test_heading_is_none_until_we_have_moved():
    """A fix at a standstill gives a position but no trustworthy heading."""
    chip = FakeChip()
    for k, v in STOPPED.items():
        setattr(chip, k, v)
    g = make_gps(chip)
    g._snapshot()

    lat, lon, heading = g.pose()
    assert (lat, lon) == (37.7749, -122.4194)
    assert heading is None      # NOT 0.0 — that would read as "pointing North"
    assert not g.has_heading()


def test_stationary_noise_does_not_overwrite_a_good_heading():
    """Below min_move_mps the reported course wanders; hold the last good one."""
    chip = FakeChip([MOVING, STOPPED])
    g = make_gps(chip)
    chip.update()
    g._snapshot()
    chip.update()
    g._snapshot()

    assert g.pose()[2] == pytest.approx(123.4)  # not the 271.0 noise value


def test_track_angle_is_wrapped_into_0_360():
    chip = FakeChip([{**MOVING, "track_angle_deg": 361.5}])
    g = make_gps(chip)
    chip.update()
    g._snapshot()
    assert g.pose()[2] == pytest.approx(1.5)


def test_unknown_speed_is_distrusted():
    """No speed field => we can't tell if the course is real, so don't use it."""
    chip = FakeChip([{**MOVING, "speed_knots": None, "speed_kmh": None}])
    g = make_gps(chip)
    chip.update()
    g._snapshot()
    assert g.pose()[2] is None


def test_speed_falls_back_to_kmh():
    """VTG-only receivers report km/h; 3.6 km/h = 1 m/s clears min_move."""
    chip = FakeChip([{**MOVING, "speed_knots": None, "speed_kmh": 7.2}])
    g = make_gps(chip)
    chip.update()
    g._snapshot()
    assert g.speed_mps() == pytest.approx(2.0)
    assert g.pose()[2] == pytest.approx(123.4)


def test_stale_fix_drops_to_none():
    """Antenna unplugged / satellites lost: the last fix must expire."""
    chip = FakeChip([MOVING])
    g = make_gps(chip, fix_timeout=1.0)
    chip.update()
    g._snapshot()
    assert g.pose() is not None

    g._last_fix -= 2.0  # pretend two seconds passed with no new sentence
    assert g.pose() is None


def test_read_loop_survives_a_parse_error(capsys):
    """Garbage on the wire raises out of adafruit_gps; the thread must live."""
    chip = FakeChip([ValueError("invalid literal for int()"), MOVING])
    g = make_gps(chip)
    g._running = True
    try:
        g._read_loop_once()   # error pass
        g._read_loop_once()   # recovery pass
    finally:
        g._running = False

    assert g.pose() is not None
    assert "read/parse error" in capsys.readouterr().out


class FakePort:
    """A serial port that records what was written to it, usable as a context
    manager (the baud handshake opens one with `with`)."""

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.written = b""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def write(self, data):
        self.written += data

    def flush(self):
        pass

    def close(self):
        pass


def fake_serial(monkeypatch, ports):
    """Patch the serial module so every Serial(...) lands in `ports`."""
    def _open(port, baud, **kwargs):
        p = FakePort(port, baud, **kwargs)
        ports.append(p)
        return p

    monkeypatch.setattr(gps_mod, "serial", types.SimpleNamespace(Serial=_open))


def test_start_configures_sentences_and_rate(monkeypatch):
    """VTG (the track angle) plus BOTH rate commands: solve rate and talk rate.

    PMTK220 on its own is the subtle failure — the module keeps solving at its
    old rate and simply repeats each position, so the sentences speed up while
    the navigation data behind them does not.
    """
    chip = FakeChip()
    fake_serial(monkeypatch, [])
    monkeypatch.setattr(gps_mod, "adafruit_gps",
                        types.SimpleNamespace(GPS=lambda uart, debug=False: chip))
    g = GPS("/dev/null", 9600, update_rate_ms=200)
    try:
        g.start()
        assert g.is_running()
    finally:
        g.stop()

    assert chip.commands[0].startswith(b"PMTK314,0,1,1,1")  # GLL off, RMC/VTG/GGA on
    assert chip.commands[1] == b"PMTK300,200,0,0,0,0"
    assert chip.commands[2] == b"PMTK220,200"


def test_start_raises_the_module_baud_first(monkeypatch):
    """PMTK251 goes out at 9600 — the only baud a cold module is listening at.

    And it goes out on EVERY start: without the CR1220 backup battery the module
    forgets the change on each power cycle, so a one-off bring-up step would
    leave the rover reading garbage after its next battery swap.
    """
    ports = []
    fake_serial(monkeypatch, ports)
    monkeypatch.setattr(gps_mod, "adafruit_gps",
                        types.SimpleNamespace(GPS=lambda uart, debug=False: FakeChip()))
    monkeypatch.setattr(gps_mod.time, "sleep", lambda *_: None)
    g = GPS("/dev/null", 57600, update_rate_ms=200)
    try:
        g.start()
    finally:
        g.stop()

    handshake, reader = ports
    assert handshake.args[1] == 9600           # spoken at the module's boot baud
    assert handshake.written == b"$PMTK251,57600*2C\r\n"
    assert reader.args[1] == 57600             # ...then we listen at the new one


def test_no_baud_handshake_when_already_at_the_module_default(monkeypatch):
    """Nothing to change at 9600, so don't open a second port to say so."""
    ports = []
    fake_serial(monkeypatch, ports)
    monkeypatch.setattr(gps_mod, "adafruit_gps",
                        types.SimpleNamespace(GPS=lambda uart, debug=False: FakeChip()))
    g = GPS("/dev/null", 9600)
    try:
        g.start()
    finally:
        g.stop()

    assert len(ports) == 1
    assert ports[0].written == b""


def test_a_failed_baud_handshake_still_starts(monkeypatch, capsys):
    """A module already at the target baud reads fine without the handshake."""
    def _open(port, baud, **kwargs):
        if baud == 9600:
            raise OSError("port busy")
        return FakePort(port, baud, **kwargs)

    monkeypatch.setattr(gps_mod, "serial", types.SimpleNamespace(Serial=_open))
    monkeypatch.setattr(gps_mod, "adafruit_gps",
                        types.SimpleNamespace(GPS=lambda uart, debug=False: FakeChip()))
    g = GPS("/dev/null", 57600)
    try:
        g.start()
        assert g.is_running()
    finally:
        g.stop()
    assert "PMTK251" in capsys.readouterr().out


@pytest.mark.parametrize("rate_ms,baud,warns", [
    (1000, 9600, False),   # the old 1 Hz setup: fits fine, must stay quiet
    (200, 9600, True),     # 5 Hz of GGA+RMC+VTG is ~9500 bps of payload
    (200, 57600, False),   # ...which is why the default moved up
])
def test_link_budget_warning(rate_ms, baud, warns, capsys):
    """Too much NMEA for the baud truncates sentences, which reads as 'no fix'."""
    g = GPS("/dev/null", baud, update_rate_ms=rate_ms)
    g._check_link_budget()
    assert ("truncate" in capsys.readouterr().out) is warns


def test_rate_beyond_the_mtk3339_warns_but_is_not_clamped(capsys):
    """10 Hz is real on a PA1616D, so warn — don't silently override the ask."""
    g = GPS("/dev/null", 57600, update_rate_ms=100)
    g._check_link_budget()
    assert "repeat positions" in capsys.readouterr().out
    assert g.update_rate_ms == 100


def test_telemetry_reports_fix_health():
    chip = FakeChip([MOVING])
    g = make_gps(chip)
    chip.update()
    g._snapshot()

    t = g.telemetry()
    assert t["fix"] == 1
    assert t["sats"] == 9
    assert t["hdop"] == 0.9
    assert t["alt"] == 12.3
    assert t["track"] == pytest.approx(123.4)
    assert t["track_age"] >= 0.0

    g._last_fix -= 60.0
    assert g.telemetry()["fix"] == 0  # a stale fix reports as no fix


# --- PoseEstimator heading source ------------------------------------------

class _StubGPS:
    def __init__(self, heading):
        self._heading = heading

    def pose(self):
        return (1.0, 2.0, self._heading)


class _StubIMU:
    def __init__(self, heading):
        self._heading = heading

    def heading(self):
        return self._heading

    def yaw_rate(self):
        return 12.0


@pytest.mark.parametrize(
    "source,gps_heading,imu_heading,expected",
    [
        ("auto", 100.0, 200.0, 200.0),  # IMU wins when calibrated
        ("auto", 100.0, None, 100.0),   # falls back to the track angle
        ("gps", 100.0, 200.0, 100.0),   # track angle only, IMU ignored
        ("gps", None, 200.0, None),     # ...even with nothing to fall back to
        ("imu", 100.0, 200.0, 200.0),
        ("imu", 100.0, None, None),     # no fallback: heading stays unknown
    ],
)
def test_heading_source(source, gps_heading, imu_heading, expected):
    est = PoseEstimator(_StubGPS(gps_heading), _StubIMU(imu_heading), source)
    assert est.pose() == (1.0, 2.0, expected)


def test_unknown_heading_source_falls_back_to_auto(capsys):
    """A typo'd env var must not stop the rover from booting."""
    est = PoseEstimator(_StubGPS(100.0), _StubIMU(200.0), "compass")
    assert est.heading_source == "auto"
    assert est.pose() == (1.0, 2.0, 200.0)
    assert "compass" in capsys.readouterr().out


def test_gps_source_still_uses_the_gyro_for_the_pid_derivative():
    est = PoseEstimator(_StubGPS(100.0), _StubIMU(None), "gps")
    assert est.heading_rate() == 12.0


def test_no_gps_means_no_pose():
    est = PoseEstimator(None, _StubIMU(200.0), "auto")
    assert est.pose() is None
