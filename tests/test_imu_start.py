"""IMU start-up contract: a sick BNO085 must never wedge the robot's boot.

These are the regressions behind a field failure where `python3 run_robot.py`
printed the GPS line and then hung forever with the ESCs armed and no control
loop: IMU.start() opened the sensor on the main thread, and the driver's
calibration handshake drains every queued packet with an uncapped loop, so with
the reports already streaming on a slow I2C bus it never returned.

No hardware here — the Pi-only libs (board/busio/adafruit_bno08x) are absent on
a dev laptop anyway, so we substitute fakes for them.
"""

import time
import types

import pytest

from robot.sensors import bno085


@pytest.fixture
def fake_libs(monkeypatch):
    """Make the module think the Pi sensor libraries are installed."""
    monkeypatch.setattr(bno085, "board", types.SimpleNamespace(SCL=1, SDA=2))
    monkeypatch.setattr(bno085, "busio",
                        types.SimpleNamespace(I2C=lambda *a, **k: object()))
    monkeypatch.setattr(bno085, "adafruit_bno08x", types.SimpleNamespace())
    monkeypatch.setattr(bno085, "BNO_REPORT_ROTATION_VECTOR", "rotvec")
    monkeypatch.setattr(bno085, "BNO_REPORT_GYROSCOPE", "gyro")


class _FakeSensor:
    """Records the order of the start-up calls and serves canned readings."""

    def __init__(self, calls, *a, **k):
        self.calls = calls
        calls.append("init")

    def begin_calibration(self):
        self.calls.append("begin_calibration")

    def enable_feature(self, feature):
        self.calls.append(f"enable:{feature}")

    @property
    def quaternion(self):
        return (0.0, 0.0, 0.0, 1.0)

    @property
    def gyro(self):
        return (0.0, 0.0, 0.0)

    @property
    def calibration_status(self):
        self.calls.append("calib_poll")
        return 3

    def save_calibration_data(self):
        self.calls.append("save")


def test_start_returns_even_if_the_sensor_never_responds(fake_libs, monkeypatch):
    """The boot thread must not wait on the sensor — that was the hang."""

    class Hanging:
        def __init__(self, *a, **k):
            while True:  # never returns, like the uncapped drain loop
                time.sleep(0.05)

    monkeypatch.setattr(bno085, "BNO08X_I2C", Hanging)

    imu = bno085.IMU(0x4A)
    t0 = time.monotonic()
    imu.start()
    elapsed = time.monotonic() - t0
    try:
        assert elapsed < 1.0, "start() blocked on the sensor and would wedge boot"
        # Wedged sensor -> no readings, so heading cleanly falls back to GPS course.
        assert imu.heading() is None
        assert imu.yaw_rate() is None
        assert imu.has_heading() is False
    finally:
        imu.stop()  # daemon thread; stop() must not block forever either


def test_calibration_starts_before_the_reports_stream(fake_libs, monkeypatch):
    """begin_calibration() must run BEFORE enable_feature().

    Reversed, the sensor is already streaming when the calibration command waits
    for its ack, and on a slow bus the driver can never drain the queue dry.
    """
    calls = []
    monkeypatch.setattr(bno085, "BNO08X_I2C",
                        lambda *a, **k: _FakeSensor(calls, *a, **k))

    imu = bno085.IMU(0x4A, persist_calibration=True)
    imu.start()
    try:
        deadline = time.monotonic() + 2.0
        while "enable:gyro" not in calls and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        imu.stop()

    assert "begin_calibration" in calls, "dynamic calibration never started"
    assert calls.index("begin_calibration") < calls.index("enable:rotvec")
    assert calls.index("begin_calibration") < calls.index("enable:gyro")


def test_calibration_status_is_polled_at_about_1hz(fake_libs, monkeypatch):
    """Each poll is a bus round-trip, so it must not run at the sample rate."""
    calls = []
    monkeypatch.setattr(bno085, "BNO08X_I2C",
                        lambda *a, **k: _FakeSensor(calls, *a, **k))
    monkeypatch.setattr(bno085, "CALIB_POLL_S", 0.2)

    imu = bno085.IMU(0x4A, update_hz=100.0)
    imu.start()
    time.sleep(0.5)
    imu.stop()

    samples = sum(1 for c in calls if c == "calib_poll")
    # ~50 samples in that window; at one poll per sample this would be ~50.
    assert 1 <= samples <= 5, f"calibration polled {samples}x, expected ~3"
    # Readings still flow, and the converged calibration is still saved once.
    assert imu.heading() is not None
    assert calls.count("save") == 1


def test_uart_rvc_backend_returns_heading(monkeypatch):
    """The UART-RVC backend should expose headings through the shared IMU API."""

    class FakeSerial:
        def __init__(self, *a, **k):
            self.args = a
            self.kwargs = k

    class FakeRVC:
        def __init__(self, uart):
            self.uart = uart
            self.heading = (12.5, 1.0, 0.5, 0.0, 0.0, 0.0)

    monkeypatch.setattr(bno085, "serial", types.SimpleNamespace(Serial=FakeSerial))
    monkeypatch.setattr(bno085, "BNO08x_RVC", FakeRVC)

    imu = bno085.IMU(transport="uart_rvc", serial_port="/dev/serial0", serial_baud=115200)
    imu.start()
    try:
        deadline = time.monotonic() + 1.0
        while imu.heading() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert imu.heading() == pytest.approx(12.5)
    finally:
        imu.stop()
