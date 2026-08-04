"""Which IMU reader this build has, given its config.

One line of dispatch, in its own module for a boring reason: the two readers
share a base class (`imu_common.HeadingSource`) and a factory living in either
of them would import the other, which is a cycle. It also gives `Robot` and the
tools one name to call instead of a conditional each.

    i2c       SHTP over I2C. Everything the chip can say — a fused rotation
              vector, a calibrated gyro, a calibration accuracy level, and
              commands back to it — on a bus with no per-packet checksum.
    uart_rvc  19-byte checksummed frames at 100 Hz, one direction only.
              Corruption is detected instead of becoming a plausible heading;
              the price is no calibration level, no gyro and no commands. See
              sensors/bno085_rvc.py, which spells out what that costs.
"""

from __future__ import annotations

from typing import Optional

from ..config import IMUConfig
from .bno085 import IMU
from .bno085_rvc import RVCIMU
from .imu_common import HeadingSource

MODES = ("i2c", "uart_rvc")


def build_imu(config: IMUConfig) -> Optional[HeadingSource]:
    """The reader this build's config asks for, or None if the IMU is off.

    An unknown mode falls back to I2C with a line in the journal rather than
    raising: a typo in an env var must cost you a heading source, not a rover
    that will not boot. Same rule as `PoseEstimator`'s heading_source.
    """
    if not config.enabled:
        return None
    mode = config.mode
    if mode not in MODES:
        print(f"[IMU] unknown mode {mode!r} — using 'i2c' "
              f"(one of {', '.join(MODES)})")
        mode = "i2c"
    if mode == "uart_rvc":
        return RVCIMU(port=config.port, baud=config.baud,
                      heading_offset_deg=config.heading_offset_deg,
                      invert=config.invert,
                      sample_timeout=config.sample_timeout)
    return IMU(config.i2c_address, config.heading_offset_deg, config.invert,
               config.min_calib, config.persist_calibration,
               sample_timeout=config.sample_timeout)
