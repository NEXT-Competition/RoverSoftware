"""Color-detection alignment controller (autonomy scaffold).

Turns the robot to face a detected colored object and creeps toward it, using a
PID on the object's horizontal offset in the camera frame.

This is intentionally decoupled from *how* the object is detected: you pass in a
`target_provider` callable that returns the normalized horizontal error in
[-1, 1] (0 = centered, +1 = far right), or None when nothing is seen. That
provider is where OpenCV color-threshold + blob detection will live. Because the
provider is injected, this controller runs and unit-tests fine with no camera.
"""

from __future__ import annotations

from typing import Callable, Optional

from .commands import DriveCommand
from .controller import Controller
from .pid import PID

# target_provider() -> horizontal error in [-1, 1], or None if no target
TargetProvider = Callable[[], Optional[float]]


class ColorAlignController(Controller):
    name = "color_align"

    def __init__(
        self,
        target_provider: Optional[TargetProvider] = None,
        forward_speed: float = 0.25,
        aligned_tolerance: float = 0.05,
        pid: Optional[PID] = None,
    ):
        self.target_provider = target_provider
        self.forward_speed = forward_speed
        self.aligned_tolerance = aligned_tolerance
        self.pid = pid or PID(kp=0.9, ki=0.0, kd=0.08, out_limit=0.8)

    def set_target_provider(self, provider: TargetProvider) -> None:
        self.target_provider = provider

    def on_activate(self) -> None:
        self.pid.reset()

    def update(self, dt: float) -> Optional[DriveCommand]:
        if self.target_provider is None:
            return DriveCommand.stopped()  # no perception wired up yet

        error = self.target_provider()
        if error is None:
            # TODO: search behavior (slow rotate) until a target reappears.
            self.pid.reset()
            return DriveCommand.stopped()

        steer = self.pid.update(error, dt)
        # Drive forward only once roughly aligned; otherwise turn in place.
        forward = self.forward_speed if abs(error) < self.aligned_tolerance * 4 else 0.0
        return DriveCommand.arcade(forward, steer)
