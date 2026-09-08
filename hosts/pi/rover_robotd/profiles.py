"""What a skill streams: ``drive_for`` and ``turn_to`` as functions of time
and heading, and the twist mixer.

robotd executes the profile and the controller applies power only (ADR-0013).
There is no encoder, so ``drive_for`` is a power for a time and ``turn_to`` is
a proportional law closed on the controller's fused yaw.  Both are recomputed
every control period from live state -- the elapsed time, the latest heading
sample -- rather than played back from a table, so a profile that is not being
stepped produces nothing at all.
"""

from __future__ import annotations

import math
from typing import Final

from rover_contracts.units import heading_error_deg, wrap_deg_180

__all__ = ["RAMP_S", "DriveForProfile", "TurnToProfile", "mix_twist"]

RAMP_S: Final = 0.100
"""``drive_for``'s linear ramp in and out, inside the requested duration."""


class DriveForProfile:
    """Both sides at ``power`` for ``duration_s``, ramped over the first and
    last :data:`RAMP_S` so the wheels do not step.  The sign is the direction."""

    __slots__ = ("power", "duration_s", "ramp_s")

    def __init__(
        self, power: float, duration_s: float, *, ramp_s: float = RAMP_S
    ) -> None:
        if not math.isfinite(power) or power == 0.0:
            raise ValueError(f"power must be finite and non-zero, got {power!r}")
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError(f"duration_s must be positive, got {duration_s!r}")
        self.power = float(power)
        self.duration_s = float(duration_s)
        # A drive shorter than two ramps ramps to its midpoint and back.
        self.ramp_s = min(float(ramp_s), self.duration_s / 2.0)

    def command(self, t_s: float) -> tuple[float, float]:
        """``(left, right)`` at ``t_s`` seconds after the start."""
        if t_s < 0.0 or t_s >= self.duration_s:
            return (0.0, 0.0)
        scale = 1.0
        if self.ramp_s > 0.0:
            scale = min(1.0, t_s / self.ramp_s, (self.duration_s - t_s) / self.ramp_s)
        power = self.power * scale
        return (power, power)

    def done(self, t_s: float) -> bool:
        return t_s >= self.duration_s

    def fraction(self, t_s: float) -> float:
        """Completion in ``0.0..1.0``, for ``state.active.progress``."""
        return min(1.0, max(0.0, t_s / self.duration_s))


class TurnToProfile:
    """Rotate in place until the heading is within ``tolerance_deg`` of
    ``target_deg`` for two consecutive feedback samples.

    The command is ``ang = clamp(kp * |error|, power_min, power_max)`` with the
    sign of the error, applied as ``left = -ang, right = +ang``; a positive
    error is a left turn.  Inside the tolerance the command is zero, so the
    floor never pushes a finished turn back out of it.  ``observe`` is fed one
    heading per feedback sample and keys on the sample stamp, so the control
    loop can call it every period without counting one sample twice.
    """

    __slots__ = (
        "target_deg",
        "tolerance_deg",
        "kp",
        "power_min",
        "power_max",
        "error_deg",
        "turned_deg",
        "_initial_error",
        "_last_heading",
        "_last_sample_ns",
        "_in_tolerance",
    )

    def __init__(
        self,
        target_deg: float,
        *,
        tolerance_deg: float,
        kp: float,
        power_min: float,
        power_max: float,
    ) -> None:
        if tolerance_deg <= 0.0 or kp <= 0.0:
            raise ValueError("tolerance_deg and kp must be positive")
        if not 0.0 <= power_min <= power_max:
            raise ValueError("need 0 <= power_min <= power_max")
        self.target_deg = float(target_deg)
        self.tolerance_deg = float(tolerance_deg)
        self.kp = float(kp)
        self.power_min = float(power_min)
        self.power_max = float(power_max)
        self.error_deg: float | None = None
        self.turned_deg = 0.0
        self._initial_error: float | None = None
        self._last_heading: float | None = None
        self._last_sample_ns: int | None = None
        self._in_tolerance = 0

    def observe(self, heading_deg: float, sample_ns: int) -> None:
        """Take the latest heading sample.  A repeated stamp changes nothing."""
        if sample_ns == self._last_sample_ns:
            return
        if self._last_heading is not None:
            self.turned_deg += wrap_deg_180(heading_deg - self._last_heading)
        self._last_heading = heading_deg
        self._last_sample_ns = sample_ns
        self.error_deg = heading_error_deg(self.target_deg, heading_deg)
        if self._initial_error is None:
            self._initial_error = abs(self.error_deg)
        if abs(self.error_deg) <= self.tolerance_deg:
            self._in_tolerance += 1
        else:
            self._in_tolerance = 0

    @property
    def done(self) -> bool:
        return self._in_tolerance >= 2

    def command(self) -> tuple[float, float]:
        """``(left, right)`` for the latest sample; zeros before the first."""
        error = self.error_deg
        if error is None or abs(error) <= self.tolerance_deg:
            return (0.0, 0.0)
        magnitude = min(self.power_max, max(self.power_min, self.kp * abs(error)))
        ang = math.copysign(magnitude, error)
        return (-ang, ang)

    def fraction(self) -> float:
        """Completion in ``0.0..1.0``: how much of the initial error is gone."""
        if self.error_deg is None or not self._initial_error:
            return 0.0
        return min(1.0, max(0.0, 1.0 - abs(self.error_deg) / self._initial_error))


def mix_twist(lin: float, ang: float, limit: float) -> tuple[float, float]:
    """A35's stream as wheel powers: ``left = lin - ang``, ``right = lin + ang``,
    each clamped to ``[limits] twist_power``.  A positive ``ang`` turns left."""
    if not (math.isfinite(lin) and math.isfinite(ang)):
        return (0.0, 0.0)
    left = max(-limit, min(limit, lin - ang))
    right = max(-limit, min(limit, lin + ang))
    return (left, right)
