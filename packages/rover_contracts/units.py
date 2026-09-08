"""The unit boundary.

A11: every bounded numeric the model emits is an integer -- whole degrees for a
heading, whole percent of full scale for a power, whole milliseconds for a
duration.  The rover is open loop, so the only continuous quantity anything
integrates is heading, and every wrap of an angle happens here so no other
module does bare arithmetic on one.

Rounding is half-away-from-zero, not Python's banker's rounding, so an integer
survives a round trip through a float.
"""

from __future__ import annotations

import math

__all__ = [
    "PERMILLE_FULL",
    "bearing_deg_from_center_x",
    "deg_to_rad",
    "heading_error_deg",
    "rad_to_deg",
    "round_half_away",
    "wrap_deg_180",
    "wrap_deg_360",
]

PERMILLE_FULL = 1000
"""Full-width divisor for ``center_x_permille`` (ARCHITECTURE 5.5)."""


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return float(value)


def round_half_away(value: float) -> int:
    """Round half away from zero."""
    value = _finite(value, "value")
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def deg_to_rad(deg: float) -> float:
    return math.radians(_finite(deg, "deg"))


def rad_to_deg(rad: float) -> float:
    return math.degrees(_finite(rad, "rad"))


def wrap_deg_180(deg: float) -> float:
    """Into (-180, 180], the range the controller's fused yaw uses."""
    deg = _finite(deg, "deg")
    wrapped = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def wrap_deg_360(deg: float) -> float:
    """Into [0, 360), the range ``turn_to`` and the WorldState use."""
    return _finite(deg, "deg") % 360.0


def heading_error_deg(target_deg: float, current_deg: float) -> float:
    """The shortest signed rotation from ``current`` to ``target``, in
    (-180, 180].  Positive means turn left (counter-clockwise from above), the
    same sign as a positive yaw rate."""
    target = _finite(target_deg, "target_deg")
    current = _finite(current_deg, "current_deg")
    return wrap_deg_180(target - current)


def bearing_deg_from_center_x(center_x_permille: int, hfov_deg: float) -> float:
    """Bearing to an object the model located at ``center_x_permille``.

    ARCHITECTURE 5.5: ``(0.5 - center_x_permille/1000) * hfov_deg``, computed on
    the host.  The model never emits an angle; ``+`` is left, the same sign as
    :func:`heading_error_deg`.
    """
    if not 0 <= center_x_permille <= PERMILLE_FULL:
        raise ValueError(
            f"center_x_permille must be 0..{PERMILLE_FULL}, got {center_x_permille!r}"
        )
    return (0.5 - center_x_permille / PERMILLE_FULL) * _finite(hfov_deg, "hfov_deg")
