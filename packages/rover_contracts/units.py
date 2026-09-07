"""The unit boundary.

A11: every bounded numeric the model emits is an integer -- cm, cm/s, degrees.
Internally the rover works in SI (m, m/s, rad, rad/s) and the serial link
(ARCHITECTURE 5.1) carries integer mm/s and mrad/s.  Every crossing between
those three worlds happens here, so no other module does bare arithmetic on a
unit.

Rounding is half-away-from-zero, not Python's banker's rounding, so that
``m_to_cm(cm_to_m(n)) == n`` holds for every integer n and so the C firmware
core can reproduce the same integers.
"""

from __future__ import annotations

import math

__all__ = [
    "PERMILLE_FULL",
    "bearing_deg_from_center_x",
    "cm_to_m",
    "cms_to_mps",
    "deg_to_rad",
    "dps_to_mrad_s",
    "dps_to_radps",
    "m_to_cm",
    "m_to_mm",
    "mm_s_to_mps",
    "mm_to_m",
    "mps_to_cms",
    "mps_to_mm_s",
    "mrad_s_to_radps",
    "rad_to_deg",
    "radps_to_dps",
    "radps_to_mrad_s",
]

PERMILLE_FULL = 1000
"""Full-width divisor for ``center_x_permille`` (ARCHITECTURE 5.5)."""


def _finite(value: float, name: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return float(value)


def _round(value: float) -> int:
    """Round half away from zero."""
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def cm_to_m(cm: int) -> float:
    """Centimetres (model) to metres (SI)."""
    return cm / 100.0


def m_to_cm(m: float) -> int:
    """Metres (SI) to centimetres (model)."""
    return _round(_finite(m, "m") * 100.0)


def cms_to_mps(cms: int) -> float:
    """cm/s (model) to m/s (SI)."""
    return cms / 100.0


def mps_to_cms(mps: float) -> int:
    """m/s (SI) to cm/s (model)."""
    return _round(_finite(mps, "mps") * 100.0)


def m_to_mm(m: float) -> int:
    """Metres (SI) to millimetres (wire)."""
    return _round(_finite(m, "m") * 1000.0)


def mm_to_m(mm: int) -> float:
    """Millimetres (wire) to metres (SI)."""
    return mm / 1000.0


def mps_to_mm_s(mps: float) -> int:
    """m/s (SI) to mm/s (``V.v_mm_s``)."""
    return _round(_finite(mps, "mps") * 1000.0)


def mm_s_to_mps(mm_s: int) -> float:
    """mm/s (``T.v_meas_mm_s``) to m/s (SI)."""
    return mm_s / 1000.0


def deg_to_rad(deg: float) -> float:
    """Degrees (model) to radians (SI)."""
    return math.radians(_finite(deg, "deg"))


def rad_to_deg(rad: float) -> float:
    """Radians (SI) to degrees (model)."""
    return math.degrees(_finite(rad, "rad"))


def dps_to_radps(dps: float) -> float:
    """deg/s (model) to rad/s (SI)."""
    return math.radians(_finite(dps, "dps"))


def radps_to_dps(radps: float) -> float:
    """rad/s (SI) to deg/s (model)."""
    return math.degrees(_finite(radps, "radps"))


def radps_to_mrad_s(radps: float) -> int:
    """rad/s (SI) to mrad/s (``V.w_mrad_s``)."""
    return _round(_finite(radps, "radps") * 1000.0)


def mrad_s_to_radps(mrad_s: int) -> float:
    """mrad/s (``T.w_meas_mrad_s``) to rad/s (SI)."""
    return mrad_s / 1000.0


def dps_to_mrad_s(dps: float) -> int:
    """deg/s (model) straight to mrad/s (wire), without a lossy SI round trip."""
    return _round(math.radians(_finite(dps, "dps")) * 1000.0)


def bearing_deg_from_center_x(center_x_permille: int, hfov_deg: float) -> float:
    """Bearing to an object the model located at ``center_x_permille``.

    ARCHITECTURE 5.5: ``(0.5 - center_x_permille/1000) * hfov_deg``, computed on
    the Pi.  The model never emits an angle; ``+`` is left is CCW, the same sign
    as ``w_mrad_s`` and ``turn.angle_deg``.
    """
    if not 0 <= center_x_permille <= PERMILLE_FULL:
        raise ValueError(
            f"center_x_permille must be 0..{PERMILLE_FULL}, got {center_x_permille!r}"
        )
    return (0.5 - center_x_permille / PERMILLE_FULL) * _finite(hfov_deg, "hfov_deg")
