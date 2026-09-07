"""Odometry, integrated on the Pi from raw ticks (A7, principle 7).

Names and meanings mirror the standard robot messages so that a later ROS 2 or
LeRobot bridge is a rename rather than a re-derivation: :class:`Pose` is
``nav_msgs/Odometry``'s ``pose.pose`` in the ``odom`` frame, :class:`Twist` is
``geometry_msgs/Twist``, and the signs are REP-103 -- **+x forward, +z up**, so
a positive ``angular_z`` is counter-clockwise and is produced by the right wheel
leading the left.  That is the same sign as ``V.w_mrad_s``, ``turn.angle_deg``
and ``bearing_deg``.

Both encoders count **up when their wheel drives the robot forward**; the
firmware owns the wheel geometry (A6) and this is the one convention it has to
match.  Counts arrive as ``i32`` and wrap, so every difference goes through
:func:`tick_delta`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final

__all__ = [
    "Geometry",
    "OdomStep",
    "Odometry",
    "Pose",
    "Twist",
    "tick_delta",
    "wrap_angle",
]

_INT32_SPAN: Final = 1 << 32
_INT32_HALF: Final = 1 << 31


def tick_delta(now: int, previous: int) -> int:
    """Signed difference between two ``i32`` encoder counts, wrap-safe."""
    return ((now - previous + _INT32_HALF) % _INT32_SPAN) - _INT32_HALF


def wrap_angle(rad: float) -> float:
    """Fold an angle into ``(-pi, +pi]``."""
    folded = math.remainder(rad, math.tau)
    return math.pi if folded == -math.pi else folded


@dataclass(frozen=True, slots=True)
class Geometry:
    """``[robot]`` geometry, in SI."""

    wheel_radius_m: float
    track_m: float
    ticks_per_rev: int

    @property
    def metres_per_tick(self) -> float:
        return math.tau * self.wheel_radius_m / self.ticks_per_rev


@dataclass(frozen=True, slots=True)
class Pose:
    """``nav_msgs/Odometry`` ``pose.pose``, flattened to the plane."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class Twist:
    """``geometry_msgs/Twist``, flattened to the plane."""

    linear_x_mps: float = 0.0
    angular_z_radps: float = 0.0


@dataclass(frozen=True, slots=True)
class OdomStep:
    """One integration step: what moved, and how fast it was moving."""

    ds_m: float
    dyaw_rad: float
    dt_s: float
    twist: Twist


class Odometry:
    """Dead reckoning from raw ticks.

    The pose is integrated over the step's *mid* heading rather than its start
    heading, which is the exact answer for a constant-curvature arc to second
    order and keeps a 45 degree sweep from accumulating a visible bias.
    """

    def __init__(self, geometry: Geometry) -> None:
        self.geometry = geometry
        self.pose = Pose()
        self.path_m = 0.0
        self._left: int | None = None
        self._right: int | None = None

    def rebase(self, left_ticks: int, right_ticks: int) -> None:
        """Adopt new raw counters without moving the pose.

        Used on an MCU restart: the counters restart at whatever the new boot
        reports and the difference across the gap is not motion we observed.
        """
        self._left = left_ticks
        self._right = right_ticks

    def update(self, left_ticks: int, right_ticks: int, dt_s: float) -> OdomStep:
        """Integrate one telemetry frame.  ``dt_s`` is host-monotonic."""
        if self._left is None or self._right is None:
            self.rebase(left_ticks, right_ticks)
            return OdomStep(0.0, 0.0, dt_s, Twist())

        per_tick = self.geometry.metres_per_tick
        left_m = tick_delta(left_ticks, self._left) * per_tick
        right_m = tick_delta(right_ticks, self._right) * per_tick
        self._left = left_ticks
        self._right = right_ticks

        ds = (left_m + right_m) / 2.0
        dyaw = (right_m - left_m) / self.geometry.track_m
        mid_yaw = self.pose.yaw_rad + dyaw / 2.0
        self.pose = Pose(
            x_m=self.pose.x_m + ds * math.cos(mid_yaw),
            y_m=self.pose.y_m + ds * math.sin(mid_yaw),
            yaw_rad=wrap_angle(self.pose.yaw_rad + dyaw),
        )
        self.path_m += abs(ds)
        twist = (
            Twist(ds / dt_s, dyaw / dt_s) if dt_s > 0.0 else Twist()
        )
        return OdomStep(ds, dyaw, dt_s, twist)
