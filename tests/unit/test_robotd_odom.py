"""Odometry integrated from raw ticks: signs, units and wraparound."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_robotd.odom import (  # noqa: E402
    Geometry,
    Odometry,
    tick_delta,
    wrap_angle,
)

GEOMETRY = Geometry(wheel_radius_m=0.045, track_m=0.150, ticks_per_rev=2200)


def _odom() -> Odometry:
    odom = Odometry(GEOMETRY)
    odom.rebase(0, 0)
    return odom


def test_metres_per_tick_matches_the_configured_wheel() -> None:
    assert GEOMETRY.metres_per_tick == pytest.approx(
        2 * math.pi * 0.045 / 2200, rel=1e-12
    )


def test_one_wheel_revolution_is_one_circumference() -> None:
    odom = _odom()
    step = odom.update(2200, 2200, 1.0)
    assert step.ds_m == pytest.approx(2 * math.pi * 0.045, rel=1e-12)
    assert step.dyaw_rad == pytest.approx(0.0, abs=1e-15)
    assert odom.pose.x_m == pytest.approx(2 * math.pi * 0.045, rel=1e-12)
    assert odom.pose.y_m == pytest.approx(0.0, abs=1e-15)


def test_forward_is_plus_x_and_the_right_wheel_leading_is_ccw() -> None:
    """REP-103: +x forward, +z up.  The right wheel leading turns left."""
    odom = _odom()
    step = odom.update(-100, 100, 0.1)
    assert step.ds_m == pytest.approx(0.0, abs=1e-15)
    assert step.dyaw_rad > 0.0
    assert odom.pose.yaw_rad > 0.0


def test_a_pure_spin_moves_no_path() -> None:
    odom = _odom()
    odom.update(-500, 500, 0.5)
    assert odom.path_m == pytest.approx(0.0, abs=1e-12)


def test_yaw_change_uses_the_track_width() -> None:
    odom = _odom()
    per_tick = GEOMETRY.metres_per_tick
    ticks = 400
    step = odom.update(-ticks, ticks, 0.4)
    expected = 2 * ticks * per_tick / GEOMETRY.track_m
    assert step.dyaw_rad == pytest.approx(expected, rel=1e-12)


def test_a_quarter_turn_then_a_drive_lands_on_plus_y() -> None:
    odom = _odom()
    per_tick = GEOMETRY.metres_per_tick
    spin_ticks = round(math.pi / 2 * GEOMETRY.track_m / 2 / per_tick)
    odom.update(-spin_ticks, spin_ticks, 0.5)
    assert odom.pose.yaw_rad == pytest.approx(math.pi / 2, abs=2e-3)
    forward = round(0.5 / per_tick)
    odom.update(-spin_ticks + forward, spin_ticks + forward, 1.0)
    assert odom.pose.x_m == pytest.approx(0.0, abs=2e-3)
    assert odom.pose.y_m == pytest.approx(0.5, abs=2e-3)


def test_twist_is_the_step_over_its_own_dt() -> None:
    odom = _odom()
    step = odom.update(1100, 1100, 0.5)
    assert step.twist.linear_x_mps == pytest.approx(step.ds_m / 0.5, rel=1e-12)
    assert step.twist.angular_z_radps == pytest.approx(0.0, abs=1e-15)


def test_ticks_wrap_at_int32_without_a_pose_jump() -> None:
    odom = Odometry(GEOMETRY)
    odom.rebase(2**31 - 10, 2**31 - 10)
    step = odom.update(-(2**31) + 10, -(2**31) + 10, 0.02)
    assert step.ds_m == pytest.approx(20 * GEOMETRY.metres_per_tick, rel=1e-12)


@pytest.mark.parametrize(
    ("now", "previous", "expected"),
    [
        (5, 3, 2),
        (3, 5, -2),
        (-(2**31), 2**31 - 1, 1),
        (2**31 - 1, -(2**31), -1),
        (0, 0, 0),
    ],
)
def test_tick_delta_is_wrap_safe(now: int, previous: int, expected: int) -> None:
    assert tick_delta(now, previous) == expected


def test_rebase_adopts_counters_without_moving_the_pose() -> None:
    """An MCU restart restarts the counters; the gap is not motion we saw."""
    odom = _odom()
    odom.update(1000, 1000, 0.1)
    before = odom.pose
    odom.rebase(-4_000_000, 999_999)
    step = odom.update(-4_000_000, 999_999, 0.1)
    assert step.ds_m == 0.0
    assert odom.pose == before


def test_the_first_update_only_seeds_the_counters() -> None:
    odom = Odometry(GEOMETRY)
    step = odom.update(123_456, 654_321, 0.02)
    assert step.ds_m == 0.0 and step.dyaw_rad == 0.0
    assert odom.pose.x_m == 0.0


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi)],
)
def test_wrap_angle_folds_into_a_half_open_turn(angle: float, expected: float) -> None:
    assert wrap_angle(angle) == pytest.approx(expected)
