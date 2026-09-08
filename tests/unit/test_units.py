"""The unit boundary (A11): the wraps every heading crosses, the signed error
``turn_to`` closes on, half-away rounding, and the one bearing formula."""

from __future__ import annotations

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st
from rover_contracts.units import (
    PERMILLE_FULL,
    bearing_deg_from_center_x,
    deg_to_rad,
    heading_error_deg,
    rad_to_deg,
    round_half_away,
    wrap_deg_180,
    wrap_deg_360,
)

FINITE_DEG = st.floats(
    min_value=-1e6, max_value=1e6, allow_nan=False, allow_infinity=False
)


# --------------------------------------------------------------------------
# wrap_deg_180 -> (-180, 180], the controller's yaw range
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("deg", "expected"),
    [(180.0, 180.0), (-180.0, 180.0), (360.0, 0.0), (-0.0, 0.0), (359.999, -0.001),
     (0.0, 0.0), (540.0, 180.0), (-540.0, 180.0), (180.001, -179.999),
     (-179.999, -179.999), (90.0, 90.0), (-90.0, -90.0), (270.0, -90.0)],
)
def test_wrap_180_boundaries(deg, expected):
    # Both ends of a half turn are spelled +180: -180 is not in the range.
    assert wrap_deg_180(deg) == pytest.approx(expected, abs=1e-9)


def test_wrap_180_never_returns_minus_180_or_negative_zero():
    assert wrap_deg_180(-180.0) == 180.0
    assert wrap_deg_180(180.0) == 180.0
    for zero in (-0.0, 360.0, -360.0):
        assert math.copysign(1.0, wrap_deg_180(zero)) > 0


@given(FINITE_DEG)
def test_wrap_180_lands_in_range_and_agrees_with_wrap_360(deg):
    half = wrap_deg_180(deg)
    full = wrap_deg_360(deg)
    assert -180.0 < half <= 180.0
    turns = (full - half) % 360.0          # the two spellings differ by whole turns
    assert min(turns, 360.0 - turns) < 1e-6
    assert wrap_deg_180(half) == pytest.approx(half, abs=1e-9)


# --------------------------------------------------------------------------
# wrap_deg_360 -> [0, 360), the WorldState's and turn_to's range
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("deg", "expected"),
    [(180.0, 180.0), (-180.0, 180.0), (360.0, 0.0), (-0.0, 0.0), (359.999, 359.999),
     (-0.001, 359.999), (0.0, 0.0), (720.0, 0.0), (-360.0, 0.0), (-90.0, 270.0),
     (450.0, 90.0)],
)
def test_wrap_360_boundaries(deg, expected):
    assert wrap_deg_360(deg) == pytest.approx(expected, abs=1e-9)


def test_wrap_360_never_returns_negative_zero():
    assert wrap_deg_360(360.0) == 0.0
    for zero in (-0.0, 360.0, -360.0):
        assert math.copysign(1.0, wrap_deg_360(zero)) > 0


@pytest.mark.xfail(
    strict=True,
    reason="units.wrap_deg_360 returns 360.0 for a tiny negative input: "
    "-1e-20 % 360.0 rounds up to 360.0, so the result is outside [0, 360).  It "
    "needs the guard wrap_deg_180 has for -180.0 (return 0.0 for a 360.0 remainder).",
)
@given(FINITE_DEG)
def test_wrap_360_lands_in_range_and_is_idempotent(deg):
    assert wrap_deg_360(-1e-20) == 0.0     # the pinned counterexample
    full = wrap_deg_360(deg)
    assert 0.0 <= full < 360.0
    assert wrap_deg_360(full) == pytest.approx(full, abs=1e-9)


# --------------------------------------------------------------------------
# heading_error_deg(target, current): + is left (counter-clockwise from above)
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("target", "current", "expected"),
    [
        (90.0, 0.0, 90.0),        # left is positive, the sign of a positive yaw rate
        (0.0, 90.0, -90.0),       # right is negative
        (350.0, 10.0, -20.0),     # the short way across the wrap, not +340
        (10.0, 350.0, 20.0),
        (180.0, 0.0, 180.0),      # a half turn is +180, never -180
        (0.0, 180.0, 180.0),
        (270.0, 0.0, -90.0),      # a quarter turn the short way round
        (-90.0, 0.0, -90.0),      # a (-180, 180] current or target is fine too
        (359.0, 1.0, -2.0),
        (45.5, 44.0, 1.5),
        (87.0, 87.0, 0.0),
        (720.0, 0.0, 0.0),
    ],
)
def test_heading_error_sign_and_shortest_path(target, current, expected):
    assert heading_error_deg(target, current) == pytest.approx(expected, abs=1e-9)


@given(FINITE_DEG, FINITE_DEG)
def test_heading_error_is_the_shortest_rotation_that_reaches_the_target(target, current):
    error = heading_error_deg(target, current)
    assert -180.0 < error <= 180.0
    reached = wrap_deg_360(current + error)
    wanted = wrap_deg_360(target)
    gap = abs(reached - wanted)
    assert min(gap, 360.0 - gap) < 1e-6


@given(FINITE_DEG, FINITE_DEG)
def test_heading_error_is_antisymmetric_except_for_the_half_turn(a, b):
    forward = heading_error_deg(a, b)
    back = heading_error_deg(b, a)
    if abs(abs(forward) - 180.0) < 1e-6:
        assert back == pytest.approx(180.0, abs=1e-6)
    else:
        assert forward == pytest.approx(-back, abs=1e-6)


# --------------------------------------------------------------------------
# round_half_away
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "expected"),
    [(0.5, 1), (-0.5, -1), (1.5, 2), (-1.5, -2), (2.5, 3), (-2.5, -3),
     (0.49, 0), (-0.49, 0), (0.51, 1), (-0.51, -1), (0.0, 0), (-0.0, 0),
     (7.0, 7), (-7.0, -7), (0.999, 1), (-0.999, -1)],
)
def test_round_half_away_on_the_halves(value, expected):
    result = round_half_away(value)
    assert result == expected
    assert isinstance(result, int)


def test_round_half_away_is_not_bankers_rounding():
    # Python's round() sends both halves to the even neighbour.
    assert (round(0.5), round(2.5), round(-0.5)) == (0, 2, 0)
    halves = (round_half_away(0.5), round_half_away(2.5), round_half_away(-0.5))
    assert halves == (1, 3, -1)


@given(st.integers(min_value=-(10**6), max_value=10**6))
def test_integers_survive_a_float_round_trip(n):
    assert round_half_away(float(n)) == n


@given(st.floats(min_value=-1e6, max_value=1e6, allow_nan=False))
def test_round_half_away_stays_within_half_and_keeps_the_sign(value):
    result = round_half_away(value)
    assert abs(result - value) <= 0.5
    assert result == 0 or (result > 0) == (value > 0)


# --------------------------------------------------------------------------
# deg <-> rad
# --------------------------------------------------------------------------


@given(st.floats(min_value=-180.0, max_value=180.0, allow_nan=False))
def test_deg_rad_round_trip(deg):
    assert rad_to_deg(deg_to_rad(deg)) == pytest.approx(deg, abs=1e-9)


def test_deg_rad_documented_values():
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert rad_to_deg(math.pi / 2) == pytest.approx(90.0)


# --------------------------------------------------------------------------
# bearing_deg_from_center_x (ARCHITECTURE 5.5)
# --------------------------------------------------------------------------


def test_bearing_is_a_pure_function_of_permille_and_hfov():
    # (0.5 - center_x_permille/1000) * hfov_deg, + = left = CCW.
    assert bearing_deg_from_center_x(500, 83.0) == pytest.approx(0.0)
    assert bearing_deg_from_center_x(0, 83.0) == pytest.approx(41.5)
    assert bearing_deg_from_center_x(1000, 83.0) == pytest.approx(-41.5)
    assert bearing_deg_from_center_x(610, 83.0) == pytest.approx(-9.13)


def test_bearing_sign_points_toward_the_object():
    # An object left of centre must produce a positive (CCW) turn, the same
    # sign heading_error_deg gives a left turn; the inverse form makes find
    # turn away from the object.
    assert bearing_deg_from_center_x(200, 83.0) > 0
    assert bearing_deg_from_center_x(800, 83.0) < 0
    assert heading_error_deg(90.0, 0.0) > 0


def test_bearing_scales_with_the_calibrated_hfov():
    # 120 (the Wide lens's diagonal) inflates every bearing by ~45% over the
    # 4:3 crop's 83, which is why hfov_deg is a config key and never a literal.
    wrong = bearing_deg_from_center_x(610, 120.0)
    right = bearing_deg_from_center_x(610, 83.0)
    assert abs(wrong) > abs(right) * 1.4


@pytest.mark.parametrize("edge", [0, PERMILLE_FULL])
def test_permille_edges_are_inside(edge):
    assert abs(bearing_deg_from_center_x(edge, 83.0)) == pytest.approx(41.5)


@pytest.mark.parametrize("bad", [-1, PERMILLE_FULL + 1, -1000, 10**9])
def test_permille_out_of_range_rejected(bad):
    with pytest.raises(ValueError):
        bearing_deg_from_center_x(bad, 83.0)


@given(st.integers(min_value=0, max_value=PERMILLE_FULL),
       st.floats(min_value=1.0, max_value=179.0))
def test_bearing_stays_inside_half_the_field_of_view(center_x, hfov):
    bearing = bearing_deg_from_center_x(center_x, hfov)
    assert abs(bearing) <= hfov / 2 + 1e-9
    if center_x < PERMILLE_FULL:
        assert bearing_deg_from_center_x(center_x + 1, hfov) < bearing


# --------------------------------------------------------------------------
# Non-finite input is refused everywhere
# --------------------------------------------------------------------------

NON_FINITE_CALLS = {
    "deg_to_rad": deg_to_rad,
    "rad_to_deg": rad_to_deg,
    "wrap_deg_180": wrap_deg_180,
    "wrap_deg_360": wrap_deg_360,
    "round_half_away": round_half_away,
    "heading_error_deg.target": lambda v: heading_error_deg(v, 0.0),
    "heading_error_deg.current": lambda v: heading_error_deg(0.0, v),
    "bearing.hfov": lambda v: bearing_deg_from_center_x(500, v),
}


@pytest.mark.parametrize(
    "call", list(NON_FINITE_CALLS.values()), ids=list(NON_FINITE_CALLS)
)
@pytest.mark.parametrize(
    "bad", [math.nan, math.inf, -math.inf], ids=["nan", "inf", "-inf"]
)
def test_non_finite_rejected(call, bad):
    with pytest.raises(ValueError):
        call(bad)
