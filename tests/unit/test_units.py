"""The unit boundary: every conversion the model's integers cross (A11)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from hypothesis import given
from hypothesis import strategies as st

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.units import (  # noqa: E402
    bearing_deg_from_center_x,
    cm_to_m,
    cms_to_mps,
    deg_to_rad,
    dps_to_mrad_s,
    dps_to_radps,
    m_to_cm,
    m_to_mm,
    mm_s_to_mps,
    mm_to_m,
    mps_to_cms,
    mps_to_mm_s,
    mrad_s_to_radps,
    rad_to_deg,
    radps_to_dps,
    radps_to_mrad_s,
)


def test_documented_values():
    assert cm_to_m(40) == pytest.approx(0.40)
    assert cms_to_mps(15) == pytest.approx(0.15)
    assert mps_to_mm_s(0.250) == 250
    assert radps_to_mrad_s(0.210) == 210
    assert dps_to_mrad_s(60) == 1047  # ARCHITECTURE 6's 1.047 rad/s
    assert mm_to_m(231) == pytest.approx(0.231)
    assert m_to_mm(0.045) == 45


@given(st.integers(min_value=-100, max_value=100))
def test_cm_round_trip(cm):
    assert m_to_cm(cm_to_m(cm)) == cm


@given(st.integers(min_value=5, max_value=30))
def test_cms_round_trip(cms):
    assert mps_to_cms(cms_to_mps(cms)) == cms


@given(st.integers(min_value=-32768, max_value=32767))
def test_mm_s_round_trip(mm_s):
    assert mps_to_mm_s(mm_s_to_mps(mm_s)) == mm_s


@given(st.integers(min_value=-32768, max_value=32767))
def test_mrad_s_round_trip(mrad_s):
    assert radps_to_mrad_s(mrad_s_to_radps(mrad_s)) == mrad_s


@given(st.integers(min_value=-1000, max_value=1000))
def test_mm_round_trip(mm):
    assert m_to_mm(mm_to_m(mm)) == mm


@given(st.floats(min_value=-180.0, max_value=180.0, allow_nan=False))
def test_deg_rad_round_trip(deg):
    assert rad_to_deg(deg_to_rad(deg)) == pytest.approx(deg, abs=1e-9)
    assert radps_to_dps(dps_to_radps(deg)) == pytest.approx(deg, abs=1e-9)


def test_rounding_is_half_away_from_zero():
    # Python's round() is banker's rounding and would give 0 for both of these,
    # which the C core would not reproduce.
    assert m_to_cm(0.005) == 1
    assert m_to_cm(-0.005) == -1
    assert m_to_cm(0.015) == 2
    assert mps_to_mm_s(0.0005) == 1
    assert mps_to_mm_s(-0.0005) == -1


@pytest.mark.parametrize(
    "func",
    [m_to_cm, mps_to_cms, m_to_mm, mps_to_mm_s, deg_to_rad, rad_to_deg,
     dps_to_radps, radps_to_dps, radps_to_mrad_s, dps_to_mrad_s],
)
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_rejected(func, bad):
    with pytest.raises(ValueError):
        func(bad)


def test_bearing_is_a_pure_function_of_permille_and_hfov():
    # ARCHITECTURE 5.5: (0.5 - center_x_permille/1000) * hfov_deg, + = left = CCW.
    assert bearing_deg_from_center_x(500, 83.0) == pytest.approx(0.0)
    assert bearing_deg_from_center_x(0, 83.0) == pytest.approx(41.5)
    assert bearing_deg_from_center_x(1000, 83.0) == pytest.approx(-41.5)
    assert bearing_deg_from_center_x(610, 83.0) == pytest.approx(-9.13)


def test_bearing_sign_points_toward_the_object():
    # An object left of centre must produce a positive (CCW) turn; the inverse
    # form makes find turn away from the object.
    assert bearing_deg_from_center_x(200, 83.0) > 0
    assert bearing_deg_from_center_x(800, 83.0) < 0


def test_bearing_scales_with_the_calibrated_hfov():
    # 120 (the Wide lens's diagonal) inflates every bearing by ~45% over the
    # 4:3 crop's 83, which is why hfov_deg is a config key and never a literal.
    wrong = bearing_deg_from_center_x(610, 120.0)
    right = bearing_deg_from_center_x(610, 83.0)
    assert abs(wrong) > abs(right) * 1.4


@pytest.mark.parametrize("bad", [-1, 1001])
def test_permille_out_of_range_rejected(bad):
    with pytest.raises(ValueError):
        bearing_deg_from_center_x(bad, 83.0)
