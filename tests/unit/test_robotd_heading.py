"""The heading tracker: wrap, ``yaw_sign``, rate and age."""

from __future__ import annotations

import pytest
from rover_robotd.heading import HeadingTracker

MS = 1_000_000


def test_a_fresh_tracker_has_no_sample() -> None:
    tracker = HeadingTracker(1)
    assert not tracker.available
    assert tracker.heading_deg == 0.0
    assert tracker.yaw_rate_dps is None
    assert tracker.age_ms(10 * MS) is None


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [(0.0, 0.0), (90.0, 90.0), (180.0, 180.0), (-180.0, 180.0), (190.0, -170.0), (-190.0, 170.0), (540.0, 180.0)],
)
def test_the_heading_is_wrapped_into_the_half_open_turn(yaw: float, expected: float) -> None:
    tracker = HeadingTracker(1)
    tracker.update(yaw, 0)
    assert tracker.heading_deg == pytest.approx(expected)
    assert -180.0 < tracker.heading_deg <= 180.0


def test_yaw_sign_flips_the_boards_convention() -> None:
    """``[link] yaw_sign = -1`` makes a board whose yaw grows turning right
    report a heading that grows turning left."""
    flipped = HeadingTracker(-1)
    flipped.update(30.0, 0)
    assert flipped.heading_deg == pytest.approx(-30.0)
    flipped.update(-180.0, 50 * MS)
    assert flipped.heading_deg == pytest.approx(180.0)


def test_the_rate_is_the_wrapped_change_over_the_arrival_gap() -> None:
    tracker = HeadingTracker(1)
    tracker.update(10.0, 0)
    assert tracker.yaw_rate_dps is None, "one sample has no rate"
    tracker.update(15.0, 50 * MS)
    assert tracker.yaw_rate_dps == pytest.approx(100.0)


def test_the_rate_survives_the_wrap_at_180() -> None:
    tracker = HeadingTracker(1)
    tracker.update(175.0, 0)
    tracker.update(-175.0, 100 * MS)  # ten degrees further left, not 350 right
    assert tracker.yaw_rate_dps == pytest.approx(100.0)


def test_a_sample_that_does_not_advance_the_clock_has_no_rate() -> None:
    tracker = HeadingTracker(1)
    tracker.update(0.0, 5 * MS)
    tracker.update(10.0, 5 * MS)
    assert tracker.yaw_rate_dps is None
    assert tracker.heading_deg == 10.0


def test_age_is_measured_from_the_arrival_stamp() -> None:
    tracker = HeadingTracker(1)
    tracker.update(0.0, 1_000 * MS)
    assert tracker.age_ms(1_000 * MS) == 0.0
    assert tracker.age_ms(1_120 * MS) == pytest.approx(120.0)
    assert tracker.age_ms(999 * MS) == 0.0, "a clock going backwards is not a negative age"


def test_reset_keeps_the_last_heading_but_forgets_the_continuity() -> None:
    tracker = HeadingTracker(1)
    tracker.update(45.0, 0)
    tracker.update(50.0, 50 * MS)
    tracker.reset()
    assert tracker.heading_deg == 50.0
    assert not tracker.available
    assert tracker.yaw_rate_dps is None
    tracker.update(90.0, 200 * MS)
    assert tracker.yaw_rate_dps is None, "the first sample after a reset has no rate"


def test_only_plus_or_minus_one_is_a_sign() -> None:
    with pytest.raises(ValueError, match="yaw_sign"):
        HeadingTracker(0)
