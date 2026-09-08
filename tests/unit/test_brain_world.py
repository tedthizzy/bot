"""The WorldState brain builds from robotd's ``state`` message.

ARCHITECTURE 5.6 as ADR-0013 amends it: a heading in 0..359, a front range in
centimetres that is ``null`` when unknown (I-16: null is not a clear path), an
``obstacle_ahead`` that is what blocks forward motion at the controller, and a
budget read from robotd's own ledger (I-15).  No pose, no speed, no metre.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.main import world_from_state  # noqa: E402
from rover_contracts.config import LimitsConfig, SafetyConfig  # noqa: E402
from rover_contracts.messages import ResultStatus, StateMessage  # noqa: E402
from rover_contracts.wave_proto import StopFlag  # noqa: E402
from rover_contracts.worldstate import RecentlySeen, WorldState  # noqa: E402

LIMITS = LimitsConfig()
SAFETY = SafetyConfig()  # tof_stop_mm 250


def state(
    *,
    heading_deg: float = -93.4,
    stop_flags: int = 0,
    front_m: float | None = 1.204,
    motion: bool = False,
    motion_s: float = 7.9,
    bumper: bool = False,
    pct: int = 62,
) -> StateMessage:
    return StateMessage.model_validate(
        {
            "t_utc_ns": 1,
            "t_mono_ns": 2,
            "rover": {
                "fw": "bot-0.1",
                "hb_ok": True,
                "stop_flags": stop_flags,
                "feedback_age_ms": 40,
                "cmd_left": 0.0,
                "cmd_right": 0.0,
                "heading_deg": heading_deg,
                "yaw_rate_dps": 0.0,
                "roll_deg": 0.0,
                "pitch_deg": 0.0,
                "temp_c": 30.0,
                "clamp_count": 0,
                "motion": motion,
            },
            "twist": {"lin": 0.0, "ang": 0.0},
            "front_m": front_m,
            "bumper": bumper,
            "estop_sw": False,
            "battery": {"pack_v": 11.7, "pct": pct},
            "active": None,
            "budget": {"motion_s": motion_s},
            "ready": True,
            "reason": "",
        }
    )


def build(message: StateMessage | None, **over: Any) -> WorldState:
    kwargs: dict[str, Any] = {
        "limits": LIMITS,
        "safety": SAFETY,
        "allow_motion": True,
    } | over
    return world_from_state(message, **kwargs)


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (-93.4, 267),
        (87.0, 87),
        (180.0, 180),
        (-0.4, 0),
        (-0.6, 359),
        (179.6, 180),
        (0.5, 1),
    ],
)
def test_the_heading_is_the_yaw_folded_into_0_to_359(yaw: float, expected: int) -> None:
    world = build(state(heading_deg=yaw))
    assert world.heading_deg == expected
    assert 0 <= world.heading_deg <= 359


@pytest.mark.parametrize(
    ("front_m", "expected"),
    [(1.204, 120), (0.0, 0), (0.255, 26), (3.99, 399), (6.0, 400), (None, None)],
)
def test_the_front_range_is_whole_centimetres_or_null(
    front_m: float | None, expected: int | None
) -> None:
    assert build(state(front_m=front_m)).front_range_cm == expected


def test_a_null_range_is_unknown_not_an_obstacle_and_not_clear() -> None:
    """I-16 is robotd's and the firmware's to enforce; the model is told null
    means unknown, and the flags say whether forward is blocked."""
    world = build(state(front_m=None))
    assert world.front_range_cm is None
    assert world.obstacle_ahead is False
    blocked = build(state(front_m=None, stop_flags=int(StopFlag.TOF)))
    assert blocked.obstacle_ahead is True


@pytest.mark.parametrize(
    ("flags", "expected"),
    [
        (0, False),
        (int(StopFlag.TOF), True),
        (int(StopFlag.BUMPER), True),
        (int(StopFlag.TOF | StopFlag.BUMPER), True),
        (int(StopFlag.HEARTBEAT | StopFlag.COAST), False),
        (int(StopFlag.LOWBAT), False),
    ],
)
def test_obstacle_ahead_follows_the_forward_blocking_stop_flags(
    flags: int, expected: bool
) -> None:
    assert build(state(stop_flags=flags, front_m=2.0)).obstacle_ahead is expected


@pytest.mark.parametrize(
    ("front_m", "expected"),
    [(0.2, True), (0.249, True), (0.25, False), (0.3, False), (2.0, False)],
)
def test_a_valid_range_inside_tof_stop_mm_is_an_obstacle(
    front_m: float, expected: bool
) -> None:
    assert build(state(front_m=front_m)).obstacle_ahead is expected
    wider = build(state(front_m=front_m), safety=SafetyConfig(tof_stop_mm=400))
    assert wider.obstacle_ahead is (front_m * 1000 < 400)


def test_the_rest_is_read_straight_off_the_state() -> None:
    world = build(state(motion=True, bumper=True, pct=41, motion_s=7.9))
    assert world.moving is True
    assert world.bumper is True
    assert world.battery_pct == 41
    assert world.motion_budget_left.seconds == 7  # robotd's ledger, whole seconds


def test_the_power_cap_is_the_configured_default_in_percent() -> None:
    assert build(state()).power_cap_pct == 20
    assert build(state(), limits=LimitsConfig(power_default=0.30)).power_cap_pct == 30
    assert build(state(), limits=LimitsConfig(power_default=0.12)).power_cap_pct == 12


def test_without_a_state_message_the_path_ahead_is_not_known_to_be_clear() -> None:
    world = build(None)
    assert world.heading_deg == 0
    assert world.front_range_cm is None
    assert world.obstacle_ahead is True
    assert world.battery_pct == 0 and world.moving is False and world.bumper is False
    assert world.motion_budget_left.seconds == int(LIMITS.budget_motion_s)  # 12


def test_permission_and_budget_decide_the_offered_skills() -> None:
    assert {"drive_for", "turn_to", "find"} <= set(build(state()).allowed_skills)
    unauthorized = build(state(), allow_motion=False).allowed_skills
    assert set(unauthorized).isdisjoint({"drive_for", "turn_to", "find"})
    spent = build(state(motion_s=0.4)).allowed_skills  # rounds to 0 s left
    assert set(spent).isdisjoint({"drive_for", "turn_to", "find"})
    assert {"stop", "say", "describe_scene", "set_face"} <= set(spent)


def test_memory_and_the_last_result_ride_along() -> None:
    seen = [RecentlySeen(label="red mug", heading_deg=320, age_s=94)]
    world = build(
        state(),
        last_result=ResultStatus.DONE,
        last_scene="a kitchen",
        recently_seen=seen,
    )
    assert world.last_result is ResultStatus.DONE
    assert world.last_scene == "a kitchen"
    assert world.recently_seen == seen


def test_the_world_state_has_no_pose_and_nothing_in_metres() -> None:
    dumped = build(state()).model_dump()
    assert "pose_cm" not in dumped and "speed_cap_cms" not in dumped
    assert "front_at_max" not in dumped
    assert set(dumped["motion_budget_left"]) == {"seconds"}
