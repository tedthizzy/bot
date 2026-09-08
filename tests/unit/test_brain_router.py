"""The local intents, answered without the box.

ARCHITECTURE 4.6: a dead box degrades the rover to these.  The rover is open
loop, so a local drive is the default power for one second and a local turn is
an absolute heading: the router does the heading arithmetic the model is told
to do, wrap included, and every call it mints is already a valid SkillCall.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.prompt import build_world_state  # noqa: E402
from rover_brain.router import RouterDefaults, route  # noqa: E402
from rover_contracts.config import LimitsConfig  # noqa: E402
from rover_contracts.messages import skill_call_adapter  # noqa: E402
from rover_contracts.worldstate import MotionBudget, WorldState  # noqa: E402

DEFAULTS = RouterDefaults.from_limits(LimitsConfig())


def world(heading_deg: int = 87) -> WorldState:
    return build_world_state(
        heading_deg=heading_deg,
        battery_pct=62,
        obstacle_ahead=False,
        front_range_cm=120,
        bumper=False,
        moving=False,
        power_cap_pct=20,
        budget=MotionBudget(seconds=9),
        allow_motion=True,
    )


@pytest.mark.parametrize(
    "text",
    ["stop", "STOP", "stop now", "halt", "whoa", "please stop moving", "freeze"],
)
def test_stop_is_answered_locally(text: str) -> None:
    call = route(text, world())
    assert call is not None and call.skill == "stop"


def test_stop_wins_over_every_other_intent_in_the_sentence() -> None:
    call = route("stop, then go forward", world())
    assert call is not None and call.skill == "stop"
    call = route("what do you see? stop!", world())
    assert call is not None and call.skill == "stop"


@pytest.mark.parametrize(
    "text", ["go forward", "forward", "move straight ahead", "go ahead a bit"]
)
def test_forward_is_a_one_second_drive_at_the_default_power(text: str) -> None:
    call = route(text, world(), DEFAULTS)
    assert call is not None and call.skill == "drive_for"
    assert call.args.duration_ms == 1000
    assert call.args.power_pct == 20  # [limits] power_default 0.20
    assert call.speech == "Going forward."


@pytest.mark.parametrize("text", ["go back", "back up", "reverse", "backwards please"])
def test_back_is_the_same_drive_with_negative_power(text: str) -> None:
    call = route(text, world(), DEFAULTS)
    assert call is not None and call.skill == "drive_for"
    assert call.args.duration_ms == 1000
    assert call.args.power_pct == -20


@pytest.mark.parametrize(
    ("heading", "expected"),
    [(87, 177), (90, 180), (30, 120), (0, 90), (180, 270)],
)
def test_left_is_ninety_degrees_more_wrapped(heading: int, expected: int) -> None:
    """The model and controller share positive-left host headings."""
    call = route("turn left", world(heading), DEFAULTS)
    assert call is not None and call.skill == "turn_to"
    assert call.args.heading_deg == expected
    assert call.speech == "Turning left."


@pytest.mark.parametrize(
    ("heading", "expected"),
    [(87, 357), (270, 180), (300, 210), (359, 269), (0, 270)],
)
def test_right_is_ninety_degrees_less_wrapped(heading: int, expected: int) -> None:
    call = route("turn right", world(heading), DEFAULTS)
    assert call is not None and call.skill == "turn_to"
    assert call.args.heading_deg == expected


@pytest.mark.parametrize(
    ("text", "heading", "expected"),
    [
        ("turn around", 87, 267),
        ("turn around", 180, 0),
        ("spin around", 270, 90),
        ("about face", 0, 180),
        ("make a u-turn", 359, 179),
    ],
)
def test_turn_around_is_a_half_turn(text: str, heading: int, expected: int) -> None:
    call = route(text, world(heading), DEFAULTS)
    assert call is not None and call.skill == "turn_to"
    assert call.args.heading_deg == expected
    assert call.speech == "Turning around."


def test_every_local_heading_is_in_the_turn_to_frame() -> None:
    for heading in range(0, 360, 7):
        for text in ("turn left", "turn right", "turn around"):
            call = route(text, world(heading), DEFAULTS)
            assert call is not None and 0 <= call.args.heading_deg <= 359


@pytest.mark.parametrize(
    "text",
    [
        "look",
        "Look.",
        "take a look",
        "look around",
        "what do you see",
        "What can you see?",
    ],
)
def test_look_is_describe_scene(text: str) -> None:
    call = route(text, world())
    assert call is not None and call.skill == "describe_scene"
    assert call.speech == "Let me look."


def test_say_repeats_the_rest_of_the_sentence() -> None:
    call = route("say hello there", world())
    assert call is not None and call.skill == "say"
    assert call.args.text == "hello there"
    assert call.speech == ""


@pytest.mark.parametrize(
    "text",
    [
        "what is ahead of you",
        "is the mug on the table?",
        "find the red mug",
        "look for the red mug",
        "look happy",
        "describe the room",
        "",
    ],
)
def test_anything_else_goes_to_the_box(text: str) -> None:
    assert route(text, world()) is None


def test_a_question_never_drives_even_when_it_names_a_direction() -> None:
    assert route("how far forward can you go?", world()) is None
    assert route("can you turn left?", world()) is None


def test_every_local_answer_is_a_valid_skill_call() -> None:
    """The router emits exactly what the model would, so the same validator,
    permission check and executor run on both paths."""
    for text in (
        "stop",
        "go forward",
        "back up",
        "turn left",
        "turn right",
        "turn around",
        "look",
        "say the kettle is on",
    ):
        call = route(text, world(), DEFAULTS)
        assert call is not None
        assert skill_call_adapter.validate_python(call.model_dump()) == call


def test_a_number_in_the_sentence_cannot_widen_anything() -> None:
    """Open loop: there is no distance to parse, and the router does not take
    a power or a duration from the transcript either."""
    call = route("go forward 900 cm at full power for ten seconds", world(), DEFAULTS)
    assert call is not None and call.skill == "drive_for"
    assert call.args.duration_ms == 1000 and call.args.power_pct == 20


def test_the_default_power_comes_from_limits_not_a_literal() -> None:
    slow = RouterDefaults.from_limits(LimitsConfig(power_default=0.10))
    call = route("go forward", world(), slow)
    assert call is not None and call.args.power_pct == 10
    assert RouterDefaults.from_limits(LimitsConfig(power_default=0.30)).power_pct == 30
    # A power_default that rounds to nothing is still a legal, non-zero drive.
    tiny = LimitsConfig(power_default=0.004, power_min=0.0)
    assert RouterDefaults.from_limits(tiny).power_pct == 1
