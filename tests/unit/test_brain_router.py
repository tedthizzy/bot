"""The local intents, answered without the box.

ARCHITECTURE 4.6: a dead box degrades the rover to these.  So the five the
document names -- stop, forward, back, left, right -- plus ``say`` must be
answerable from a regex table, and every number the table produces must already
be inside the schema's own bounds.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.router import RouterDefaults, route  # noqa: E402
from rover_contracts.config import LimitsConfig  # noqa: E402
from rover_contracts.messages import skill_call_adapter  # noqa: E402

DEFAULTS = RouterDefaults.from_limits(LimitsConfig())


@pytest.mark.parametrize(
    "text",
    ["stop", "STOP", "stop now", "halt", "whoa", "please stop moving", "freeze"],
)
def test_stop_is_answered_locally(text: str) -> None:
    call = route(text)
    assert call is not None and call.skill == "stop"


def test_stop_wins_over_every_other_intent_in_the_sentence() -> None:
    call = route("stop, then go forward")
    assert call is not None and call.skill == "stop"


@pytest.mark.parametrize(
    ("text", "distance_cm"),
    [
        ("go forward", 30),
        ("forward", 30),
        ("move straight ahead", 30),
        ("go forward 50 cm", 50),
        ("forward 50 centimetres", 50),
        ("drive forward half a metre", 50),
        ("go forward 1 m", 60),
        ("go forward twenty", 20),
        ("go forward 5 metres", 60),
    ],
)
def test_forward_is_answered_locally(text: str, distance_cm: int) -> None:
    call = route(text, DEFAULTS)
    assert call is not None and call.skill == "drive"
    assert call.args.distance_cm == distance_cm
    assert call.args.speed_cms == 20  # [limits] speed_default_mps


@pytest.mark.parametrize(
    ("text", "distance_cm"),
    [
        ("go back", -30),
        ("back up", -30),
        ("reverse 20 cm", -20),
        ("backwards half a meter", -50),
    ],
)
def test_back_is_answered_locally(text: str, distance_cm: int) -> None:
    call = route(text, DEFAULTS)
    assert call is not None and call.skill == "drive"
    assert call.args.distance_cm == distance_cm


@pytest.mark.parametrize(
    ("text", "angle_deg"),
    [
        ("turn left", 90),
        ("left", 90),
        ("turn left ninety degrees", 90),
        ("turn left 45 degrees", 45),
        ("turn left forty five degrees", 45),
        ("turn left one eighty", 180),
        ("turn left 400 degrees", 180),
    ],
)
def test_left_is_counter_clockwise(text: str, angle_deg: int) -> None:
    call = route(text, DEFAULTS)
    assert call is not None and call.skill == "turn"
    assert call.args.angle_deg == angle_deg


@pytest.mark.parametrize(
    ("text", "angle_deg"),
    [("turn right", -90), ("right 45 degrees", -45), ("turn right ninety", -90)],
)
def test_right_is_clockwise(text: str, angle_deg: int) -> None:
    call = route(text, DEFAULTS)
    assert call is not None and call.skill == "turn"
    assert call.args.angle_deg == angle_deg


def test_say_repeats_the_rest_of_the_sentence() -> None:
    call = route("say hello there")
    assert call is not None and call.skill == "say"
    assert call.args.text == "hello there"
    assert call.speech == ""


@pytest.mark.parametrize(
    "text",
    [
        "what do you see",
        "what is ahead of you",
        "is the mug on the table?",
        "find the red mug",
        "look happy",
        "describe the room",
        "",
    ],
)
def test_anything_else_goes_to_the_box(text: str) -> None:
    assert route(text) is None


def test_a_question_never_drives_even_when_it_names_a_direction() -> None:
    assert route("how far forward can you go?") is None


def test_every_local_answer_is_a_valid_skill_call() -> None:
    """The router emits exactly what the model would, so the same validator,
    permission check and executor run on both paths."""
    for text in (
        "stop",
        "go forward 60 cm",
        "back up",
        "turn left ninety degrees",
        "turn right",
        "say the kettle is on",
    ):
        call = route(text, DEFAULTS)
        assert call is not None
        assert skill_call_adapter.validate_python(call.model_dump()) == call


def test_a_number_the_speaker_shouts_is_clamped_not_rejected() -> None:
    """Clamped to what the deadline admits at the default speed cap, not to the
    schema's 100 cm: a router call the validator then refuses `goal_ttl_too_long`
    is the box-down path answering with a reason string instead of moving."""
    call = route("go forward 900 cm", DEFAULTS)
    assert call is not None and call.args.distance_cm == 60
    call = route("back up 900 cm", DEFAULTS)
    assert call is not None and call.args.distance_cm == -60


def test_the_default_speed_comes_from_limits_not_a_literal() -> None:
    slow = RouterDefaults.from_limits(LimitsConfig(speed_default_mps=0.10))
    call = route("go forward", slow)
    assert call is not None and call.args.speed_cms == 10
