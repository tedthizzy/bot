"""A12 stage two, and the dispatch-time check that is stage three.

The four rejections ARCHITECTURE names -- a hallucinated skill, an
out-of-range integer, an extra field and a non-finite value -- each get a test,
because the grammar is an attack channel and this is the stage that does not
trust it.  The rest is the unit boundary: the model's integers become bus units
with no metre anywhere, the power clamp is recorded, and the deadline brain
computes is one robotd will accept.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.validate import (  # noqa: E402
    BUS_SKILLS,
    MOTION_CAPABLE,
    SPEECH_MAX_CHARS,
    ValidationFailure,
    authorized_motion,
    bus_args_for,
    crosses_bus,
    goal_ttl_ms_for,
    permit,
    power_clamped_to,
    refusal_reason,
    to_bus_message,
    turn_timeout_s,
    validate_output,
)
from rover_contracts.config import LimitsConfig  # noqa: E402
from rover_contracts.messages import (  # noqa: E402
    DriveForArgs,
    DriveForBusArgs,
    DriveForCall,
    FindArgs,
    FindCall,
    NoArgs,
    ResultReason,
    SetFaceArgs,
    SetFaceCall,
    SkillCall,
    SkillMessage,
    SkillName,
    Source,
    StopCall,
    TurnToArgs,
    TurnToBusArgs,
    TurnToCall,
)
from rover_contracts.skills import goal_deadline_s  # noqa: E402

LIMITS = LimitsConfig()
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
TURN = "01J9ZC7K000000000000000000"


def good(**args: object) -> str:
    return json.dumps(
        {
            "speech": "Heading over.",
            "skill": "drive_for",
            "args": {"duration_ms": 1000, "power_pct": 15} | args,
        }
    )


def drive(duration_ms: int = 1000, power_pct: int = 15) -> DriveForCall:
    return DriveForCall(
        speech="",
        skill="drive_for",
        args=DriveForArgs(duration_ms=duration_ms, power_pct=power_pct),
    )


def turn(heading_deg: int = 357) -> TurnToCall:
    return TurnToCall(
        speech="", skill="turn_to", args=TurnToArgs(heading_deg=heading_deg)
    )


def message(call: SkillCall, limits: LimitsConfig = LIMITS) -> SkillMessage:
    return to_bus_message(
        call,
        cmd_id=CMD,
        turn_id=TURN,
        seq=42,
        issued_mono_ns=123456789012,
        limits=limits,
        authorized=True,
    )


# -- what the validator accepts ---------------------------------------------


def test_each_of_the_seven_skills_validates() -> None:
    lines = {
        "drive_for": good(),
        "turn_to": (
            '{"speech":"Turning left.","skill":"turn_to","args":{"heading_deg":357}}'
        ),
        "stop": '{"speech":"Stopping.","skill":"stop","args":{}}',
        "say": (
            '{"speech":"","skill":"say","args":{"text":"There is a mug on the table."}}'
        ),
        "describe_scene": '{"speech":"Let me look.","skill":"describe_scene","args":{}}',
        "find": (
            '{"speech":"Looking for it.","skill":"find",'
            '"args":{"object":"red mug","max_sweeps":8}}'
        ),
        "set_face": '{"speech":"","skill":"set_face","args":{"expr":"happy"}}',
    }
    assert set(lines) == {s.value for s in SkillName}
    for skill, line in lines.items():
        assert validate_output(line).skill == skill


def test_whitespace_around_the_object_is_tolerated() -> None:
    assert validate_output(f"\n  {good()}  \n").skill == "drive_for"


def test_the_edges_of_every_integer_bound_are_inside() -> None:
    for line in (
        good(duration_ms=100, power_pct=-30),
        good(duration_ms=2000, power_pct=30),
        good(power_pct=1),
        good(power_pct=-1),
        '{"speech":"","skill":"turn_to","args":{"heading_deg":0}}',
        '{"speech":"","skill":"turn_to","args":{"heading_deg":359}}',
        '{"speech":"","skill":"find","args":{"object":"m","max_sweeps":1}}',
    ):
        validate_output(line)


# -- the four rejections -----------------------------------------------------


def test_a_hallucinated_skill_is_rejected() -> None:
    with pytest.raises(ValidationFailure) as caught:
        validate_output('{"speech":"Off I go.","skill":"teleport","args":{}}')
    assert caught.value.reason == "unknown_skill"
    assert "drive_for" in caught.value.detail and "turn_to" in caught.value.detail
    # The retired names are not offered to the model as alternatives.
    assert " drive," not in caught.value.detail and " turn," not in caught.value.detail


@pytest.mark.parametrize(
    ("line", "field"),
    [
        (good(duration_ms=2001), "duration_ms"),
        (good(duration_ms=99), "duration_ms"),
        (good(power_pct=31), "power_pct"),
        (good(power_pct=-31), "power_pct"),
        (
            '{"speech":"","skill":"turn_to","args":{"heading_deg":360}}',
            "heading_deg",
        ),
        ('{"speech":"","skill":"turn_to","args":{"heading_deg":-1}}', "heading_deg"),
        (
            '{"speech":"","skill":"find","args":{"object":"mug","max_sweeps":9}}',
            "max_sweeps",
        ),
        (
            '{"speech":"","skill":"find","args":{"object":"mug","max_sweeps":0}}',
            "max_sweeps",
        ),
        ('{"speech":"","skill":"say","args":{"text":""}}', "text"),
    ],
)
def test_an_out_of_range_value_is_rejected(line: str, field: str) -> None:
    with pytest.raises(ValidationFailure) as caught:
        validate_output(line)
    assert caught.value.reason == "out_of_range"
    assert field in caught.value.detail


def test_a_zero_power_is_not_a_drive() -> None:
    """Inside the integer bound and still refused: the model is told to use stop."""
    with pytest.raises(ValidationFailure) as caught:
        validate_output(good(power_pct=0))
    assert caught.value.reason == "out_of_range"
    assert "power_pct" in caught.value.detail and "stop" in caught.value.detail


def test_an_extra_field_is_rejected() -> None:
    with pytest.raises(ValidationFailure) as caught:
        validate_output(good(boost=True))
    assert caught.value.reason == "extra_field"
    assert "boost" in caught.value.detail

    with pytest.raises(ValidationFailure) as caught:
        validate_output(good()[:-1] + ',"cmd_id":"01J9ZC7K3QF2M8XR4V6T0YAHBD"}')
    assert caught.value.reason == "extra_field"


def test_a_non_finite_value_is_rejected() -> None:
    """All three spellings: the JSON constants, and a float that overflows."""
    for line in (
        '{"speech":"","skill":"drive_for","args":{"duration_ms":NaN,"power_pct":15}}',
        '{"speech":"","skill":"drive_for",'
        '"args":{"duration_ms":1000,"power_pct":Infinity}}',
    ):
        with pytest.raises(ValidationFailure) as caught:
            validate_output(line)
        assert caught.value.reason == "not_finite"
    with pytest.raises(ValidationFailure):
        validate_output(
            '{"speech":"","skill":"drive_for","args":{"duration_ms":1e999,"power_pct":15}}'
        )


def test_malformed_json_is_rejected_before_anything_reads_it() -> None:
    for line in ("", "   ", "not json", "[1,2]", '{"speech":"x"', '"a string"'):
        with pytest.raises(ValidationFailure) as caught:
            validate_output(line)
        assert caught.value.reason in {"malformed_json", "empty"}


def test_a_float_or_a_string_where_an_integer_belongs_is_rejected() -> None:
    """A11: every bounded numeric is an integer, and strict mode means it."""
    with pytest.raises(ValidationFailure):
        validate_output(good(duration_ms=1000.5))
    with pytest.raises(ValidationFailure):
        validate_output(good(power_pct="15"))
    with pytest.raises(ValidationFailure):
        validate_output('{"speech":"","skill":"turn_to","args":{"heading_deg":"90"}}')


def test_an_over_long_speech_is_truncated_not_rejected() -> None:
    line = json.dumps(
        {
            "speech": "x" * 400,
            "skill": "drive_for",
            "args": {"duration_ms": 1000, "power_pct": 15},
        }
    )
    call = validate_output(line)
    assert len(call.speech) == SPEECH_MAX_CHARS


# -- authorized_motion -------------------------------------------------------


@pytest.mark.parametrize(
    ("text", "confidence", "expected"),
    [
        ("go forward", None, True),
        ("go forward", 0.9, True),
        ("go forward", 0.5, True),
        ("go forward", 0.49, False),
        ("g", None, False),
        ("  ", None, False),
        ("go", 0.0, False),
    ],
)
def test_authorized_motion_is_architecture_sevens_rule(
    text: str, confidence: float | None, expected: bool
) -> None:
    assert (
        authorized_motion(text, confidence, min_chars=2, min_confidence=0.5) is expected
    )


# -- stage three -------------------------------------------------------------


def test_the_motion_skills_are_the_catalogs() -> None:
    assert set(MOTION_CAPABLE) == {"drive_for", "turn_to", "find"}
    assert set(BUS_SKILLS) == {"drive_for", "turn_to", "say", "describe_scene"}


def test_an_unauthorized_utterance_cannot_dispatch_motion() -> None:
    find = FindCall(speech="", skill="find", args=FindArgs(object="mug", max_sweeps=3))
    for call in (drive(), turn(), find):
        assert permit(call, authorized=False, limits=LIMITS) is (
            ResultReason.UNAUTHORIZED_UTTERANCE
        )
        assert permit(call, authorized=True, limits=LIMITS) is None


def test_an_unauthorized_utterance_may_still_speak_and_look() -> None:
    call = SetFaceCall(speech="", skill="set_face", args=SetFaceArgs(expr="happy"))
    assert permit(call, authorized=False, limits=LIMITS) is None


def test_a_duration_over_the_configured_ceiling_is_refused_locally() -> None:
    """The schema allows 2000 ms; a config that lowers drive_for_max_s makes
    robotd refuse it out_of_bounds, so brain refuses it first with the reason."""
    limits = LimitsConfig(drive_for_max_s=1.0)
    with pytest.raises(ValidationFailure) as caught:
        bus_args_for(drive(duration_ms=1500), limits)
    assert caught.value.reason == "out_of_range"
    assert "duration_ms" in caught.value.detail and "1000 ms" in caught.value.detail
    assert permit(drive(duration_ms=1500), authorized=True, limits=limits) is (
        ResultReason.OUT_OF_BOUNDS
    )
    assert permit(drive(duration_ms=1000), authorized=True, limits=limits) is None


def test_a_deadline_over_goal_ttl_ms_max_is_refused_locally() -> None:
    limits = LimitsConfig(goal_ttl_ms_max=3000)
    with pytest.raises(ValidationFailure) as caught:
        goal_ttl_ms_for(drive(duration_ms=2000), limits)  # T2 = 3.5 s
    assert caught.value.reason == "goal_ttl_too_long"
    assert permit(drive(duration_ms=2000), authorized=True, limits=limits) is (
        ResultReason.GOAL_TTL_TOO_LONG
    )
    assert refusal_reason(caught.value) is ResultReason.GOAL_TTL_TOO_LONG
    assert refusal_reason(ValidationFailure("out_of_range", "x")) is (
        ResultReason.OUT_OF_BOUNDS
    )


# -- the deadline ------------------------------------------------------------


def test_goal_ttl_is_the_skills_own_time_through_one_formula() -> None:
    # duration x 1.5 + 0.5 s, in whole milliseconds, rounded up.
    assert goal_ttl_ms_for(drive(duration_ms=1000), LIMITS) == 2000
    assert goal_ttl_ms_for(drive(duration_ms=2000), LIMITS) == 3500
    assert goal_ttl_ms_for(drive(duration_ms=100), LIMITS) == 650
    assert goal_ttl_ms_for(drive(duration_ms=333), LIMITS) == math.ceil(
        goal_deadline_s(0.333) * 1000
    )


def test_a_turns_deadline_is_its_timeout_without_drive_overhead() -> None:
    assert turn_timeout_s(LIMITS) == 4.0
    assert goal_ttl_ms_for(turn(), LIMITS) == 4000
    assert message(turn()).goal_ttl_ms == 4000
    assert message(turn()).args.timeout_s == 4.0
    shorter = LimitsConfig(turn_timeout_max_s=2.0)
    assert turn_timeout_s(shorter) == 2.0
    assert goal_ttl_ms_for(turn(), shorter) == 2000


def test_the_turn_timeout_never_puts_T2_over_the_ceiling() -> None:
    """Every legal goal_ttl_ms_max: the T2 robotd re-derives from the timeout
    brain sends is never above the goal_ttl_ms brain sends, by one millisecond
    or any other amount."""
    for ttl_max in range(600, 5001):
        limits = LimitsConfig(goal_ttl_ms_max=ttl_max)
        timeout = turn_timeout_s(limits)
        assert 0.0 < timeout <= limits.turn_timeout_max_s
        assert (
            math.ceil(goal_deadline_s(timeout, skill=SkillName.TURN_TO) * 1000) <= ttl_max
        )
        assert goal_ttl_ms_for(turn(), limits) <= ttl_max


def test_a_short_turn_has_no_artificial_drive_deadline_floor() -> None:
    limits = LimitsConfig(goal_ttl_ms_max=400)
    assert turn_timeout_s(limits) == 0.4
    assert goal_ttl_ms_for(turn(), limits) == 400


def test_non_motion_skills_get_a_short_fixed_expiry() -> None:
    say = validate_output('{"speech":"","skill":"say","args":{"text":"hi"}}')
    assert goal_ttl_ms_for(say, LIMITS) == 2000
    assert goal_ttl_ms_for(say, LimitsConfig(goal_ttl_ms_max=1500)) == 1500


# -- the unit boundary -------------------------------------------------------


def test_only_the_host_supplies_identifiers_and_units() -> None:
    bus = message(drive(duration_ms=1000, power_pct=15))
    assert bus.source is Source.BRAIN
    assert bus.cmd_id == CMD and bus.turn_id == TURN and bus.seq == 42
    assert bus.goal_ttl_ms == 2000
    assert isinstance(bus.args, DriveForBusArgs)
    assert bus.args.duration_s == pytest.approx(1.0)
    assert bus.args.power == pytest.approx(0.15)
    assert bus.trace is not None and bus.trace.authorized_motion is True


def test_the_bus_carries_seconds_and_power_and_nothing_metric() -> None:
    assert set(DriveForBusArgs.model_fields) == {"duration_s", "power"}
    assert set(TurnToBusArgs.model_fields) == {
        "heading_deg",
        "timeout_s",
        "tolerance_deg",
    }
    for name in (*DriveForBusArgs.model_fields, *TurnToBusArgs.model_fields):
        assert not name.endswith(("_m", "_mps", "_cm", "_cms", "_rad", "_dps"))


def test_reverse_is_a_negative_power() -> None:
    assert message(drive(power_pct=-15)).args.power == pytest.approx(-0.15)


def test_a_power_above_the_default_is_clamped_and_the_clamp_recorded() -> None:
    """ARCHITECTURE 6: above the cap in force the value is clamped, not
    rejected -- and reported, so the completion can say so."""
    over = drive(power_pct=30)
    assert power_clamped_to(over, LIMITS) == pytest.approx(0.20)
    assert message(over).args.power == pytest.approx(0.20)
    back = drive(power_pct=-30)
    assert power_clamped_to(back, LIMITS) == pytest.approx(0.20)
    assert message(back).args.power == pytest.approx(-0.20)
    assert power_clamped_to(drive(power_pct=21), LIMITS) == pytest.approx(0.20)


def test_a_power_at_or_below_the_default_is_not_clamped() -> None:
    assert power_clamped_to(drive(power_pct=20), LIMITS) is None
    assert power_clamped_to(drive(power_pct=15), LIMITS) is None
    assert power_clamped_to(turn(), LIMITS) is None
    assert message(drive(power_pct=20)).args.power == pytest.approx(0.20)


def test_the_clamp_follows_the_configured_default() -> None:
    slow = LimitsConfig(power_default=0.10)
    assert power_clamped_to(drive(power_pct=15), slow) == pytest.approx(0.10)
    assert message(drive(power_pct=15), slow).args.power == pytest.approx(0.10)


def test_a_heading_passes_through_with_the_configured_turn_defaults() -> None:
    bus = message(turn(heading_deg=357))
    assert isinstance(bus.args, TurnToBusArgs)
    assert bus.args.heading_deg == 357.0
    assert bus.args.timeout_s == 4.0
    assert bus.args.tolerance_deg == 5.0
    wider = LimitsConfig(turn_tolerance_deg=8.0)
    assert message(turn(), wider).args.tolerance_deg == 8.0
    assert message(turn(heading_deg=0)).args.heading_deg == 0.0


def test_stop_is_never_carried_as_a_skill() -> None:
    call = StopCall(speech="Stopping.", skill="stop", args=NoArgs())
    assert crosses_bus(call) is False
    with pytest.raises(ValueError, match="not dispatched"):
        message(call)


def test_set_face_and_find_stay_brain_local() -> None:
    face = SetFaceCall(speech="", skill="set_face", args=SetFaceArgs(expr="happy"))
    find = FindCall(speech="", skill="find", args=FindArgs(object="mug", max_sweeps=3))
    assert crosses_bus(face) is False and crosses_bus(find) is False
    with pytest.raises(ValueError, match="not dispatched"):
        bus_args_for(find, LIMITS)


def test_say_and_describe_scene_cross_the_bus() -> None:
    say = validate_output('{"speech":"","skill":"say","args":{"text":"hi"}}')
    look = validate_output('{"speech":"","skill":"describe_scene","args":{}}')
    assert crosses_bus(say) and crosses_bus(look)
    assert message(say).args.text == "hi"
    assert isinstance(message(look).args, NoArgs)
