"""A12 stage two, and the dispatch-time check that is stage three.

The four rejections ARCHITECTURE names -- a hallucinated skill, an
out-of-range integer, an extra field and a non-finite value -- each get a test,
because the grammar is an attack channel and this is the stage that does not
trust it.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.validate import (  # noqa: E402
    SPEECH_MAX_CHARS,
    ValidationFailure,
    authorized_motion,
    crosses_bus,
    goal_ttl_ms_for,
    permit,
    to_bus_message,
    validate_output,
)
from rover_contracts.config import LimitsConfig  # noqa: E402
from rover_contracts.messages import (  # noqa: E402
    DriveArgs,
    DriveBusArgs,
    DriveCall,
    NoArgs,
    ResultReason,
    SetFaceArgs,
    SetFaceCall,
    SkillName,
    Source,
    StopCall,
    TurnArgs,
    TurnCall,
)
from rover_contracts.skills import goal_deadline_s  # noqa: E402
from rover_contracts.units import deg_to_rad  # noqa: E402

LIMITS = LimitsConfig()
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
TURN = "01J9ZC7K000000000000000000"


def good(**args: object) -> str:
    return json.dumps(
        {
            "speech": "Heading over.",
            "skill": "drive",
            "args": {"distance_cm": 40, "speed_cms": 15} | args,
        }
    )


# -- what the validator accepts ---------------------------------------------


def test_the_architectures_own_examples_validate() -> None:
    for line in (
        '{"speech":"Heading over.","skill":"drive",'
        '"args":{"distance_cm":40,"speed_cms":15}}',
        '{"speech":"Turning left.","skill":"turn","args":{"angle_deg":45,"rate_dps":40}}',
        '{"speech":"Stopping.","skill":"stop","args":{}}',
        '{"speech":"","skill":"say","args":{"text":"There is a mug on the table."}}',
        '{"speech":"Let me look.","skill":"describe_scene","args":{}}',
        '{"speech":"Looking for it.","skill":"find",'
        '"args":{"object":"red mug","max_sweeps":8}}',
        '{"speech":"","skill":"set_face","args":{"expr":"happy"}}',
    ):
        assert validate_output(line).skill in set(SkillName)


def test_whitespace_around_the_object_is_tolerated() -> None:
    assert validate_output(f"\n  {good()}  \n").skill == "drive"


# -- the four rejections -----------------------------------------------------


def test_a_hallucinated_skill_is_rejected() -> None:
    with pytest.raises(ValidationFailure) as caught:
        validate_output('{"speech":"Off I go.","skill":"teleport","args":{}}')
    assert caught.value.reason == "unknown_skill"
    assert "drive" in caught.value.detail


def test_an_out_of_range_integer_is_rejected() -> None:
    for line, field in (
        (good(distance_cm=101), "distance_cm"),
        (good(speed_cms=31), "speed_cms"),
        (good(speed_cms=4), "speed_cms"),
        (
            '{"speech":"","skill":"turn","args":{"angle_deg":181,"rate_dps":40}}',
            "angle_deg",
        ),
        (
            '{"speech":"","skill":"find","args":{"object":"mug","max_sweeps":9}}',
            "max_sweeps",
        ),
    ):
        with pytest.raises(ValidationFailure) as caught:
            validate_output(line)
        assert caught.value.reason == "out_of_range"
        assert field in caught.value.detail


def test_an_extra_field_is_rejected() -> None:
    with pytest.raises(ValidationFailure) as caught:
        validate_output(
            '{"speech":"","skill":"drive",'
            '"args":{"distance_cm":40,"speed_cms":15,"boost":true}}'
        )
    assert caught.value.reason == "extra_field"
    assert "boost" in caught.value.detail

    with pytest.raises(ValidationFailure) as caught:
        validate_output(good() [:-1] + ',"cmd_id":"01J9ZC7K3QF2M8XR4V6T0YAHBD"}')
    assert caught.value.reason == "extra_field"


def test_a_non_finite_value_is_rejected() -> None:
    """Both spellings: the JSON constants, and a float that overflows."""
    with pytest.raises(ValidationFailure) as caught:
        validate_output('{"speech":"","skill":"drive","args":{"distance_cm":NaN,"speed_cms":15}}')
    assert caught.value.reason == "not_finite"

    with pytest.raises(ValidationFailure):
        validate_output(
            '{"speech":"","skill":"drive","args":{"distance_cm":1e999,"speed_cms":15}}'
        )


def test_malformed_json_is_rejected_before_anything_reads_it() -> None:
    for line in ("", "   ", "not json", "[1,2]", '{"speech":"x"', '"a string"'):
        with pytest.raises(ValidationFailure) as caught:
            validate_output(line)
        assert caught.value.reason in {"malformed_json", "empty"}


def test_a_float_where_an_integer_belongs_is_rejected() -> None:
    """A11: every bounded numeric is an integer, and strict mode means it."""
    with pytest.raises(ValidationFailure):
        validate_output(good(distance_cm=40.5))
    with pytest.raises(ValidationFailure):
        validate_output(good(distance_cm="40"))


def test_an_over_long_speech_is_truncated_not_rejected() -> None:
    line = json.dumps(
        {
            "speech": "x" * 400,
            "skill": "drive",
            "args": {"distance_cm": 40, "speed_cms": 15},
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


def test_an_unauthorized_utterance_cannot_dispatch_motion() -> None:
    call = DriveCall(
        speech="", skill="drive", args=DriveArgs(distance_cm=40, speed_cms=15)
    )
    assert permit(call, authorized=False, limits=LIMITS) is (
        ResultReason.UNAUTHORIZED_UTTERANCE
    )
    assert permit(call, authorized=True, limits=LIMITS) is None


def test_an_unauthorized_utterance_may_still_speak_and_look() -> None:
    call = SetFaceCall(speech="", skill="set_face", args=SetFaceArgs(expr="happy"))
    assert permit(call, authorized=False, limits=LIMITS) is None


def test_the_schema_legal_but_unexecutable_drive_is_refused_locally() -> None:
    """ARCHITECTURE 6's own example: drive(100 cm, 5 cm/s) is 30.5 s."""
    call = DriveCall(
        speech="", skill="drive", args=DriveArgs(distance_cm=100, speed_cms=5)
    )
    with pytest.raises(ValidationFailure) as caught:
        goal_ttl_ms_for(call, LIMITS)
    assert caught.value.reason == "goal_ttl_too_long"
    assert permit(call, authorized=True, limits=LIMITS) is ResultReason.GOAL_TTL_TOO_LONG


def test_goal_ttl_is_computed_from_the_profile() -> None:
    call = DriveCall(
        speech="", skill="drive", args=DriveArgs(distance_cm=40, speed_cms=15)
    )
    # 0.40 m / 0.15 m/s x 1.5 + 0.5 s = 4.5 s
    assert goal_ttl_ms_for(call, LIMITS) == 4500
    turn = TurnCall(speech="", skill="turn", args=TurnArgs(angle_deg=90, rate_dps=60))
    assert goal_ttl_ms_for(turn, LIMITS) == 2750


def test_the_ttl_brain_computes_is_never_below_the_T2_robotd_derives() -> None:
    """One formula, one set of arguments.  brain divided degrees by deg/s while
    robotd divides radians by rad/s, and the float round trip through
    math.radians put robotd's ceil one millisecond higher on 742 of the ~10 900
    legal (angle, rate) pairs -- so robotd refused the deadline brain itself
    computed, with `goal_ttl_too_short`."""
    below = []
    for angle in range(-180, 181):
        if angle == 0:
            continue
        for rate in range(5, 61):
            call = TurnCall(
                speech="",
                skill="turn",
                args=TurnArgs(angle_deg=angle, rate_dps=rate),
            )
            try:
                brain_ttl = goal_ttl_ms_for(call, LIMITS)
            except ValidationFailure:
                continue  # refused locally as goal_ttl_too_long
            robotd_t2 = math.ceil(
                goal_deadline_s(deg_to_rad(angle), deg_to_rad(rate)) * 1000.0
            )
            if brain_ttl < robotd_t2:
                below.append((angle, rate, brain_ttl, robotd_t2))
    assert not below, f"{len(below)} pairs robotd would reject, e.g. {below[:3]}"


# -- the host's own identifiers ---------------------------------------------


def test_only_the_host_supplies_identifiers_and_units() -> None:
    call = DriveCall(
        speech="Heading over.",
        skill="drive",
        args=DriveArgs(distance_cm=40, speed_cms=15),
    )
    message = to_bus_message(
        call,
        cmd_id=CMD,
        turn_id=TURN,
        seq=42,
        issued_mono_ns=123456789012,
        limits=LIMITS,
        authorized=True,
    )
    assert message.source is Source.BRAIN
    assert message.cmd_id == CMD and message.turn_id == TURN and message.seq == 42
    assert message.goal_ttl_ms == 4500
    assert isinstance(message.args, DriveBusArgs)
    assert message.args.distance_m == pytest.approx(0.40)
    assert message.args.speed_mps == pytest.approx(0.15)
    assert message.trace is not None and message.trace.authorized_motion is True


def test_stop_is_never_carried_as_a_skill() -> None:
    call = StopCall(speech="Stopping.", skill="stop", args=NoArgs())
    assert crosses_bus(call) is False
    with pytest.raises(ValueError, match="not dispatched"):
        to_bus_message(
            call,
            cmd_id=CMD,
            turn_id=TURN,
            seq=1,
            issued_mono_ns=1,
            limits=LIMITS,
            authorized=True,
        )


def test_set_face_and_find_stay_brain_local() -> None:
    face = SetFaceCall(speech="", skill="set_face", args=SetFaceArgs(expr="happy"))
    assert crosses_bus(face) is False
