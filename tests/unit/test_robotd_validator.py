"""ARCHITECTURE 4.2, row by row: one case per rejection reason, plus the
replay window (I-12) and the late-response race (I-11).

The table below is the point of the file: every ``result.reason`` robotd can
answer has a named case that produces it, so a refusal is attributable to one
row rather than to "validation failed".
"""

from __future__ import annotations

import math
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.config import (  # noqa: E402
    BusConfig,
    LimitsConfig,
    RobotConfig,
)
from rover_contracts.messages import (  # noqa: E402
    BusCap,
    HelloMessage,
    ResultReason,
    SkillMessage,
    Source,
    TwistMessage,
)
from rover_contracts.serial_codec import (  # noqa: E402
    TOF_ERROR_MM,
    TOF_NO_TARGET_MM,
    Fault,
)
from rover_contracts.skills import goal_deadline_s  # noqa: E402
from rover_contracts.units import deg_to_rad  # noqa: E402
from rover_robotd.arbiter import ActiveCommand, CommandKind  # noqa: E402
from rover_robotd.session import ClientSession  # noqa: E402
from rover_robotd.validator import (  # noqa: E402
    AcceptedSkill,
    AcceptedTwist,
    Rejection,
    ValidationContext,
    Validator,
)

MS = 1_000_000
NOW = 10_000 * MS
TURN = "01J9ZC7K000000000000000000"
OLD_TURN = "01J9ZC7J000000000000000000"
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
CMD2 = "01J9ZC7K3QF2M8XR4V6T0YAHBE"


def config(**bus: Any) -> RobotConfig:
    """A default config, optionally with ``[bus]`` overrides."""
    return RobotConfig(bus=BusConfig(**bus)) if bus else RobotConfig()


def session(source: Source = Source.BRAIN, caps: tuple[str, ...] = ("skill",)):
    """A connection with ``source`` bound at ``hello`` (ARCHITECTURE 4.2)."""
    client = ClientSession()
    client.bind(
        HelloMessage(source=source, pid=1234, caps=[BusCap(cap) for cap in caps]),
        expected_uid=None,
    )
    return client


def context(**overrides: Any) -> ValidationContext:
    base: dict[str, Any] = {
        "now_mono_ns": NOW,
        "telemetry_age_ms": 18.0,
        "mcu_fault": 0,
        "front_range_mm": 2000,
        "estop_sw": False,
        "current_turn_id": TURN,
        "active": None,
        "last_motion_start_mono_ns": None,
        "last_motion_turn_id": None,
        "remaining_path_m": 1.5,
        "remaining_motion_s": 12.0,
    }
    base.update(overrides)
    return ValidationContext(**base)


def sender_ttl_ms(skill: str, args: dict[str, Any]) -> int:
    """The ``goal_ttl_ms`` brain would send for this call.

    ARCHITECTURE 4.2: "brain computes ``goal_ttl_ms`` from the profile, never
    from a constant".  A constant here is what let the clamp case certify a
    path that rejected every real drive: with 5000 hard-coded, 5000 >= the
    *clamped* T2 and the case passed; with the value brain actually sends --
    derived from the speed it *requested* -- it was answered
    ``goal_ttl_too_short``.
    """
    if skill == "turn":
        seconds = goal_deadline_s(
            deg_to_rad(args["angle_deg"]), deg_to_rad(args["rate_dps"])
        )
    else:
        seconds = goal_deadline_s(args["distance_m"], args["speed_mps"])
    return math.ceil(seconds * 1000.0)


def drive(**overrides: Any) -> SkillMessage:
    skill = overrides.get("skill", "drive")
    args = overrides.get("args", {"distance_m": 0.40, "speed_mps": 0.15})
    payload: dict[str, Any] = {
        "source": "brain",
        "cmd_id": CMD,
        "seq": 42,
        "turn_id": TURN,
        "issued_mono_ns": 123,
        "goal_ttl_ms": sender_ttl_ms(skill, args),
        "skill": "drive",
        "args": {"distance_m": 0.40, "speed_mps": 0.15},
        "obs": {"frame_id": "cam-000917", "frame_mono_ns": NOW - 100 * MS},
        # brain sets this on every dispatch (validate.to_bus_message), and a
        # motion skill without it is refused: an absent flag is not the same
        # thing as section 7's deliberately-authorized null STT confidence.
        "trace": {"authorized_motion": True},
    }
    payload.update(overrides)
    return SkillMessage.model_validate(payload)


def say(**overrides: Any) -> SkillMessage:
    payload: dict[str, Any] = {
        "source": "brain",
        "cmd_id": CMD2,
        "seq": 7,
        "turn_id": TURN,
        "issued_mono_ns": 123,
        "goal_ttl_ms": 1000,
        "skill": "say",
        "args": {"text": "there is a mug on the table"},
    }
    payload.update(overrides)
    return SkillMessage.model_validate(payload)


def twist(**overrides: Any) -> TwistMessage:
    payload: dict[str, Any] = {
        "source": "teleop",
        "cmd_id": CMD,
        "seq": 901,
        "twist": {"linear_x_mps": 0.15, "angular_z_radps": 0.35},
    }
    payload.update(overrides)
    return TwistMessage.model_validate(payload)


TELEOP_BUS = {
    "allow_sources": ["brain", "web", "teleop"],
    "allow_stream": ["teleop"],
}


# ---------------------------------------------------------------------------
# One case per rejection reason
# ---------------------------------------------------------------------------


def _unknown_skill(validator: Validator) -> Rejection:
    verdict = validator.parse(
        {
            "v": 1,
            "type": "skill",
            "source": "brain",
            "cmd_id": CMD,
            "seq": 1,
            "turn_id": TURN,
            "issued_mono_ns": 1,
            "goal_ttl_ms": 1000,
            "skill": "fly",
            "args": {},
        }
    )
    assert isinstance(verdict, Rejection)
    return verdict


def _bad_args(validator: Validator) -> Rejection:
    verdict = validator.parse(
        {
            "v": 1,
            "type": "skill",
            "source": "brain",
            "cmd_id": CMD,
            "seq": 1,
            "turn_id": TURN,
            "issued_mono_ns": 1,
            "goal_ttl_ms": 1000,
            "skill": "drive",
            "args": {"distance_m": 0.4, "speed_mps": 0.15, "extra": 1},
        }
    )
    assert isinstance(verdict, Rejection)
    return verdict


def _out_of_bounds(_: Validator) -> Rejection:
    """``[limits] drive_m`` may be lowered below the catalog's 1.0 m."""
    lowered = RobotConfig(limits=LimitsConfig(drive_m=0.25))
    return _reject(
        Validator(lowered), drive(args={"distance_m": 0.40, "speed_mps": 0.15})
    )


def _source_not_allowed(validator: Validator) -> Rejection:
    """A connection that says hello as brain then sends teleop's source."""
    return _reject(validator, drive(source="teleop"), client=session(Source.BRAIN))


def _estop_active(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(estop_sw=True))


def _stale_seq(validator: Validator) -> Rejection:
    client = session()
    assert isinstance(
        validator.check_skill(drive(seq=42), client, context()), AcceptedSkill
    )
    return _reject(validator, drive(cmd_id=CMD2, seq=41), client=client)


def _not_ready(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(telemetry_age_ms=200.0))


def _faulted(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(mcu_fault=int(Fault.OVERCURRENT)))


def _obstacle(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(mcu_fault=int(Fault.TOF_STOP)))


def _obs_stale(validator: Validator) -> Rejection:
    return _reject(
        validator,
        drive(obs={"frame_id": "cam-000917", "frame_mono_ns": NOW - 9_000 * MS}),
    )


def _goal_ttl_too_short(validator: Validator) -> Rejection:
    """drive(0.40 m, 0.15 m/s) is a 4.5 s T2; asking for 3 s is a bug the
    sender should see, not something to silently truncate."""
    return _reject(validator, drive(goal_ttl_ms=3000))


def _goal_ttl_too_long(validator: Validator) -> Rejection:
    """The schema-legal drive(100 cm, 5 cm/s) at 30.5 s.

    ``goal_ttl_ms`` is the schema maximum rather than the profile's 30 500 ms,
    because the field itself is bounded at 5000: a sender that computed the
    real T2 could not put it on the wire.  This is the row that catches one
    that sent the maximum instead.
    """
    return _reject(
        validator,
        drive(args={"distance_m": 1.0, "speed_mps": 0.05}, goal_ttl_ms=5000),
    )


def _duplicate_cmd(validator: Validator) -> Rejection:
    client = session()
    assert isinstance(validator.check_skill(drive(), client, context()), AcceptedSkill)
    return _reject(validator, drive(seq=43), client=client)


def _stale_turn(validator: Validator) -> Rejection:
    return _reject(validator, drive(turn_id=OLD_TURN))


def _unauthorized_utterance(validator: Validator) -> Rejection:
    return _reject(validator, drive(trace={"authorized_motion": False}))


def _rate_limited(validator: Validator) -> Rejection:
    """The cooldown applies to the first motion of a new instruction."""
    return _reject(
        validator,
        drive(),
        ctx=context(
            last_motion_start_mono_ns=NOW - 500 * MS, last_motion_turn_id=OLD_TURN
        ),
    )


def _budget_exceeded(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(remaining_path_m=0.05))


REJECTIONS: list[tuple[ResultReason, Callable[[Validator], Rejection]]] = [
    (ResultReason.UNKNOWN_SKILL, _unknown_skill),
    (ResultReason.BAD_ARGS, _bad_args),
    (ResultReason.OUT_OF_BOUNDS, _out_of_bounds),
    (ResultReason.SOURCE_NOT_ALLOWED, _source_not_allowed),
    (ResultReason.ESTOP_ACTIVE, _estop_active),
    (ResultReason.STALE_SEQ, _stale_seq),
    (ResultReason.NOT_READY, _not_ready),
    (ResultReason.FAULTED, _faulted),
    (ResultReason.OBSTACLE, _obstacle),
    (ResultReason.OBS_STALE, _obs_stale),
    (ResultReason.GOAL_TTL_TOO_SHORT, _goal_ttl_too_short),
    (ResultReason.GOAL_TTL_TOO_LONG, _goal_ttl_too_long),
    (ResultReason.DUPLICATE_CMD, _duplicate_cmd),
    (ResultReason.STALE_TURN, _stale_turn),
    (ResultReason.UNAUTHORIZED_UTTERANCE, _unauthorized_utterance),
    (ResultReason.RATE_LIMITED, _rate_limited),
    (ResultReason.BUDGET_EXCEEDED, _budget_exceeded),
]


def _reject(
    validator: Validator,
    message: SkillMessage,
    *,
    client: ClientSession | None = None,
    ctx: ValidationContext | None = None,
) -> Rejection:
    verdict = validator.check_skill(
        message, client or session(), ctx or context()
    )
    assert isinstance(verdict, Rejection), f"expected a rejection, got {verdict}"
    return verdict


@pytest.mark.parametrize(
    ("reason", "case"), REJECTIONS, ids=[str(r) for r, _ in REJECTIONS]
)
def test_every_rejection_reason_has_a_case(
    reason: ResultReason, case: Callable[[Validator], Rejection]
) -> None:
    rejection = case(Validator(config()))
    assert rejection.reason is reason, rejection
    assert rejection.detail, "a rejection must say which row refused it"


def test_the_table_covers_every_reason_robotd_can_answer() -> None:
    """The reasons robotd never answers from the validator, and why."""
    runtime_only = {
        ResultReason.NONE,
        ResultReason.SPEED_CLAMPED,  # a clamp is reported, never a rejection
        ResultReason.TTL_EXPIRED,  # T2, raised by the control loop
        ResultReason.MCU_NACK,  # the controller refused, after dispatch
        ResultReason.BOX_LOST,  # brain owns the box link (A20)
    }
    covered = {reason for reason, _ in REJECTIONS}
    assert covered | runtime_only == set(ResultReason)


# ---------------------------------------------------------------------------
# Acceptance, clamping and the cap in force
# ---------------------------------------------------------------------------


def test_a_legal_drive_is_accepted_with_the_effective_deadline() -> None:
    verdict = Validator(config()).check_skill(drive(), session(), context())
    assert isinstance(verdict, AcceptedSkill)
    assert verdict.moves
    assert verdict.deadline_ms == 4500  # min(goal_ttl_ms, T2)
    assert verdict.est_path_m == pytest.approx(0.40)
    assert verdict.speed_clamped_to_cms is None


@pytest.mark.parametrize(
    ("front_mm", "expected_cap"),
    [
        (2000, 0.30),
        (TOF_NO_TARGET_MM, 0.30),
        (1001, 0.30),
        (1000, 0.20),
        (400, 0.20),
        (TOF_ERROR_MM, 0.20),
        (None, 0.20),
    ],
)
def test_the_speed_cap_unlocks_only_beyond_one_metre(
    front_mm: int | None, expected_cap: float
) -> None:
    assert Validator(config()).speed_cap_mps(front_mm) == expected_cap


@pytest.mark.parametrize("front_mm", [400, 800])
def test_a_speed_above_the_cap_is_clamped_and_reported(front_mm: int) -> None:
    """ARCHITECTURE 6: above the cap in force the value is *clamped*, not
    rejected, and reported as ``detail.speed_clamped_to_cms``.

    ``front_mm=800`` is the ordinary case -- any wall between the 250 mm stop
    zone and the 1000 mm unlock threshold, no fault -- and it is the one the
    constant ``goal_ttl_ms`` hid: brain sends 2500 ms for this call, robotd's
    T2 for the *clamped* 0.20 m/s is 3500 ms, and comparing the two rejected it
    ``goal_ttl_too_short``.
    """
    args = {"distance_m": 0.40, "speed_mps": 0.30}
    verdict = Validator(config()).check_skill(
        drive(args=args, goal_ttl_ms=sender_ttl_ms("drive", args)),
        session(),
        context(front_range_mm=front_mm),
    )
    assert isinstance(verdict, AcceptedSkill)
    assert verdict.args.speed_mps == pytest.approx(0.20)
    assert verdict.speed_clamped_to_cms == 20
    # The deadline covers the drive the clamp created, not the one the sender
    # asked for: 0.40 m at 0.20 m/s is 4.5 s, and publishing the sender's 2.5 s
    # would abort a drive robotd itself made longer.
    assert verdict.deadline_ms == 3500


def test_a_clamped_drive_keeps_a_deadline_it_can_finish_inside() -> None:
    """The other half of the same rule: a drive that needs no clamp keeps
    4.2's ``min(goal_ttl_ms, T2)`` exactly."""
    verdict = Validator(config()).check_skill(
        drive(goal_ttl_ms=5000), session(), context()
    )
    assert isinstance(verdict, AcceptedSkill)
    assert verdict.speed_clamped_to_cms is None
    assert verdict.deadline_ms == 4500


def test_reverse_is_capped_at_thirty_centimetres_under_an_obstacle() -> None:
    verdict = Validator(config()).check_skill(
        drive(args={"distance_m": -0.50, "speed_mps": 0.20}),
        session(),
        context(mcu_fault=int(Fault.BUMPER)),
    )
    assert isinstance(verdict, AcceptedSkill)
    assert verdict.args.distance_m == pytest.approx(-0.30)


def test_rotation_stays_legal_under_an_obstacle_bit() -> None:
    """I-5's Pi-side half: forward is refused, rotation is not."""
    turn = drive(
        skill="turn", args={"angle_deg": 45.0, "rate_dps": 40.0}, goal_ttl_ms=2500
    )
    verdict = Validator(config()).check_skill(
        turn, session(), context(mcu_fault=int(Fault.TOF_STOP))
    )
    assert isinstance(verdict, AcceptedSkill)


@pytest.mark.parametrize(
    "trace",
    [None, {}, {"model": "rover-vlm"}, {"authorized_motion": None}],
    ids=["no-trace", "empty-trace", "trace-without-the-flag", "flag-is-null"],
)
def test_a_motion_skill_without_the_flag_is_unauthorized(
    trace: dict[str, object] | None,
) -> None:
    """Section 7 makes a *null STT confidence* authorized, deliberately.  It
    says nothing about a missing flag, and treating one as authorized
    re-enables motion with no authorizing utterance for any sender that never
    learned to set it -- which is what I-21 asserts cannot happen."""
    verdict = Validator(config()).check_skill(
        drive(trace=trace), session(), context()
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.UNAUTHORIZED_UTTERANCE


@pytest.mark.parametrize(
    "trace", [None, {}], ids=["no-trace", "empty-trace"]
)
def test_a_non_motion_skill_needs_no_flag(trace: dict[str, object] | None) -> None:
    """say and describe_scene must stay permitted when motion is not, so the
    requirement is on the motion rows only."""
    verdict = Validator(config()).check_skill(say(trace=trace), session(), context())
    assert isinstance(verdict, AcceptedSkill)


def test_a_non_motion_skill_skips_every_motion_row() -> None:
    """say and describe_scene stay permitted when motion is not, so a stale
    link cannot silently mute the robot."""
    verdict = Validator(config()).check_skill(
        say(),
        session(),
        context(telemetry_age_ms=9_000.0, mcu_fault=int(Fault.OVERCURRENT)),
    )
    assert isinstance(verdict, AcceptedSkill)
    assert not verdict.moves


def test_a_say_over_three_hundred_characters_does_not_parse() -> None:
    verdict = Validator(config()).parse(
        {
            "v": 1,
            "type": "skill",
            "source": "brain",
            "cmd_id": CMD,
            "seq": 1,
            "turn_id": TURN,
            "issued_mono_ns": 1,
            "goal_ttl_ms": 1000,
            "skill": "say",
            "args": {"text": "x" * 301},
        }
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.BAD_ARGS


def test_the_cooldown_exempts_later_skills_of_the_same_instruction() -> None:
    """Without the exemption find's sweep loop is rate_limited on its second
    turn (ARCHITECTURE 4.2)."""
    verdict = Validator(config()).check_skill(
        drive(),
        session(),
        context(last_motion_start_mono_ns=NOW - 100 * MS, last_motion_turn_id=TURN),
    )
    assert isinstance(verdict, AcceptedSkill)


# ---------------------------------------------------------------------------
# Replay and the late-response race
# ---------------------------------------------------------------------------


def test_a_repeated_cmd_id_cannot_execute_motion_twice() -> None:
    """I-12: resend an accepted skill verbatim."""
    validator = Validator(config())
    client = session()
    first = validator.check_skill(drive(), client, context())
    assert isinstance(first, AcceptedSkill)
    repeat = validator.check_skill(drive(seq=99), client, context())
    assert isinstance(repeat, Rejection)
    assert repeat.reason is ResultReason.DUPLICATE_CMD


def test_the_replay_window_holds_the_last_sixty_four_pairs() -> None:
    validator = Validator(config())
    client = session()
    for index in range(70):
        message = drive(cmd_id=f"01J9ZC7K3QF2M8XR4V6T0Y{index:04d}", seq=index + 1)
        assert isinstance(
            validator.check_skill(message, client, context()), AcceptedSkill
        )
    assert not validator.replay.seen(Source.BRAIN, "01J9ZC7K3QF2M8XR4V6T0Y0000")
    assert validator.replay.seen(Source.BRAIN, "01J9ZC7K3QF2M8XR4V6T0Y0069")


def test_a_late_response_carrying_the_previous_turn_cannot_start_motion() -> None:
    """I-11: the box answer that lands after a stop or a new turn boundary."""
    validator = Validator(config())
    verdict = validator.check_skill(
        drive(turn_id=OLD_TURN), session(), context(current_turn_id=TURN)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.STALE_TURN


def test_a_skill_before_any_turn_boundary_is_stale() -> None:
    verdict = Validator(config()).check_skill(
        drive(), session(), context(current_turn_id=None)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.STALE_TURN


def test_a_rejected_command_is_not_added_to_the_replay_window() -> None:
    validator = Validator(config())
    validator.check_skill(drive(turn_id=OLD_TURN), session(), context())
    assert not validator.replay.seen(Source.BRAIN, CMD)


# ---------------------------------------------------------------------------
# twist (A35): the subset of 4.2 that applies to a stream
# ---------------------------------------------------------------------------


def test_a_twist_is_refused_while_allow_stream_is_empty() -> None:
    verdict = Validator(config()).check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context()
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_a_twist_is_accepted_once_the_stream_is_opted_in() -> None:
    verdict = Validator(config(**TELEOP_BUS)).check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context()
    )
    assert isinstance(verdict, AcceptedTwist)


def test_a_connection_bound_as_brain_may_not_send_teleop_twists() -> None:
    """G4-b's named case: hello as brain, then source teleop."""
    verdict = Validator(config(**TELEOP_BUS)).check_twist(
        twist(), session(Source.BRAIN, ("twist",)), context()
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_a_forward_twist_is_refused_under_an_obstacle_bit() -> None:
    verdict = Validator(config(**TELEOP_BUS)).check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context(mcu_fault=int(Fault.CLIFF))
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.OBSTACLE


def test_a_reverse_twist_survives_an_obstacle_bit() -> None:
    verdict = Validator(config(**TELEOP_BUS)).check_twist(
        twist(twist={"linear_x_mps": -0.10, "angular_z_radps": 0.0}),
        session(Source.TELEOP, ("twist",)),
        context(mcu_fault=int(Fault.CLIFF)),
    )
    assert isinstance(verdict, AcceptedTwist)


def test_a_twist_needs_fresh_telemetry() -> None:
    verdict = Validator(config(**TELEOP_BUS)).check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context(telemetry_age_ms=500.0)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.NOT_READY


def test_an_out_of_bounds_twist_does_not_parse() -> None:
    verdict = Validator(config(**TELEOP_BUS)).parse(
        {
            "v": 1,
            "type": "twist",
            "source": "teleop",
            "cmd_id": CMD,
            "seq": 1,
            "twist": {"linear_x_mps": 3.0, "angular_z_radps": 0.0},
        }
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.BAD_ARGS


def test_a_non_finite_twist_does_not_parse() -> None:
    verdict = Validator(config(**TELEOP_BUS)).parse(
        {
            "v": 1,
            "type": "twist",
            "source": "teleop",
            "cmd_id": CMD,
            "seq": 1,
            "twist": {"linear_x_mps": float("nan"), "angular_z_radps": 0.0},
        }
    )
    assert isinstance(verdict, Rejection)


def test_a_lower_priority_skill_cannot_take_the_wheels_from_a_web_stream() -> None:
    active = ActiveCommand(
        cmd_id="other",
        kind=CommandKind.TWIST,
        source=Source.WEB,
        session_id="deadbeef",
        started_mono_ns=NOW,
        deadline_mono_ns=NOW + 10**10,
    )
    verdict = Validator(config()).check_skill(
        drive(), session(), context(active=active)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.NOT_READY


def test_a_second_motion_skill_is_refused_while_one_is_in_flight() -> None:
    active = ActiveCommand(
        cmd_id="other",
        kind=CommandKind.SKILL,
        source=Source.BRAIN,
        session_id="deadbeef",
        started_mono_ns=NOW,
        deadline_mono_ns=NOW + 10**10,
        skill="drive",
    )
    verdict = Validator(config()).check_skill(
        drive(), session(), context(active=active)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.RATE_LIMITED


# ---------------------------------------------------------------------------
# clear (not stop-class)
# ---------------------------------------------------------------------------


def test_clear_is_refused_from_brain() -> None:
    """G4-l: brain, whose job is acting on model output, must not be able to
    clear a stop authority."""
    from rover_contracts.messages import ClearMessage

    message = ClearMessage(source=Source.BRAIN, faults=["estop_sw"])
    rejection = Validator(config()).check_clear(message, session(Source.BRAIN))
    assert rejection is not None
    assert rejection.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_clear_is_accepted_from_web() -> None:
    from rover_contracts.messages import ClearMessage

    message = ClearMessage(source=Source.WEB, faults=["estop_sw"])
    assert Validator(config()).check_clear(message, session(Source.WEB)) is None


# ---------------------------------------------------------------------------
# The connection binding itself
# ---------------------------------------------------------------------------


def test_a_second_hello_on_one_connection_is_refused() -> None:
    client = session()
    with pytest.raises(ValueError, match="second hello"):
        client.bind(
            HelloMessage(source=Source.WEB, pid=1, caps=[BusCap.SKILL]),
            expected_uid=None,
        )


def test_an_unbound_connection_may_not_command() -> None:
    verdict = Validator(config()).check_skill(drive(), ClientSession(), context())
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_a_peer_uid_that_does_not_match_the_unit_user_is_refused() -> None:
    from rover_robotd.session import PeerCredentials

    client = ClientSession(peer=PeerCredentials(uid=1000, gid=1000, pid=42))
    with pytest.raises(ValueError, match="may not claim source"):
        client.bind(
            HelloMessage(source=Source.WEB, pid=42, caps=[BusCap.SKILL]),
            expected_uid=999,
        )


def test_a_twist_is_bounded_by_the_configured_twist_limits() -> None:
    """A35 calls twist the widest motion surface in the design, and A33 says a
    gate reads the running value from welcome.limits.  Checking only the static
    catalog row left both [limits] twist keys unenforced anywhere in robotd, so
    rover-web's joystick honoured them and every other allow-listed client did
    not."""
    tight = RobotConfig(
        bus=BusConfig(**TELEOP_BUS),
        limits=LimitsConfig(twist_linear_mps=0.05, twist_angular_radps=0.20),
    )
    verdict = Validator(tight).check_twist(
        twist(twist={"linear_x_mps": 0.30, "angular_z_radps": 0.0}),
        session(Source.TELEOP, ("twist",)),
        context(),
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.OUT_OF_BOUNDS
    assert "twist_linear_mps" in verdict.detail

    verdict = Validator(tight).check_twist(
        twist(twist={"linear_x_mps": 0.0, "angular_z_radps": -1.0}),
        session(Source.TELEOP, ("twist",)),
        context(),
    )
    assert isinstance(verdict, Rejection)
    assert "twist_angular_radps" in verdict.detail

    inside = Validator(tight).check_twist(
        twist(twist={"linear_x_mps": 0.04, "angular_z_radps": 0.1}),
        session(Source.TELEOP, ("twist",)),
        context(),
    )
    assert isinstance(inside, AcceptedTwist)
