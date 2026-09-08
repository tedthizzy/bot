"""ARCHITECTURE 4.2, row by row, with ADR-0013's catalog: one case per
rejection reason, the power clamp, forward blocking, the replay window (I-12)
and the late-response race (I-11).

The table below is the point of the file: every ``result.reason`` robotd can
answer has a named case that produces it, so a refusal is attributable to one
row rather than to "validation failed".
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import pytest
from rover_contracts.config import BusConfig, LimitsConfig, RobotConfig
from rover_contracts.messages import (
    BusCap,
    ClearMessage,
    HelloMessage,
    ResultReason,
    SkillMessage,
    Source,
    TwistMessage,
)
from rover_contracts.skills import goal_deadline_s
from rover_contracts.wave_proto import StopFlag
from rover_robotd.arbiter import ActiveCommand, CommandKind
from rover_robotd.clients import ClientSession, PeerCredentials
from rover_robotd.validator import (
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


def config(**overrides: Any) -> RobotConfig:
    """A default config, optionally with ``bus`` / ``limits`` overrides."""
    kwargs: dict[str, Any] = {}
    if "bus" in overrides:
        kwargs["bus"] = BusConfig(**overrides["bus"])
    if "limits" in overrides:
        kwargs["limits"] = LimitsConfig(**overrides["limits"])
    return RobotConfig(**kwargs)


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
        "feedback_age_ms": 18.0,
        "firmware_ok": True,
        "stop_flags": 0,
        "estop_sw": False,
        "current_turn_id": TURN,
        "active": None,
        "last_motion_start_mono_ns": None,
        "last_motion_turn_id": None,
        "remaining_motion_s": 12.0,
    }
    base.update(overrides)
    return ValidationContext(**base)


def drive_ttl_ms(duration_s: float) -> int:
    """The ``goal_ttl_ms`` a sender derives from the T2 formula."""
    return math.ceil(goal_deadline_s(duration_s) * 1000.0)


def drive(**overrides: Any) -> SkillMessage:
    args = overrides.get("args", {"duration_s": 1.0, "power": 0.15})
    payload: dict[str, Any] = {
        "source": "brain",
        "cmd_id": CMD,
        "seq": 42,
        "turn_id": TURN,
        "issued_mono_ns": 123,
        "goal_ttl_ms": drive_ttl_ms(args["duration_s"]),
        "skill": "drive_for",
        "args": args,
        "obs": {"frame_id": "cam-000917", "frame_mono_ns": NOW - 100 * MS},
        # brain sets this on every dispatch, and a motion skill without it is
        # refused: an absent flag is not the same thing as section 7's
        # deliberately-authorized null STT confidence.
        "trace": {"authorized_motion": True},
    }
    payload.update(overrides)
    return SkillMessage.model_validate(payload)


def turn(**overrides: Any) -> SkillMessage:
    args = overrides.get("args", {"heading_deg": 90.0})
    payload: dict[str, Any] = {
        "source": "brain",
        "cmd_id": CMD,
        "seq": 42,
        "turn_id": TURN,
        "issued_mono_ns": 123,
        "goal_ttl_ms": math.ceil(args.get("timeout_s", 4.0) * 1000),
        "skill": "turn_to",
        "args": args,
        "obs": {"frame_id": "cam-000917", "frame_mono_ns": NOW - 100 * MS},
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
        "twist": {"lin": 0.15, "ang": 0.05},
    }
    payload.update(overrides)
    return TwistMessage.model_validate(payload)


TELEOP_BUS = {
    "allow_sources": ["brain", "web", "teleop"],
    "allow_stream": ["teleop"],
}


def _reject(
    validator: Validator,
    message: SkillMessage,
    *,
    client: ClientSession | None = None,
    ctx: ValidationContext | None = None,
) -> Rejection:
    verdict = validator.check_skill(message, client or session(), ctx or context())
    assert isinstance(verdict, Rejection), f"expected a rejection, got {verdict}"
    return verdict


def _accept(
    validator: Validator,
    message: SkillMessage,
    *,
    client: ClientSession | None = None,
    ctx: ValidationContext | None = None,
) -> AcceptedSkill:
    verdict = validator.check_skill(message, client or session(), ctx or context())
    assert isinstance(verdict, AcceptedSkill), f"expected acceptance, got {verdict}"
    return verdict


# ---------------------------------------------------------------------------
# One case per rejection reason
# ---------------------------------------------------------------------------


def _skill_payload(**fields: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "v": 1,
        "type": "skill",
        "source": "brain",
        "cmd_id": CMD,
        "seq": 1,
        "turn_id": TURN,
        "issued_mono_ns": 1,
        "goal_ttl_ms": 2000,
        "skill": "drive_for",
        "args": {"duration_s": 1.0, "power": 0.15},
    }
    payload.update(fields)
    return payload


def _unknown_skill(validator: Validator) -> Rejection:
    verdict = validator.parse(_skill_payload(skill="fly", args={}))
    assert isinstance(verdict, Rejection)
    return verdict


def _bad_args(validator: Validator) -> Rejection:
    verdict = validator.parse(
        _skill_payload(args={"duration_s": 1.0, "power": 0.15, "extra": 1})
    )
    assert isinstance(verdict, Rejection)
    return verdict


def _out_of_bounds(_: Validator) -> Rejection:
    """``[limits] drive_for_max_s`` may be lowered below the catalog's 2 s."""
    lowered = Validator(config(limits={"drive_for_max_s": 0.5}))
    return _reject(lowered, drive(args={"duration_s": 1.0, "power": 0.15}))


def _source_not_allowed(validator: Validator) -> Rejection:
    """A connection that says hello as brain then sends teleop's source."""
    return _reject(validator, drive(source="teleop"), client=session(Source.BRAIN))


def _estop_active(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(estop_sw=True))


def _stale_seq(validator: Validator) -> Rejection:
    client = session()
    _accept(validator, drive(seq=42), client=client)
    return _reject(validator, drive(cmd_id=CMD2, seq=41), client=client)


def _feedback_stale(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(feedback_age_ms=200.0))


def _unpatched_firmware(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(firmware_ok=False))


def _faulted(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(stop_flags=int(StopFlag.LOWBAT)))


def _obstacle(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(stop_flags=int(StopFlag.TOF)))


def _obs_stale(validator: Validator) -> Rejection:
    return _reject(
        validator,
        drive(obs={"frame_id": "cam-000917", "frame_mono_ns": NOW - 9_000 * MS}),
    )


def _goal_ttl_too_short(validator: Validator) -> Rejection:
    """drive_for(1.0 s) has a 2.0 s deadline; asking for 1.5 s is a bug the
    sender should see, not something to silently truncate."""
    return _reject(validator, drive(goal_ttl_ms=1500))


def _goal_ttl_too_long(_: Validator) -> Rejection:
    """With ``goal_ttl_ms_max`` lowered to 1500 ms, a 1 s drive's 2 s deadline
    no longer fits, whatever ``goal_ttl_ms`` the sender put on the wire."""
    lowered = Validator(config(limits={"goal_ttl_ms_max": 1500}))
    return _reject(lowered, drive(goal_ttl_ms=1500))


def _duplicate_cmd(validator: Validator) -> Rejection:
    client = session()
    _accept(validator, drive(), client=client)
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


def _not_ready(validator: Validator) -> Rejection:
    """A brain skill may not take the wheels from a web stream."""
    active = ActiveCommand(
        cmd_id="other",
        kind=CommandKind.TWIST,
        source=Source.WEB,
        session_id="deadbeef",
        started_mono_ns=NOW,
        deadline_mono_ns=NOW + 10**10,
    )
    return _reject(validator, drive(), ctx=context(active=active))


def _budget_exceeded(validator: Validator) -> Rejection:
    return _reject(validator, drive(), ctx=context(remaining_motion_s=0.5))


REJECTIONS: list[tuple[ResultReason, Callable[[Validator], Rejection]]] = [
    (ResultReason.UNKNOWN_SKILL, _unknown_skill),
    (ResultReason.BAD_ARGS, _bad_args),
    (ResultReason.OUT_OF_BOUNDS, _out_of_bounds),
    (ResultReason.SOURCE_NOT_ALLOWED, _source_not_allowed),
    (ResultReason.ESTOP_ACTIVE, _estop_active),
    (ResultReason.STALE_SEQ, _stale_seq),
    (ResultReason.FEEDBACK_STALE, _feedback_stale),
    (ResultReason.UNPATCHED_FIRMWARE, _unpatched_firmware),
    (ResultReason.FAULTED, _faulted),
    (ResultReason.OBSTACLE, _obstacle),
    (ResultReason.OBS_STALE, _obs_stale),
    (ResultReason.GOAL_TTL_TOO_SHORT, _goal_ttl_too_short),
    (ResultReason.GOAL_TTL_TOO_LONG, _goal_ttl_too_long),
    (ResultReason.DUPLICATE_CMD, _duplicate_cmd),
    (ResultReason.STALE_TURN, _stale_turn),
    (ResultReason.UNAUTHORIZED_UTTERANCE, _unauthorized_utterance),
    (ResultReason.RATE_LIMITED, _rate_limited),
    (ResultReason.NOT_READY, _not_ready),
    (ResultReason.BUDGET_EXCEEDED, _budget_exceeded),
]


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
        ResultReason.POWER_CLAMPED,  # a clamp is reported on an acceptance
        ResultReason.TTL_EXPIRED,  # the deadline, raised by the control loop
        ResultReason.HEADING_UNAVAILABLE,  # a turn losing its heading, mid-turn
        ResultReason.BOX_LOST,  # brain owns the box link (A20)
    }
    covered = {reason for reason, _ in REJECTIONS}
    assert covered | runtime_only == set(ResultReason)


# ---------------------------------------------------------------------------
# Acceptance, the power clamp and the deadlines
# ---------------------------------------------------------------------------


def test_a_legal_drive_is_accepted_with_its_deadline_and_motion_estimate() -> None:
    verdict = _accept(Validator(config()), drive())
    assert verdict.moves
    assert verdict.deadline_ms == 2000  # min(goal_ttl_ms, T2 = 1.0 * 1.5 + 0.5)
    assert verdict.est_motion_s == pytest.approx(1.0)
    assert verdict.power_clamped_to is None
    assert verdict.args == drive().args


def test_a_longer_goal_ttl_keeps_the_deadline_at_t2() -> None:
    verdict = _accept(Validator(config()), drive(goal_ttl_ms=5000))
    assert verdict.deadline_ms == 2000


@pytest.mark.parametrize("power", [0.30, 0.25, 0.21])
def test_a_power_above_the_default_is_clamped_and_reported(power: float) -> None:
    """ADR-0013: above ``power_default`` the value is *clamped*, not rejected,
    and the acceptance carries ``power_clamped_to``."""
    verdict = _accept(
        Validator(config()), drive(args={"duration_s": 1.0, "power": power})
    )
    assert verdict.args.power == pytest.approx(0.20)  # type: ignore[attr-defined]
    assert verdict.power_clamped_to == pytest.approx(0.20)


def test_a_reverse_power_clamps_with_its_sign() -> None:
    verdict = _accept(
        Validator(config()), drive(args={"duration_s": 1.0, "power": -0.30})
    )
    assert verdict.args.power == pytest.approx(-0.20)  # type: ignore[attr-defined]
    assert verdict.power_clamped_to == pytest.approx(0.20)


def test_a_power_at_or_below_the_default_is_untouched() -> None:
    verdict = _accept(Validator(config()), drive(args={"duration_s": 1.0, "power": 0.20}))
    assert verdict.power_clamped_to is None


def test_a_power_above_the_configured_maximum_is_out_of_bounds() -> None:
    """``power_max`` may sit below the catalog's 0.30; above it is a refusal,
    not a clamp."""
    tight = Validator(
        config(limits={"power_max": 0.25, "power_default": 0.20, "twist_power": 0.25})
    )
    rejection = _reject(tight, drive(args={"duration_s": 1.0, "power": 0.28}))
    assert rejection.reason is ResultReason.OUT_OF_BOUNDS
    assert "power_max" in rejection.detail


# ---------------------------------------------------------------------------
# Stop flags: forward blocking and low battery
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("flag", [StopFlag.TOF, StopFlag.BUMPER])
def test_forward_is_blocked_but_reverse_and_turning_pass(flag: StopFlag) -> None:
    validator = Validator(config())
    ctx = context(stop_flags=int(flag))
    forward = _reject(validator, drive(), ctx=ctx)
    assert forward.reason is ResultReason.OBSTACLE
    reverse = _accept(validator, drive(args={"duration_s": 1.0, "power": -0.15}), ctx=ctx)
    assert reverse.moves
    assert isinstance(_accept(validator, turn(cmd_id=CMD2), ctx=ctx), AcceptedSkill)


def test_low_battery_refuses_every_motion_including_reverse_and_turning() -> None:
    validator = Validator(config())
    ctx = context(stop_flags=int(StopFlag.LOWBAT | StopFlag.TOF))
    for message in (
        drive(),
        drive(args={"duration_s": 1.0, "power": -0.15}),
        turn(),
    ):
        assert _reject(validator, message, ctx=ctx).reason is ResultReason.FAULTED
    stream = Validator(config(bus=TELEOP_BUS)).check_twist(
        twist(twist={"lin": -0.1, "ang": 0.0}), session(Source.TELEOP, ("twist",)), ctx
    )
    assert isinstance(stream, Rejection)
    assert stream.reason is ResultReason.FAULTED


def test_the_heartbeat_and_coast_flags_block_nothing_at_dispatch() -> None:
    """Both clear on the next speed line, which the accepted goal will send."""
    ctx = context(stop_flags=int(StopFlag.HEARTBEAT | StopFlag.COAST))
    assert isinstance(_accept(Validator(config()), drive(), ctx=ctx), AcceptedSkill)


def test_low_battery_does_not_mute_a_non_motion_skill() -> None:
    verdict = _accept(
        Validator(config()), say(), ctx=context(stop_flags=int(StopFlag.LOWBAT))
    )
    assert not verdict.moves


# ---------------------------------------------------------------------------
# turn_to
# ---------------------------------------------------------------------------


def test_a_turn_is_accepted_with_its_own_timeout_as_the_deadline() -> None:
    verdict = _accept(Validator(config()), turn(goal_ttl_ms=5000))
    assert verdict.deadline_ms == 4000
    assert verdict.est_motion_s == 0.0, "how long a turn takes is not known up front"


def test_a_turn_whose_goal_ttl_is_below_its_timeout_is_too_short() -> None:
    rejection = _reject(Validator(config()), turn(goal_ttl_ms=3000))
    assert rejection.reason is ResultReason.GOAL_TTL_TOO_SHORT


def test_a_turn_timeout_above_the_configured_maximum_is_out_of_bounds() -> None:
    tight = Validator(config(limits={"turn_timeout_max_s": 2.0}))
    rejection = _reject(tight, turn(args={"heading_deg": 90.0, "timeout_s": 3.0}))
    assert rejection.reason is ResultReason.OUT_OF_BOUNDS


def test_a_turn_is_refused_only_once_the_budget_is_spent() -> None:
    validator = Validator(config())
    assert isinstance(
        _accept(validator, turn(), ctx=context(remaining_motion_s=0.3)), AcceptedSkill
    )
    rejection = _reject(validator, turn(cmd_id=CMD2), ctx=context(remaining_motion_s=0.0))
    assert rejection.reason is ResultReason.BUDGET_EXCEEDED


def test_a_drive_longer_than_the_remaining_budget_is_refused_up_front() -> None:
    """Open loop: a drive's motion time is its duration, exactly, so a drive
    that cannot finish inside the budget never starts."""
    rejection = _reject(
        Validator(config()),
        drive(args={"duration_s": 2.0, "power": 0.15}),
        ctx=context(remaining_motion_s=1.9),
    )
    assert rejection.reason is ResultReason.BUDGET_EXCEEDED
    assert isinstance(
        _accept(
            Validator(config()),
            drive(args={"duration_s": 1.0, "power": 0.15}),
            ctx=context(remaining_motion_s=1.0),
        ),
        AcceptedSkill,
    )


# ---------------------------------------------------------------------------
# Non-motion skills skip the motion rows
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "trace",
    [None, {}, {"model": "rover-vlm"}, {"authorized_motion": None}],
    ids=["no-trace", "empty-trace", "trace-without-the-flag", "flag-is-null"],
)
def test_a_motion_skill_without_the_flag_is_unauthorized(
    trace: dict[str, object] | None,
) -> None:
    rejection = _reject(Validator(config()), drive(trace=trace))
    assert rejection.reason is ResultReason.UNAUTHORIZED_UTTERANCE


@pytest.mark.parametrize("trace", [None, {}], ids=["no-trace", "empty-trace"])
def test_a_non_motion_skill_needs_no_flag(trace: dict[str, object] | None) -> None:
    assert isinstance(_accept(Validator(config()), say(trace=trace)), AcceptedSkill)


def test_a_non_motion_skill_skips_every_motion_row() -> None:
    """say and describe_scene stay permitted when motion is not, so a stale
    link or stock firmware cannot silently mute the robot."""
    verdict = _accept(
        Validator(config()),
        say(),
        ctx=context(
            feedback_age_ms=None,
            firmware_ok=False,
            stop_flags=int(StopFlag.LOWBAT),
            remaining_motion_s=0.0,
        ),
    )
    assert not verdict.moves


def test_a_say_over_three_hundred_characters_does_not_parse() -> None:
    verdict = Validator(config()).parse(
        _skill_payload(skill="say", args={"text": "x" * 301}, goal_ttl_ms=1000)
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.BAD_ARGS


def test_no_feedback_at_all_is_stale_feedback() -> None:
    rejection = _reject(Validator(config()), drive(), ctx=context(feedback_age_ms=None))
    assert rejection.reason is ResultReason.FEEDBACK_STALE


def test_the_cooldown_exempts_later_skills_of_the_same_instruction() -> None:
    """Without the exemption find's sweep loop is rate_limited on its second
    turn (ARCHITECTURE 4.2)."""
    verdict = _accept(
        Validator(config()),
        drive(),
        ctx=context(last_motion_start_mono_ns=NOW - 100 * MS, last_motion_turn_id=TURN),
    )
    assert verdict.moves


# ---------------------------------------------------------------------------
# Replay and the late-response race
# ---------------------------------------------------------------------------


def test_a_repeated_cmd_id_cannot_execute_motion_twice() -> None:
    """I-12: resend an accepted skill verbatim."""
    validator = Validator(config())
    client = session()
    _accept(validator, drive(), client=client)
    repeat = _reject(validator, drive(seq=99), client=client)
    assert repeat.reason is ResultReason.DUPLICATE_CMD


def test_the_replay_window_holds_the_last_sixty_four_pairs() -> None:
    validator = Validator(config())
    client = session()
    for index in range(70):
        message = drive(cmd_id=f"01J9ZC7K3QF2M8XR4V6T0Y{index:04d}", seq=index + 1)
        _accept(validator, message, client=client)
    assert not validator.replay.seen(Source.BRAIN, "01J9ZC7K3QF2M8XR4V6T0Y0000")
    assert validator.replay.seen(Source.BRAIN, "01J9ZC7K3QF2M8XR4V6T0Y0069")


def test_a_late_response_carrying_the_previous_turn_cannot_start_motion() -> None:
    """I-11: the box answer that lands after a stop or a new turn boundary."""
    rejection = _reject(
        Validator(config()), drive(turn_id=OLD_TURN), ctx=context(current_turn_id=TURN)
    )
    assert rejection.reason is ResultReason.STALE_TURN


def test_a_skill_before_any_turn_boundary_is_stale() -> None:
    rejection = _reject(Validator(config()), drive(), ctx=context(current_turn_id=None))
    assert rejection.reason is ResultReason.STALE_TURN


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
    verdict = Validator(config(bus=TELEOP_BUS)).check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context()
    )
    assert isinstance(verdict, AcceptedTwist)


def test_a_connection_bound_as_brain_may_not_send_teleop_twists() -> None:
    verdict = Validator(config(bus=TELEOP_BUS)).check_twist(
        twist(), session(Source.BRAIN, ("twist",)), context()
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_a_forward_twist_is_refused_under_a_forward_block() -> None:
    verdict = Validator(config(bus=TELEOP_BUS)).check_twist(
        twist(),
        session(Source.TELEOP, ("twist",)),
        context(stop_flags=int(StopFlag.BUMPER)),
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.OBSTACLE


def test_a_reverse_or_turning_twist_survives_a_forward_block() -> None:
    validator = Validator(config(bus=TELEOP_BUS))
    ctx = context(stop_flags=int(StopFlag.TOF))
    for payload in ({"lin": -0.10, "ang": 0.0}, {"lin": 0.0, "ang": 0.2}):
        verdict = validator.check_twist(
            twist(twist=payload), session(Source.TELEOP, ("twist",)), ctx
        )
        assert isinstance(verdict, AcceptedTwist), verdict


def test_a_twist_needs_fresh_feedback_from_the_fork() -> None:
    validator = Validator(config(bus=TELEOP_BUS))
    stale = validator.check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context(feedback_age_ms=500.0)
    )
    assert isinstance(stale, Rejection)
    assert stale.reason is ResultReason.FEEDBACK_STALE
    stock = validator.check_twist(
        twist(), session(Source.TELEOP, ("twist",)), context(firmware_ok=False)
    )
    assert isinstance(stock, Rejection)
    assert stock.reason is ResultReason.UNPATCHED_FIRMWARE


def test_an_out_of_bounds_twist_does_not_parse() -> None:
    verdict = Validator(config(bus=TELEOP_BUS)).parse(
        {
            "v": 1,
            "type": "twist",
            "source": "teleop",
            "cmd_id": CMD,
            "seq": 1,
            "twist": {"lin": 0.31, "ang": 0.0},
        }
    )
    assert isinstance(verdict, Rejection)
    assert verdict.reason is ResultReason.BAD_ARGS


def test_a_non_finite_twist_does_not_parse() -> None:
    verdict = Validator(config(bus=TELEOP_BUS)).parse(
        {
            "v": 1,
            "type": "twist",
            "source": "teleop",
            "cmd_id": CMD,
            "seq": 1,
            "twist": {"lin": float("nan"), "ang": 0.0},
        }
    )
    assert isinstance(verdict, Rejection)


def test_a_second_motion_skill_is_refused_while_one_is_in_flight() -> None:
    active = ActiveCommand(
        cmd_id="other",
        kind=CommandKind.SKILL,
        source=Source.BRAIN,
        session_id="deadbeef",
        started_mono_ns=NOW,
        deadline_mono_ns=NOW + 10**10,
        skill="drive_for",
    )
    rejection = _reject(Validator(config()), drive(), ctx=context(active=active))
    assert rejection.reason is ResultReason.RATE_LIMITED


# ---------------------------------------------------------------------------
# clear (not stop-class) and the connection binding
# ---------------------------------------------------------------------------


def test_clear_is_refused_from_brain() -> None:
    """brain, whose job is acting on model output, must not be able to clear a
    stop authority."""
    message = ClearMessage(source=Source.BRAIN, faults=["estop_sw"])
    rejection = Validator(config()).check_clear(message, session(Source.BRAIN))
    assert rejection is not None
    assert rejection.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_clear_is_accepted_from_web() -> None:
    message = ClearMessage(source=Source.WEB, faults=["estop_sw"])
    assert Validator(config()).check_clear(message, session(Source.WEB)) is None


def test_a_second_hello_on_one_connection_is_refused() -> None:
    client = session()
    with pytest.raises(ValueError, match="second hello"):
        client.bind(
            HelloMessage(source=Source.WEB, pid=1, caps=[BusCap.SKILL]),
            expected_uid=None,
        )


def test_an_unbound_connection_may_not_command() -> None:
    rejection = _reject(Validator(config()), drive(), client=ClientSession())
    assert rejection.reason is ResultReason.SOURCE_NOT_ALLOWED


def test_a_peer_uid_that_does_not_match_the_unit_user_is_refused() -> None:
    client = ClientSession(peer=PeerCredentials(uid=1000, gid=1000, pid=42))
    with pytest.raises(ValueError, match="may not claim source"):
        client.bind(
            HelloMessage(source=Source.WEB, pid=42, caps=[BusCap.SKILL]),
            expected_uid=999,
        )


def test_seq_is_strictly_increasing_per_connection() -> None:
    client = session()
    assert client.seq_ok(5)
    client.note_seq(5)
    assert not client.seq_ok(5)
    assert not client.seq_ok(4)
    assert client.seq_ok(6)
