"""Every check of ARCHITECTURE 4.2, in the order 4.2 states them, with the
bounds of ADR-0013's catalog.

The model proposes, robotd validates, the controller decides whether motion is
safe (principle 1).  This module is the middle layer and nothing else in robotd
may accept a command: each check returns a typed :class:`Rejection` carrying
one of the speakable ``result.reason`` values, so a refusal is always
attributable to one row of the table rather than to "validation failed".

Stop-class messages -- ``stop``, ``estop`` and ``cancel`` -- never reach this
module.  I-22 makes them skip every row below and forbids ever answering one
``rejected``, so they are dispatched on the raw ``"type"`` before strict parsing
(see :mod:`rover_robotd.main`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, cast

from pydantic import BaseModel, ValidationError
from rover_contracts.config import RobotConfig
from rover_contracts.messages import (
    ClearMessage,
    ClientMessage,
    DriveForBusArgs,
    ResultReason,
    SkillMessage,
    Source,
    TurnToBusArgs,
    TwistMessage,
    TwistPayload,
    client_adapter,
)
from rover_contracts.skills import SKILLS, TWIST, TWIST_BOUNDS, Bound, goal_deadline_s
from rover_contracts.wave_proto import StopFlag

from rover_robotd.arbiter import ActiveCommand, Arbitration, CommandKind, arbitrate
from rover_robotd.clients import ClientSession, ReplayWindow

__all__ = [
    "AcceptedSkill",
    "AcceptedTwist",
    "Rejection",
    "ValidationContext",
    "Validator",
]


@dataclass(frozen=True, slots=True)
class Rejection:
    """One refused command, with the row of 4.2 that refused it."""

    reason: ResultReason
    detail: str = ""


@dataclass(frozen=True, slots=True)
class AcceptedSkill:
    """A skill that passed every row, with the values robotd will execute.

    ``args`` may differ from ``message.args``: ``drive_for``'s power is clamped
    to ``[limits] power_default`` rather than rejected, and ``power_clamped_to``
    says so for the ``accepted`` result.
    """

    message: SkillMessage
    args: BaseModel
    moves: bool
    deadline_ms: int
    est_motion_s: float
    arbitration: Arbitration
    power_clamped_to: float | None = None


@dataclass(frozen=True, slots=True)
class AcceptedTwist:
    """A streamed command that passed the twist subset of 4.2."""

    message: TwistMessage
    arbitration: Arbitration


@dataclass(frozen=True, slots=True)
class ValidationContext:
    """Everything the checks read, sampled once so a decision is consistent."""

    now_mono_ns: int
    feedback_age_ms: float | None
    firmware_ok: bool
    stop_flags: int
    estop_sw: bool
    current_turn_id: str | None
    active: ActiveCommand | None
    last_motion_start_mono_ns: int | None
    last_motion_turn_id: str | None
    remaining_motion_s: float


def _bound_value(bound: Bound, args: BaseModel) -> float | None:
    value = getattr(args, bound.field, None)
    if isinstance(value, str):
        return len(value)
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    return float(value)


class Validator:
    """The 4.2 table, as code.  Holds no live state beyond the replay window."""

    def __init__(self, config: RobotConfig, replay: ReplayWindow | None = None) -> None:
        self.config = config
        self.replay = replay if replay is not None else ReplayWindow()

    # -- stage one: strict parsing -----------------------------------------

    def parse(self, payload: Any) -> ClientMessage | Rejection:
        """A12 stage two: strict, ``extra="forbid"``, no non-finite numbers.

        An unrecognised ``skill`` name is separated out here because
        ``unknown_skill`` and ``bad_args`` are different answers to the sender
        and pydantic reports both as one discriminator failure.
        """
        if isinstance(payload, dict) and payload.get("type") == "skill":
            name = payload.get("skill")
            if not isinstance(name, str) or name not in SKILLS:
                return Rejection(ResultReason.UNKNOWN_SKILL, f"skill={name!r}")
        try:
            return cast(ClientMessage, client_adapter.validate_python(payload))
        except ValidationError as exc:
            return Rejection(ResultReason.BAD_ARGS, _first_error(exc))

    # -- skills -------------------------------------------------------------

    def check_skill(
        self, message: SkillMessage, session: ClientSession, ctx: ValidationContext
    ) -> AcceptedSkill | Rejection:
        """Every row of 4.2 for a ``skill``, in the document's order."""
        spec = SKILLS[message.skill]
        moves = spec.moves

        # 1. known skill / strict bounds.
        bounds = self._check_bounds(message.skill, message.args)
        if bounds is not None:
            return bounds

        # 2. source bound to the connection and in allow_sources.
        source = self._check_source(
            message.source, session, tuple(self.config.bus.allow_sources)
        )
        if source is not None:
            return source

        # 2b. the software e-stop latch refuses every skill from every source.
        if ctx.estop_sw:
            return Rejection(ResultReason.ESTOP_ACTIVE, "estop_sw latched")

        # 3. seq strictly increasing per (source, client session).
        if not session.seq_ok(message.seq):
            return Rejection(
                ResultReason.STALE_SEQ, f"seq={message.seq} last={session.last_seq}"
            )
        session.note_seq(message.seq)

        args: BaseModel = message.args
        power_clamped_to: float | None = None
        est_motion_s = 0.0
        deadline_ms = message.goal_ttl_ms

        if moves:
            # 4. feedback inside T0, from firmware confirmed as the fork.
            link = self._check_link(ctx)
            if link is not None:
                return link

            # 5. no flag that blocks all motion; no forward component while
            # the controller blocks forward.
            flags = self._check_flags(ctx.stop_flags, message.skill, message.args)
            if flags is not None:
                return flags

            # 6. observation age on the monotonic clock (I-23).
            stale = self._check_observation(message, ctx)
            if stale is not None:
                return stale

            # 7. the power clamp, then goal_ttl_ms against the deadline.
            prepared = self._prepare_motion(message)
            if isinstance(prepared, Rejection):
                return prepared
            args, power_clamped_to, est_motion_s, deadline_ms = prepared

        # 8. ULID cmd_id not in the last-64 replay window.
        if self.replay.seen(message.source, message.cmd_id):
            return Rejection(ResultReason.DUPLICATE_CMD, message.cmd_id)

        # 9. turn_id current.
        if not ctx.current_turn_id or message.turn_id != ctx.current_turn_id:
            return Rejection(
                ResultReason.STALE_TURN,
                f"turn_id={message.turn_id} current={ctx.current_turn_id}",
            )

        arbitration = Arbitration.START
        if moves:
            # 10. the utterance that justified this must have been authorized.
            # The flag is *required* on a motion skill, not merely respected
            # when present: section 7 makes a null STT confidence authorized
            # deliberately, and says nothing about a missing flag.  Treating an
            # absent trace as authorized re-enables motion with no authorizing
            # utterance -- exactly what I-21 asserts cannot happen -- for any
            # sender that never learned to set it.
            if message.trace is None or message.trace.authorized_motion is not True:
                return Rejection(
                    ResultReason.UNAUTHORIZED_UTTERANCE,
                    "authorized_motion=false"
                    if message.trace is not None
                    else "no trace on a motion skill",
                )

            # 11. one motion in flight; cooldown per turn_id.
            cooldown = self._check_cooldown(message.turn_id, ctx)
            if cooldown is not None:
                return cooldown
            arbitration = arbitrate(
                message.source, CommandKind.SKILL, message.cmd_id, ctx.active
            )
            blocked = _arbitration_rejection(arbitration)
            if blocked is not None:
                return blocked

            # 12. the per-instruction budget (I-15), in motion seconds.
            if ctx.remaining_motion_s <= 0.0 or est_motion_s > ctx.remaining_motion_s:
                return Rejection(
                    ResultReason.BUDGET_EXCEEDED,
                    f"needs {est_motion_s:.1f} s of motion, "
                    f"{ctx.remaining_motion_s:.1f} s left on this instruction",
                )

        self.replay.remember(message.source, message.cmd_id)
        return AcceptedSkill(
            message=message,
            args=args,
            moves=moves,
            deadline_ms=deadline_ms,
            est_motion_s=est_motion_s,
            arbitration=arbitration,
            power_clamped_to=power_clamped_to,
        )

    # -- twist --------------------------------------------------------------

    def check_twist(
        self, message: TwistMessage, session: ClientSession, ctx: ValidationContext
    ) -> AcceptedTwist | Rejection:
        """The twist subset of 4.2: bounds, source plus ``allow_stream``, seq,
        the link, the stop flags and arbitration.  A stream has no goal TTL, no
        ``turn_id`` and no replay window -- it is one arbitration unit."""
        bounds = self._check_bounds(TWIST, message.twist)
        if bounds is not None:
            return bounds

        allow = tuple(
            s
            for s in self.config.bus.allow_sources
            if s in self.config.bus.allow_stream
        )
        source = self._check_source(message.source, session, allow)
        if source is not None:
            return source

        if ctx.estop_sw:
            return Rejection(ResultReason.ESTOP_ACTIVE, "estop_sw latched")

        if not session.seq_ok(message.seq):
            return Rejection(
                ResultReason.STALE_SEQ, f"seq={message.seq} last={session.last_seq}"
            )
        session.note_seq(message.seq)

        link = self._check_link(ctx)
        if link is not None:
            return link

        flags = self._check_flags(ctx.stop_flags, TWIST, message.twist)
        if flags is not None:
            return flags

        arbitration = arbitrate(
            message.source, CommandKind.TWIST, message.cmd_id, ctx.active
        )
        blocked = _arbitration_rejection(arbitration)
        if blocked is not None:
            return blocked
        return AcceptedTwist(message=message, arbitration=arbitration)

    # -- clear --------------------------------------------------------------

    def check_clear(
        self, message: ClearMessage, session: ClientSession
    ) -> Rejection | None:
        """``clear`` is not stop-class: it is restricted to
        ``[bus] clear_sources`` and checked against the bound source, because
        brain -- whose whole job is acting on model output -- must not be able
        to clear a stop authority."""
        return self._check_source(
            message.source, session, tuple(self.config.bus.clear_sources)
        )

    # -- individual rows ----------------------------------------------------

    def _check_bounds(self, skill: str, args: BaseModel) -> Rejection | None:
        if skill == TWIST:
            bounds = TWIST_BOUNDS
        else:
            spec = SKILLS.get(skill)
            if spec is None:
                return Rejection(ResultReason.UNKNOWN_SKILL, skill)
            bounds = spec.bounds
        for bound in bounds:
            value = _bound_value(bound, args)
            if value is None:
                continue
            if not math.isfinite(value) or not bound.contains(value):
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"{skill}.{bound.field}={value} outside "
                    f"[{bound.lo}, {bound.hi}] {bound.unit}",
                )
        # The configured [limits], which may sit below the catalog (A33).
        limits = self.config.limits
        if isinstance(args, DriveForBusArgs):
            if args.duration_s > limits.drive_for_max_s:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"drive_for.duration_s={args.duration_s} above "
                    f"drive_for_max_s={limits.drive_for_max_s}",
                )
            if abs(args.power) > limits.power_max:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"drive_for.power={args.power} above power_max={limits.power_max}",
                )
        if isinstance(args, TurnToBusArgs) and args.timeout_s > limits.turn_timeout_max_s:
            return Rejection(
                ResultReason.OUT_OF_BOUNDS,
                f"turn_to.timeout_s={args.timeout_s} above "
                f"turn_timeout_max_s={limits.turn_timeout_max_s}",
            )
        return None

    def _check_source(
        self, source: Source, session: ClientSession, allow: tuple[str, ...]
    ) -> Rejection | None:
        if not session.bound:
            return Rejection(
                ResultReason.SOURCE_NOT_ALLOWED, "no hello on this connection"
            )
        if source is not session.source:
            return Rejection(
                ResultReason.SOURCE_NOT_ALLOWED,
                f"declared {source!r}, bound {session.source!r}",
            )
        if str(source) not in allow:
            return Rejection(
                ResultReason.SOURCE_NOT_ALLOWED, f"{source!r} not in {list(allow)}"
            )
        return None

    def _check_link(self, ctx: ValidationContext) -> Rejection | None:
        limit = self.config.safety.feedback_max_age_ms
        if ctx.feedback_age_ms is None:
            return Rejection(ResultReason.FEEDBACK_STALE, "no feedback from the rover")
        if ctx.feedback_age_ms > limit:
            return Rejection(
                ResultReason.FEEDBACK_STALE,
                f"feedback {ctx.feedback_age_ms:.0f} ms old, limit {limit} ms",
            )
        if not ctx.firmware_ok:
            return Rejection(
                ResultReason.UNPATCHED_FIRMWARE,
                "the controller is not running the firmware fork",
            )
        return None

    def _check_flags(
        self, stop_flags: int, skill: str, args: BaseModel
    ) -> Rejection | None:
        flags = StopFlag(stop_flags & 0x1F)
        if flags.blocks_all:
            return Rejection(
                ResultReason.FAULTED, f"stop_flags=0x{int(flags):X} (low battery)"
            )
        if flags.blocks_forward and _forward_component(skill, args) > 0.0:
            return Rejection(
                ResultReason.OBSTACLE, f"forward refused, stop_flags=0x{int(flags):X}"
            )
        return None

    def _check_observation(
        self, message: SkillMessage, ctx: ValidationContext
    ) -> Rejection | None:
        if message.obs is None:
            return Rejection(ResultReason.OBS_STALE, "no observation attached")
        age_ms = (ctx.now_mono_ns - message.obs.frame_mono_ns) / 1e6
        limit = self.config.safety.obs_max_age_ms
        if age_ms > limit or age_ms < 0.0:
            return Rejection(
                ResultReason.OBS_STALE,
                f"observation {age_ms:.0f} ms old, limit {limit} ms",
            )
        return None

    def _prepare_motion(
        self, message: SkillMessage
    ) -> tuple[BaseModel, float | None, float, int] | Rejection:
        """Clamp the power, then hold ``goal_ttl_ms`` to the deadline.

        ``drive_for``'s deadline is T2, ``goal_deadline_s(duration_s)``; its
        motion estimate is the duration itself, exactly, because the rover is
        open loop.  ``turn_to`` ends by construction at its own ``timeout_s``,
        so that is its deadline, and its motion time is unknown until the
        heading says so: the budget is charged as it turns and refuses only
        what is already spent.
        """
        limits = self.config.limits
        args: BaseModel = message.args
        power_clamped_to: float | None = None

        if isinstance(args, DriveForBusArgs):
            power = args.power
            if abs(power) > limits.power_default:
                power = math.copysign(limits.power_default, power)
                power_clamped_to = limits.power_default
                args = DriveForBusArgs(duration_s=args.duration_s, power=power)
            est_motion_s = args.duration_s
            t2_ms = math.ceil(goal_deadline_s(args.duration_s) * 1000.0)
        elif isinstance(args, TurnToBusArgs):
            est_motion_s = 0.0
            t2_ms = math.ceil(args.timeout_s * 1000.0)
        else:  # pragma: no cover - only drive_for and turn_to move
            return Rejection(ResultReason.UNKNOWN_SKILL, message.skill)

        if message.goal_ttl_ms > limits.goal_ttl_ms_max:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_LONG,
                f"goal_ttl_ms={message.goal_ttl_ms} above "
                f"goal_ttl_ms_max={limits.goal_ttl_ms_max}",
            )
        if t2_ms > limits.goal_ttl_ms_max:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_LONG,
                f"deadline {t2_ms} ms above goal_ttl_ms_max={limits.goal_ttl_ms_max}",
            )
        # Two deadlines governing one command, disagreeing, is a bug the sender
        # should see rather than a drive silently cut short.
        if message.goal_ttl_ms < t2_ms:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_SHORT,
                f"goal_ttl_ms={message.goal_ttl_ms} below the deadline {t2_ms} ms",
            )
        return (args, power_clamped_to, est_motion_s, min(message.goal_ttl_ms, t2_ms))

    def _check_cooldown(
        self, turn_id: str, ctx: ValidationContext
    ) -> Rejection | None:
        """``motion_cooldown_ms`` applies to the **first** motion of a
        ``turn_id``; later skills sharing that id are exempt, because the
        per-instruction budget is the limiter inside a turn."""
        cooldown_ms = self.config.limits.motion_cooldown_ms
        if cooldown_ms <= 0 or ctx.last_motion_start_mono_ns is None:
            return None
        if turn_id == ctx.last_motion_turn_id:
            return None
        elapsed_ms = (ctx.now_mono_ns - ctx.last_motion_start_mono_ns) / 1e6
        if elapsed_ms < cooldown_ms:
            return Rejection(
                ResultReason.RATE_LIMITED,
                f"{elapsed_ms:.0f} ms since the last motion, "
                f"cooldown {cooldown_ms} ms",
            )
        return None


def _forward_component(skill: str, args: BaseModel) -> float:
    """The forward power a command would apply.  A turn has none."""
    if isinstance(args, DriveForBusArgs):
        return args.power
    if isinstance(args, TwistPayload):
        return args.lin
    return 0.0


def _arbitration_rejection(arbitration: Arbitration) -> Rejection | None:
    if arbitration is Arbitration.BUSY:
        return Rejection(ResultReason.RATE_LIMITED, "one motion already in flight")
    if arbitration is Arbitration.OUTRANKED:
        return Rejection(
            ResultReason.NOT_READY, "a higher-priority source owns the wheels"
        )
    return None


def _first_error(exc: ValidationError) -> str:
    errors = exc.errors()
    if not errors:  # pragma: no cover - pydantic always reports at least one
        return "invalid"
    first = errors[0]
    location = ".".join(str(part) for part in first["loc"])
    return f"{location}: {first['msg']}"[:200]
