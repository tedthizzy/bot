"""Every check of ARCHITECTURE 4.2, in the order 4.2 states them.

The model proposes, robotd validates, the MCU decides whether motion is safe
(principle 1).  This module is the middle layer and nothing else in robotd may
accept a command: each check returns a typed :class:`Rejection` carrying one of
the speakable ``result.reason`` values, so a refusal is always attributable to
one row of the table rather than to "validation failed".

Stop-class messages -- ``stop``, ``estop`` and ``cancel`` -- never reach this
module.  I-22 makes them skip every row below and forbids ever answering one
``rejected``, so they are dispatched on the raw ``"type"`` before strict parsing
(see :mod:`rover_robotd.main`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Final, cast

from pydantic import BaseModel, ValidationError
from rover_contracts.config import RobotConfig
from rover_contracts.messages import (
    ClearMessage,
    ClientMessage,
    DriveBusArgs,
    ResultReason,
    SkillMessage,
    Source,
    TurnBusArgs,
    TwistMessage,
    TwistPayload,
    client_adapter,
)
from rover_contracts.serial_codec import (
    LATCHED_FAULTS,
    OBSTACLE_FAULTS,
    TOF_ERROR_MM,
    TOF_NO_TARGET_MM,
)
from rover_contracts.skills import SKILLS, Bound, goal_deadline_s
from rover_contracts.units import deg_to_rad, mps_to_cms

from rover_robotd.arbiter import ActiveCommand, Arbitration, CommandKind, arbitrate
from rover_robotd.session import ClientSession, ReplayWindow

__all__ = [
    "OBSTACLE_REVERSE_CAP_M",
    "SPEED_UNLOCK_RANGE_MM",
    "AcceptedSkill",
    "AcceptedTwist",
    "Rejection",
    "ValidationContext",
    "Validator",
]

SPEED_UNLOCK_RANGE_MM: Final = 1000
"""ARCHITECTURE 6: ``speed_mps`` is unlocked above ``speed_default_mps`` only
while the front ToF is valid and beyond 1.0 m; a 65534 no-target return
satisfies that and a 65535 error does not."""

OBSTACLE_REVERSE_CAP_M: Final = 0.30
"""ARCHITECTURE 6: ``distance_cm`` is capped at -30 while any obstacle-class bit
is set.  Reverse is unsensed, and the MCU independently clamps it to 150 mm/s."""


@dataclass(frozen=True, slots=True)
class Rejection:
    """One refused command, with the row of 4.2 that refused it."""

    reason: ResultReason
    detail: str = ""


@dataclass(frozen=True, slots=True)
class AcceptedSkill:
    """A skill that passed every row, with the values robotd will execute.

    ``args`` may differ from ``message.args``: ``drive``'s speed is clamped to
    the cap in force rather than rejected, and its distance is clamped to
    :data:`OBSTACLE_REVERSE_CAP_M` under an obstacle-class bit.
    """

    message: SkillMessage
    args: BaseModel
    moves: bool
    deadline_ms: int
    est_path_m: float
    est_motion_s: float
    arbitration: Arbitration
    speed_clamped_to_cms: int | None = None


@dataclass(frozen=True, slots=True)
class AcceptedTwist:
    """A streamed velocity that passed the twist subset of 4.2."""

    message: TwistMessage
    arbitration: Arbitration


@dataclass(frozen=True, slots=True)
class ValidationContext:
    """Everything the checks read, sampled once so a decision is consistent."""

    now_mono_ns: int
    telemetry_age_ms: float | None
    mcu_fault: int
    front_range_mm: int | None
    estop_sw: bool
    current_turn_id: str | None
    active: ActiveCommand | None
    last_motion_start_mono_ns: int | None
    last_motion_turn_id: str | None
    remaining_path_m: float
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

    # -- the cap in force ---------------------------------------------------

    def speed_cap_mps(self, front_range_mm: int | None) -> float:
        """``speed_default_mps``, unlocked to ``speed_mps`` on a clear path."""
        limits = self.config.limits
        if front_range_mm is None or front_range_mm == TOF_ERROR_MM:
            return limits.speed_default_mps
        unlocked = (
            front_range_mm == TOF_NO_TARGET_MM
            or front_range_mm > SPEED_UNLOCK_RANGE_MM
        )
        return limits.speed_mps if unlocked else limits.speed_default_mps

    # -- skills -------------------------------------------------------------

    def check_skill(
        self, message: SkillMessage, session: ClientSession, ctx: ValidationContext
    ) -> AcceptedSkill | Rejection:
        """Every row of 4.2 for a ``skill``, in the document's order."""
        spec = SKILLS[message.skill]
        moves = spec.moves

        # 1. known skill / strict bounds in SI.
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
        clamped_to_cms: int | None = None
        est_path_m = 0.0
        est_motion_s = 0.0
        deadline_ms = message.goal_ttl_ms

        if moves:
            # 4. telemetry age within T0.
            fresh = self._check_telemetry(ctx)
            if fresh is not None:
                return fresh

            # 5. no latched fault; no forward component under an obstacle bit.
            faults = self._check_faults(ctx.mcu_fault, message.skill, message.args)
            if faults is not None:
                return faults

            # 6. observation age on the monotonic clock (I-23).
            stale = self._check_observation(message, ctx)
            if stale is not None:
                return stale

            # 7. goal_ttl_ms in range and consistent with the T2 estimate.
            prepared = self._prepare_motion(message, ctx)
            if isinstance(prepared, Rejection):
                return prepared
            args, clamped_to_cms, est_path_m, est_motion_s, deadline_ms = prepared

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

            # 12. the per-instruction budget (I-15).
            if est_path_m > ctx.remaining_path_m or est_motion_s > ctx.remaining_motion_s:
                return Rejection(
                    ResultReason.BUDGET_EXCEEDED,
                    f"needs {est_path_m:.3f} m / {est_motion_s:.1f} s, "
                    f"left {ctx.remaining_path_m:.3f} m / {ctx.remaining_motion_s:.1f} s",
                )

        self.replay.remember(message.source, message.cmd_id)
        return AcceptedSkill(
            message=message,
            args=args,
            moves=moves,
            deadline_ms=deadline_ms,
            est_path_m=est_path_m,
            est_motion_s=est_motion_s,
            arbitration=arbitration,
            speed_clamped_to_cms=clamped_to_cms,
        )

    # -- twist --------------------------------------------------------------

    def check_twist(
        self, message: TwistMessage, session: ClientSession, ctx: ValidationContext
    ) -> AcceptedTwist | Rejection:
        """The twist subset of 4.2: bounds, source plus ``allow_stream``, seq,
        telemetry age, faults and arbitration.  A stream has no goal TTL, no
        ``turn_id`` and no replay window -- it is one arbitration unit."""
        bounds = self._check_bounds("twist", message.twist)
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

        fresh = self._check_telemetry(ctx)
        if fresh is not None:
            return fresh

        faults = self._check_faults(ctx.mcu_fault, "twist", message.twist)
        if faults is not None:
            return faults

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
        spec = SKILLS.get(skill)
        if spec is None:
            return Rejection(ResultReason.UNKNOWN_SKILL, skill)
        for bound in spec.bounds:
            value = _bound_value(bound, args)
            if value is None:
                continue
            if not math.isfinite(value) or not bound.contains(value):
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"{skill}.{bound.field}={value} outside "
                    f"[{bound.lo}, {bound.hi}] {bound.unit}",
                )
        limits = self.config.limits
        if isinstance(args, DriveBusArgs) and abs(args.distance_m) > limits.drive_m:
            return Rejection(
                ResultReason.OUT_OF_BOUNDS,
                f"drive.distance_m={args.distance_m} above drive_m={limits.drive_m}",
            )
        if isinstance(args, TurnBusArgs):
            if abs(args.angle_deg) > limits.turn_deg:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"turn.angle_deg={args.angle_deg} above turn_deg={limits.turn_deg}",
                )
            if args.rate_dps > limits.rate_dps:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"turn.rate_dps={args.rate_dps} above rate_dps={limits.rate_dps}",
                )
        # A35 calls twist the widest motion surface in the design, and A33 says
        # every [limits] key is republished so a gate can read the running
        # value.  Checking only the static catalog row left both twist keys
        # unenforced anywhere in robotd.
        if isinstance(args, TwistPayload):
            if abs(args.linear_x_mps) > limits.twist_linear_mps:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"twist.linear_x_mps={args.linear_x_mps} above "
                    f"twist_linear_mps={limits.twist_linear_mps}",
                )
            if abs(args.angular_z_radps) > limits.twist_angular_radps:
                return Rejection(
                    ResultReason.OUT_OF_BOUNDS,
                    f"twist.angular_z_radps={args.angular_z_radps} above "
                    f"twist_angular_radps={limits.twist_angular_radps}",
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

    def _check_telemetry(self, ctx: ValidationContext) -> Rejection | None:
        limit = self.config.serial.cmd_gate_max_age_ms
        if ctx.telemetry_age_ms is None:
            return Rejection(ResultReason.NOT_READY, "no telemetry yet")
        if ctx.telemetry_age_ms > limit:
            return Rejection(
                ResultReason.NOT_READY,
                f"telemetry {ctx.telemetry_age_ms:.0f} ms old, limit {limit} ms",
            )
        return None

    def _check_faults(
        self, fault: int, skill: str, args: BaseModel
    ) -> Rejection | None:
        latched = fault & int(LATCHED_FAULTS)
        if latched:
            return Rejection(ResultReason.FAULTED, f"fault=0x{latched:X}")
        obstacle = fault & int(OBSTACLE_FAULTS)
        if obstacle and _forward_component(skill, args) > 0.0:
            return Rejection(
                ResultReason.OBSTACLE, f"forward refused, fault=0x{obstacle:X}"
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
        self, message: SkillMessage, ctx: ValidationContext
    ) -> tuple[BaseModel, int | None, float, float, int] | Rejection:
        """Apply the clamps of ARCHITECTURE 6, then the two goal-TTL rules."""
        limits = self.config.limits
        args: BaseModel = message.args
        clamped_to_cms: int | None = None

        if isinstance(args, DriveBusArgs):
            cap = self.speed_cap_mps(ctx.front_range_mm)
            speed = args.speed_mps
            if speed > cap:
                speed = cap
                clamped_to_cms = mps_to_cms(cap)
            distance = args.distance_m
            if ctx.mcu_fault & int(OBSTACLE_FAULTS):
                distance = max(distance, -OBSTACLE_REVERSE_CAP_M)
            args = DriveBusArgs(distance_m=distance, speed_mps=speed)
            est_path_m = abs(distance)
            est_motion_s = goal_deadline_s(distance, speed)
            # The sender derived goal_ttl_ms from the speed it asked for, so
            # that is the T2 its deadline has to be tested against.
            sender_motion_s = goal_deadline_s(distance, message.args.speed_mps)
        elif isinstance(args, TurnBusArgs):
            est_path_m = 0.0
            est_motion_s = goal_deadline_s(
                deg_to_rad(args.angle_deg), deg_to_rad(args.rate_dps)
            )
            sender_motion_s = est_motion_s
        else:  # pragma: no cover - only drive and turn move
            return Rejection(ResultReason.UNKNOWN_SKILL, message.skill)

        t2_ms = math.ceil(est_motion_s * 1000.0)
        t2_sender_ms = math.ceil(sender_motion_s * 1000.0)
        if message.goal_ttl_ms > limits.goal_ttl_ms_max:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_LONG,
                f"goal_ttl_ms={message.goal_ttl_ms} above "
                f"goal_ttl_ms_max={limits.goal_ttl_ms_max}",
            )
        ceiling_ms = min(
            limits.goal_ttl_ms_max, math.floor(ctx.remaining_motion_s * 1000.0)
        )
        if t2_ms > ceiling_ms:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_LONG,
                f"T2={t2_ms} ms above min(goal_ttl_ms_max, remaining budget)"
                f"={ceiling_ms} ms",
            )
        # Against the *sender's* T2, not the clamped one.  A clamp always makes
        # T2 larger -- a slower drive takes longer -- so testing the sender's
        # deadline against the clamped estimate rejected every model drive above
        # speed_default_cms within a metre of anything, with `goal_ttl_too_short`
        # in place of the clamp-and-report ARCHITECTURE 6 promises.
        if message.goal_ttl_ms < t2_sender_ms:
            return Rejection(
                ResultReason.GOAL_TTL_TOO_SHORT,
                f"goal_ttl_ms={message.goal_ttl_ms} below T2={t2_sender_ms} ms",
            )
        # 4.2's effective deadline is min(goal_ttl_ms, T2).  When the clamp is
        # the validator's own doing, the deadline the clamped drive needs is the
        # validator's to extend too -- otherwise the drive it just accepted is
        # aborted `timeout` part-way, which is the disagreement between two
        # deadlines that rule exists to prevent.  `detail.speed_clamped_to_cms`
        # and `deadline_in_ms` both report the change.
        deadline_ms = (
            min(t2_ms, limits.goal_ttl_ms_max)
            if clamped_to_cms is not None
            else min(message.goal_ttl_ms, t2_ms)
        )
        return (args, clamped_to_cms, est_path_m, est_motion_s, deadline_ms)

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
    """The +x component a command would produce, in m/s or m."""
    if isinstance(args, DriveBusArgs):
        return args.distance_m
    linear = getattr(args, "linear_x_mps", None)
    return float(linear) if isinstance(linear, int | float) else 0.0


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
