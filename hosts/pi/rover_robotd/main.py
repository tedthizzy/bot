"""``rover-robotd``: the only process that can move the robot.

This module is the wiring.  Every rule it enforces is stated in one of the
modules beside it -- :mod:`~rover_robotd.validator` owns the 4.2 table,
:mod:`~rover_robotd.arbiter` owns who has the wheels, :mod:`~rover_robotd.budget`
owns I-15, :mod:`~rover_robotd.profiles` owns what a skill streams and
:mod:`~rover_robotd.link` owns the port -- so what is left here is dispatch,
the control loop and the published state.

Two shapes are load-bearing and are easy to lose in a refactor:

* **Stop-class first.**  ``stop``, ``estop`` and ``cancel`` are dispatched on
  the raw ``"type"`` before any strict parsing, so a malformed stop is still a
  stop and is never answered ``rejected`` (I-22).
* **One loop owns the goal and the wire.**  The control loop recomputes the
  active command every period and hands the result to :meth:`Link.command`,
  zeros when nothing is active.  Nothing else sends a speed line, so a frozen
  robotd stops sending and the firmware's heartbeat zeroes the motors within
  ``[safety] heartbeat_ms`` (I-1, I-14).  A separate keep-alive task would
  defeat exactly that, which is why there is none.

The systemd watchdog ping is liveness, not readiness: it is sent from the
control loop while it is scheduling, whether or not the controller is there.
Gating it on fresh feedback would restart-loop robotd every 10 s whenever the
rover is unplugged.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
import signal
import socket
import time
from pathlib import Path
from typing import Any, Final, Literal, cast

from rover_contracts.config import RobotConfig, load_config
from rover_contracts.ids import is_ulid, new_session_id, new_turn_id
from rover_contracts.messages import (
    ClearableFault,
    ClearMessage,
    ErrorMessage,
    EventKind,
    EventMessage,
    HelloMessage,
    PingMessage,
    ResultDetail,
    ResultMessage,
    ResultReason,
    ResultStatus,
    SkillMessage,
    Source,
    StateActive,
    StateBattery,
    StateBudget,
    StateMessage,
    StateRover,
    StateTwist,
    SubscribeMessage,
    SubscribeTopic,
    TurnMessage,
    TwistMessage,
    WelcomeMessage,
)
from rover_contracts.wave_proto import Banner, Feedback, StopFlag

from rover_robotd import __version__
from rover_robotd.arbiter import ActiveCommand, Arbiter, Arbitration, CommandKind
from rover_robotd.budget import BudgetLedger
from rover_robotd.bus import Bus, BusConnection
from rover_robotd.clients import ReplayWindow, TurnTracker, resolve_uid
from rover_robotd.episodes import EpisodeRecorder
from rover_robotd.heading import HeadingTracker
from rover_robotd.link import Link
from rover_robotd.log import RobotdLog
from rover_robotd.profiles import DriveForProfile, TurnToProfile, mix_twist
from rover_robotd.validator import (
    AcceptedSkill,
    AcceptedTwist,
    Rejection,
    ValidationContext,
    Validator,
)

__all__ = ["Robotd", "main", "sd_notify", "serve", "watchdog_period_s"]

log = logging.getLogger("rover.robotd")

_BATTERY_CELLS: Final = 3
"""3S pack; ``[battery]`` holds the per-cell OCV table and ``bus_v`` is the
pack voltage (docs/protocol.md)."""

SkillLiteral = Literal[
    "drive_for", "turn_to", "say", "describe_scene", "find", "set_face"
]
"""``state.active.skill``'s closed set, as ``messages.StateActive`` declares it."""

_STOP_TYPES: Final = frozenset({"stop", "estop", "cancel"})

_FLAG_EVENTS: Final[tuple[tuple[StopFlag, EventKind, str], ...]] = (
    (StopFlag.TOF, EventKind.TOF_BLOCK, "tof"),
    (StopFlag.BUMPER, EventKind.BUMPER, "bumper"),
    (StopFlag.LOWBAT, EventKind.LOW_BATTERY, "low_battery"),
)
"""The stop flags that are published as events when they appear, and named in
``fault_cleared`` when they clear.  ``HEARTBEAT`` is reported from the ``hb``
field and ``COAST`` is robotd's own doing."""


# ---------------------------------------------------------------------------
# systemd Type=notify, with no dependency and no effect when unconfigured
# ---------------------------------------------------------------------------


def sd_notify(state: bytes) -> bool:
    """Send one datagram to ``$NOTIFY_SOCKET``; a no-op when it is unset.

    ``Type=notify`` needs this or systemd kills robotd at ``TimeoutStartSec``
    on the first Pi boot and then restart-loops it every 10 s.  On the Mac
    ``NOTIFY_SOCKET`` is never set and every call returns ``False``.
    """
    address = os.environ.get("NOTIFY_SOCKET")
    if not address:
        return False
    if address[0] == "@":  # the abstract-namespace form
        address = "\0" + address[1:]
    # SOCK_CLOEXEC is Linux-only, and systemd is the only caller that gets here.
    kind = socket.SOCK_DGRAM | getattr(socket, "SOCK_CLOEXEC", 0)
    try:
        with socket.socket(socket.AF_UNIX, kind) as sock:
            sock.connect(address)
            sock.sendall(state)
    except OSError as exc:  # pragma: no cover - systemd only
        log.warning("sd_notify failed: %s", exc)
        return False
    return True


def watchdog_period_s(default: float = 5.0) -> float:
    """``WatchdogSec/2``, from ``$WATCHDOG_USEC``.  ``WatchdogSec=10`` -> 5 s."""
    raw = os.environ.get("WATCHDOG_USEC")
    if not raw or not raw.isdigit():
        return default
    return max(0.5, int(raw) / 2e6)


class Robotd:
    """The whole daemon: bus, link, arbitration, profiles, heading and logs."""

    def __init__(
        self, config: RobotConfig, *, clock: Any = time.monotonic_ns
    ) -> None:
        self.config = config
        self.clock = clock
        self.session_id = new_session_id()

        self.link = Link(
            config,
            on_feedback=self._on_feedback,
            on_up=self._on_link_up,
            on_lost=self._on_link_lost,
            on_restart=self._on_rover_restart,
            clock=clock,
        )
        self.arbiter = Arbiter()
        self.validator = Validator(config, ReplayWindow())
        self.budget = BudgetLedger(config.limits.budget_motion_s)
        self.heading = HeadingTracker(config.link.yaw_sign)
        self.turns = TurnTracker()
        self.episodes = EpisodeRecorder(Path(config.log.dir))
        self.logs = RobotdLog(
            config.log.dir, state_decimate_hz=config.log.state_decimate_hz
        )
        self.bus = Bus(
            config.bus.sock,
            on_message=self._on_message,
            on_disconnect=self._on_disconnect,
        )

        self.estop_path = Path(config.bus.sock).parent / "estop"
        self.estop_sw = self.estop_path.exists()
        self.ready = False
        self.reason = "link_down"
        self.tasks: list[asyncio.Task[None]] = []

        self._uids: dict[str, int | None] = {
            str(source): resolve_uid(user)
            for source, user in config.bus.source_uids.items()
        }
        self._last_hb: bool | None = None
        self._last_flags = StopFlag(0)
        self._last_clamp_count: int | None = None
        self._feedback_was_fresh = False
        self._last_state_ns = 0
        self._last_signature: tuple[Any, ...] | None = None
        self._stopping = False

    # -- lifecycle ----------------------------------------------------------

    async def run(self) -> None:
        """Boot to IDLE and serve until cancelled."""
        if self.estop_sw:
            log.warning(
                "estop_sw was latched at %s; motion stays refused", self.estop_path
            )
        await self.bus.start()
        self.tasks = [
            asyncio.create_task(self.link.run(), name="link"),
            asyncio.create_task(self._control_loop(), name="control"),
        ]
        sd_notify(b"READY=1")
        try:
            await asyncio.gather(*self.tasks)
        except asyncio.CancelledError:
            raise
        finally:
            self._stopping = True
            # Zeros go out while the port is still open; cancelling the link
            # task first would tear the transport down before they are sent.
            await self.link.close()
            for task in self.tasks:
                task.cancel()
            await asyncio.gather(*self.tasks, return_exceptions=True)
            await self.aclose()

    async def aclose(self) -> None:
        self.episodes.stop()
        await self.link.close()
        await self.bus.close()
        self.logs.close()

    # -- bus ----------------------------------------------------------------

    async def _on_disconnect(self, connection: BusConnection) -> None:
        """A dead client loses whatever it owned, and the turn moves on."""
        active = self.arbiter.active
        if active is not None and active.session_id == connection.session.session_id:
            self._abort(
                ResultStatus.ABORTED, ResultReason.NOT_READY, "client disconnected"
            )
            self.turns.adopt(new_turn_id())

    async def _on_message(
        self, connection: BusConnection, payload: dict[str, Any]
    ) -> None:
        message_type = payload.get("type")
        try:
            if message_type in _STOP_TYPES:
                self._handle_stop_class(connection, message_type, payload)
                return
            self._dispatch(connection, payload)
        except Exception:
            # I-8's fuzz corpus reaches this method.  One hostile message must
            # cost that message, never the connection and never the daemon.
            log.exception("dispatching %r failed", message_type)
            connection.send(
                ErrorMessage(code="bad_message", detail=str(message_type)[:64])
            )

    def _dispatch(self, connection: BusConnection, payload: dict[str, Any]) -> None:
        parsed = self.validator.parse(payload)
        if isinstance(parsed, Rejection):
            self._answer_rejection(connection, payload, parsed)
            return
        if isinstance(parsed, HelloMessage):
            self._handle_hello(connection, parsed)
        elif isinstance(parsed, SubscribeMessage):
            connection.session.subscribe(parsed)
        elif isinstance(parsed, PingMessage):
            connection.session.last_ping_mono_ns = self.clock()
        elif isinstance(parsed, TurnMessage):
            self._handle_turn(connection, parsed)
        elif isinstance(parsed, ClearMessage):
            self._handle_clear(connection, parsed)
        elif isinstance(parsed, SkillMessage):
            self._handle_skill(connection, parsed)
        elif isinstance(parsed, TwistMessage):
            self._handle_twist(connection, parsed)

    def _handle_hello(self, connection: BusConnection, message: HelloMessage) -> None:
        try:
            connection.session.bind(
                message, expected_uid=self._uids.get(str(message.source))
            )
        except ValueError as exc:
            connection.send(ErrorMessage(code="hello_refused", detail=str(exc)[:256]))
            return
        connection.send(self._welcome())

    def _handle_turn(self, connection: BusConnection, message: TurnMessage) -> None:
        """The bare turn boundary: without it a superseded plan could still
        drive, because robotd would only learn a ``turn_id`` from a skill that
        had already arrived (I-11)."""
        if not self._source_allowed(connection, message.source):
            connection.send(
                ErrorMessage(code="source_not_allowed", detail=str(message.source))
            )
            return
        if not self.turns.adopt(message.turn_id):
            return
        active = self.arbiter.active
        if active is not None and active.turn_id != message.turn_id:
            self._abort(ResultStatus.PREEMPTED, ResultReason.STALE_TURN, "new turn")

    def _handle_clear(self, connection: BusConnection, message: ClearMessage) -> None:
        """``clear`` is not stop-class and does not re-enable motion by itself.

        Only ``estop_sw`` is robotd's to clear.  The obstacle and low-battery
        flags are the controller's: it owns the samples and the rule, clears
        them itself when the condition ends, and the wire has no command for
        them.  Naming them here is acknowledged and changes nothing.
        """
        rejection = self.validator.check_clear(message, connection.session)
        if rejection is not None:
            connection.send(
                ErrorMessage(
                    code=str(rejection.reason) or "rejected", detail=rejection.detail
                )
            )
            return
        if ClearableFault.ESTOP_SW in message.faults:
            self.estop_sw = False
            self.estop_path.unlink(missing_ok=True)
        self._publish_event(
            EventKind.FAULT_CLEARED,
            detail={
                "faults": ",".join(str(f) for f in message.faults),
                "estop_sw": self.estop_sw,
            },
        )

    def _handle_stop_class(
        self, connection: BusConnection, message_type: str, payload: dict[str, Any]
    ) -> None:
        """I-22: never validated away, never answered ``rejected``.

        The source is taken from the connection binding when there is one and
        from the raw field otherwise, because a stop must survive a message
        that would not have parsed strictly.
        """
        source = connection.session.source
        if source is None:
            raw = payload.get("source")
            source = Source(raw) if raw in set(Source) else None
        if source is None or str(source) not in self.config.bus.allow_sources:
            log.warning("stop-class %r from a source that may not command: %r",
                        message_type, payload.get("source"))
            return

        if message_type == "estop":
            self.estop_sw = True
            self._persist_estop()
            self._abort(ResultStatus.ABORTED, ResultReason.ESTOP_ACTIVE, "estop")
            self.turns.adopt(new_turn_id())
        elif message_type == "stop":
            self._abort(ResultStatus.ABORTED, ResultReason.NONE, "stop")
            self.turns.adopt(new_turn_id())
        else:
            # A cancel names one command.  Cancelling a command that is not the
            # active one must not stop the one that is.
            active = self.arbiter.active
            wanted = payload.get("cmd_id")
            if active is not None and (wanted is None or wanted == active.cmd_id):
                self._abort(ResultStatus.ABORTED, ResultReason.NONE, "cancel")
        # The stream itself is the stop: with nothing active, the next control
        # period sends zeros, and the firmware zeroes on its own if it never
        # arrives.
        self.logs.command({"decision": message_type, "source": str(source)})

    # -- skills and twists --------------------------------------------------

    def _handle_skill(self, connection: BusConnection, message: SkillMessage) -> None:
        verdict = self.validator.check_skill(message, connection.session, self._context())
        if isinstance(verdict, Rejection):
            self._publish_result(
                message.cmd_id, ResultStatus.REJECTED, verdict.reason,
                seq=message.seq, detail_text=verdict.detail, connection=connection,
            )
            return
        if not verdict.moves:
            # say / describe_scene / find / set_face execute in brain; robotd's
            # part is the bound check, and A31 forbids reporting a completion
            # the executor has not made.
            self._publish_result(
                message.cmd_id, ResultStatus.ACCEPTED, ResultReason.NONE,
                seq=message.seq, connection=connection,
            )
            return
        self._start_motion(connection, verdict)

    def _start_motion(self, connection: BusConnection, verdict: AcceptedSkill) -> None:
        message = verdict.message
        now = self.clock()
        limits = self.config.limits
        args: Any = verdict.args
        profile: DriveForProfile | TurnToProfile
        if message.skill == "drive_for":
            profile = DriveForProfile(args.power, args.duration_s)
        else:
            profile = TurnToProfile(
                args.heading_deg,
                # The configured tolerance is the floor: the IMU holds what it
                # holds, and a tighter request is honoured as well as it can be.
                tolerance_deg=max(args.tolerance_deg, limits.turn_tolerance_deg),
                kp=limits.turn_kp,
                power_min=limits.power_min,
                power_max=limits.power_max,
            )
            if self.heading.sample_ns is not None:
                profile.observe(self.heading.heading_deg, self.heading.sample_ns)
        command = ActiveCommand(
            cmd_id=message.cmd_id,
            kind=CommandKind.SKILL,
            source=message.source,
            session_id=connection.session.session_id,
            started_mono_ns=now,
            deadline_mono_ns=now + verdict.deadline_ms * 1_000_000,
            skill=message.skill,
            turn_id=message.turn_id,
            seq=message.seq,
            profile=profile,
            power_clamped_to=verdict.power_clamped_to,
        )
        connection.session.last_ping_mono_ns = now
        self._install(command, verdict.arbitration)
        clamped = verdict.power_clamped_to is not None
        self._publish_result(
            message.cmd_id,
            ResultStatus.ACCEPTED,
            ResultReason.POWER_CLAMPED if clamped else ResultReason.NONE,
            seq=message.seq,
            detail=ResultDetail(power_clamped_to=verdict.power_clamped_to)
            if clamped
            else None,
            connection=connection,
        )

    def _handle_twist(self, connection: BusConnection, message: TwistMessage) -> None:
        verdict = self.validator.check_twist(message, connection.session, self._context())
        if isinstance(verdict, Rejection):
            self._publish_result(
                message.cmd_id, ResultStatus.REJECTED, verdict.reason,
                seq=message.seq, detail_text=verdict.detail, connection=connection,
            )
            return
        self._renew_twist(connection, verdict)

    def _renew_twist(self, connection: BusConnection, verdict: AcceptedTwist) -> None:
        message = verdict.message
        now = self.clock()
        active = self.arbiter.active
        if verdict.arbitration is Arbitration.RENEW and active is not None:
            active.twist_lin = message.twist.lin
            active.twist_ang = message.twist.ang
            active.renewed_mono_ns = now
            active.cmd_id = message.cmd_id
            connection.session.last_ping_mono_ns = now
            return
        command = ActiveCommand(
            cmd_id=message.cmd_id,
            kind=CommandKind.TWIST,
            source=message.source,
            session_id=connection.session.session_id,
            started_mono_ns=now,
            deadline_mono_ns=now + self.config.limits.twist_renew_ms * 1_000_000,
            twist_lin=message.twist.lin,
            twist_ang=message.twist.ang,
            renewed_mono_ns=now,
        )
        connection.session.last_ping_mono_ns = now
        self._install(command, verdict.arbitration)
        self.episodes.start()
        self._publish_result(
            message.cmd_id, ResultStatus.ACCEPTED, ResultReason.NONE,
            seq=message.seq, connection=connection,
        )

    def _install(self, command: ActiveCommand, arbitration: Arbitration) -> None:
        loser = self.arbiter.start(command)
        if loser is not None and arbitration is Arbitration.PREEMPT:
            if loser.kind is CommandKind.TWIST:
                self.episodes.stop()
            self._publish_result(
                loser.cmd_id, ResultStatus.PREEMPTED, ResultReason.NONE, seq=loser.seq,
                detail=self._detail_for(loser), connection=self._owner(loser),
            )

    # -- the control loop ---------------------------------------------------

    async def _control_loop(self) -> None:
        """The goal owner and the only sender of speed lines, at command_hz."""
        period = 1.0 / max(1, self.config.link.command_hz)
        watchdog_s = watchdog_period_s()
        loop = asyncio.get_running_loop()
        next_at = loop.time()
        last_ping = 0.0
        while not self._stopping:
            next_at += period
            delay = next_at - loop.time()
            if delay < -period:
                next_at = loop.time()
                delay = 0.0
            await asyncio.sleep(max(0.0, delay))
            now_wall = loop.time()
            if now_wall - last_ping >= watchdog_s:
                last_ping = now_wall
                sd_notify(b"WATCHDOG=1")
            now = self.clock()
            try:
                self._cycle(now, period)
            except Exception:  # pragma: no cover - a bug here must not wedge
                log.exception("control cycle failed")

    def _cycle(self, now: int, period_s: float) -> None:
        """One control period: judge readiness, step the goal, send, charge."""
        self._update_readiness(now)
        left, right = self._step(now)
        left, right = self.link.command(left, right, now)
        active = self.arbiter.active
        if (
            (left or right)
            and active is not None
            and active.kind is CommandKind.SKILL
            and active.turn_id is not None
        ):
            self.budget.charge(active.turn_id, period_s)
        self._publish_state(now)

    def _update_readiness(self, now: int) -> None:
        fresh = self.link.feedback_fresh(now)
        if self._feedback_was_fresh and not fresh:
            # T0 tripped: the controller stopped talking while the link is up.
            self._publish_event(
                EventKind.FEEDBACK_STALE,
                detail={"age_ms": self.link.feedback_age_ms(now)},
            )
        self._feedback_was_fresh = fresh
        ready, reason = self._readiness(now)
        if reason != self.reason and reason == "unpatched_firmware":
            self._publish_event(
                EventKind.UNPATCHED_FIRMWARE,
                detail={"fw": self.link.fw, "why": self.link.firmware_refusal()},
            )
        self.ready, self.reason = ready, reason

    def _step(self, now: int) -> tuple[float, float]:
        """What to stream this period: the active command's values, or zeros."""
        active = self.arbiter.active
        if active is None:
            return (0.0, 0.0)
        if self._owner_is_gone(active, now):
            self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "client ping gap")
            return (0.0, 0.0)
        if not self.link.up:
            self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "link_down")
            return (0.0, 0.0)
        if not self.link.feedback_fresh(now):
            # T0: a turn has lost its heading, anything else its controller.
            reason = (
                ResultReason.HEADING_UNAVAILABLE
                if active.skill == "turn_to"
                else ResultReason.FEEDBACK_STALE
            )
            self._abort(ResultStatus.ABORTED, reason, "feedback stale")
            return (0.0, 0.0)
        if not self.link.firmware_ok:
            self._abort(
                ResultStatus.ABORTED, ResultReason.UNPATCHED_FIRMWARE, "firmware"
            )
            return (0.0, 0.0)
        if active.kind is CommandKind.TWIST:
            return self._step_twist(active, now)
        return self._step_skill(active, now)

    def _step_twist(self, active: ActiveCommand, now: int) -> tuple[float, float]:
        renew_ns = self.config.limits.twist_renew_ms * 1_000_000
        if now - active.renewed_mono_ns > renew_ns:
            # A stream is one arbitration unit and ends after 200 ms of silence.
            self.episodes.stop()
            self._finish(active, ResultStatus.DONE, ResultReason.NONE)
            return (0.0, 0.0)
        left, right = mix_twist(
            active.twist_lin, active.twist_ang, self.config.limits.twist_power
        )
        self.episodes.record(
            t_mono_ns=now,
            left=left,
            right=right,
            heading_deg=self.heading.heading_deg,
            yaw_rate_dps=self.heading.yaw_rate_dps,
            feedback=self.link.feedback,
            source=str(active.source),
        )
        return (left, right)

    def _step_skill(self, active: ActiveCommand, now: int) -> tuple[float, float]:
        profile = active.profile
        if isinstance(profile, DriveForProfile):
            elapsed_s = (now - active.started_mono_ns) / 1e9
            if profile.done(elapsed_s):
                self._finish(active, ResultStatus.DONE, ResultReason.NONE)
                return (0.0, 0.0)
            left, right = profile.command(elapsed_s)
        elif isinstance(profile, TurnToProfile):
            if self.heading.sample_ns is not None:
                profile.observe(self.heading.heading_deg, self.heading.sample_ns)
            if profile.done:
                self._finish(active, ResultStatus.DONE, ResultReason.NONE)
                return (0.0, 0.0)
            left, right = profile.command()
        else:  # pragma: no cover - a moving skill always has a profile
            self._finish(active, ResultStatus.DONE, ResultReason.NONE)
            return (0.0, 0.0)
        if now >= active.deadline_mono_ns:
            self._finish(active, ResultStatus.TIMEOUT, ResultReason.TTL_EXPIRED)
            return (0.0, 0.0)
        if active.turn_id is not None and self.budget.exhausted(active.turn_id):
            self._finish(active, ResultStatus.ABORTED, ResultReason.BUDGET_EXCEEDED)
            return (0.0, 0.0)
        return (left, right)

    def _owner(self, active: ActiveCommand) -> BusConnection | None:
        """The connection that issued ``active``, while it is still open."""
        for connection in self.bus.connections:
            if connection.session.session_id == active.session_id:
                return connection
        return None

    def _owner_is_gone(self, active: ActiveCommand, now: int) -> bool:
        """Socket EOF **or** a ``client_ping_gap_ms`` gap: the gap is what
        covers ``kill -STOP`` on brain, where no EOF ever arrives."""
        connection = self._owner(active)
        if connection is None:
            return True
        gap_ns = self.config.bus.client_ping_gap_ms * 1_000_000
        return now - connection.session.last_ping_mono_ns > gap_ns

    def _finish(
        self, active: ActiveCommand, status: ResultStatus, reason: ResultReason
    ) -> None:
        self.arbiter.finish(active.cmd_id)
        self._publish_result(
            active.cmd_id, status, reason, seq=active.seq,
            detail=self._detail_for(active), connection=self._owner(active),
        )

    def _abort(
        self, status: ResultStatus, reason: ResultReason, detail_text: str = ""
    ) -> None:
        active = self.arbiter.active
        if active is None:
            return
        if active.kind is CommandKind.TWIST:
            self.episodes.stop()
        self.arbiter.finish(active.cmd_id)
        self._publish_result(
            active.cmd_id, status, reason, seq=active.seq,
            detail=self._detail_for(active), detail_text=detail_text,
            connection=self._owner(active),
        )

    # -- the rover side -----------------------------------------------------

    def _on_feedback(self, feedback: Feedback, arrival_ns: int) -> None:
        self.heading.update(feedback.yaw_deg, arrival_ns)
        self.logs.feedback(feedback, arrival_ns)

        if feedback.hb is not None:
            if self._last_hb is not None and feedback.hb != self._last_hb:
                self._publish_event(
                    EventKind.HEARTBEAT_RECOVERED
                    if feedback.hb
                    else EventKind.HEARTBEAT_TIMEOUT
                )
            self._last_hb = feedback.hb

        flags = feedback.st if feedback.st is not None else StopFlag(0)
        added = flags & ~self._last_flags
        removed = self._last_flags & ~flags
        self._last_flags = flags
        for flag, kind, name in _FLAG_EVENTS:
            if added & flag:
                self._publish_event(kind, fault=name, detail=self._flag_detail(feedback))
            if removed & flag:
                self._publish_event(EventKind.FAULT_CLEARED, fault=name)

        if feedback.clamp_count is not None:
            if (
                self._last_clamp_count is not None
                and feedback.clamp_count != self._last_clamp_count
            ):
                # robotd never sends above the cap, so a clamp the controller
                # counted is a value it did not get from this process.
                self._publish_event(
                    EventKind.CAP_CLAMP, detail={"clamp_count": feedback.clamp_count}
                )
            self._last_clamp_count = feedback.clamp_count

        active = self.arbiter.active
        if active is not None and active.moves:
            if flags.blocks_all:
                self._abort(ResultStatus.ABORTED, ResultReason.FAULTED, "low battery")
            elif flags.blocks_forward and active.forward_power() > 0.0:
                # The controller has already zeroed forward motion; the goal
                # would run out its time going nowhere.
                self._abort(
                    ResultStatus.ABORTED, ResultReason.OBSTACLE, "forward blocked"
                )

    def _on_link_up(self, link: Link) -> None:
        banner = link.banner
        self._publish_event(
            EventKind.LINK_UP,
            detail={
                "fw": banner.fw if banner else None,
                "hb_ms": banner.hb_ms if banner else None,
                "cap": banner.cap if banner else None,
                "proto": banner.proto if banner else None,
            },
        )

    def _on_link_lost(self, link: Link) -> None:
        self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "link_lost")
        self._reset_controller_state()
        self._publish_event(EventKind.LINK_LOST)

    def _on_rover_restart(self, banner: Banner) -> None:
        """The controller rebooted mid-session: whatever was running was
        computed against a controller that no longer exists (I-13)."""
        self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "rover_restart")
        self._reset_controller_state()
        self._publish_event(EventKind.ROVER_RESTART, detail={"fw": banner.fw})

    def _reset_controller_state(self) -> None:
        self.heading.reset()
        self._last_hb = None
        self._last_flags = StopFlag(0)
        self._last_clamp_count = None
        self._feedback_was_fresh = False

    # -- publishing ---------------------------------------------------------

    def _welcome(self) -> WelcomeMessage:
        return WelcomeMessage(
            session=self.session_id,
            robotd_version=__version__,
            rover_fw=self.link.fw,
            limits=self.config.limits,
            safety=self.config.safety,
        )

    def _publish_result(
        self,
        cmd_id: str,
        status: ResultStatus,
        reason: ResultReason,
        *,
        seq: int | None = None,
        detail: ResultDetail | None = None,
        detail_text: str = "",
        connection: BusConnection | None = None,
    ) -> None:
        """The answer to one command.

        It reaches every ``result`` subscriber **and** the connection that
        issued the command, subscribed or not: a sender must never be left not
        knowing what happened to a command it sent.
        """
        message = ResultMessage(
            cmd_id=cmd_id, seq=seq, status=status, reason=reason,
            detail=detail, t_utc_ns=time.time_ns(),
        )
        for client in self.bus.connections:
            if client is connection or client.wants(SubscribeTopic.RESULT):
                client.send(message)
        self.logs.command(
            {"decision": str(status), "cmd_id": cmd_id, "reason": str(reason),
             "detail": detail_text}
        )

    def _publish_event(
        self,
        kind: EventKind,
        *,
        fault: str | None = None,
        detail: dict[str, Any] | None = None,
    ) -> None:
        message = EventMessage(
            kind=kind, fault=fault, detail=detail, t_utc_ns=time.time_ns()
        )
        self.bus.broadcast(message, SubscribeTopic.EVENT)
        self.logs.event(message)

    def _publish_state(self, now: int) -> None:
        state = self._state(now)
        signature = (
            state.rover.hb_ok,
            state.rover.stop_flags,
            state.rover.fw,
            state.active.cmd_id if state.active else None,
            state.ready,
        )
        period_ns = int(1e9 / max(1, self.config.bus.state_hz))
        if signature == self._last_signature and now - self._last_state_ns < period_ns:
            return
        self._last_signature = signature
        self._last_state_ns = now
        self.bus.broadcast_state(state)

    def _state(self, now: int) -> StateMessage:
        feedback = self.link.feedback
        age = self.link.feedback_age_ms(now)
        active = self.arbiter.active
        flags = feedback.st if feedback is not None and feedback.st is not None else 0
        left, right = self.link.cmd_left, self.link.cmd_right
        return StateMessage(
            t_utc_ns=time.time_ns(),
            t_mono_ns=now,
            rover=StateRover(
                fw=self.link.fw,
                hb_ok=bool(feedback.hb) if feedback is not None else False,
                stop_flags=int(flags) & 0x1F,
                feedback_age_ms=int(age) if age is not None else 0,
                cmd_left=left,
                cmd_right=right,
                heading_deg=self.heading.heading_deg,
                yaw_rate_dps=self.heading.yaw_rate_dps,
                roll_deg=feedback.roll_deg if feedback is not None else 0.0,
                pitch_deg=feedback.pitch_deg if feedback is not None else 0.0,
                temp_c=feedback.temp_c if feedback is not None else 0.0,
                clamp_count=_u16(feedback.clamp_count) if feedback is not None else 0,
                motion=bool(feedback is not None and (feedback.left or feedback.right)),
            ),
            twist=StateTwist(lin=(left + right) / 2.0, ang=(right - left) / 2.0),
            front_m=(
                feedback.tof_mm / 1000.0
                if feedback is not None and feedback.tof_valid
                else None
            ),
            bumper=bool(feedback.bumper) if feedback is not None else False,
            estop_sw=self.estop_sw,
            battery=self._battery(feedback),
            active=StateActive(
                cmd_id=active.cmd_id,
                skill=cast(SkillLiteral, active.skill),
                source=active.source,
                progress=active.fraction(now),
                deadline_in_ms=max(0, (active.deadline_mono_ns - now) // 1_000_000),
            )
            if active is not None and active.kind is CommandKind.SKILL and active.skill
            else None,
            budget=StateBudget(motion_s=self.budget.remaining(self.turns.current or "")),
            ready=self.ready,
            reason=self.reason,
        )

    def _readiness(self, now: int) -> tuple[bool, str]:
        link = self.link
        if not link.connected or not link.up:
            return False, "link_down"
        if not link.feedback_fresh(now):
            return False, "feedback_stale"
        if not link.firmware_ok:
            return False, "unpatched_firmware"
        if self.estop_sw:
            return False, "estop_active"
        feedback = link.feedback
        if feedback is not None and feedback.st is not None and feedback.st.blocks_all:
            return False, "faulted"
        return True, ""

    def _battery(self, feedback: Feedback | None) -> StateBattery:
        if feedback is None:
            return StateBattery(pack_v=0.0, pct=0)
        pack_v = max(0.0, feedback.bus_v)
        return StateBattery(pack_v=pack_v, pct=self._pct(pack_v))

    def _pct(self, pack_v: float) -> int:
        ocv = self.config.battery.ocv_per_cell
        soc = self.config.battery.soc_pct
        per_cell = pack_v / _BATTERY_CELLS
        if per_cell >= ocv[0]:
            return soc[0]
        for high, low, soc_high, soc_low in zip(ocv, ocv[1:], soc, soc[1:], strict=False):
            if low <= per_cell <= high:
                fraction = (per_cell - low) / (high - low)
                return round(soc_low + fraction * (soc_high - soc_low))
        return soc[-1]

    def _detail_for(self, active: ActiveCommand) -> ResultDetail:
        duration_ms = max(0, (self.clock() - active.started_mono_ns) // 1_000_000)
        profile = active.profile
        if isinstance(profile, TurnToProfile):
            return ResultDetail(
                duration_ms=duration_ms,
                turned_deg=profile.turned_deg,
                heading_error_deg=profile.error_deg,
            )
        return ResultDetail(
            duration_ms=duration_ms, power_clamped_to=active.power_clamped_to
        )

    def _flag_detail(self, feedback: Feedback) -> dict[str, Any]:
        return {
            "front_m": feedback.tof_mm / 1000.0 if feedback.tof_valid else None,
            "bumper": feedback.bumper,
            "bus_v": feedback.bus_v,
        }

    # -- helpers ------------------------------------------------------------

    def _context(self) -> ValidationContext:
        now = self.clock()
        feedback = self.link.feedback
        flags = feedback.st if feedback is not None and feedback.st is not None else 0
        turn_id = self.turns.current or ""
        return ValidationContext(
            now_mono_ns=now,
            feedback_age_ms=self.link.feedback_age_ms(now),
            firmware_ok=self.link.up and self.link.firmware_ok,
            stop_flags=int(flags),
            estop_sw=self.estop_sw,
            current_turn_id=self.turns.current,
            active=self.arbiter.active,
            last_motion_start_mono_ns=self.arbiter.last_motion_start_mono_ns,
            last_motion_turn_id=self.arbiter.last_motion_turn_id,
            remaining_motion_s=self.budget.remaining(turn_id),
        )

    def _source_allowed(self, connection: BusConnection, source: Source) -> bool:
        session = connection.session
        return (
            session.bound
            and source is session.source
            and str(source) in self.config.bus.allow_sources
        )

    def _persist_estop(self) -> None:
        """``estop_sw`` survives a crash-restart, so one does not silently
        clear a stop authority."""
        try:
            self.estop_path.parent.mkdir(parents=True, exist_ok=True)
            self.estop_path.write_text(
                json.dumps({"estop_sw": True, "t_utc_ns": time.time_ns()}) + "\n"
            )
        except OSError as exc:  # pragma: no cover
            log.error("cannot persist estop_sw to %s: %s", self.estop_path, exc)

    def _answer_rejection(
        self, connection: BusConnection, payload: dict[str, Any], rejection: Rejection
    ) -> None:
        cmd_id = payload.get("cmd_id")
        if is_ulid(cmd_id):
            assert isinstance(cmd_id, str)
            self._publish_result(
                cmd_id, ResultStatus.REJECTED, rejection.reason,
                detail_text=rejection.detail, connection=connection,
            )
            return
        connection.send(
            ErrorMessage(code="bad_message", detail=rejection.detail[:256] or "invalid")
        )


def _u16(value: int | None) -> int:
    """``clamp_count`` as the bus declares it, whatever a firmware sent: a
    value outside 0..65535 must not stop state publication."""
    if value is None:
        return 0
    return max(0, min(0xFFFF, value))


async def serve(config: RobotConfig) -> None:
    """Run robotd until SIGTERM or SIGINT, then unwind.

    systemd stops a unit with SIGTERM.  Without a handler the interpreter dies
    where it stands: ``Robotd.run``'s finally never runs, so the final zeros
    are never sent, ``RobotdLog`` is not closed and an open teleop episode is
    never fsynced.
    """
    loop = asyncio.get_running_loop()
    task = asyncio.create_task(Robotd(config).run(), name="robotd")
    for signum in (signal.SIGTERM, signal.SIGINT):
        # Windows and some embedded loops have no add_signal_handler; the
        # KeyboardInterrupt path below still covers SIGINT there.
        with contextlib.suppress(NotImplementedError):
            loop.add_signal_handler(signum, task.cancel)
    with contextlib.suppress(asyncio.CancelledError):
        await task


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rover-robotd", description=__doc__)
    parser.add_argument("--config", default="config/robot.toml")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    config = load_config(args.config)
    with contextlib.suppress(KeyboardInterrupt):  # pragma: no cover
        asyncio.run(serve(config))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
