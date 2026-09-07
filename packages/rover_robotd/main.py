"""``rover-robotd``: the only process that can move the robot.

This module is the wiring.  Every rule it enforces is stated in one of the
modules beside it -- :mod:`~rover_robotd.validator` owns the 4.2 table,
:mod:`~rover_robotd.arbiter` owns who has the wheels, :mod:`~rover_robotd.budget`
owns I-15, :mod:`~rover_robotd.profiles` owns the setpoint cell of I-14 and
:mod:`~rover_robotd.link` owns the port -- so what is left here is dispatch,
the control loop, the arm policy and the published state.

Three shapes are load-bearing and are easy to lose in a refactor:

* **Stop-class first.**  ``stop``, ``estop`` and ``cancel`` are dispatched on
  the raw ``"type"`` before any strict parsing, so a malformed stop is still a
  stop and is never answered ``rejected`` (I-22).
* **Two independent loops.**  The control loop recomputes the profile and
  *stamps* the setpoint cell; the link's writer loop *reads* it at 20 Hz.  If
  this loop freezes, the stamp expires and the writer streams zeros -- a frozen
  goal owner stops the wheels without anything detecting the freeze (I-14).
* **The watchdog ping is liveness, not readiness.**  It is sent unconditionally
  from the control loop while it is scheduling; gating it on fresh telemetry
  would restart-loop robotd every 10 s whenever the MCU is unplugged, which is
  its state until deploy step 10.
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
    OdomDelta,
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
    StateMcu,
    StateMessage,
    StatePose,
    StateRails,
    StateRanges,
    StateTof,
    StateTwist,
    StateWheels,
    SubscribeMessage,
    SubscribeTopic,
    TurnMessage,
    TwistMessage,
    WelcomeMessage,
)
from rover_contracts.serial_codec import (
    LATCHED_FAULTS,
    TOF_ERROR_MM,
    TOF_NO_TARGET_MM,
    AckFrame,
    CtrlFlag,
    EventFrame,
    Fault,
    Frame,
    McuState,
    TelemetryFrame,
    ack_type_code,
    fault_mask,
    fault_names,
)
from rover_contracts.units import (
    deg_to_rad,
    mm_s_to_mps,
    mm_to_m,
    mrad_s_to_radps,
    rad_to_deg,
)

from rover_robotd import __version__
from rover_robotd.arbiter import ActiveCommand, Arbiter, Arbitration, CommandKind
from rover_robotd.budget import BudgetLedger, BudgetLimits
from rover_robotd.bus import Bus, BusConnection
from rover_robotd.episodes import EpisodeRecorder
from rover_robotd.link import Link
from rover_robotd.log import RobotdLog
from rover_robotd.odom import Geometry, Odometry, wrap_angle
from rover_robotd.profiles import SetpointCell, TrapezoidProfile
from rover_robotd.session import ReplayWindow, TurnTracker, resolve_uid
from rover_robotd.validator import (
    AcceptedSkill,
    AcceptedTwist,
    Rejection,
    ValidationContext,
    Validator,
)

__all__ = ["Robotd", "main", "sd_notify", "serve", "watchdog_period_s"]

log = logging.getLogger("rover.robotd")

DRIVE_TOLERANCE_M: Final = 0.01
"""One centimetre: the granularity the model commands in (A11)."""

TURN_TOLERANCE_RAD: Final = deg_to_rad(1.0)

_BATTERY_CELLS: Final = 3
"""3S pack (ARCHITECTURE 3); ``[battery]`` holds the per-cell OCV table."""

SkillLiteral = Literal["drive", "turn", "say", "describe_scene", "find", "set_face"]
"""``state.active.skill``'s closed set, as ``messages.StateActive`` declares it."""

_STOP_TYPES: Final = frozenset({"stop", "estop", "cancel"})
_ARM_ACK_CODE: Final = ack_type_code("A")


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
    """The whole daemon: bus, link, arbitration, profiles, odometry and logs."""

    def __init__(
        self, config: RobotConfig, *, clock: Any = time.monotonic_ns
    ) -> None:
        self.config = config
        self.clock = clock
        self.session_id = new_session_id()

        self.cell = SetpointCell()
        self.link = Link(
            config, self.cell, on_frame=self._on_frame, on_resync=self._on_resync,
            clock=clock,
        )
        self.arbiter = Arbiter()
        self.validator = Validator(config, ReplayWindow())
        self.budget = BudgetLedger(
            BudgetLimits(config.limits.budget_path_m, config.limits.budget_motion_s)
        )
        self.odom = Odometry(
            Geometry(
                config.robot.wheel_radius_m,
                config.robot.track_m,
                config.robot.ticks_per_rev,
            )
        )
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

        self._uids: dict[str, int | None] = {
            str(source): resolve_uid(user)
            for source, user in config.bus.source_uids.items()
        }
        self._pending_ds = 0.0
        self._pending_dyaw = 0.0
        self._last_telemetry_ns: int | None = None
        self._last_fault = 0
        self._idle_since: int | None = None
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
        tasks = [
            asyncio.create_task(self.link.run(), name="link"),
            asyncio.create_task(self._control_loop(), name="control"),
        ]
        sd_notify(b"READY=1")
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            raise
        finally:
            self._stopping = True
            # Brake and disarm on the port while it is still open.  Cancelling
            # the link task first runs Link.run's per-iteration finally, which
            # is _teardown(): the fd goes to None, Link.close's `connected and
            # _ready` guard is then false, and the shutdown S+D 4.2 requires
            # are never sent -- so `systemctl restart rover-robotd` mid-drive
            # left the MCU armed with a live setpoint until the 300 ms frame
            # TTL and the 5 s TTL-disarm, instead of an ordered stop.
            await self.link.close()
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
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
            self.link.disarm()

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
        """``clear`` is not stop-class and does **not** re-arm."""
        rejection = self.validator.check_clear(message, connection.session)
        if rejection is not None:
            connection.send(
                ErrorMessage(
                    code=str(rejection.reason) or "rejected", detail=rejection.detail
                )
            )
            return
        names = [str(f) for f in message.faults]
        if ClearableFault.ESTOP_SW in message.faults:
            self.estop_sw = False
            self.estop_path.unlink(missing_ok=True)
            names.remove(str(ClearableFault.ESTOP_SW))
        # robotd never clears obstacle-class bits: the MCU owns the samples and
        # the rule.  Only latched bits are ever named in a C frame.
        mask = fault_mask(names) & int(LATCHED_FAULTS) if names else 0
        if mask:
            self.link.clear(mask)
        self._publish_event(
            EventKind.FAULT_CLEARED,
            detail={"mask": mask, "estop_sw": self.estop_sw},
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
            self.cell.zero()
            self.link.stop(mode=0)
            self.link.disarm()
            self.estop_sw = True
            self._persist_estop()
            self._abort(ResultStatus.ABORTED, ResultReason.ESTOP_ACTIVE, "estop")
            self.turns.adopt(new_turn_id())
            self._publish_event(
                EventKind.FAULT_SET, fault="estop_sw", detail={"source": str(source)}
            )
        elif message_type == "stop":
            self.cell.zero()
            self.link.stop(mode=0)
            self._abort(ResultStatus.ABORTED, ResultReason.NONE, "stop")
            self.turns.adopt(new_turn_id())
        else:
            # A cancel names one command.  Cancelling a command that is not the
            # active one must not stop the one that is.
            active = self.arbiter.active
            wanted = payload.get("cmd_id")
            if active is not None and (wanted is None or wanted == active.cmd_id):
                self._abort(ResultStatus.ABORTED, ResultReason.NONE, "cancel")
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
        if not self._arm():
            self._publish_result(
                message.cmd_id, ResultStatus.REJECTED, ResultReason.NOT_READY,
                seq=message.seq, detail_text="arm preconditions not met",
                connection=connection,
            )
            return
        self._start_motion(connection, verdict)

    def _start_motion(self, connection: BusConnection, verdict: AcceptedSkill) -> None:
        message = verdict.message
        now = self.clock()
        limits = self.config.limits
        args = verdict.args
        if message.skill == "drive":
            profile = TrapezoidProfile(
                target=args.distance_m,  # type: ignore[attr-defined]
                cruise=args.speed_mps,  # type: ignore[attr-defined]
                accel=limits.accel_mps2,
                tolerance=DRIVE_TOLERANCE_M,
            )
        else:
            profile = TrapezoidProfile(
                target=deg_to_rad(args.angle_deg),  # type: ignore[attr-defined]
                cruise=deg_to_rad(args.rate_dps),  # type: ignore[attr-defined]
                accel=limits.alpha_radps2,
                tolerance=TURN_TOLERANCE_RAD,
            )
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
            start_pose=self.odom.pose,
            speed_clamped_to_cms=verdict.speed_clamped_to_cms,
        )
        connection.session.last_ping_mono_ns = now
        self._install(command, verdict.arbitration)
        self._publish_result(
            message.cmd_id, ResultStatus.ACCEPTED, ResultReason.NONE, seq=message.seq,
            detail=ResultDetail(speed_clamped_to_cms=verdict.speed_clamped_to_cms)
            if verdict.speed_clamped_to_cms is not None
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
        if not self._arm():
            self._publish_result(
                message.cmd_id, ResultStatus.REJECTED, ResultReason.NOT_READY,
                seq=message.seq, detail_text="arm preconditions not met",
                connection=connection,
            )
            return
        self._renew_twist(connection, verdict)

    def _renew_twist(self, connection: BusConnection, verdict: AcceptedTwist) -> None:
        message = verdict.message
        now = self.clock()
        active = self.arbiter.active
        if verdict.arbitration is Arbitration.RENEW and active is not None:
            active.twist_v_mps = message.twist.linear_x_mps
            active.twist_w_radps = message.twist.angular_z_radps
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
            twist_v_mps=message.twist.linear_x_mps,
            twist_w_radps=message.twist.angular_z_radps,
            renewed_mono_ns=now,
            start_pose=self.odom.pose,
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
        self._idle_since = None

    # -- the control loop ---------------------------------------------------

    async def _control_loop(self) -> None:
        period = 1.0 / max(1, self.config.serial.setpoint_hz)
        watchdog_s = watchdog_period_s()
        loop = asyncio.get_running_loop()
        next_at = loop.time()
        last_ns = self.clock()
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
            dt_s = max(0.0, (now - last_ns) / 1e9)
            last_ns = now
            try:
                self._step(now, dt_s)
                self._publish_state(now)
            except Exception:  # pragma: no cover - a bug here must not wedge
                log.exception("control cycle failed")

    def _step(self, now: int, dt_s: float) -> None:
        active = self.arbiter.active
        # Consumed here, at the top, so every exit path leaves them empty.  An
        # abort that returned early left up to one telemetry period of odometry
        # charged to whatever skill the bus handler installed next -- at 0.3 m/s
        # and setpoint_hz 20 that is 15 mm, past DRIVE_TOLERANCE_M, which can
        # complete a short drive before it starts.
        pending_ds, pending_dyaw = self._pending_ds, self._pending_dyaw
        self._pending_ds = self._pending_dyaw = 0.0
        if active is None:
            self._maybe_disarm(now)
            return
        if self._owner_is_gone(active, now):
            self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "client ping gap")
            return
        if self.link.link_down:
            # T0: stop emitting V, mark the link down, fail the active goal.
            self._abort(ResultStatus.ABORTED, ResultReason.NOT_READY, "mcu_link_down")
            return
        if active.kind is CommandKind.TWIST:
            self._step_twist(active, now)
        else:
            self._step_skill(active, now, dt_s, pending_ds, pending_dyaw)

    def _step_twist(self, active: ActiveCommand, now: int) -> None:
        renew_ns = self.config.limits.twist_renew_ms * 1_000_000
        if now - active.renewed_mono_ns > renew_ns:
            # A stream is one arbitration unit and ends after 200 ms of silence.
            self.episodes.stop()
            self._finish(active, ResultStatus.DONE, ResultReason.NONE)
            return
        self.cell.stamp(active.twist_v_mps, active.twist_w_radps, now)
        telemetry = self.link.telemetry
        if telemetry is not None:
            self.episodes.record(
                t_mono_ns=now,
                linear_x_mps=active.twist_v_mps,
                angular_z_radps=active.twist_w_radps,
                pose=self.odom.pose,
                left_ticks=telemetry.left_ticks,
                right_ticks=telemetry.right_ticks,
                mcu_us=telemetry.mcu_us,
                source=str(active.source),
            )

    def _step_skill(
        self,
        active: ActiveCommand,
        now: int,
        dt_s: float,
        pending_ds: float,
        pending_dyaw: float,
    ) -> None:
        profile = active.profile
        assert profile is not None
        linear = active.skill == "drive"
        moved = pending_ds if linear else pending_dyaw
        active.progress += moved
        command = profile.step(active.progress, dt_s)
        if linear:
            self.cell.stamp(command, 0.0, now)
        else:
            self.cell.stamp(0.0, command, now)
        if active.turn_id is not None:
            self.budget.charge(
                active.turn_id,
                abs(pending_ds),
                dt_s if abs(command) > 0.0 else 0.0,
            )
        if profile.done(active.progress):
            self._finish(active, ResultStatus.DONE, ResultReason.NONE)
            return
        if now >= active.deadline_mono_ns:
            self._finish(active, ResultStatus.TIMEOUT, ResultReason.TTL_EXPIRED)
            return
        if active.monitor.update(now, moved, command, dt_s):
            self._finish(active, ResultStatus.TIMEOUT, ResultReason.TTL_EXPIRED)
            return
        remaining = self.budget.remaining(active.turn_id or "")
        if remaining.path_m <= 0.0 or remaining.motion_s <= 0.0:
            self._finish(active, ResultStatus.ABORTED, ResultReason.BUDGET_EXCEEDED)

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
        self.cell.zero()
        self.link.stop(mode=0)
        self.arbiter.finish(active.cmd_id)
        self._idle_since = self.clock()
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
        self.cell.zero()
        self.link.stop(mode=0)
        self.arbiter.finish(active.cmd_id)
        self._idle_since = self.clock()
        self._publish_result(
            active.cmd_id, status, reason, seq=active.seq,
            detail=self._detail_for(active), detail_text=detail_text,
            connection=self._owner(active),
        )

    # -- arm policy ---------------------------------------------------------

    def _arm(self) -> bool:
        """``A`` on the first motion command of a turn, when every precondition
        of 4.2's arm policy holds: live session, telemetry age under T0, no
        blocking-all fault, ``ctrl_flags`` b7 clear (I-18)."""
        if self.link.armed:
            return True
        telemetry = self.link.telemetry
        age = self.link.telemetry_age_ms()
        if (
            not self.link.ready
            or telemetry is None
            or age is None
            or age > self.config.serial.cmd_gate_max_age_ms
            or telemetry.fault & int(LATCHED_FAULTS)
            or telemetry.ctrl_flags & int(CtrlFlag.DEBUG_BUILD)
            # An uncalibrated controller accepts A and then silently zeroes
            # every forward v (4.1).  Refusing here turns that into an
            # attributable not_ready rather than a progress timeout.
            or not telemetry.ctrl_flags & int(CtrlFlag.CAL_VALID)
            or self.estop_sw
        ):
            return False
        return self.link.arm() is not None

    def _maybe_disarm(self, now: int) -> None:
        if not self.link.armed:
            self._idle_since = None
            return
        if self._idle_since is None:
            self._idle_since = now
            return
        idle_ns = self.config.limits.motion_idle_disarm_ms * 1_000_000
        if now - self._idle_since >= idle_ns:
            self.link.disarm()
            self._idle_since = None

    # -- the serial side ----------------------------------------------------

    def _on_frame(self, frame: Frame) -> None:
        if isinstance(frame, TelemetryFrame):
            self._on_telemetry(frame)
        elif isinstance(frame, AckFrame):
            if frame.ack_type == _ARM_ACK_CODE and frame.result != 0:
                self._abort(ResultStatus.ABORTED, ResultReason.MCU_NACK, "arm denied")
        elif isinstance(frame, EventFrame):
            kind = _EVENT_KINDS.get(frame.event)
            if kind is None:
                # The protocol stays at version 2 by design, so firmware may
                # gain event 16 while this table is old and nothing rejects the
                # frame.  Publishing it as `fault_set` with `fault: null` puts
                # a fault that did not happen on the bus and in events-*.jsonl,
                # where every consumer keying off fault_set misreads it.
                log.warning(
                    "unknown E event %d (arg=%d); not published", frame.event, frame.arg
                )
                return
            self._publish_event(
                kind, detail={"arg": frame.arg, "mcu_us": frame.mcu_us}
            )

    def _on_telemetry(self, frame: TelemetryFrame) -> None:
        arrival = self.link.telemetry_arrival_ns or self.clock()
        if self._last_telemetry_ns is not None:
            dt_s = max(1e-6, (arrival - self._last_telemetry_ns) / 1e9)
            step = self.odom.update(frame.left_ticks, frame.right_ticks, dt_s)
            self._pending_ds += step.ds_m
            self._pending_dyaw += step.dyaw_rad
        else:
            self.odom.rebase(frame.left_ticks, frame.right_ticks)
        self._last_telemetry_ns = arrival
        self.logs.telemetry(frame, arrival)

        if frame.fault != self._last_fault:
            added = frame.fault & ~self._last_fault
            removed = self._last_fault & ~frame.fault
            self._last_fault = frame.fault
            transitions = (
                (added, EventKind.FAULT_SET),
                (removed, EventKind.FAULT_CLEARED),
            )
            for mask, kind in transitions:
                for name in fault_names(mask):
                    self._publish_event(
                        kind, fault=name, detail=self._fault_detail(name, frame)
                    )
            if frame.fault & int(LATCHED_FAULTS):
                self.link.disarm()
                self._abort(ResultStatus.ABORTED, ResultReason.FAULTED, "latched fault")

    def _on_resync(
        self, old_session: int, new_session: int, reset_reason: int | None, reason: str
    ) -> None:
        """A port open, a ``T.SESS`` change or a ``B`` are one event: abort,
        re-seed, re-send ``H``, and require a fresh ``A`` (I-3, I-13)."""
        self._last_telemetry_ns = None
        self._last_fault = 0
        telemetry = self.link.telemetry
        if telemetry is not None:
            self.odom.rebase(telemetry.left_ticks, telemetry.right_ticks)
        self._abort(ResultStatus.ABORTED, ResultReason.MCU_NACK, reason)
        if old_session != new_session:
            self._publish_event(
                EventKind.MCU_RESTART,
                detail={
                    "old_session": old_session,
                    "new_session": new_session,
                    "reset_reason": reset_reason,
                    "reason": reason,
                },
            )

    # -- publishing ---------------------------------------------------------

    def _welcome(self) -> WelcomeMessage:
        return WelcomeMessage(
            session=self.session_id,
            robotd_version=__version__,
            mcu_session=self.link.session or 1,
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
            state.mcu.state, state.mcu.fault, state.armed,
            state.active.cmd_id if state.active else None,
        )
        period_ns = int(1e9 / max(1, self.config.bus.state_hz))
        if signature == self._last_signature and now - self._last_state_ns < period_ns:
            return
        self._last_signature = signature
        self._last_state_ns = now
        self.bus.broadcast_state(state)

    def _state(self, now: int) -> StateMessage:
        telemetry = self.link.telemetry
        age = self.link.telemetry_age_ms(now)
        active = self.arbiter.active
        front_mm = telemetry.tof_front_mm if telemetry is not None else None
        mcu_state = _mcu_state(telemetry)
        ready, reason = self._readiness(telemetry, age, mcu_state)
        return StateMessage(
            t_utc_ns=time.time_ns(),
            t_mono_ns=now,
            mcu=StateMcu(
                state=mcu_state,
                fault=telemetry.fault if telemetry is not None else 0,
                session=self.link.session,
                age_ms=int(age) if age is not None else 0,
                last_ack_seq=telemetry.ack_seq if telemetry is not None else 0,
                loop_late_pct=telemetry.loop_late_pct if telemetry is not None else 0,
                # T.rx_drop: the MCU's *down*-direction drop counter, the one
                # LINK_CRC latches on and the only symptom of a degrading
                # Pi->MCU link.  robotd's own up-direction count is a different
                # direction and a different subject, and publishing it under
                # `mcu` left every bus consumer blind to the wire fault (I-2).
                rx_drop=telemetry.rx_drop if telemetry is not None else 0,
                motion=bool(telemetry.motion) if telemetry is not None else False,
            ),
            armed=self.link.armed,
            pose=StatePose(
                x_m=self.odom.pose.x_m,
                y_m=self.odom.pose.y_m,
                yaw_rad=self.odom.pose.yaw_rad,
            ),
            twist=StateTwist(
                linear_x_mps=mm_s_to_mps(telemetry.v_meas_mm_s) if telemetry else 0.0,
                angular_z_radps=mrad_s_to_radps(telemetry.w_meas_mrad_s)
                if telemetry
                else 0.0,
            ),
            wheels=StateWheels(
                left_ticks=telemetry.left_ticks if telemetry else 0,
                right_ticks=telemetry.right_ticks if telemetry else 0,
                ticks_per_rev=self.config.robot.ticks_per_rev,
                wheel_radius_m=self.config.robot.wheel_radius_m,
                track_m=self.config.robot.track_m,
            ),
            ranges_m=StateRanges(
                front=_range_m(front_mm),
                cliff=_range_m(telemetry.tof_cliff_mm) if telemetry else None,
            ),
            front_at_max=front_mm == TOF_NO_TARGET_MM,
            tof=StateTof(
                front_l_ok=bool(telemetry.ctrl_flags & CtrlFlag.TOF_FL_OK)
                if telemetry
                else False,
                front_r_ok=bool(telemetry.ctrl_flags & CtrlFlag.TOF_FR_OK)
                if telemetry
                else False,
            ),
            bumper=(not telemetry.ctrl_flags & CtrlFlag.BUMPER_CLEAR)
            if telemetry
            else False,
            estop_hw=not (telemetry.ctrl_flags & CtrlFlag.ESTOP_RELEASED)
            if telemetry
            else False,
            estop_sw=self.estop_sw,
            battery=self._battery(telemetry),
            rails=StateRails(servo=bool(telemetry.rails & 0x1) if telemetry else False),
            active=StateActive(
                cmd_id=active.cmd_id,
                skill=cast(SkillLiteral, active.skill),
                source=active.source,
                progress=active.fraction(),
                deadline_in_ms=max(0, (active.deadline_mono_ns - now) // 1_000_000),
            )
            if active is not None and active.kind is CommandKind.SKILL and active.skill
            else None,
            budget=self._state_budget(),
            ready=ready,
            reason=reason,
        )

    def _state_budget(self) -> StateBudget:
        """I-15's remaining budget for the current instruction, from the ledger
        that enforces it -- so brain's WorldState reports what robotd applies
        rather than a second estimate of its own."""
        remaining = self.budget.remaining(self.turns.current or "")
        return StateBudget(path_m=remaining.path_m, motion_s=remaining.motion_s)

    def _readiness(
        self,
        telemetry: TelemetryFrame | None,
        age: float | None,
        mcu_state: McuState,
    ) -> tuple[bool, str]:
        if telemetry is None or age is None or not self.link.link_alive:
            return False, "mcu_link_down"
        if self.estop_sw or mcu_state is McuState.ESTOP:
            return False, "estop_active"
        # `is FAULT` rather than the fault mask alone: the MCU enters FAULT only
        # on a latched bit, so the two agree -- except for a state value outside
        # the enum, which _mcu_state maps here so an unreadable controller fails
        # toward not-ready rather than toward ready.
        if mcu_state is McuState.FAULT or telemetry.fault & int(LATCHED_FAULTS):
            return False, "faulted"
        return True, ""

    def _battery(self, telemetry: TelemetryFrame | None) -> StateBattery:
        if telemetry is None:
            return StateBattery(pack_v=0.0, oc_v=0.0, current_a=0.0, pct=0)
        pack_v = telemetry.vbat_mv / 1000.0
        current_a = telemetry.imotor_ma / 1000.0
        # A25's ladder is evaluated on V_oc, never on the sagged terminal
        # voltage, so the field named oc_v has to carry the compensation the
        # MCU applies: publishing pack_v under that name understates the pack
        # by ~0.26 V at 4 A, which is exactly what the compensation removes.
        oc_v = pack_v + current_a * self.config.safety.r_pack_mohm / 1000.0
        return StateBattery(
            pack_v=pack_v, oc_v=oc_v, current_a=current_a, pct=self._pct(oc_v)
        )

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
        pose = self.odom.pose
        duration_ms = max(0, (self.clock() - active.started_mono_ns) // 1_000_000)
        linear = active.skill == "drive"
        return ResultDetail(
            traveled_m=active.progress if linear else None,
            turned_deg=None if linear else rad_to_deg(active.progress),
            duration_ms=duration_ms,
            odom_delta=OdomDelta(
                x_m=pose.x_m - active.start_pose.x_m,
                y_m=pose.y_m - active.start_pose.y_m,
                # Both headings are already folded into (-pi, +pi], so their
                # difference is not the swept angle across the boundary.
                yaw_rad=wrap_angle(pose.yaw_rad - active.start_pose.yaw_rad),
            ),
            speed_clamped_to_cms=active.speed_clamped_to_cms,
        )

    # -- helpers ------------------------------------------------------------

    def _context(self) -> ValidationContext:
        telemetry = self.link.telemetry
        turn_id = self.turns.current or ""
        remaining = self.budget.remaining(turn_id)
        return ValidationContext(
            now_mono_ns=self.clock(),
            telemetry_age_ms=self.link.telemetry_age_ms(),
            mcu_fault=telemetry.fault if telemetry is not None else 0,
            front_range_mm=telemetry.tof_front_mm if telemetry is not None else None,
            estop_sw=self.estop_sw,
            current_turn_id=self.turns.current,
            active=self.arbiter.active,
            last_motion_start_mono_ns=self.arbiter.last_motion_start_mono_ns,
            last_motion_turn_id=self.arbiter.last_motion_turn_id,
            remaining_path_m=remaining.path_m,
            remaining_motion_s=remaining.motion_s,
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

    def _fault_detail(self, name: str, frame: TelemetryFrame) -> dict[str, Any]:
        if name == Fault.TOF_STOP.name.lower():
            return {"range_m": _range_m(frame.tof_front_mm)}
        return {"fault_mask": frame.fault}

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


_EVENT_KINDS: Final[dict[int, EventKind]] = {
    1: EventKind.ARM_OK,
    2: EventKind.ARM_DENIED,
    3: EventKind.TTL_EXPIRED,
    4: EventKind.TTL_RECOVERED,
    5: EventKind.FAULT_SET,
    6: EventKind.FAULT_CLEARED,
    7: EventKind.CAP_CLAMP,
    8: EventKind.WDT_REBOOT,
    9: EventKind.BROWNOUT,
    10: EventKind.I2C_ERROR,
    11: EventKind.TOF_STATUS,
    12: EventKind.SESSION_RESET,
    13: EventKind.LOOP_OVERRUN,
    14: EventKind.STALL,
    15: EventKind.CAL_STORED,
}
"""The Pi-side text table for the ``E`` codes; the numbers live in 5.1."""


def _mcu_state(telemetry: TelemetryFrame | None) -> McuState:
    """``T.state`` as an enum, mapping anything outside 0..5 to ``FAULT``.

    ``TelemetryFrame.state`` is a plain int over the u8 range the codec
    accepts, so one CRC-valid frame carrying a seventh state -- a firmware that
    grew one without a proto bump -- made every later ``_publish_state`` raise
    inside the control loop's catch-all.  robotd then stopped publishing
    ``state`` while still driving: brain's half-duplex TTS gate reads
    ``moving`` off the last state it saw, so it would speak through a drive and
    defeat A29, and ``ready`` freezes with it.  Unknown fails toward not-ready.
    """
    if telemetry is None:
        return McuState.BOOT
    try:
        return McuState(telemetry.state)
    except ValueError:
        log.warning("T.state=%d is outside 0..5; treating it as FAULT", telemetry.state)
        return McuState.FAULT


def _range_m(mm: int | None) -> float | None:
    """65535 publishes as ``null``; 65534 as 6.0 beside ``front_at_max``,
    never as 65.534 -- the distinction I-16 tests has to survive into the bus."""
    if mm is None or mm == TOF_ERROR_MM:
        return None
    if mm == TOF_NO_TARGET_MM:
        return 6.0
    return mm_to_m(mm)


async def serve(config: RobotConfig) -> None:
    """Run robotd until SIGTERM or SIGINT, then unwind.

    systemd stops a unit with SIGTERM.  Without a handler the interpreter dies
    where it stands: ``Robotd.run``'s finally never runs, so the shutdown
    ``S``+``D`` of 4.2 are never sent, ``RobotdLog`` is not closed and an open
    teleop episode is never fsynced.
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
