"""Which command owns the wheels (ARCHITECTURE 4.2, "Arbitration").

One motion is in flight at a time.  An accepted ``twist`` preempts an active
``skill`` and vice versa, the loser reported ``preempted``; ``stop`` and
``estop`` preempt both from any allow-listed source in any state (I-22).
Priority between sources is ``web > teleop > brain``, in those exact strings.

Two commands of the same kind do not preempt each other: a second motion
``skill`` while one is running is the "one motion in flight" row of 4.2 and is
refused, and a ``twist`` from the source already streaming is a renewal of the
one arbitration unit rather than a new one.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from rover_contracts.messages import Source

from rover_robotd.profiles import DriveForProfile, TurnToProfile

__all__ = [
    "SOURCE_PRIORITY",
    "ActiveCommand",
    "Arbiter",
    "Arbitration",
    "CommandKind",
    "arbitrate",
]


class CommandKind(StrEnum):
    """The two things that can own the wheels."""

    SKILL = "skill"
    TWIST = "twist"


SOURCE_PRIORITY: Final[dict[Source, int]] = {
    Source.WEB: 3,
    Source.TELEOP: 2,
    Source.BRAIN: 1,
    Source.PHONE: 0,
}
"""``web > teleop > brain``.  ``phone`` is the later head of ARCHITECTURE 14 and
sits below all three; it is never in a stop path."""


class Arbitration(StrEnum):
    """What a newly validated command may do to the active one."""

    START = "start"
    """Nothing is active."""

    PREEMPT = "preempt"
    """The active command loses and is reported ``preempted``."""

    RENEW = "renew"
    """The same stream from the same source: one arbitration unit continues."""

    BUSY = "busy"
    """One motion in flight, and this is another of the same kind."""

    OUTRANKED = "outranked"
    """A lower-priority source may not take the wheels from a higher one."""


@dataclass(slots=True)
class ActiveCommand:
    """The one command that owns the wheels, and everything the control loop
    needs to keep executing it."""

    cmd_id: str
    kind: CommandKind
    source: Source
    session_id: str
    started_mono_ns: int
    deadline_mono_ns: int
    skill: str | None = None
    turn_id: str | None = None
    seq: int | None = None
    profile: DriveForProfile | TurnToProfile | None = None
    twist_lin: float = 0.0
    twist_ang: float = 0.0
    renewed_mono_ns: int = 0
    power_clamped_to: float | None = None

    @property
    def moves(self) -> bool:
        """A ``twist`` always moves; a ``skill`` moves when it has a profile."""
        return self.kind is CommandKind.TWIST or self.profile is not None

    def forward_power(self) -> float:
        """The forward component this command drives with -- what the
        controller's TOF and bumper flags block.  A turn has none."""
        if self.kind is CommandKind.TWIST:
            return self.twist_lin
        if isinstance(self.profile, DriveForProfile):
            return self.profile.power
        return 0.0

    def fraction(self, now_mono_ns: int) -> float:
        if isinstance(self.profile, DriveForProfile):
            return self.profile.fraction((now_mono_ns - self.started_mono_ns) / 1e9)
        if isinstance(self.profile, TurnToProfile):
            return self.profile.fraction()
        return 0.0


def arbitrate(
    source: Source, kind: CommandKind, cmd_id: str, active: ActiveCommand | None
) -> Arbitration:
    """What ``source``'s new ``kind`` command may do to ``active``."""
    if active is None:
        return Arbitration.START
    if kind is CommandKind.TWIST and active.kind is CommandKind.TWIST:
        if source is active.source:
            return Arbitration.RENEW
        return (
            Arbitration.PREEMPT
            if SOURCE_PRIORITY[source] >= SOURCE_PRIORITY[active.source]
            else Arbitration.OUTRANKED
        )
    if kind is CommandKind.SKILL and active.kind is CommandKind.SKILL:
        if cmd_id == active.cmd_id:
            return Arbitration.RENEW
        return Arbitration.BUSY
    return (
        Arbitration.PREEMPT
        if SOURCE_PRIORITY[source] >= SOURCE_PRIORITY[active.source]
        else Arbitration.OUTRANKED
    )


class Arbiter:
    """Holds the active command.  It publishes nothing: the caller reports the
    loser it hands back, which keeps arbitration testable without a bus."""

    def __init__(self) -> None:
        self.active: ActiveCommand | None = None
        self.last_motion_start_mono_ns: int | None = None
        self.last_motion_turn_id: str | None = None

    def start(self, command: ActiveCommand) -> ActiveCommand | None:
        """Install ``command``; return the one it displaced, if any."""
        loser = self.active if self.active is not command else None
        self.active = command
        if command.moves:
            self.last_motion_start_mono_ns = command.started_mono_ns
            self.last_motion_turn_id = command.turn_id
        return loser

    def finish(self, cmd_id: str | None = None) -> ActiveCommand | None:
        """Retire the active command, optionally only if it is ``cmd_id``."""
        active = self.active
        if active is None:
            return None
        if cmd_id is not None and active.cmd_id != cmd_id:
            return None
        self.active = None
        return active
