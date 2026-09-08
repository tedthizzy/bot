"""The skill catalog of ARCHITECTURE 6 as amended by ADR-0013, as data.

One row per skill: its name, the two argument models (what the model emits in
integers, and what crosses robotd), its operational bounds, which process
executes it, and whether it commands motion.  Everything else -- the
validator's motion/non-motion split, the ``allowed_skills`` list in the
WorldState, the drift alarm against the firmware cap -- derives from this table
rather than restating it.

The rover is open loop.  There is no distance and no speed in metres per
second anywhere in this table: a drive is a power for a time, a turn is a
heading closed on the controller's fused yaw.  Bounds carrying a ``fw_cap``
name are the ones the firmware fork also enforces in its own units;
``tests/unit/test_caps_match.py`` asserts each of them sits inside
``firmware/General_Driver/bot_config.h``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from pydantic import BaseModel

from rover_contracts.messages import (
    DriveForArgs,
    DriveForBusArgs,
    FindArgs,
    NoArgs,
    SayArgs,
    SayBusArgs,
    SetFaceArgs,
    SkillName,
    TurnToArgs,
    TurnToBusArgs,
    TwistPayload,
)

__all__ = [
    "Bound",
    "CATALOG",
    "ExecutorOwner",
    "FIND_BUDGET_S",
    "MOTION_SKILLS",
    "NON_MOTION_SKILLS",
    "SKILLS",
    "SkillSpec",
    "TWIST",
    "Unit",
    "goal_deadline_s",
    "power_from_pct",
]

FIND_BUDGET_S: Final = 60.0
"""Whole-operation deadline for a local find, including every adapter await."""


def goal_deadline_s(
    duration_s: float, *, skill: SkillName = SkillName.DRIVE_FOR
) -> float:
    """Shared T2 calculation; each host still enforces the result independently.

    Drives allow half their duration plus 0.5 seconds for completion. A turn's
    timeout already is its complete deadline, not an estimated drive duration.
    """
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError(f"duration must be positive, got {duration_s!r}")
    if skill == SkillName.DRIVE_FOR:
        return duration_s * 1.5 + 0.5
    if skill == SkillName.TURN_TO:
        return duration_s
    raise ValueError(f"{skill} has no motor deadline")


def power_from_pct(power_pct: int) -> float:
    """Legacy ``power_pct`` means hundredths of a Waveshare power unit.

    Preserve 30 -> 0.30. Full scale is 0.5, so 30 is 60% duty, not 30% duty.
    """
    return round(power_pct / 100.0, 3)


class ExecutorOwner(StrEnum):
    """Which process turns an accepted skill into behaviour."""

    ROBOTD = "robotd"
    BRAIN = "brain"


class Unit(StrEnum):
    """The unit a bound is stated in, on the bus."""

    S = "s"
    POWER = "power"
    DEG = "deg"
    COUNT = "count"
    CHARS = "chars"


@dataclass(frozen=True, slots=True)
class Bound:
    """One operational bound, in the units the robotd bus uses.

    ``fw_cap`` names the compiled firmware constant the bound must sit inside;
    it is ``None`` for a bound the firmware has no opinion about, such as a
    duration (the firmware holds no goal) or a string length.
    """

    field: str
    lo: float
    hi: float
    unit: Unit
    lo_exclusive: bool = False
    hi_exclusive: bool = False
    fw_cap: str | None = None

    def contains(self, value: float) -> bool:
        low_ok = value > self.lo if self.lo_exclusive else value >= self.lo
        high_ok = value < self.hi if self.hi_exclusive else value <= self.hi
        return low_ok and high_ok


@dataclass(frozen=True, slots=True)
class SkillSpec:
    """One row of the catalog.

    ``model_args`` is ``None`` for ``twist``, which the model may not emit;
    ``bus_args`` is ``None`` for ``stop``, which brain translates into the
    stop-class bus message rather than carrying as a skill, so that it never has
    to pass strict parsing to be recognised as a stop (I-22).
    """

    name: str
    model_args: type[BaseModel] | None
    bus_args: type[BaseModel] | None
    bounds: tuple[Bound, ...]
    executor: ExecutorOwner
    moves: bool


TWIST: Final = "twist"
"""Not a model skill: a streamed command, and its own bus message (A35)."""

CATALOG: Final[tuple[SkillSpec, ...]] = (
    SkillSpec(
        name=SkillName.DRIVE_FOR,
        model_args=DriveForArgs,
        bus_args=DriveForBusArgs,
        bounds=(
            Bound("duration_s", 0.0, 2.0, Unit.S, lo_exclusive=True),
            Bound("power", -0.30, 0.30, Unit.POWER, fw_cap="BOT_POWER_CAP"),
        ),
        executor=ExecutorOwner.ROBOTD,
        moves=True,
    ),
    SkillSpec(
        name=SkillName.TURN_TO,
        model_args=TurnToArgs,
        bus_args=TurnToBusArgs,
        bounds=(
            Bound("heading_deg", 0.0, 360.0, Unit.DEG, hi_exclusive=True),
            Bound("timeout_s", 0.0, 4.0, Unit.S, lo_exclusive=True),
            Bound("tolerance_deg", 2.0, 20.0, Unit.DEG),
        ),
        executor=ExecutorOwner.ROBOTD,
        moves=True,
    ),
    SkillSpec(
        name=SkillName.STOP,
        model_args=NoArgs,
        bus_args=None,
        bounds=(),
        executor=ExecutorOwner.ROBOTD,
        moves=False,
    ),
    SkillSpec(
        name=SkillName.SAY,
        model_args=SayArgs,
        bus_args=SayBusArgs,
        bounds=(Bound("text", 1, 300, Unit.CHARS),),
        executor=ExecutorOwner.BRAIN,
        moves=False,
    ),
    SkillSpec(
        name=SkillName.DESCRIBE_SCENE,
        model_args=NoArgs,
        bus_args=NoArgs,
        bounds=(),
        executor=ExecutorOwner.BRAIN,
        moves=False,
    ),
    SkillSpec(
        name=SkillName.FIND,
        model_args=FindArgs,
        bus_args=FindArgs,
        bounds=(Bound("max_sweeps", 1, 8, Unit.COUNT),),
        executor=ExecutorOwner.BRAIN,
        moves=True,
    ),
    SkillSpec(
        name=SkillName.SET_FACE,
        model_args=SetFaceArgs,
        bus_args=SetFaceArgs,
        bounds=(),
        executor=ExecutorOwner.BRAIN,
        moves=False,
    ),
)

SKILLS: Final[dict[str, SkillSpec]] = {spec.name: spec for spec in CATALOG}
MOTION_SKILLS: Final[frozenset[str]] = frozenset(s.name for s in CATALOG if s.moves)
NON_MOTION_SKILLS: Final[frozenset[str]] = frozenset(
    s.name for s in CATALOG if not s.moves
)

TWIST_BOUNDS: Final[tuple[Bound, ...]] = (
    Bound("lin", -0.30, 0.30, Unit.POWER, fw_cap="BOT_POWER_CAP"),
    Bound("ang", -0.30, 0.30, Unit.POWER, fw_cap="BOT_POWER_CAP"),
)
"""The streamed command's bounds, stated once beside the catalog so the drift
alarm covers them too.  :class:`TwistPayload` carries the same numbers."""

_TWIST_FIELDS = set(TwistPayload.model_fields)
assert {b.field for b in TWIST_BOUNDS} == _TWIST_FIELDS, "twist bounds drifted"
