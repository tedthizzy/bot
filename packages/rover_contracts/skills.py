"""The skill catalog of ARCHITECTURE 6, as data.

One row per skill: its name, the two argument models (what the model emits in
integers, and what crosses robotd in SI), its operational bounds, which process
executes it, and whether it commands motion.  Everything else -- the validator's
motion/non-motion split, the ``allowed_skills`` list in the WorldState, the
drift alarm against the firmware caps -- derives from this table rather than
restating it.

Bounds carrying an ``mcu_cap`` name are the ones the controller also enforces in
its own compiled units; ``tests/unit/test_caps_match.py`` asserts each of them
sits inside ``firmware/core/rover_config.h``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from pydantic import BaseModel

from rover_contracts.messages import (
    DriveArgs,
    DriveBusArgs,
    FindArgs,
    NoArgs,
    SayArgs,
    SayBusArgs,
    SetFaceArgs,
    SkillName,
    TurnArgs,
    TurnBusArgs,
    TwistPayload,
)
from rover_contracts.units import dps_to_mrad_s, mps_to_mm_s, radps_to_mrad_s

__all__ = [
    "Bound",
    "CATALOG",
    "ExecutorOwner",
    "MOTION_SKILLS",
    "NON_MOTION_SKILLS",
    "SKILLS",
    "SkillSpec",
    "TWIST",
    "Unit",
    "goal_deadline_s",
    "mcu_cap_magnitude",
]


def goal_deadline_s(distance: float, speed: float) -> float:
    """T2's deadline estimate (A19, 4.2): ``|d| / v * 1.5 + 0.5`` seconds.

    ``distance`` and ``speed`` are in matching units -- m and m/s for a drive,
    rad and rad/s for a turn -- so one formula serves both.  It lives here and
    not in robotd because brain computes the ``goal_ttl_ms`` robotd then checks
    against T2: two implementations of one formula disagree by a float ulp and
    robotd refuses the deadline brain itself derived.
    """
    if speed <= 0.0:
        raise ValueError(f"speed must be positive, got {speed!r}")
    return abs(distance) / speed * 1.5 + 0.5


class ExecutorOwner(StrEnum):
    """Which process turns an accepted skill into behaviour."""

    ROBOTD = "robotd"
    BRAIN = "brain"


class Unit(StrEnum):
    """The unit a bound is stated in, on the bus."""

    M = "m"
    MPS = "m/s"
    DEG = "deg"
    DPS = "deg/s"
    RADPS = "rad/s"
    COUNT = "count"
    CHARS = "chars"


@dataclass(frozen=True, slots=True)
class Bound:
    """One operational bound, in the units the robotd bus uses.

    ``mcu_cap`` names the compiled controller cap the bound must sit inside; it
    is ``None`` for a bound the MCU has no opinion about, such as a distance
    (the MCU holds no goal, A7) or a string length.
    """

    field: str
    lo: float
    hi: float
    unit: Unit
    lo_exclusive: bool = False
    hi_exclusive: bool = False
    mcu_cap: str | None = None

    def contains(self, value: float) -> bool:
        low_ok = value > self.lo if self.lo_exclusive else value >= self.lo
        high_ok = value < self.hi if self.hi_exclusive else value <= self.hi
        return low_ok and high_ok


@dataclass(frozen=True, slots=True)
class SkillSpec:
    """One row of ARCHITECTURE 6.

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
"""Not a model skill: a streamed velocity, and its own bus message (A35)."""


CATALOG: Final[tuple[SkillSpec, ...]] = (
    SkillSpec(
        name=SkillName.DRIVE,
        model_args=DriveArgs,
        bus_args=DriveBusArgs,
        bounds=(
            Bound("distance_m", -1.0, 1.0, Unit.M),
            Bound(
                "speed_mps",
                0.0,
                0.30,
                Unit.MPS,
                lo_exclusive=True,
                mcu_cap="ROVER_MAX_V_MM_S",
            ),
        ),
        executor=ExecutorOwner.ROBOTD,
        moves=True,
    ),
    SkillSpec(
        name=SkillName.TURN,
        model_args=TurnArgs,
        bus_args=TurnBusArgs,
        bounds=(
            Bound("angle_deg", -180.0, 180.0, Unit.DEG),
            Bound(
                "rate_dps",
                0.0,
                60.0,
                Unit.DPS,
                lo_exclusive=True,
                mcu_cap="ROVER_MAX_W_MRAD_S",
            ),
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
        bounds=(
            Bound("object", 1, 48, Unit.CHARS),
            Bound("max_sweeps", 1, 8, Unit.COUNT),
        ),
        executor=ExecutorOwner.BRAIN,
        moves=False,
    ),
    SkillSpec(
        name=SkillName.SET_FACE,
        model_args=SetFaceArgs,
        bus_args=SetFaceArgs,
        bounds=(),
        executor=ExecutorOwner.BRAIN,
        moves=False,
    ),
    SkillSpec(
        name=TWIST,
        model_args=None,
        bus_args=TwistPayload,
        bounds=(
            Bound(
                "linear_x_mps", -0.30, 0.30, Unit.MPS, mcu_cap="ROVER_MAX_V_MM_S"
            ),
            Bound(
                "angular_z_radps",
                -1.047,
                1.047,
                Unit.RADPS,
                mcu_cap="ROVER_MAX_W_MRAD_S",
            ),
        ),
        executor=ExecutorOwner.ROBOTD,
        moves=True,
    ),
)

SKILLS: Final[dict[str, SkillSpec]] = {spec.name: spec for spec in CATALOG}

MOTION_SKILLS: Final = frozenset(spec.name for spec in CATALOG if spec.moves)
NON_MOTION_SKILLS: Final = frozenset(spec.name for spec in CATALOG if not spec.moves)

_TO_MCU = {
    Unit.MPS: mps_to_mm_s,
    Unit.RADPS: radps_to_mrad_s,
    Unit.DPS: dps_to_mrad_s,
}


def mcu_cap_magnitude(bound: Bound) -> int:
    """The bound's widest magnitude, in the controller's own integer units.

    mm/s for a linear cap, mrad/s for an angular one.  Only defined for a bound
    that names an ``mcu_cap``.
    """
    if bound.mcu_cap is None:
        raise ValueError(f"{bound.field} names no controller cap")
    convert = _TO_MCU[bound.unit]
    return max(abs(convert(bound.lo)), abs(convert(bound.hi)))
