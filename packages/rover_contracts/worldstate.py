"""WorldState -- what the model sees, never raw sensors (ARCHITECTURE 5.6).

Field order is the order the architecture prints it in, and pydantic preserves
declaration order on dump, because A17 pins the prompt as
``static system -> image -> world state -> utterance`` and a reordered block is
a prefix-cache miss.  Nothing that changes every turn -- no timestamp, no seq,
no session -- appears here at all.
"""

from __future__ import annotations

from pydantic import Field, model_validator

from rover_contracts.messages import (
    EnumValue,
    ResultStatus,
    SkillName,
    StrictModel,
)

__all__ = ["MotionBudget", "PoseCm", "RecentlySeen", "WorldState"]


class PoseCm(StrictModel):
    """Odometry position in whole centimetres (A11)."""

    x: int
    y: int


class RecentlySeen(StrictModel):
    """One entry of the scene ring: what, which way, how long ago."""

    label: str = Field(min_length=1, max_length=48)
    where_deg: int = Field(ge=-180, le=180)
    age_s: int = Field(ge=0)


class MotionBudget(StrictModel):
    """What is left of the per-instruction budget of I-15."""

    path_cm: int = Field(ge=0)
    seconds: int = Field(ge=0)


class WorldState(StrictModel):
    """The block placed after the image and before the utterance."""

    pose_cm: PoseCm
    heading_deg: int = Field(ge=-180, le=180)
    battery_pct: int = Field(ge=0, le=100)
    obstacle_ahead: bool
    front_range_cm: int = Field(ge=0, le=600)
    front_at_max: bool
    bumper: bool
    moving: bool
    speed_cap_cms: int = Field(ge=5, le=30)
    last_result: EnumValue[ResultStatus] | None = None
    last_scene: str = Field(default="", max_length=240)
    recently_seen: list[RecentlySeen] = Field(default_factory=list, max_length=8)
    allowed_skills: list[EnumValue[SkillName]] = Field(min_length=1)
    motion_budget_left: MotionBudget

    @model_validator(mode="after")
    def _cap_agrees_with_range(self) -> WorldState:
        """``speed_cap_cms`` and ``front_range_cm`` must agree.

        ARCHITECTURE 6 unlocks 0.30 m/s only when the front ToF is valid and
        above 1.0 m; a 65534 no-target return satisfies that and renders as 600
        with ``front_at_max``.  Publishing 30 beside a 40 cm reading would make
        the one field that could respect the cap disagree with the one that
        justifies it.
        """
        unlocked = self.front_at_max or self.front_range_cm > 100
        if self.speed_cap_cms > 20 and not unlocked:
            raise ValueError(
                f"speed_cap_cms {self.speed_cap_cms} needs front_range_cm > 100 "
                f"or front_at_max, got {self.front_range_cm} / {self.front_at_max}"
            )
        return self
