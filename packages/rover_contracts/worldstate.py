"""WorldState -- what the model sees, never raw sensors (ARCHITECTURE 5.6,
amended by ADR-0013).

Field order is the order the architecture prints it in, and pydantic preserves
declaration order on dump, because A17 pins the prompt as
``static system -> image -> world state -> utterance`` and a reordered block is
a prefix-cache miss.  Nothing that changes every turn -- no timestamp, no seq,
no session -- appears here at all.

The rover is open loop, so there is no pose.  ``heading_deg`` is the one
number the model can steer by, and ``turn_to`` takes the same 0..359 frame.
"""

from __future__ import annotations

from pydantic import Field

from rover_contracts.messages import (
    EnumValue,
    ResultStatus,
    SkillName,
    StrictModel,
)

__all__ = ["MotionBudget", "RecentlySeen", "WorldState"]


class RecentlySeen(StrictModel):
    """One entry of the scene ring: what, which heading, how long ago."""

    label: str = Field(min_length=1, max_length=48)
    heading_deg: int = Field(ge=0, le=359)
    age_s: int = Field(ge=0)


class MotionBudget(StrictModel):
    """What is left of the per-instruction budget of I-15: motion seconds."""

    seconds: int = Field(ge=0)


class WorldState(StrictModel):
    """The block placed after the image and before the utterance."""

    heading_deg: int = Field(ge=0, le=359)
    battery_pct: int = Field(ge=0, le=100)
    obstacle_ahead: bool
    front_range_cm: int | None = Field(default=None, ge=0, le=400)
    """``null`` when there is no forward sensor or no reading; the model is
    told in the system prompt that ``null`` means unknown, not clear."""
    bumper: bool
    moving: bool
    power_cap_pct: int = Field(ge=5, le=30)
    last_result: EnumValue[ResultStatus] | None = None
    last_scene: str = Field(default="", max_length=240)
    recently_seen: list[RecentlySeen] = Field(default_factory=list, max_length=8)
    allowed_skills: list[EnumValue[SkillName]] = Field(min_length=1)
    motion_budget_left: MotionBudget
