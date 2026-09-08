"""The vision response schemas of ARCHITECTURE 5.5.

A13: vision uses a **separate Observation schema that cannot contain a skill**,
and neither schema carries a distance.  That is structural here, not a
convention: every model is ``extra="forbid"``, so a back end that invents
``skill``, ``bearing_deg`` or ``distance_m`` is rejected rather than trusted.

The model never emits an angle.  ``center_x_permille`` is the only geometry it
supplies; the bearing is computed on the Pi by
:func:`rover_contracts.units.bearing_deg_from_center_x` from a calibrated
``[camera] hfov_deg``.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Final, Literal

from pydantic import Field, TypeAdapter

from rover_contracts.messages import EnumValue, StrictModel

__all__ = [
    "Confidence",
    "FindObservation",
    "Hazard",
    "Lighting",
    "Observation",
    "SceneObservation",
    "observation_adapter",
]


class Confidence(StrEnum):
    """How sure the model is that it found the object."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class Lighting(StrEnum):
    """Scene illumination, which bounds how far the vision answer is worth."""

    DARK = "dark"
    DIM = "dim"
    NORMAL = "normal"
    BRIGHT = "bright"


class Hazard(StrEnum):
    """Model-reported hazards.

    ``text_in_frame`` drives ARCHITECTURE 6's speed cap, which is a
    **model-reported mitigation, not a deterministic control**: its only input
    is a field the model itself writes, and an attacker who put text in the
    frame is the same party writing it (A12).  It is reported at G1 beside the
    attack-success rate, never counted among the Pi-side controls.
    """

    CLUTTER = "clutter"
    TEXT_IN_FRAME = "text_in_frame"
    STAIRS = "stairs"
    DROP_OFF = "drop_off"
    PEOPLE = "people"
    LOW_LIGHT = "low_light"


Description = Annotated[str, Field(max_length=240)]


class FindObservation(StrictModel):
    """The answer to one ``find`` sweep.

    ``center_x_permille`` is 0..1000 across the frame; it is meaningless when
    ``present`` is false and the executor ignores it then.
    """

    kind: Literal["find"]
    present: bool
    center_x_permille: int = Field(ge=0, le=1000)
    confidence: EnumValue[Confidence]
    description: Description


class SceneObservation(StrictModel):
    """The answer to ``describe_scene``, spoken back through ``say``."""

    kind: Literal["scene"]
    description: Description
    labels: list[Annotated[str, Field(min_length=1, max_length=32)]] = Field(
        max_length=12
    )
    lighting: EnumValue[Lighting]
    hazards: list[EnumValue[Hazard]] = Field(max_length=6)


Observation = Annotated[FindObservation | SceneObservation, Field(discriminator="kind")]

observation_adapter: Final[TypeAdapter[Observation]] = TypeAdapter(Observation)
