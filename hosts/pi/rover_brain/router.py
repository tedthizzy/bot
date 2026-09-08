"""The local intent router: a regex table, no box call.

ARCHITECTURE 4.6: a dead box degrades the rover to the router's local intents,
so ``stop``, ``forward``, ``back``, ``turn left``, ``turn right``, ``turn
around``, ``look`` and ``say`` are answered here.  The router emits exactly the
same :class:`~rover_contracts.messages.SkillCall` the model would, which is
what keeps the validator, the permission check and the executor on one path --
a locally routed drive is bounded, budgeted and cancellable like any other.

The rover is open loop, so a local drive is the configured default power for
one second, and a local turn is an absolute heading: the router reads the
current ``heading_deg`` from the :class:`~rover_contracts.worldstate.WorldState`
and asks ``turn_to`` for the heading 90 degrees to the left or right of it.

Order matters and is stated once: ``stop`` wins over everything, then a look,
then a question (which is for the box, not the wheels), then ``say``, then a
turn, then a drive.  A sentence containing "stop" is a stop.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Final

from rover_contracts.config import LimitsConfig
from rover_contracts.messages import (
    DescribeSceneCall,
    DriveForArgs,
    DriveForCall,
    NoArgs,
    SayArgs,
    SayCall,
    SkillCall,
    StopCall,
    TurnToArgs,
    TurnToCall,
)
from rover_contracts.worldstate import WorldState

from rover_brain.heading import heading_left_of

__all__ = ["RouterDefaults", "route"]

_STOP = re.compile(r"\b(stop|halt|freeze|whoa|hold\s+it|hold\s+up)\b", re.I)
_LOOK = re.compile(
    r"^\s*(?:(?:have|take)\s+a\s+)?look(?:\s+around)?\s*[.!?]?\s*$"
    r"|\bwhat\s+(?:do|can)\s+you\s+see\b",
    re.I,
)
"""A bare "look" or "what do you see" is ``describe_scene``.  "look happy" and
"look for the mug" do not match and go to the box, which can tell a face from a
search."""
_QUESTION = re.compile(
    r"^\s*(what|who|where|when|why|how|is|are|do|does|did|can|could|should)\b|\?\s*$",
    re.I,
)
"""A question about the world is for the box, not for the wheels: "what is
ahead of you" contains "ahead" and must not drive.  A stop is checked first and
is never a question."""
_SAY = re.compile(r"^\s*(?:say|repeat)\b[:,]?\s+(?P<text>.+?)\s*$", re.I)
_AROUND = re.compile(r"\b(?:turn|spin)\s+around\b|\babout[- ]face\b|\bu[- ]turn\b", re.I)
_LEFT = re.compile(r"\bleft\b", re.I)
_RIGHT = re.compile(r"\bright\b", re.I)
_BACK = re.compile(r"\b(back|backward|backwards|reverse)\b", re.I)
_FORWARD = re.compile(r"\b(forward|forwards|ahead|straight)\b", re.I)

# Speech is the router's own, not the model's: these sentences are stated
# before the skill runs, exactly as A31 requires of the model's `speech`.
_SPEECH: Final = {
    "stop": "Stopping.",
    "look": "Let me look.",
    "forward": "Going forward.",
    "back": "Backing up.",
    "left": "Turning left.",
    "right": "Turning right.",
    "around": "Turning around.",
}


@dataclass(frozen=True, slots=True)
class RouterDefaults:
    """What the router fills in: the power and duration of a local drive, and
    the angle of a local turn.  ``power_pct`` comes from ``[limits]
    power_default`` so the box-down path can never ask for more than a model
    call would be clamped to."""

    power_pct: int = 20
    duration_ms: int = 1000
    turn_deg: int = 90
    around_deg: int = 180

    @classmethod
    def from_limits(cls, limits: LimitsConfig) -> RouterDefaults:
        # DriveForArgs forbids 0 and the schema tops out at 30: a power_default
        # that rounds outside that is still a legal drive at the nearest edge.
        power_pct = max(1, min(30, round(limits.power_default * 100)))
        return cls(power_pct=power_pct)


_DEFAULTS: Final = RouterDefaults()


def _turn(speech: str, world: WorldState, left_deg: int) -> TurnToCall:
    return TurnToCall(
        speech=speech,
        skill="turn_to",
        args=TurnToArgs(heading_deg=heading_left_of(world.heading_deg, left_deg)),
    )


def _drive(speech: str, defaults: RouterDefaults, sign: int) -> DriveForCall:
    return DriveForCall(
        speech=speech,
        skill="drive_for",
        args=DriveForArgs(
            duration_ms=defaults.duration_ms, power_pct=sign * defaults.power_pct
        ),
    )


def route(
    text: str, world: WorldState, defaults: RouterDefaults = _DEFAULTS
) -> SkillCall | None:
    """The local answer for ``text``, or ``None`` to ask the box.

    The text is a transcript: it is matched, never executed, and nothing in it
    can widen a bound -- the only numbers a local call carries are the
    configured defaults and a heading derived from the WorldState.
    """
    if _STOP.search(text):
        return StopCall(speech=_SPEECH["stop"], skill="stop", args=NoArgs())
    if _LOOK.search(text):
        return DescribeSceneCall(
            speech=_SPEECH["look"], skill="describe_scene", args=NoArgs()
        )
    if _QUESTION.search(text):
        return None
    if (match := _SAY.match(text)) is not None:
        spoken = match.group("text")[:240]
        return SayCall(speech="", skill="say", args=SayArgs(text=spoken))
    if _AROUND.search(text):
        return _turn(_SPEECH["around"], world, -defaults.around_deg)
    if _LEFT.search(text):
        return _turn(_SPEECH["left"], world, defaults.turn_deg)
    if _RIGHT.search(text):
        return _turn(_SPEECH["right"], world, -defaults.turn_deg)
    if _BACK.search(text):
        return _drive(_SPEECH["back"], defaults, -1)
    if _FORWARD.search(text):
        return _drive(_SPEECH["forward"], defaults, 1)
    return None
