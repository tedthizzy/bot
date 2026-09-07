"""The local intent router: a regex table, no box call.

ARCHITECTURE 4.3 and 4.6: ``stop``, ``forward``, ``back``, ``left``, ``right``
and ``say`` are answered here, so a dead box degrades the rover to these
instead of to nothing.  The router emits exactly the same
:class:`~rover_contracts.messages.SkillCall` the model would, which is what
keeps the validator, the permission check and the executor on one path --
a locally routed drive is bounded, budgeted and cancellable like any other.

Order matters and is stated once: ``stop`` wins over everything, then ``say``,
then a turn, then a drive.  A sentence containing "stop" is a stop.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Final

from rover_contracts.config import LimitsConfig
from rover_contracts.messages import (
    DriveArgs,
    DriveCall,
    NoArgs,
    SayArgs,
    SayCall,
    SkillCall,
    StopCall,
    TurnArgs,
    TurnCall,
)
from rover_contracts.units import cms_to_mps, deg_to_rad, mps_to_cms, rad_to_deg

__all__ = ["RouterDefaults", "route"]

_STOP = re.compile(r"\b(stop|halt|freeze|whoa|hold\s+it|hold\s+up)\b", re.I)
_QUESTION = re.compile(
    r"^\s*(what|who|where|when|why|how|is|are|do|does|did|can|could|should)\b|\?\s*$",
    re.I,
)
"""A question about the world is for the box, not for the wheels: "what is
ahead of you" contains "ahead" and must not drive.  A stop is checked first and
is never a question."""
_SAY = re.compile(r"^\s*(?:say|repeat)\b[:,]?\s+(?P<text>.+?)\s*$", re.I)
_BACK = re.compile(r"\b(back|backward|backwards|reverse)\b", re.I)
_FORWARD = re.compile(r"\b(forward|forwards|ahead|straight)\b", re.I)
_LEFT = re.compile(r"\bleft\b", re.I)
_RIGHT = re.compile(r"\bright\b", re.I)

_CENTIMETRES = re.compile(r"(\d+)\s*(?:cm|centimet(?:er|re)s?)\b", re.I)
_METRES = re.compile(r"(\d+(?:\.\d+)?)\s*(?:m|met(?:er|re)s?)\b", re.I)
_DEGREES = re.compile(r"(\d+)\s*(?:deg\b|degrees?\b|°)", re.I)
_BARE = re.compile(r"\b(\d+)\b")
_HALF_A_METRE = re.compile(r"\bhalf\s+a\s+met(?:er|re)\b", re.I)

_WORD_NUMBERS: Final[dict[str, int]] = {
    "one hundred and eighty": 180,
    "one hundred eighty": 180,
    "a hundred and eighty": 180,
    "one eighty": 180,
    "one hundred": 100,
    "a hundred": 100,
    "ninety": 90,
    "seventy five": 75,
    "sixty": 60,
    "forty five": 45,
    "forty-five": 45,
    "forty": 40,
    "thirty": 30,
    "twenty": 20,
    "fifteen": 15,
    "ten": 10,
    "fifty": 50,
}
"""What people actually say to a robot.  Longest phrase first, so "one eighty"
is not read as "one"."""

_WORDS = re.compile(
    "|".join(re.escape(word) for word in _WORD_NUMBERS), re.I
)

# Speech is the router's own, not the model's: these sentences are stated
# before the skill runs, exactly as A31 requires of the model's `speech`.
_SPEECH: Final = {
    "stop": "Stopping.",
    "forward": "Going forward.",
    "back": "Backing up.",
    "left": "Turning left.",
    "right": "Turning right.",
}


@dataclass(frozen=True, slots=True)
class RouterDefaults:
    """What the router fills in when the sentence gives no number, and the
    largest distance and angle the configured deadline still admits.

    The router is the box-down path, so a call it mints and the validator then
    refuses is the worst of both: the robot answers with a reason string
    instead of moving.  ``max_distance_cm`` and ``max_angle_deg`` are therefore
    derived from the same T2 arithmetic robotd applies, not left as literals.
    """

    distance_cm: int = 30
    speed_cms: int = 20
    angle_deg: int = 90
    rate_dps: int = 40
    max_distance_cm: int = 100
    max_angle_deg: int = 180

    @classmethod
    def from_limits(cls, limits: LimitsConfig) -> RouterDefaults:
        """Take every number from ``[limits]`` rather than a literal, so the
        router cannot outrun the configured caps or mint an undispatchable
        deadline."""
        speed_cms = mps_to_cms(limits.speed_default_mps)
        rate_dps = _clamp(int(limits.rate_dps), 5, 60)
        ttl_s = limits.goal_ttl_ms_max / 1000.0
        return cls(
            speed_cms=speed_cms,
            rate_dps=rate_dps,
            distance_cm=_clamp(30, 1, _reachable_cm(speed_cms, ttl_s)),
            angle_deg=_clamp(90, 1, _reachable_deg(rate_dps, ttl_s)),
            max_distance_cm=_reachable_cm(speed_cms, ttl_s),
            max_angle_deg=_reachable_deg(rate_dps, ttl_s),
        )


def _reachable_cm(speed_cms: int, ttl_s: float) -> int:
    """The largest ``distance_cm`` whose T2 estimate fits in ``ttl_s``."""
    metres = goal_deadline_reach(cms_to_mps(speed_cms), ttl_s)
    return _clamp(int(metres * 100), 1, 100)


def _reachable_deg(rate_dps: int, ttl_s: float) -> int:
    """The largest ``angle_deg`` whose T2 estimate fits in ``ttl_s``."""
    radians = goal_deadline_reach(deg_to_rad(rate_dps), ttl_s)
    return _clamp(int(rad_to_deg(radians)), 1, 180)


def goal_deadline_reach(speed: float, ttl_s: float) -> float:
    """Invert T2: the distance whose ``|d|/v * 1.5 + 0.5`` equals ``ttl_s``."""
    return max(0.0, (ttl_s - 0.5) * speed / 1.5)


_DEFAULTS: Final = RouterDefaults()


def _clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def _word_number(text: str) -> int | None:
    match = _WORDS.search(text)
    return _WORD_NUMBERS[match.group(0).lower()] if match else None


def _distance_cm(text: str, defaults: RouterDefaults) -> int:
    """A distance in centimetres, clamped to what ``drive`` can actually run."""
    top = defaults.max_distance_cm
    if _HALF_A_METRE.search(text):
        return _clamp(50, 1, top)
    if (match := _CENTIMETRES.search(text)) is not None:
        return _clamp(int(match.group(1)), 1, top)
    if (match := _METRES.search(text)) is not None:
        return _clamp(round(float(match.group(1)) * 100), 1, top)
    if (words := _word_number(text)) is not None:
        return _clamp(words, 1, top)
    if (match := _BARE.search(text)) is not None:
        return _clamp(int(match.group(1)), 1, top)
    return _clamp(defaults.distance_cm, 1, top)


def _angle_deg(text: str, defaults: RouterDefaults) -> int:
    """An angle in degrees, clamped to what ``turn`` can actually run."""
    top = defaults.max_angle_deg
    if (match := _DEGREES.search(text)) is not None:
        return _clamp(int(match.group(1)), 1, top)
    if (words := _word_number(text)) is not None:
        return _clamp(words, 1, top)
    if (match := _BARE.search(text)) is not None:
        return _clamp(int(match.group(1)), 1, top)
    return _clamp(defaults.angle_deg, 1, top)


def route(text: str, defaults: RouterDefaults = _DEFAULTS) -> SkillCall | None:
    """The local answer for ``text``, or ``None`` to ask the box.

    The text is a transcript: it is matched, never executed, and nothing in it
    can widen a bound -- every number it yields is clamped into the same range
    the schema would have enforced on the model.
    """
    if _STOP.search(text):
        return StopCall(speech=_SPEECH["stop"], skill="stop", args=NoArgs())
    if _QUESTION.search(text):
        return None
    if (match := _SAY.match(text)) is not None:
        spoken = match.group("text")[:240]
        return SayCall(speech="", skill="say", args=SayArgs(text=spoken))
    if _LEFT.search(text):
        return TurnCall(
            speech=_SPEECH["left"],
            skill="turn",
            args=TurnArgs(
                angle_deg=_angle_deg(text, defaults), rate_dps=defaults.rate_dps
            ),
        )
    if _RIGHT.search(text):
        return TurnCall(
            speech=_SPEECH["right"],
            skill="turn",
            args=TurnArgs(
                angle_deg=-_angle_deg(text, defaults),
                rate_dps=defaults.rate_dps,
            ),
        )
    if _BACK.search(text):
        return DriveCall(
            speech=_SPEECH["back"],
            skill="drive",
            args=DriveArgs(
                distance_cm=-_distance_cm(text, defaults),
                speed_cms=defaults.speed_cms,
            ),
        )
    if _FORWARD.search(text):
        return DriveCall(
            speech=_SPEECH["forward"],
            skill="drive",
            args=DriveArgs(
                distance_cm=_distance_cm(text, defaults),
                speed_cms=defaults.speed_cms,
            ),
        )
    return None
