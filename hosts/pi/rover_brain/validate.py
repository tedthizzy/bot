"""Stage two of A12, and the dispatch-time permission check that is stage three.

A12 runs three stages: a server-side grammar, then pydantic v2 ``strict`` with
``extra="forbid"``, then a permission check *at dispatch time*.  Structured
generation is itself an attack channel, so nothing here trusts the grammar to
have held: a skill that does not exist, an integer outside its bound, a field
nobody asked for and a value that is not finite are each rejected here, before
anything looks at what the skill means.

The rover is open loop, so the model's integers become bus units without a
metre anywhere: ``power_pct`` becomes ``power`` in Waveshare units and is
clamped to ``[limits] power_default`` (a clamp, not a rejection, and recorded
so the completion can say so), ``duration_ms`` becomes ``duration_s``, and a
``heading_deg`` passes through with the configured turn deadline and tolerance.

The reasons this module raises are written to be handed straight back to the
model in ARCHITECTURE 5.7's one retry (``VALIDATOR: <reason>``), so they name
the field and the bound rather than quoting a stack of pydantic internals.
"""

from __future__ import annotations

import json
import math
from typing import Final, TypeGuard

from pydantic import ValidationError
from pydantic_core import ErrorDetails
from rover_contracts.config import LimitsConfig
from rover_contracts.messages import (
    DescribeSceneCall,
    DriveForBusArgs,
    DriveForCall,
    NoArgs,
    ResultReason,
    SayBusArgs,
    SayCall,
    SkillCall,
    SkillMessage,
    SkillName,
    SkillObs,
    SkillTrace,
    Source,
    TurnToBusArgs,
    TurnToCall,
    skill_call_adapter,
)
from rover_contracts.skills import MOTION_SKILLS, goal_deadline_s, power_from_pct

__all__ = [
    "BUS_SKILLS",
    "MOTION_CAPABLE",
    "SPEECH_MAX_CHARS",
    "ValidationFailure",
    "authorized_motion",
    "bus_args_for",
    "crosses_bus",
    "goal_ttl_ms_for",
    "permit",
    "power_clamped_to",
    "to_bus_message",
    "turn_timeout_s",
    "validate_output",
]

SPEECH_MAX_CHARS: Final = 160
"""ARCHITECTURE 5.4: an over-length ``speech`` is truncated at the validator,
never rejected -- a too-chatty sentence is not a reason to refuse a valid
skill.  It is truncated because A31 makes it the sentence that must finish
playing before dispatch, so its length is the pre-dispatch deaf window."""

MOTION_CAPABLE: Final = frozenset(MOTION_SKILLS)
"""What ``authorized_motion`` gates: ``drive_for``, ``turn_to`` and ``find``,
whose every sweep dispatches a ``turn_to`` of its own."""

BUS_SKILLS: Final = frozenset(
    {SkillName.DRIVE_FOR, SkillName.TURN_TO, SkillName.SAY, SkillName.DESCRIBE_SCENE}
)
"""Which skills cross robotd as ``type:"skill"``.

ARCHITECTURE 4.2: "Non-motion skills still cross the bus -- that is where
``say``'s 300-char bound is enforced", and 4.2's validator table exists so that
``say`` and ``describe_scene`` stay permitted when motion is not.  ``stop`` is
translated into the stop-class bus message instead (I-22); ``set_face``'s only
route is ``brain.sock`` (ARCHITECTURE 6) and ``find`` is a brain loop whose
individual turns cross the bus on their own.
"""

_NON_MOTION_GOAL_TTL_MS: Final = 2000
"""``goal_ttl_ms`` is required on every ``skill`` message, but a non-motion
skill has no duration to compute one from, so it gets a short fixed expiry."""


class ValidationFailure(ValueError):
    """A model output this host will not dispatch.

    ``reason`` is the short machine tag; ``str(self)`` is the sentence handed
    back to the model on the one retry.
    """

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(detail)
        self.reason = reason
        self.detail = detail


def _no_constants(token: str) -> float:
    raise ValidationFailure(
        "not_finite", f"{token} is not a JSON number; every value must be finite"
    )


def _describe(error: ErrorDetails) -> ValidationFailure:
    """Turn the first pydantic error into a reason the model can act on."""
    loc = ".".join(str(part) for part in error["loc"] if part != "function-after")
    kind = str(error["type"])
    message = str(error["msg"])
    if kind in {"union_tag_invalid", "union_tag_not_found", "literal_error"}:
        names = ", ".join(sorted(s.value for s in SkillName))
        return ValidationFailure(
            "unknown_skill", f"skill must be one of {names}; got {loc or 'nothing'}"
        )
    if kind == "extra_forbidden":
        return ValidationFailure("extra_field", f"{loc} is not a field of this skill")
    if kind.startswith(("greater_than", "less_than")):
        return ValidationFailure("out_of_range", f"{loc} is out of range ({message})")
    if kind in {"string_too_long", "string_too_short", "too_long", "too_short"}:
        return ValidationFailure("out_of_range", f"{loc}: {message}")
    if kind == "value_error":
        # A model_validator's own sentence, e.g. "power_pct 0 is not a drive".
        return ValidationFailure(
            "out_of_range", f"{loc}: {message.removeprefix('Value error, ')}"
        )
    if kind == "finite_number":
        return ValidationFailure("not_finite", f"{loc} must be a finite number")
    return ValidationFailure("bad_type", f"{loc}: {message}")


def validate_output(raw: str) -> SkillCall:
    """Parse and validate one model output into a :class:`SkillCall`.

    Raises :class:`ValidationFailure` for malformed JSON, a hallucinated skill,
    an out-of-range integer, an unexpected field and a non-finite value.  The
    only thing it repairs is an over-length ``speech`` (5.4).
    """
    text = raw.strip()
    if not text:
        raise ValidationFailure("empty", "no output; emit one JSON object")
    try:
        parsed = json.loads(text, parse_constant=_no_constants)
    except json.JSONDecodeError as exc:
        raise ValidationFailure(
            "malformed_json", f"output is not one JSON object ({exc.msg})"
        ) from exc
    if not isinstance(parsed, dict):
        raise ValidationFailure(
            "malformed_json", f"output must be a JSON object, got {type(parsed).__name__}"
        )
    speech = parsed.get("speech")
    if isinstance(speech, str) and len(speech) > SPEECH_MAX_CHARS:
        parsed["speech"] = speech[:SPEECH_MAX_CHARS]
    try:
        return skill_call_adapter.validate_python(parsed)
    except ValidationError as exc:
        raise _describe(exc.errors()[0]) from exc


def authorized_motion(
    text: str, confidence: float | None, *, min_chars: int, min_confidence: float
) -> bool:
    """ARCHITECTURE 7's rule, verbatim.

    A ``null`` confidence -- text, PTT, ``roverctl utter`` and every backend
    that supplies none -- counts as authorized, which is what keeps the whole
    Mac test plan able to move.
    """
    if confidence is not None and not math.isfinite(confidence):
        return False
    return len(text.strip()) >= min_chars and (
        confidence is None or confidence >= min_confidence
    )


def turn_timeout_s(limits: LimitsConfig) -> float:
    """A turn's timeout is its deadline, bounded by the configured TTL ceiling."""
    timeout = min(limits.turn_timeout_max_s, limits.goal_ttl_ms_max / 1000.0)
    if (
        math.ceil(goal_deadline_s(timeout, skill=SkillName.TURN_TO) * 1000)
        > limits.goal_ttl_ms_max
    ):
        timeout = math.nextafter(timeout, 0.0)
    return timeout


def power_clamped_to(call: SkillCall, limits: LimitsConfig) -> float | None:
    """The power a ``drive_for`` is clamped to, or ``None`` when it is not.

    A request above ``[limits] power_default`` is clamped, not rejected, and the
    clamp is reported so the completion can say "at my top power" rather than
    let the model believe it got what it asked for.
    """
    if call.skill != "drive_for":
        return None
    if abs(power_from_pct(call.args.power_pct)) > limits.power_default:
        return limits.power_default
    return None


def bus_args_for(
    call: SkillCall, limits: LimitsConfig
) -> DriveForBusArgs | TurnToBusArgs | SayBusArgs | NoArgs:
    """The model's integers as the bus-unit arguments robotd validates.

    Raises :class:`ValidationFailure` for a ``duration_ms`` above the configured
    ``drive_for_max_s`` -- config may lower a ceiling and robotd would refuse
    it ``out_of_bounds``, so the model hears about it on the retry instead.
    """
    if not crosses_bus(call):
        raise ValueError(f"{call.skill} is not dispatched over the robotd bus")
    if call.skill == "drive_for":
        duration_s = call.args.duration_ms / 1000.0
        if duration_s > limits.drive_for_max_s:
            raise ValidationFailure(
                "out_of_range",
                f"duration_ms is above the configured "
                f"{round(limits.drive_for_max_s * 1000)} ms",
            )
        power = power_from_pct(call.args.power_pct)
        clamp = power_clamped_to(call, limits)
        if clamp is not None:
            power = math.copysign(clamp, power)
        return DriveForBusArgs(duration_s=duration_s, power=power)
    if call.skill == "turn_to":
        return TurnToBusArgs(
            heading_deg=float(call.args.heading_deg),
            timeout_s=turn_timeout_s(limits),
            tolerance_deg=limits.turn_tolerance_deg,
        )
    if call.skill == "say":
        return SayBusArgs(text=call.args.text)
    return NoArgs()


def goal_ttl_ms_for(call: SkillCall, limits: LimitsConfig) -> int:
    """The T2 estimate for this call, in milliseconds.

    ARCHITECTURE 4.2: computed from the skill's own time, never from a
    constant, and rejected rather than truncated when it does not fit inside
    ``goal_ttl_ms_max``.  The formula and its argument are robotd's, exactly:
    it re-derives T2 from the seconds it receives and rejects a ``goal_ttl_ms``
    below it, so a second implementation disagrees by one millisecond and
    robotd refuses the deadline brain itself computed.
    """
    if call.skill == "drive_for":
        seconds = call.args.duration_ms / 1000.0
    elif call.skill == "turn_to":
        seconds = turn_timeout_s(limits)
    else:
        return min(_NON_MOTION_GOAL_TTL_MS, limits.goal_ttl_ms_max)
    estimate = math.ceil(goal_deadline_s(seconds, skill=SkillName(call.skill)) * 1000)
    if estimate > limits.goal_ttl_ms_max:
        raise ValidationFailure(
            "goal_ttl_too_long",
            f"{call.skill} would need {estimate / 1000:.1f} s, over the "
            f"{limits.goal_ttl_ms_max / 1000:.1f} s deadline; ask for a shorter drive",
        )
    return max(100, estimate)


def permit(
    call: SkillCall, *, authorized: bool, limits: LimitsConfig
) -> ResultReason | None:
    """A12 stage three, run at dispatch time and never before the network.

    Returns the refusal reason, or ``None`` when the call may be dispatched.
    """
    if call.skill in MOTION_CAPABLE and not authorized:
        return ResultReason.UNAUTHORIZED_UTTERANCE
    if crosses_bus(call):
        try:
            bus_args_for(call, limits)
            goal_ttl_ms_for(call, limits)
        except ValidationFailure as exc:
            return refusal_reason(exc)
    return None


def refusal_reason(failure: ValidationFailure) -> ResultReason:
    """The speakable reason for a call this host refused to dispatch."""
    if failure.reason == "goal_ttl_too_long":
        return ResultReason.GOAL_TTL_TOO_LONG
    return ResultReason.OUT_OF_BOUNDS


def crosses_bus(
    call: SkillCall,
) -> TypeGuard[DriveForCall | TurnToCall | SayCall | DescribeSceneCall]:
    """True when this call is dispatched to robotd as ``type:"skill"``."""
    return call.skill in BUS_SKILLS


def to_bus_message(
    call: SkillCall,
    *,
    cmd_id: str,
    turn_id: str,
    seq: int,
    issued_mono_ns: int,
    limits: LimitsConfig,
    authorized: bool,
    obs: SkillObs | None = None,
    model: str | None = None,
    prompt_sha256: str | None = None,
) -> SkillMessage:
    """Attach the host's own identifiers and expiry to a validated call.

    Nothing the model emitted becomes an identifier: ``cmd_id``, ``turn_id``,
    ``seq``, ``issued_mono_ns``, ``goal_ttl_ms``, ``obs`` and ``source`` are all
    the host's, and the model's integers are converted here into the units the
    bus carries.  ``authorized_motion`` rides in ``trace`` so G1 can assert it
    per trial (ARCHITECTURE 7).
    """
    if not crosses_bus(call):
        raise ValueError(f"{call.skill} is not dispatched over the robotd bus")
    return SkillMessage(
        source=Source.BRAIN,
        cmd_id=cmd_id,
        seq=seq,
        turn_id=turn_id,
        issued_mono_ns=issued_mono_ns,
        goal_ttl_ms=goal_ttl_ms_for(call, limits),
        skill=call.skill,
        args=bus_args_for(call, limits),
        obs=obs,
        trace=SkillTrace(
            model=model, prompt_sha256=prompt_sha256, authorized_motion=authorized
        ),
    )
