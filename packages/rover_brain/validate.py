"""Stage two of A12, and the dispatch-time permission check that is stage three.

A12 runs three stages: a server-side grammar, then pydantic v2 ``strict`` with
``extra="forbid"``, then a permission check *at dispatch time*.  Structured
generation is itself an attack channel, so nothing here trusts the grammar to
have held: a skill that does not exist, an integer outside its bound, a field
nobody asked for and a value that is not finite are each rejected here, before
anything looks at what the skill means.

The reasons this module raises are written to be handed straight back to the
model in ARCHITECTURE 5.7's one retry (``VALIDATOR: <reason>``), so they name
the field and the bound rather than quoting a stack of pydantic internals.
"""

from __future__ import annotations

import json
import math
from typing import Any, Final

from pydantic import ValidationError
from rover_contracts.config import LimitsConfig
from rover_contracts.messages import (
    BUS_SKILL_ARGS,
    DriveBusArgs,
    ResultReason,
    SayBusArgs,
    SkillCall,
    SkillMessage,
    SkillName,
    SkillObs,
    SkillTrace,
    Source,
    TurnBusArgs,
    skill_call_adapter,
)
from rover_contracts.skills import MOTION_SKILLS, goal_deadline_s
from rover_contracts.units import cm_to_m, cms_to_mps, deg_to_rad

__all__ = [
    "BUS_SKILLS",
    "MOTION_CAPABLE",
    "SPEECH_MAX_CHARS",
    "ValidationFailure",
    "authorized_motion",
    "crosses_bus",
    "goal_ttl_ms_for",
    "permit",
    "to_bus_message",
    "validate_output",
]

SPEECH_MAX_CHARS: Final = 160
"""ARCHITECTURE 5.4: an over-length ``speech`` is truncated at the validator,
never rejected -- a too-chatty sentence is not a reason to refuse a valid
skill.  It is truncated because A31 makes it the sentence that must finish
playing before dispatch, so its length is the pre-dispatch deaf window."""

MOTION_CAPABLE: Final = frozenset(MOTION_SKILLS) | {SkillName.FIND}
"""What ``authorized_motion`` gates.  ``find`` carries ``moves=False`` in the
catalog because brain, not robotd, executes it -- but every sweep it runs
dispatches a ``turn``, so an unauthorized utterance must not start one."""

BUS_SKILLS: Final = frozenset(
    {SkillName.DRIVE, SkillName.TURN, SkillName.SAY, SkillName.DESCRIBE_SCENE}
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
skill has no profile to compute one from (ARCHITECTURE 4.2 computes it from the
profile for motion only), so it gets a short fixed expiry."""


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


def _describe(error: dict[str, Any]) -> ValidationFailure:
    """Turn the first pydantic error into a reason the model can act on."""
    loc = ".".join(str(part) for part in error["loc"] if part != "function-after")
    kind = str(error["type"])
    if kind in {"union_tag_invalid", "union_tag_not_found", "literal_error"}:
        names = ", ".join(sorted(s.value for s in SkillName))
        return ValidationFailure(
            "unknown_skill", f"skill must be one of {names}; got {loc or 'nothing'}"
        )
    if kind == "extra_forbidden":
        return ValidationFailure("extra_field", f"{loc} is not a field of this skill")
    if kind.startswith(("greater_than", "less_than")):
        limit = error.get("ctx", {}).get(kind.rsplit("_", 1)[-1], "its bound")
        return ValidationFailure(
            "out_of_range", f"{loc} is out of range ({error['msg']}, limit {limit})"
        )
    if kind in {"string_too_long", "string_too_short", "too_long", "too_short"}:
        return ValidationFailure("out_of_range", f"{loc}: {error['msg']}")
    if kind == "finite_number":
        return ValidationFailure("not_finite", f"{loc} must be a finite number")
    return ValidationFailure("bad_type", f"{loc}: {error['msg']}")


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


def goal_ttl_ms_for(call: SkillCall, limits: LimitsConfig) -> int:
    """The T2 estimate for this call, in milliseconds.

    ARCHITECTURE 4.2: ``|d|/v x 1.5 + 0.5 s``, computed from the profile and
    never from a constant, and rejected rather than truncated when it does not
    fit inside ``goal_ttl_ms_max`` -- which is how the schema-legal
    ``drive(distance_cm=100, speed_cms=5)`` at 30.5 s is refused.
    """
    # The formula and its arguments are robotd's, exactly: it re-derives T2
    # from the SI it receives and rejects a goal_ttl_ms below it, so a second
    # implementation -- or the same one fed degrees instead of radians --
    # disagrees by one millisecond and robotd refuses the deadline brain itself
    # computed.
    if call.skill == SkillName.DRIVE:
        seconds = goal_deadline_s(
            cm_to_m(call.args.distance_cm), cms_to_mps(call.args.speed_cms)
        )
    elif call.skill == SkillName.TURN:
        seconds = goal_deadline_s(
            deg_to_rad(call.args.angle_deg), deg_to_rad(call.args.rate_dps)
        )
    else:
        return min(_NON_MOTION_GOAL_TTL_MS, limits.goal_ttl_ms_max)
    estimate = math.ceil(seconds * 1000)
    if estimate > limits.goal_ttl_ms_max:
        raise ValidationFailure(
            "goal_ttl_too_long",
            f"{call.skill} would take {estimate / 1000:.1f} s, over the "
            f"{limits.goal_ttl_ms_max / 1000:.1f} s deadline; go slower over a "
            "shorter distance",
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
    try:
        goal_ttl_ms_for(call, limits)
    except ValidationFailure:
        return ResultReason.GOAL_TTL_TOO_LONG
    return None


def crosses_bus(call: SkillCall) -> bool:
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
    the host's, and the model's integers are converted here into the SI the bus
    carries.  ``authorized_motion`` rides in ``trace`` so G1 can assert it per
    trial (ARCHITECTURE 7).
    """
    if not crosses_bus(call):
        raise ValueError(f"{call.skill} is not dispatched over the robotd bus")
    if call.skill == SkillName.DRIVE:
        args: Any = DriveBusArgs(
            distance_m=cm_to_m(call.args.distance_cm),
            speed_mps=cms_to_mps(call.args.speed_cms),
        )
    elif call.skill == SkillName.TURN:
        args = TurnBusArgs(
            angle_deg=float(call.args.angle_deg), rate_dps=float(call.args.rate_dps)
        )
    elif call.skill == SkillName.SAY:
        args = SayBusArgs(text=call.args.text)
    else:
        args = BUS_SKILL_ARGS[call.skill]()
    return SkillMessage(
        source=Source.BRAIN,
        cmd_id=cmd_id,
        seq=seq,
        turn_id=turn_id,
        issued_mono_ns=issued_mono_ns,
        goal_ttl_ms=goal_ttl_ms_for(call, limits),
        skill=call.skill,
        args=args,
        obs=obs,
        trace=SkillTrace(
            model=model, prompt_sha256=prompt_sha256, authorized_motion=authorized
        ),
    )
