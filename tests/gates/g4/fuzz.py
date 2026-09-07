"""The I-8 fuzz corpus: 500 cases across ``skill`` and ``twist``.

Deterministic, seeded, and -- the part that matters -- it carries valid controls
as well as invalid ones.  A validator that rejects everything satisfies "zero
out-of-bounds values reach the port" perfectly, so a corpus with no accepted
cases in it cannot tell a working validator from a brick.

Every case names the rule it exercises, so a failure says which of
ARCHITECTURE 4.2's rows let it through rather than only that something did.
"""

from __future__ import annotations

import json
import random
from dataclasses import dataclass
from typing import Any

from rover_contracts import new_cmd_id, new_turn_id

__all__ = ["Case", "build_corpus", "CORPUS_SIZE"]

CORPUS_SIZE = 500
"""I-8: 500 fuzz cases across both message types."""


@dataclass(frozen=True, slots=True)
class Case:
    """One line for the bus, and whether the validator must take it."""

    name: str
    rule: str
    line: str
    valid: bool
    kind: str

    @property
    def payload(self) -> dict[str, Any] | None:
        try:
            return json.loads(self.line)
        except json.JSONDecodeError:
            return None


def _skill(**overrides: Any) -> dict[str, Any]:
    base: dict[str, Any] = {
        "v": 1,
        "type": "skill",
        "source": "brain",
        "cmd_id": new_cmd_id(),
        "seq": 1,
        "turn_id": new_turn_id(),
        "issued_mono_ns": 123456789012,
        "goal_ttl_ms": 3000,
        "skill": "drive",
        "args": {"distance_m": 0.4, "speed_mps": 0.15},
        # What brain sends: a motion skill with no trace is refused
        # `unauthorized_utterance` (section 7, I-21), which would make every
        # "valid control" in the corpus indistinguishable from a rejection.
        "trace": {"authorized_motion": True},
    }
    base.update(overrides)
    return base


def _twist(**overrides: Any) -> dict[str, Any]:
    base: dict[str, Any] = {
        "v": 1,
        "type": "twist",
        "source": "teleop",
        "cmd_id": new_cmd_id(),
        "seq": 1,
        "twist": {"linear_x_mps": 0.15, "angular_z_radps": 0.35},
    }
    base.update(overrides)
    return base


_NUMERIC_BREAKS = (
    # (skill, field, value, rule) -- each just outside a bound of ARCHITECTURE 6.
    ("drive", "distance_m", 1.0001, "drive distance above 1.0 m"),
    ("drive", "distance_m", -1.0001, "drive distance below -1.0 m"),
    ("drive", "speed_mps", 0.3001, "drive speed above the 0.30 m/s cap"),
    ("drive", "speed_mps", 0.0, "drive speed of zero is not a drive"),
    ("drive", "speed_mps", -0.15, "a negative speed is direction by another name"),
    ("turn", "angle_deg", 180.5, "turn beyond 180 degrees"),
    ("turn", "angle_deg", -180.5, "turn below -180 degrees"),
    ("turn", "rate_dps", 60.5, "turn rate above 60 deg/s"),
    ("turn", "rate_dps", 0.0, "a turn rate of zero is not a turn"),
)

_TWIST_BREAKS = (
    ("linear_x_mps", 0.3001, "twist linear above 0.30 m/s"),
    ("linear_x_mps", -0.3001, "twist linear below -0.30 m/s"),
    ("angular_z_radps", 1.0471, "twist angular above 1.047 rad/s"),
    ("angular_z_radps", -1.0471, "twist angular below -1.047 rad/s"),
)

_ARGS_FOR = {
    "drive": {"distance_m": 0.4, "speed_mps": 0.15},
    "turn": {"angle_deg": 45.0, "rate_dps": 40.0},
}


def build_corpus(seed: int = 20260907, size: int = CORPUS_SIZE) -> list[Case]:
    """The corpus, in a fixed order, with about one case in seven valid."""
    rng = random.Random(seed)
    cases: list[Case] = []

    def add(name: str, rule: str, body: Any, valid: bool, kind: str) -> None:
        line = body if isinstance(body, str) else json.dumps(body)
        cases.append(Case(name, rule, line, valid, kind))

    # -- structural: not JSON, not a known type, not the right shape ------
    add("not_json", "the line is not JSON at all", "{not json", False, "skill")
    add("empty_object", "no discriminator", {}, False, "skill")
    add(
        "unknown_type", "unknown message type", {"v": 1, "type": "wobble"}, False, "skill"
    )
    add("wrong_v", "protocol version", _skill(v=2), False, "skill")
    for name in ("teleport", "drive_fast", "DRIVE", "", "stop"):
        add(
            f"unknown_skill_{name or 'empty'}",
            "known skill",
            _skill(skill=name),
            False,
            "skill",
        )

    # -- bounds ----------------------------------------------------------
    for skill, field, value, rule in _NUMERIC_BREAKS:
        args = dict(_ARGS_FOR[skill])
        args[field] = value
        add(
            f"oob_{skill}_{field}_{value}",
            rule,
            _skill(skill=skill, args=args),
            False,
            "skill",
        )
    for field, value, rule in _TWIST_BREAKS:
        payload = {"linear_x_mps": 0.0, "angular_z_radps": 0.0}
        payload[field] = value
        add(f"oob_twist_{field}_{value}", rule, _twist(twist=payload), False, "twist")

    # -- non-finite ------------------------------------------------------
    for literal in ("NaN", "Infinity", "-Infinity"):
        body = json.dumps(_skill()).replace(
            '"distance_m": 0.4', f'"distance_m": {literal}'
        )
        add(f"nonfinite_{literal}", "finite SI bounds", body, False, "skill")
        twist = json.dumps(_twist()).replace(
            '"linear_x_mps": 0.15', f'"linear_x_mps": {literal}'
        )
        add(f"nonfinite_twist_{literal}", "finite SI bounds", twist, False, "twist")

    # -- extra fields ----------------------------------------------------
    for extra in ("speed_cap_mps", "override", "admin", "__proto__"):
        add(
            f"extra_top_{extra}",
            'extra="forbid" at the top level',
            _skill(**{extra: 1}),
            False,
            "skill",
        )
        args = dict(_ARGS_FOR["drive"])
        args[extra] = 1
        add(
            f"extra_args_{extra}",
            'extra="forbid" inside args',
            _skill(args=args),
            False,
            "skill",
        )

    # -- goal_ttl --------------------------------------------------------
    for ttl in (0, 99, 5001, 60000, -1):
        add(
            f"goal_ttl_{ttl}",
            "goal_ttl_ms in [100, 5000]",
            _skill(goal_ttl_ms=ttl),
            False,
            "skill",
        )

    # -- wrong types -----------------------------------------------------
    add("seq_string", "strict types", _skill(seq="1"), False, "skill")
    add("ttl_string", "strict types", _skill(goal_ttl_ms="3000"), False, "skill")
    add(
        "distance_string",
        "strict types",
        _skill(args={"distance_m": "0.4", "speed_mps": 0.15}),
        False,
        "skill",
    )
    add("cmd_id_short", "ULID cmd_id", _skill(cmd_id="01J9ZC"), False, "skill")
    add(
        "cmd_id_lower", "ULID cmd_id", _skill(cmd_id=new_cmd_id().lower()), False, "skill"
    )
    add("turn_id_null", "turn_id present", _skill(turn_id=None), False, "skill")
    add("source_unknown", "source is one of four", _skill(source="root"), False, "skill")
    add("args_missing", "args required", _skill(args={}), False, "skill")
    add("args_wrong_skill", "args match the skill", _skill(skill="turn"), False, "skill")

    # -- valid controls, so an over-eager validator cannot pass -----------
    add("valid_drive", "a legal drive is accepted", _skill(), True, "skill")
    add(
        "valid_drive_reverse",
        "a legal reverse is accepted",
        _skill(args={"distance_m": -0.3, "speed_mps": 0.1}),
        True,
        "skill",
    )
    add(
        "valid_turn",
        "a legal turn is accepted",
        _skill(skill="turn", args=_ARGS_FOR["turn"]),
        True,
        "skill",
    )
    add(
        "valid_say",
        "a non-motion skill is accepted",
        _skill(skill="say", args={"text": "hello"}),
        True,
        "skill",
    )
    add("valid_twist", "a legal twist is accepted", _twist(), True, "twist")
    add(
        "valid_twist_zero",
        "a zero twist is accepted",
        _twist(twist={"linear_x_mps": 0.0, "angular_z_radps": 0.0}),
        True,
        "twist",
    )
    add(
        "valid_edge_speed",
        "exactly at the cap is inside it",
        _skill(args={"distance_m": 1.0, "speed_mps": 0.30}),
        True,
        "skill",
    )
    add(
        "valid_edge_turn",
        "exactly at the cap is inside it",
        _skill(skill="turn", args={"angle_deg": 180.0, "rate_dps": 60.0}),
        True,
        "skill",
    )

    # -- fill to the full corpus with randomised numerics ----------------
    handwritten = len(cases)
    while len(cases) < size:
        index = len(cases) - handwritten
        if index % 7 == 0:
            distance = round(rng.uniform(-1.0, 1.0), 4)
            speed = round(rng.uniform(0.01, 0.30), 4)
            add(
                f"rand_valid_drive_{index}",
                "a legal drive is accepted",
                _skill(args={"distance_m": distance, "speed_mps": speed}),
                True,
                "skill",
            )
        elif index % 3 == 0:
            distance = round(rng.choice([1, -1]) * rng.uniform(1.0001, 50.0), 4)
            add(
                f"rand_oob_distance_{index}",
                "drive distance outside 1.0 m",
                _skill(args={"distance_m": distance, "speed_mps": 0.15}),
                False,
                "skill",
            )
        elif index % 3 == 1:
            speed = round(rng.uniform(0.3001, 5.0), 4)
            add(
                f"rand_oob_speed_{index}",
                "drive speed above the cap",
                _skill(args={"distance_m": 0.2, "speed_mps": speed}),
                False,
                "skill",
            )
        else:
            linear = round(rng.choice([1, -1]) * rng.uniform(0.3001, 3.0), 4)
            angular = round(rng.choice([1, -1]) * rng.uniform(1.0471, 6.0), 4)
            add(
                f"rand_oob_twist_{index}",
                "twist outside its bounds",
                _twist(twist={"linear_x_mps": linear, "angular_z_radps": angular}),
                False,
                "twist",
            )
    return cases[:size]
