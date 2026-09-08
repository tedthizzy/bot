"""Seeded 500-line skill/twist corpus with accepted controls and named faults."""

from __future__ import annotations

import json
import random
from dataclasses import dataclass

CORPUS_SIZE = 500


@dataclass(frozen=True)
class Case:
    name: str
    line: str
    valid: bool
    kind: str


def build_corpus(seed=20260907, size=CORPUS_SIZE):
    rng = random.Random(seed)
    cases = []

    def add(name, body, valid=False):
        line = body if isinstance(body, str) else json.dumps(body)
        kind = body.get("type", "skill") if isinstance(body, dict) else "skill"
        cases.append(Case(name, line, valid, kind))

    def skill(**changes):
        return {
            "v": 1,
            "type": "skill",
            "source": "brain",
            "cmd_id": f"01J9ZC7K3QF2M8XR4V6T{len(cases):06d}",
            "seq": 1,
            "turn_id": "01J9ZC7K000000000000000000",
            "issued_mono_ns": 1,
            "goal_ttl_ms": 5000,
            "skill": "drive_for",
            "args": {"duration_s": 0.4, "power": 0.15},
            "trace": {"authorized_motion": True},
            **changes,
        }

    def twist(**changes):
        return {
            "v": 1,
            "type": "twist",
            "source": "teleop",
            "cmd_id": f"01J9ZC7K3QF2M8XR4V6T{len(cases):06d}",
            "seq": 1,
            "twist": {"lin": 0.15, "ang": 0.05},
            **changes,
        }

    for name, body in (
        ("not_json", "{not json"),
        ("empty", {}),
        ("unknown_type", {"v": 1, "type": "wobble"}),
        ("wrong_version", skill(v=2)),
    ):
        add(name, body)
    for name in ("teleport", "drive", "turn", "DRIVE_FOR", "", "stop"):
        add(f"unknown_skill_{name}", skill(skill=name))
    defaults = {
        "drive_for": {"duration_s": 0.4, "power": 0.15},
        "turn_to": {"heading_deg": 45.0, "timeout_s": 4.0},
    }
    for name, field, values in (
        ("drive_for", "duration_s", [0, -0.1, 2.0001]),
        ("drive_for", "power", [-0.3001, 0, 0.3001]),
        ("turn_to", "heading_deg", [-0.1, 360]),
        ("turn_to", "timeout_s", [0, 4.0001]),
        ("turn_to", "tolerance_deg", [1.999, 20.001]),
    ):
        for value in values:
            add(
                f"bound_{name}_{field}_{value}",
                skill(skill=name, args={**defaults[name], field: value}),
            )
    for field in ("lin", "ang"):
        for value in (-0.3001, 0.3001, float("nan"), float("inf"), -float("inf")):
            add(
                f"twist_{field}_{value}",
                twist(twist={"lin": 0.0, "ang": 0.0, field: value}),
            )
    for value in (float("nan"), float("inf"), -float("inf"), "0.4"):
        add(f"duration_type_{value}", skill(args={"duration_s": value, "power": 0.15}))
    for field in ("override", "admin", "__proto__", "power_max"):
        add(f"extra_{field}", skill(**{field: 1}))
        add(f"extra_args_{field}", skill(args={**defaults["drive_for"], field: 1}))
    for value in (0, 99, 5001, -1, "3000"):
        add(f"ttl_{value}", skill(goal_ttl_ms=value))
    for field, value in (
        ("seq", "1"),
        ("cmd_id", "bad"),
        ("cmd_id", "x" * 26),
        ("turn_id", None),
        ("source", "root"),
        ("args", {}),
    ):
        add(f"shape_{field}_{value}", skill(**{field: value}))
    add("wrong_args", skill(skill="turn_to"))
    for name, args in defaults.items():
        add(f"valid_{name}", skill(skill=name, args=args), True)
    add("valid_reverse", skill(args={"duration_s": 0.3, "power": -0.1}), True)
    add("valid_edge", skill(args={"duration_s": 2.0, "power": 0.3}), True)
    add(
        "valid_turn_edge",
        skill(skill="turn_to", args={"heading_deg": 359.0, "timeout_s": 4.0}),
        True,
    )
    add("valid_say", skill(skill="say", args={"text": "hello"}), True)
    add("valid_twist", twist(), True)
    add("valid_zero_twist", twist(twist={"lin": 0.0, "ang": 0.0}), True)
    while len(cases) < size:
        index = len(cases)
        if index % 7 == 0:
            add(
                f"valid_random_{index}",
                skill(
                    args={
                        "duration_s": rng.uniform(0.01, 2),
                        "power": rng.choice([-1, 1]) * rng.uniform(0.01, 0.3),
                    }
                ),
                True,
            )
        elif index % 3 == 0:
            add(
                f"duration_random_{index}",
                skill(args={"duration_s": rng.uniform(2.001, 50), "power": 0.15}),
            )
        elif index % 3 == 1:
            add(
                f"power_random_{index}",
                skill(args={"duration_s": 0.4, "power": rng.uniform(0.301, 5)}),
            )
        else:
            add(
                f"twist_random_{index}",
                twist(
                    twist={"lin": rng.uniform(0.301, 5), "ang": rng.uniform(-5, -0.301)}
                ),
            )
    return cases[:size]
