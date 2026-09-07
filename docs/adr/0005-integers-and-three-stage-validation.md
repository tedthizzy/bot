# 0005 — Every bounded numeric the model emits is an integer

**Revises** A11, A12, A13 · **Overrules** spec v1 §3.5 and §4.3 · **Status** Accepted

## Context

Spec v1 let the model emit SI floats — `distance_m: 0.4`, `speed_mps: 0.15` —
and treated constrained decoding as a safety layer.

## Decision

The model emits **integers only**: centimetres, cm/s, degrees, deg/s, permille.
Validation is three stages: server-side grammar → pydantic v2 `strict`,
`extra="forbid"` → **a permission check at dispatch time**. Vision uses a
separate Observation schema that **cannot contain a skill**, and neither schema
carries a distance.

## Consequences

llama.cpp-class back ends constrain bounds on integers but silently skip float
bounds they cannot express in GBNF, so a float bound is a bound that may not
exist. This is `[INFERRED]` — no research note's verification section confirms
the equivalent xgrammar claim — and open item 16 makes it directly testable at
G1 against Ollama. If the bound does hold for floats, A11 becomes a preference.

Structured generation is itself an attack channel: DictAttack measures 94.3–99.5%
attack success across 13 models. The grammar therefore buys parse reliability and
**never** safety. The permission check runs after the network, not before, which
is one of three independent reasons a late response cannot move the robot.

The model never emits an angle. `center_x_permille` (0–1000) is the only geometry
it supplies; `bearing_deg` is computed on the Pi from a calibrated `hfov_deg`.
A model-supplied bearing would hand it a route straight to the executed turn
angle, and `box/schema/find.json` is `additionalProperties: false` so a back end
that invents one is rejected.
