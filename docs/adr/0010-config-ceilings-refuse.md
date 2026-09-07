# 0010 — A value over a safety ceiling refuses startup; it is never clamped

**Revises** A33 · **Overrules** the chair's `robot.yaml` · **Status** Accepted

## Context

The chair proposed `robot.yaml` with environment overrides and no ceiling. The
natural implementation clamps an out-of-range value and carries on.

## Decision

Configuration is `config/robot.toml` via `tomllib`, overridable as
`ROVER__SECTION__KEY`. **The validator holds a compiled ceiling for every
`[limits]` and `[safety]` key and refuses to start above it — it never clamps.**
Every applied override logs at WARN, and **every** `[limits]` and `[safety]` key
is republished in `welcome.limits` / `welcome.safety`.

"Clamp" is reserved for the MCU's `V`-frame behaviour and for non-safety keys.

## Consequences

Without a ceiling, `ROVER__LIMITS__SPEED_MPS=3.0` takes effect silently. The MCU
bounds the damage — that is what I-4 is for — but the gates would then be
asserting against the file rather than against what runs.

Republishing in `welcome` is the other half. I-15 explicitly reads
`budget_path_m` and `budget_motion_s` from `welcome`, not from the file, so a
gate measures the running value.

A value **over** a safety ceiling is an operator error. A value *under* one is
not constrained by this mechanism, and for eleven keys the dangerous direction
is downward — see `docs/deviations.md` under `rover_contracts`. Seven of those
eleven are the `safety_hash` mirror keys, which preflight pins in both
directions by CRC-32 against the firmware's compiled constants; the rest are a
known gap with a recommended fix.

Credentials never appear in the file: a key ending `_key`, `_token` or `_secret`
holding a literal is a load error, and `[box] api_key_env` names a variable.
