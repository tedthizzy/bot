# Decision records

`ARCHITECTURE.md` is **frozen at v1.** Every number, field name, threshold and
file path in it is a contract that nine components are being written against
concurrently. It is not edited in place.

**The governance rule.** A change to the architecture happens through a new
decision record in this directory, and only through one. A record that proposes
a change must:

1. name the ARCHITECTURE decision id it revises (`A1`–`A38`) or the invariant it
   touches (`I-1`–`I-24`);
2. cite **gate evidence** — a result in `logs/gates/` with a measurement in it,
   not an argument. `docs/gates.md` says what a gate result looks like;
3. state what breaks. If a component was written against the old value, say
   which one and what its test asserts.

A record with no gate evidence behind it is a proposal, not a decision, and its
status stays `Proposed`. Several of the open items in ARCHITECTURE's closing
section are waiting for exactly this: open item 1 (braking deceleration) can
only be settled by G2, and A21's stated fallback is already written down so that
settling it is a status change here rather than an architecture rewrite.

Deviations found while *implementing* a decision — "the document says X, the
code does X, and X looks wrong" — go in `docs/deviations.md` instead. That file
is a log of implementation notes. This directory is for changing the contract.

## The records

These twelve document the reversals `ARCHITECTURE.md` made against its own
inputs: the design brief, spec v1, critique v2, recommendations v3, and the
thirteen research notes. Each one overruled something a prior document said, and
each is a place a reader will otherwise ask "why not the obvious thing?".

| # | subject | revises | overrules |
|---|---|---|---|
| [0001](0001-firmware-core-split.md) | freestanding C core the simulator links | A2, A3 | spec v1's single `main.cpp` |
| [0002](0002-ascii-line-protocol.md) | ASCII lines with CRC-16, no COBS, no log frame | A5 | the position papers, spec v1 |
| [0003](0003-body-velocity-host-profiles.md) | body velocity on the wire; the host owns profiles | A6, A7 | spec v1 §4.1, orchestrator |
| [0004](0004-four-processes-ndjson-bus.md) | four Pi processes on an NDJSON Unix socket | A9, A10 | the chair's three processes and WebSocket bus |
| [0005](0005-integers-and-three-stage-validation.md) | every model numeric is an integer | A11, A12, A13 | spec v1 §3.5, §4.3 |
| [0006](0006-tof-thresholds-and-the-angular-clamp.md) | 250/600 mm, an angular clamp, a distance-triggered fallback | A21 | spec v1 §3.1 |
| [0007](0007-estop-breaks-the-coil.md) | the button breaks the relay coil, not the motor rail | A22, A25 | spec v1 §8, power_thermal |
| [0008](0008-fifty-to-one-and-four-current-layers.md) | 50:1 gearing, four current-limit layers | A23, A24 | esp32_firmware rec 8 |
| [0009](0009-half-duplex-and-the-uncounted-stop-word.md) | half-duplex; the spoken stop word counts for nothing | A29, A31 | spec v1 §3.3 |
| [0010](0010-config-ceilings-refuse.md) | a value over a safety ceiling refuses startup | A33 | the chair's `robot.yaml` |
| [0011](0011-no-docker-on-the-pi.md) | Docker on the Mac, never on the Pi | A36 | — |
| [0012](0012-gate-sizing.md) | 450 G1 requests, 8 `find` sweeps | A14, A38 | the brief, ml_serving's 6 |
