# 0003 — Body velocity on the wire; the host owns the profile

**Revises** A6, A7 · **Overrules** spec v1 §4.1, the orchestrator note · **Status** Accepted

## Context

Spec v1 sent two wheel speeds. The orchestrator note wanted the MCU to execute
whole motions — "drive 1 m" — so a host stall could not interrupt a movement
mid-way.

## Decision

The link carries **body velocity**: `v_mm_s`, `w_mrad_s`, REP-103 signs
(+x forward, +z up = CCW). Wheel geometry lives in firmware. **robotd executes
motion profiles; the MCU executes velocity only.**

## Consequences

I-5 needs the MCU to block *forward* while still allowing reverse and rotation.
Two wheel speeds cannot express that distinction: a controller given
`left=+100, right=-100` cannot tell a spin from a drive without reconstructing
the body frame, which is the firmware's job anyway.

If the MCU could finish a 1 m drive alone, then killing the Pi mid-drive would
not stop the robot. Every stop authority in the design assumes the wheels stop
when the setpoint stream stops.

The cost is that robotd streams `V` at 20 Hz and a jitter spike on a loaded Pi 4
is now a control-path concern (open item 3, measured at G3a). The mitigations
are named — `CPUAffinity`, `Nice`, Wi-Fi power save, and a 400 ms TTL the
protocol already accepts — and the `Setpoint{valid_until}` ownership rule is
what makes moving the writer to its own thread safe.
