# 0008 — 50:1 gearing, and four independent current-limit layers

**Revises** A23, A24 · **Overrules** esp32_firmware rec 8 (90:1) · **Status** Accepted

## Context

The firmware research recommended 90:1 JGB37-520 motors on the strength of a
170 rpm no-load figure. Its own verification section refuted that number: the
90:1 variant is **107 rpm**, not 170. The Cytron MDD3A was chosen without noting
what it does not have.

## Decision

Drive **2× JGB37-520 50:1** (192 rpm at 12 V, 2200 CPR) on an MDD3A, 3S Li-ion.

**Four current-limit layers**, not one:

1. INA226 `ALERT` at 6.0 A → MCPWM one-shot brake.
2. **Per-channel software I²t**: `∫ max(0, I_ch − 3.0 A)² dt ≥ 6 A²·s`, with
   `I_ch` from the motor model `(duty_ch × V_bat − k_e × ω_ch) / R_motor`,
   rescaled so the pair sums to the measured `I_total`, and both channels forced
   to 0 when `|duty_L| + |duty_R| < 0.01`.
3. A **per-wheel** slip/stall detector: `|duty_ch| > 40%` with
   `|v_wheel_meas| < 5%` of `|v_wheel_cmd|` for 200 ms, or `< 40%` for 500 ms.
4. NTC on the heatsink → `DRIVER_HOT`.

## Consequences

At 0.3 m/s the 50:1 wheels turn 64 rpm, a third of the 192 rpm no-load speed.
3S is also what the later merge track's 12 V servos need.

The MDD3A has **no over-current protection and no thermal protection**, and is
rated **per channel**. One INA226 on the pack cannot see a 4.0 A / 0.5 A split:
that is 4.5 A total, under the 6 A `ALERT`, while one channel cooks.

A duty-proportional split cannot see it either — a jammed wheel draws far more
current at the *same* duty as its free partner, so a duty ratio reports the pair
as equal. The motor model inverts correctly because a jam is low ω at high duty.
`k_e` and `R_motor` are measured once at G2 beside `R_pack_mΩ`.

The stall detector is **per wheel** because with one wheel held and one free the
body-frame `v_meas` is half of `v_cmd`, which never trips a 5% test. 200 ms so
I-10's 250 ms holds end to end after one 10 ms control period.
