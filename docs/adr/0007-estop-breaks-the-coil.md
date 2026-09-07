# 0007 — The button breaks the relay coil, and the undervoltage ladder is never suspended

**Revises** A22, A25 · **Overrules** spec v1 §8, the power_thermal note · **Status** Accepted

## Context

Spec v1 put the mushroom button in the motor rail. The power_thermal research
suspended the low-battery check above 2.0 A of motor current, on the grounds
that sag makes the terminal voltage unreadable under load.

## Decision

**The e-stop breaks the relay coil, not the motor rail.** Pi and logic tap
upstream of the contacts. A **1000 µF low-ESR cap and an SMBJ16A TVS** sit
across the MDD3A supply, downstream of the contacts, with a 2.2 Ω NTC inrush
limiter in series with the cap.

The battery ladder — 10.5 V warn, 9.9 V refuse and DISARM, 9.6 V disable — is
evaluated on a **sag-compensated open-circuit estimate**
`V_oc = vbat_mv + imotor_ma × R_pack_mΩ/1000`, debounced 10 s, **and is never
suspended.**

## Consequences

A 22 mm button is not rated to break 10 A of inductive DC. Breaking the coil
means the button switches a coil current, and the 30 A relay switches the motor
rail.

The cap and TVS are the half of this that is easy to omit and expensive to
learn: opening the contacts while the motors spin regenerates into a rail with
**no battery on it**, and the MDD3A's absolute maximum input is 16 V. Without
them the e-stop is the single action most likely to destroy the driver.

Sensing at coil node A, *above* the coil, is what makes "the human pressed it"
and "firmware disarmed" distinguishable. Below the coil they read identically,
and the BOM buys one NC contact block, so there is no second contact to read.

Suspending the undervoltage check above 2 A has no time bound, and >2 A is
normal on carpet. The check could therefore be disabled indefinitely by exactly
the load it exists to survive, handing protection to the BMS — whose trip also
kills the Pi buck. The INA226 supplies both terms of `V_oc`; `R_pack_mΩ` is
measured once at G2. I-7 tests it with a 60 s sustained 3 A load.
