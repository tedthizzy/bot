# 0006 — 250/600 mm, an angular clamp, and a fallback triggered on distance

**Revises** A21 · **Overrules** spec v1 §3.1 · **Status** Accepted, fallback Proposed pending G2

## Context

Spec v1 set a forward stop threshold and a slow zone but clamped only linear
speed, and justified the margin with a 3.1× figure.

## Decision

Stop at **250 mm**, slow zone **600 mm**, `v ≤ (d − 250 mm)/1.0 s` capped at
150 mm/s, **plus `|w| ≤ 500 mrad/s` inside the same zone**, two samples to stop.

**Stated fallback:** if the measured obstacle-to-halt distance at 0.30 m/s
exceeds 150 mm at G2, move to stop 350 mm, slow zone 800 mm, or hard-cap
250 mm/s.

## Consequences

The worst case at 0.30 m/s is **≈129 mm**, from the design's own four numbers:
30 ms inter-measurement + 20 ms for the first completed reading + ≤20 ms for the
50 Hz sensor task + 30 + 20 ms for the confirming sample + ≤10 ms of control
quantisation = 39 mm of travel, plus braking of 45 mm at 1.0 m/s² or 75 mm at
the design's own 0.6 m/s² floor. Margin on 250 mm is **1.9–2.2×**, not 3.1×.

Half the error budget is latency, which a deceleration measurement cannot see.
That is why the fallback triggers on measured **distance** and why G2 records
braking deceleration *first*: I-1's own time criteria derive from it.

The angular clamp is the half of the recommendation spec v1 dropped. A 0.2 m
body radius at 1.2 rad/s sweeps a corner at 0.24 m/s into an arc that no forward
cone covers, so clamping only `v` leaves the robot free to swing into what it
just stopped for.

Obstacle-class bits also clamp **reverse to −150 mm/s**, because reverse is
unsensed. Escalation to the latched class is **not timed**: a 2 s rule always
fires — stopping at the threshold leaves the cause present by definition — so
every wall approach would latch and need a human.
