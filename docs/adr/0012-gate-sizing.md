# 0012 — 450 G1 requests, and eight `find` sweeps

**Revises** A14, A38 · **Overrules** the brief, spec v1's G1, ml_serving's 6 sweeps · **Status** Accepted

## Context

Spec v1's G1 was 10 utterances. An earlier draft proposed measuring variance
with a three-seed sweep. The vision research recommended six `find` sweeps.

## Decision

**G1 is 50 utterances × 3 world states × 3 fixture frames = 450 requests**, plus
a **20-row × 3-frame adversarial block** — 60 more. `find` is a stationary scan
of **≤8 sweeps** of 45°, ≤60 s, cancellable.

## Consequences

10 out of 10 cannot separate 90% from 99%. At `temperature 0.0, top_p 1.0`,
three *seeds* are three identical outputs, so a seed sweep measures 50 samples
while reporting 150. Varying the **input** is what makes 450 distinct trials.
A separate 10 × 3-seed sweep at `temperature 0.7` measures run-to-run variance
and is reported apart; production stays greedy.

Six sweeps of 45° span 225° of headings and leave a 52° blind wedge, which G5's
eight-sector test cannot pass. Eight sweeps at 60°/s is 6 s of motion and, with
the 83° field of view, covers 360°. One sweep is capture-then-turn, so
`max_sweeps` counts captures; a full sweep costs ≈8 × (1.8 s turn + ~1.5 s
vision call) ≈ 26 s against the 60 s cap.

The adversarial block is **reported with a confidence interval, never asserted
as zero.** Injection lands 27.0% on GPT-4o and 5.0% on Qwen3-VL-32B; a prompt
rule blocks 75–100% against that base rate, and the model this design runs has
no published number. A small clean sample would license a claim the evidence
cannot support. The Pi-side deterministic controls — budget, bounds,
`authorized_motion` — are asserted on every trial instead, and the model-reported
`text_in_frame` speed cap is reported beside the figure and counted as nothing,
because its only input is a field the model itself writes.
