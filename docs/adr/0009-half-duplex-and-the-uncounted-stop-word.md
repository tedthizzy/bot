# 0009 — Half-duplex audio, and the spoken stop word counts for nothing

**Revises** A29, A31 · **Overrules** spec v1 §3.3 · **Status** Accepted

## Context

Spec v1 treated the spoken stop word as a stop authority and let text-to-speech
play over movement. Barge-in was assumed to follow from having hardware echo
cancellation in the microphone board.

## Decision

v1 audio is **half-duplex**. The three **counted** stop authorities are the
hardware e-stop, the web STOP button, and the MCU frame TTL. The spoken stop
word is a **best-effort fourth channel**: the recognizer stays live during
motion, G5 measures its latency, and **no invariant counts it.** The README says
so in the user-facing text.

**No speech plays while `mcu.state == ARMED_MOVING`.** The intent sentence is
spoken in `SPEAKING_INTENT` and the skill is not dispatched until it finishes.
Completion speech is a fixed table. The acknowledgement is a tone, not words.

## Consequences

The ReSpeaker Lite does have hardware echo cancellation. What is unverifiable is
whether it uses USB playback as its reference (open item 5) — so no echo
reference may be assumed, and silence during motion is what keeps the recognizer
live while the wheels turn.

Dispatch-after-intent costs ~0.8–1.2 s once per turn. That cost is what makes
half-duplex safe: with speech playing over `EXECUTING`, the recognizer is deaf
for the first 1–1.5 s of every drive, up to 0.45 m of motion.

The residual deaf window is the intent sentence itself, before any wheel turns.
G5 measures the stop word with a queued `say` in flight and **reports** the
number rather than gating on it. A tone cannot be wrong, and nothing says a
movement succeeded until the executor reports it.
