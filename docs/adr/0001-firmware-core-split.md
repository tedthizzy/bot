# 0001 — The firmware core is freestanding C that the simulator links

**Revises** A2, A3 · **Overrules** spec v1's single `main.cpp` · **Status** Accepted

## Context

Directive 3 says nothing ships that cannot run on the MacBook first. Spec v1 put
the whole controller in one Arduino-style `main.cpp` against the ESP32 HAL. That
file cannot be compiled, tested or fuzzed anywhere but on the device, so every
safety invariant would have waited for hardware.

## Decision

Firmware splits into `firmware/core/` — freestanding C11 with **zero IDF
headers**: line codec, CRC-16, the seq/session/TTL/arm state machine, cap clamp,
asymmetric slew limiter, PI controller, fault classifier, ToF zone logic, cliff
logic, encoder plausibility, per-wheel slip/stall, per-channel I²t — and
`firmware/main/`, which is IDF glue. `mcu-sim` links **the same core** through a
ctypes shim to a first-order wheel plant over a pty.

State lives in an opaque `rover_core_t` with no file-scope storage, so ASan and
the shim can hold several instances at once, and the core owns no I/O.

The toolchain is pure ESP-IDF v5.5.5, not Arduino: MCPWM GPIO faults, PCNT
`accum_count`, GPTimer and Task-WDT panic reset are IDF-only.

## Consequences

**19 of 24 safety invariants go green before any hardware exists.** The five
that cannot are e-stop electrics (I-6), battery cutoffs (I-7), stall current
(I-10), the release-console check (I-18) and driver pin state through reset
(I-24) — all of them electrical.

The core builds under host clang with `-Werror -fsanitize=address,undefined` and
runs the same `tests/contract/serial_vectors.jsonl` the Python codec does, so
the two implementations of one protocol are pinned to the same bytes.

The cost is one more interface to keep honest: `firmware/core/include/rover_core.h`
is stated in ARCHITECTURE 4.1 rather than left to two agents to converge on.
