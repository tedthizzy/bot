# 0002 — ASCII lines with CRC-16, no COBS, no free-text log frame

**Revises** A5 · **Overrules** the position papers, spec v1 · **Status** Accepted

## Context

The position papers proposed COBS framing with a CRC-8. Both choices were argued
on bandwidth and simplicity. Neither argument survives the numbers.

## Decision

The wire format is a **versioned ASCII line protocol, integers and hex only,
CRC-16/CCITT-FALSE** (poly 0x1021, init 0xFFFF, no reflection, no final xor).

```
frame ::= "$" body "*" CRC "\n"
body  ::= TYPE "," VER "," SEQ "," SESS [ "," FIELD ]*
```

Every field is an integer or unsigned hex. No floats, no quoted strings, no
spaces, no text — no exceptions. The v1 `L` free-text log frame is **deleted**
and replaced by `E` event codes whose numbers are pinned by golden vectors.

## Consequences

A typical telemetry line is ~100 bytes; at 50 Hz that is 50 kbit/s, **5.4% of
921600**. COBS buys no bandwidth at 5% utilisation, and its one real benefit —
deterministic resync — is already there: `\n` cannot occur in an integer-only
body. Deleting the `L` frame is what keeps that true, which is precisely why it
had to go.

CRC-8 is Hamming distance 2 at this frame length: roughly one corrupted frame in
256 passes the check, and a frame that passes is a wheel command.

The cost is that a human cannot type a log line onto the link. That is the
point: A37 turns the debug console off in release builds for the same reason —
a writable text path into the motor controller is a second writer bypassing
session, seq and CRC.
