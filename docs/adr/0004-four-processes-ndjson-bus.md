# 0004 — Four Pi processes on an NDJSON Unix socket

**Revises** A9, A10 · **Overrules** the chair's three processes and WebSocket bus · **Status** Accepted

## Context

The chair proposed three processes and a WebSocket bus, with the camera inside
the brain process. A ZeroMQ variant with `CONFLATE` was also on the table.

## Decision

**Four** processes: `robotd`, `cam`, `brain`, `web`. IPC is **NDJSON over a Unix
socket**, mode 0660, group `rover`. Frames use a second socket, newest-only.
Only `rover-web` speaks WebSocket, and only to a browser.

## Consequences

The camera is its own process because picamera2 needs apt's `python3-libcamera`
and libcamera stalls are the commonest Pi camera hang. A stalled camera must not
stall the FSM, and its staleness must gate motion (I-23) — which it can only do
if the two are separable.

A filesystem permission is how "only robotd writes the port" becomes "only group
`rover` can command motion". A TCP WebSocket bus has no equivalent; it would
make the bus reachable by anything on the host. `SO_PEERCRED` at `hello` then
binds the declared `source` to the peer's uid, without which the source
allow-list is self-declared and not an isolation boundary at all.

ZeroMQ's `CONFLATE` silently drops messages. For a `stop` message that is
catastrophic, and I-22 says stop-class messages are never validated away.

The cost is a fourth unit to supervise and a third socket (`brain.sock`, 5.9)
for the conversational layer, because robotd is forbidden conversational state.
