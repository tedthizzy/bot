# tools — how this is tested without hardware

`mcu_sim/` runs the real firmware core against a simple wheel model and speaks
the real wire protocol over a pseudo terminal, so robotd cannot tell it from the
board. It can inject an obstacle, a stall, a flat battery, dropped frames and
added latency.

`fakebox/` is a deterministic stand-in for the model endpoint. It returns
schema-valid skill calls for a fixed set of phrases and, on request, misbehaves
on purpose: slow, timed out, malformed, schema-invalid, a hallucinated skill
name, or values outside the bounds.

`sim/` starts the whole system on one machine and tears it down cleanly.
