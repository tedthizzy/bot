# box — the GPU host

Serves one vision-language model behind an OpenAI-compatible endpoint and does
nothing else. It receives a world state, a skill list and sometimes one image,
and returns one proposed skill call.

The endpoint is unauthenticated, so it belongs on a trusted local network only.

`schema/` is generated from roverlib and is what constrains the model's output.
Constrained decoding guarantees the shape of the answer, never its intent, so
the bounds are enforced again on the host and again on the controller.
