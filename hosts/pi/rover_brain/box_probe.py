"""``python -m rover_brain.box_probe`` -- deploy step 2, five calls, 30 seconds.

ARCHITECTURE 4.6: brain probes the box at boot and writes ``box_caps.json``
(``supports_json_schema``, ``supports_oneof``, ``supports_images``,
``image_tokens_observed``, ``ttft_cold_ms``, ``ttft_warm_ms``,
``decode_tok_s``).  It answers open item 8 -- whether ``response_format:
json_schema`` and ``oneOf`` hold on this back end -- in half a minute, and if
``supports_oneof`` is false the compat profile of 5.4 is what runs.

The five calls are, in order: the model list, a flat schema, the ``oneOf``
schema, an image cold, and the same image again warm.  The last two are the
same request twice on purpose: prefix caching hashes multimodal input, so the
warm TTFT is the number A17's retry rests on.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import sys
import time
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Final

from rover_contracts.config import BoxConfig

from rover_brain.box import (
    SKILL_CALL_FLAT_SCHEMA,
    SKILL_CALL_SCHEMA,
    OpenAITransport,
    Transport,
)

__all__ = ["PROBE_IMAGE_JPEG", "BoxCaps", "load_caps", "main", "probe"]

_DEFAULT_IMAGE: Final = Path("tests/fixtures/frames/f_kitchen.jpg")
"""Where the committed 640x480 fixtures actually live.  A18's ~300 image
tokens are the point of ``image_tokens_observed``, and a 1x1 cannot show
them, so falling back to the built-in pixel is a WARN and not a silent
substitution."""
_DEFAULT_OUT: Final = Path("box_caps.json")
_PROMPT: Final = "Reply with the stop skill and an empty speech string."

PROBE_IMAGE_JPEG: Final = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0a"
    "HBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAARCAABAAEDASIAAhEBAxEB/8QAHwAA"
    "AQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIh"
    "MUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpT"
    "VFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5"
    "usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAA"
    "AAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEI"
    "FEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVm"
    "Z2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK"
    "0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD3+iiigD//2Q=="
)
"""A 1x1 JPEG, used only when no real frame is given.  ``supports_images`` is
what it answers; ``image_tokens_observed`` from a 1x1 is not A18's 300, and the
caps file records which image produced the number."""


@dataclass(frozen=True, slots=True)
class BoxCaps:
    """What deploy step 2 writes and step 3 reads."""

    url: str
    model: str
    supports_json_schema: bool
    supports_oneof: bool
    supports_images: bool
    image_tokens_observed: int
    ttft_cold_ms: int
    ttft_warm_ms: int
    decode_tok_s: float
    image_source: str
    models: tuple[str, ...] = ()

    def write(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n", "utf-8")


def load_caps(path: str | Path = _DEFAULT_OUT) -> BoxCaps | None:
    """Read a caps file written by an earlier probe, or ``None``."""
    target = Path(path)
    if not target.exists():
        return None
    data = json.loads(target.read_text("utf-8"))
    data["models"] = tuple(data.get("models", ()))
    return BoxCaps(**data)


@dataclass(frozen=True, slots=True)
class _Timed:
    text: str
    ttft_ms: int
    total_ms: int
    usage: Mapping[str, Any] | None
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error


async def _timed(transport: Transport, payload: Mapping[str, Any]) -> _Timed:
    started = time.monotonic()
    first_at = 0.0
    parts: list[str] = []
    usage: Mapping[str, Any] | None = None
    try:
        async for chunk in transport.stream(payload):
            if not first_at and chunk.text:
                first_at = time.monotonic()
            parts.append(chunk.text)
            usage = chunk.usage or usage
    except Exception as exc:  # a probe reports the failure, it does not raise
        return _Timed("", 0, 0, None, f"{type(exc).__name__}: {exc}")
    now = time.monotonic()
    return _Timed(
        "".join(parts),
        int(((first_at or now) - started) * 1000),
        int((now - started) * 1000),
        usage,
    )


def _payload(
    config: BoxConfig,
    *,
    schema: Mapping[str, Any] | None,
    image_jpeg: bytes | None = None,
) -> dict[str, Any]:
    content: list[dict[str, Any]] = []
    if image_jpeg is not None:
        encoded = base64.b64encode(image_jpeg).decode("ascii")
        content.append(
            {
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{encoded}"},
            }
        )
    content.append({"type": "text", "text": _PROMPT})
    payload: dict[str, Any] = {
        "model": config.model,
        "messages": [{"role": "user", "content": content}],
        # A16 applies to every request, probe included.
        "extra_body": {"chat_template_kwargs": {"enable_thinking": False}},
        "max_tokens": config.max_tokens,
        "temperature": 0.0,
        "top_p": 1.0,
        "stream": True,
    }
    if schema is not None:
        payload["response_format"] = {
            "type": "json_schema",
            "json_schema": {"name": "skill_call", "strict": True, "schema": dict(schema)},
        }
    return payload


def _prompt_tokens(reply: _Timed) -> int:
    return int((reply.usage or {}).get("prompt_tokens", 0))


async def probe(
    config: BoxConfig,
    transport: Transport | None = None,
    *,
    image_jpeg: bytes | None = None,
    image_source: str = "builtin-1x1",
) -> BoxCaps:
    """Run the five calls and return the capabilities they establish."""
    client = transport if transport is not None else OpenAITransport(config)
    image = image_jpeg if image_jpeg is not None else PROBE_IMAGE_JPEG
    try:
        models = await client.models()
    except Exception:
        models = ()
    flat = await _timed(client, _payload(config, schema=SKILL_CALL_FLAT_SCHEMA))
    oneof = await _timed(client, _payload(config, schema=SKILL_CALL_SCHEMA))
    cold = await _timed(
        client, _payload(config, schema=SKILL_CALL_SCHEMA, image_jpeg=image)
    )
    warm = await _timed(
        client, _payload(config, schema=SKILL_CALL_SCHEMA, image_jpeg=image)
    )
    completion = int((warm.usage or {}).get("completion_tokens", 0))
    decode_ms = max(1, warm.total_ms - warm.ttft_ms)
    return BoxCaps(
        url=config.url,
        model=config.model,
        supports_json_schema=flat.ok,
        supports_oneof=oneof.ok,
        supports_images=cold.ok,
        image_tokens_observed=max(0, _prompt_tokens(cold) - _prompt_tokens(oneof)),
        ttft_cold_ms=cold.ttft_ms,
        ttft_warm_ms=warm.ttft_ms,
        decode_tok_s=round(completion * 1000 / decode_ms, 1),
        image_source=image_source,
        models=models,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rover_brain.box_probe", description=__doc__)
    parser.add_argument("--url", default=BoxConfig().url, help="[box] url")
    parser.add_argument("--model", default=BoxConfig().model, help="served-model-name")
    parser.add_argument("--image", type=Path, default=_DEFAULT_IMAGE)
    parser.add_argument("--out", type=Path, default=_DEFAULT_OUT)
    args = parser.parse_args(argv)

    config = BoxConfig(url=args.url, model=args.model)
    if args.image.exists():
        image, source = args.image.read_bytes(), str(args.image)
    else:
        print(
            f"WARN {args.image} is missing; probing with the built-in 1x1 JPEG. "
            "image_tokens_observed will not be A18's ~300 and proves nothing "
            "about a 640x480 frame.",
            file=sys.stderr,
        )
        image, source = PROBE_IMAGE_JPEG, "builtin-1x1"
    caps = asyncio.run(probe(config, image_jpeg=image, image_source=source))
    caps.write(args.out)
    print(json.dumps(asdict(caps), indent=2))
    if not caps.supports_oneof:
        print(
            '\nsupports_oneof is false: set [box] structured_output_mode="json_object" '
            "or run brain with the compat profile (ARCHITECTURE 5.4)."
        )
    return 0 if caps.supports_json_schema else 1


if __name__ == "__main__":  # pragma: no cover - the entry point deploy step 2 runs
    raise SystemExit(main())
