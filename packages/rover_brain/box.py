"""The box client: one ``/v1/chat/completions`` call per turn, and its retry.

What this module is responsible for, in the order the architecture states it:

* A17 -- the request is built ``static system -> image -> world state ->
  utterance``, and the one retry **appends** to that list so the image stays
  and the retry is a prefix-cache hit on the same frame.
* A11 -- every bounded numeric in the schema is an integer, because llama.cpp
  class back ends constrain bounds on integers but not on floats and silently
  skip what they cannot express.
* A16 -- ``enable_thinking: false`` on **every** request, client-side, and
  ``guided_json`` (removed in vLLM 0.12.0) is never used.
* A12 -- the reply is validated strictly by :mod:`rover_brain.validate` before
  anything reads what it means, and the model is never trusted with an
  identifier: ``cmd_id``, ``turn_id``, ``seq`` and the expiry are the host's.
* A13 -- observations go through a separate schema that cannot contain a skill.

The transport is a protocol with two methods, so the whole client runs against
a stub in tests and against vLLM on the box.  ``openai`` is imported inside the
transport rather than at module scope: nothing else here needs it, and the box
being absent must not stop brain from importing.
"""

from __future__ import annotations

import asyncio
import contextlib
import os
from collections.abc import AsyncIterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Final, Literal, Protocol

from rover_contracts.config import BoxConfig
from rover_contracts.messages import Face, SkillCall, SkillName
from rover_contracts.observations import (
    Confidence,
    Hazard,
    Lighting,
    Observation,
    observation_adapter,
)
from rover_contracts.worldstate import WorldState

from rover_brain.prompt import (
    append_retry,
    build_messages,
    build_observation_messages,
    prompt_sha256,
)
from rover_brain.validate import ValidationFailure, validate_output

__all__ = [
    "FIND_SCHEMA",
    "SCENE_SCHEMA",
    "SKILL_CALL_FLAT_SCHEMA",
    "SKILL_CALL_SCHEMA",
    "BoxClient",
    "BoxError",
    "BoxRejected",
    "BoxTimeout",
    "BoxUnavailable",
    "ChatChunk",
    "OpenAITransport",
    "Plan",
    "Transport",
]

_SPEECH_MAX: Final = 160


# --------------------------------------------------------------------------
# Schemas (ARCHITECTURE 5.4 and 5.5)
# --------------------------------------------------------------------------


def _branch(
    skill: str, properties: Mapping[str, Any], required: Sequence[str]
) -> dict[str, Any]:
    """One ``oneOf`` branch, ``speech`` first so it decodes first and feeds
    sentence-streamed TTS."""
    return {
        "type": "object",
        "additionalProperties": False,
        "required": ["speech", "skill", "args"],
        "properties": {
            "speech": {"type": "string", "maxLength": _SPEECH_MAX},
            "skill": {"const": skill},
            "args": {
                "type": "object",
                "additionalProperties": False,
                "required": list(required),
                "properties": dict(properties),
            },
        },
    }


def _integer(minimum: int, maximum: int) -> dict[str, Any]:
    return {"type": "integer", "minimum": minimum, "maximum": maximum}


SKILL_CALL_SCHEMA: Final[dict[str, Any]] = {
    "oneOf": [
        _branch(
            SkillName.DRIVE,
            {"distance_cm": _integer(-100, 100), "speed_cms": _integer(5, 30)},
            ("distance_cm", "speed_cms"),
        ),
        _branch(
            SkillName.TURN,
            {"angle_deg": _integer(-180, 180), "rate_dps": _integer(5, 60)},
            ("angle_deg", "rate_dps"),
        ),
        _branch(SkillName.STOP, {}, ()),
        _branch(
            SkillName.SAY,
            {"text": {"type": "string", "maxLength": 240}},
            ("text",),
        ),
        _branch(SkillName.DESCRIBE_SCENE, {}, ()),
        _branch(
            SkillName.FIND,
            {
                "object": {"type": "string", "maxLength": 48},
                "max_sweeps": _integer(1, 8),
            },
            ("object", "max_sweeps"),
        ),
        _branch(
            SkillName.SET_FACE,
            {"expr": {"type": "string", "enum": [face.value for face in Face]}},
            ("expr",),
        ),
    ]
}
"""The strict profile: a top-level ``oneOf`` over seven branches, every object
``additionalProperties: false``, every bounded numeric an integer."""

SKILL_CALL_FLAT_SCHEMA: Final[dict[str, Any]] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["speech", "skill", "args"],
    "properties": {
        "speech": {"type": "string", "maxLength": _SPEECH_MAX},
        "skill": {"type": "string", "enum": [skill.value for skill in SkillName]},
        "args": {"type": "object"},
    },
}
"""The compat profile of 5.4, for a back end that chokes on ``oneOf``: flat,
with open ``args`` and the discrimination left to pydantic.  ``box_probe``
selects it; nothing is relaxed by using it, because stage two rejects exactly
what the grammar would have."""

FIND_SCHEMA: Final[dict[str, Any]] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["kind", "present", "center_x_permille", "confidence", "description"],
    "properties": {
        "kind": {"const": "find"},
        "present": {"type": "boolean"},
        "center_x_permille": _integer(0, 1000),
        "confidence": {"type": "string", "enum": [c.value for c in Confidence]},
        "description": {"type": "string", "maxLength": 240},
    },
}
"""A13/5.5: no skill, no distance, and no ``bearing_deg`` -- the bearing is
computed on the Pi from a calibrated ``hfov_deg``, and
``additionalProperties: false`` is what rejects a back end that invents one."""

SCENE_SCHEMA: Final[dict[str, Any]] = {
    "type": "object",
    "additionalProperties": False,
    "required": ["kind", "description", "labels", "lighting", "hazards"],
    "properties": {
        "kind": {"const": "scene"},
        "description": {"type": "string", "maxLength": 240},
        "labels": {
            "type": "array",
            "maxItems": 12,
            "items": {"type": "string", "maxLength": 32},
        },
        "lighting": {"type": "string", "enum": [light.value for light in Lighting]},
        "hazards": {
            "type": "array",
            "maxItems": 6,
            "items": {"type": "string", "enum": [h.value for h in Hazard]},
        },
    },
}


# --------------------------------------------------------------------------
# Transport
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ChatChunk:
    """One streamed delta.  ``usage`` arrives on the final chunk, if at all."""

    text: str = ""
    usage: Mapping[str, Any] | None = None


class Transport(Protocol):
    """Everything this client does to the network, in two methods."""

    def stream(self, payload: Mapping[str, Any]) -> AsyncIterator[ChatChunk]:
        """Yield the reply as it decodes."""

    async def models(self) -> tuple[str, ...]:
        """``GET /v1/models`` -- the liveness probe and the probe's first call."""


class OpenAITransport:
    """vLLM over the OpenAI-compatible API.

    ``openai`` and ``httpx`` are imported here, not at module scope: brain
    imports and its tests run with the box absent.  ``max_retries=0`` because
    this design owns its retry policy (5.7) and a client-side retry would
    silently double a turn's latency budget.
    """

    def __init__(self, config: BoxConfig) -> None:
        import httpx
        from openai import AsyncOpenAI

        # The key is named by [box] api_key_env and never written to a file;
        # vLLM ignores it, but the client requires a non-empty string.
        key = os.environ.get(config.api_key_env) or "no-key"
        self._client = AsyncOpenAI(
            base_url=config.url,
            api_key=key,
            max_retries=0,
            timeout=httpx.Timeout(config.timeout_s, connect=config.connect_s),
        )

    async def stream(self, payload: Mapping[str, Any]) -> AsyncIterator[ChatChunk]:
        body = dict(payload)
        extra = body.pop("extra_body", None)
        # `async with` on the stream, so a GeneratorExit thrown in when the
        # caller closes the iterator early also closes the HTTP response and
        # returns its connection to the pool.
        async with await self._client.chat.completions.create(
            **body, extra_body=extra, stream_options={"include_usage": True}
        ) as stream:
            async for chunk in stream:
                text = ""
                if chunk.choices and chunk.choices[0].delta.content:
                    text = chunk.choices[0].delta.content
                usage = (
                    chunk.usage.model_dump() if getattr(chunk, "usage", None) else None
                )
                if text or usage:
                    yield ChatChunk(text, usage)

    async def models(self) -> tuple[str, ...]:
        page = await self._client.models.list()
        return tuple(model.id for model in page.data)


# --------------------------------------------------------------------------
# Failures
# --------------------------------------------------------------------------


class BoxError(RuntimeError):
    """Anything that stops a turn from producing a validated skill."""

    def __init__(self, reason: str, detail: str) -> None:
        super().__init__(detail)
        self.reason = reason
        self.detail = detail


class BoxTimeout(BoxError):
    """T3: no connection, no first token in 2.5 s, or no reply in 8 s."""


class BoxRejected(BoxError):
    """Two outputs in a row this host will not dispatch."""


class BoxUnavailable(BoxError):
    """The transport itself failed -- refused, reset, 500."""


@dataclass(frozen=True, slots=True)
class Plan:
    """One validated answer, with the provenance G1 asserts per trial."""

    call: SkillCall
    raw: str
    retried: bool
    prompt_sha256: str
    usage: Mapping[str, Any] | None = None


@dataclass(frozen=True, slots=True)
class _Reply:
    text: str
    usage: Mapping[str, Any] | None


# --------------------------------------------------------------------------
# Client
# --------------------------------------------------------------------------


class BoxClient:
    """One box, one model, one schema profile."""

    def __init__(
        self,
        config: BoxConfig,
        transport: Transport | None = None,
        *,
        compat: bool = False,
    ) -> None:
        self._config = config
        self._transport = transport if transport is not None else OpenAITransport(config)
        self._compat = compat

    @property
    def compat(self) -> bool:
        """True when the flat profile is in force (the probe said no ``oneOf``)."""
        return self._compat

    async def plan(
        self,
        *,
        world: WorldState,
        utterance: str,
        image_jpeg: bytes | None = None,
    ) -> Plan:
        """One turn: build, stream, validate, and retry once by appending."""
        messages = build_messages(world, utterance, image_jpeg=image_jpeg)
        digest = prompt_sha256(messages)
        schema = SKILL_CALL_FLAT_SCHEMA if self._compat else SKILL_CALL_SCHEMA
        reply = await self._collect(self._payload(messages, "skill_call", schema))
        try:
            return Plan(
                validate_output(reply.text), reply.text, False, digest, reply.usage
            )
        except ValidationFailure as first:
            # The image stays; the retry only appends (A17).
            second_try = append_retry(messages, reply.text, first.detail)
            again = await self._collect(self._payload(second_try, "skill_call", schema))
            try:
                return Plan(
                    validate_output(again.text), again.text, True, digest, again.usage
                )
            except ValidationFailure as second:
                raise BoxRejected(second.reason, second.detail) from second

    async def observe(
        self,
        kind: Literal["find", "scene"],
        *,
        image_jpeg: bytes,
        target: str | None = None,
    ) -> Observation:
        """One vision call.  The schema cannot carry a skill (A13)."""
        messages = build_observation_messages(kind, image_jpeg=image_jpeg, target=target)
        schema = FIND_SCHEMA if kind == "find" else SCENE_SCHEMA
        payload = self._payload(messages, f"{kind}_observation", schema)
        reply = await self._collect(payload)
        try:
            return observation_adapter.validate_json(reply.text)
        except ValueError as exc:
            raise BoxRejected("bad_observation", str(exc)) from exc

    async def probe(self) -> bool:
        """T3's liveness probe: cheap, bounded, and run only while a
        brain-owned motion command is active, so it costs nothing at idle."""
        try:
            async with asyncio.timeout(self._config.health_probe_s):
                await self._transport.models()
        except Exception:  # a timeout, a refusal and a 500 all mean "not there"
            return False
        return True

    def _payload(
        self, messages: Sequence[Mapping[str, Any]], name: str, schema: Mapping[str, Any]
    ) -> dict[str, Any]:
        """The request of ARCHITECTURE 5.7.

        ``enable_thinking: false`` rides on every request rather than in a
        server default that a redeploy would lose (A16), and no ``guided_*``
        parameter appears anywhere: it was removed in vLLM 0.12.0.
        """
        response_format: dict[str, Any]
        if self._config.structured_output_mode == "json_schema":
            response_format = {
                "type": "json_schema",
                "json_schema": {"name": name, "strict": True, "schema": dict(schema)},
            }
        else:
            response_format = {"type": "json_object"}
        return {
            "model": self._config.model,
            "messages": list(messages),
            "response_format": response_format,
            "extra_body": {"chat_template_kwargs": {"enable_thinking": False}},
            "max_tokens": self._config.max_tokens,
            "temperature": self._config.temperature,
            "top_p": 1.0,
            "stream": True,
        }

    async def _collect(self, payload: Mapping[str, Any]) -> _Reply:
        """Stream one reply under both deadlines: 2.5 s to the first token,
        8 s in total.  A late response is cancelled here; that it cannot move
        the robot is I-11's job, not this timeout's.

        The iterator is closed on every exit path.  Abandoning it mid-``async
        for`` leaves the underlying HTTP response open, and httpx returns a
        pooled connection only when the body is closed -- so a box that is up
        but slow leaks one connection per timed-out turn until the 100-slot pool
        is gone and every later request blocks until ``connect_s``.
        """
        parts: list[str] = []
        usage: Mapping[str, Any] | None = None
        chunks: AsyncIterator[ChatChunk] | None = None
        try:
            async with asyncio.timeout(self._config.timeout_s):
                chunks = self._transport.stream(payload).__aiter__()
                try:
                    async with asyncio.timeout(self._config.ttft_s):
                        first = await anext(chunks)
                except TimeoutError as exc:
                    raise BoxTimeout(
                        "ttft", f"no first token in {self._config.ttft_s} s"
                    ) from exc
                except StopAsyncIteration as exc:
                    raise BoxRejected("empty", "the box returned nothing") from exc
                parts.append(first.text)
                usage = first.usage or usage
                async for chunk in chunks:
                    parts.append(chunk.text)
                    usage = chunk.usage or usage
        except TimeoutError as exc:
            raise BoxTimeout(
                "total", f"no complete reply in {self._config.timeout_s} s"
            ) from exc
        except (BoxError, asyncio.CancelledError):
            raise
        except Exception as exc:  # transport-level: refused, reset, 500
            raise BoxUnavailable("transport", f"{type(exc).__name__}: {exc}") from exc
        finally:
            aclose = getattr(chunks, "aclose", None)
            if aclose is not None:
                with contextlib.suppress(Exception):
                    await aclose()
        return _Reply("".join(parts), usage)
