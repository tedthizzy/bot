"""The box client against a stubbed transport.

Covers the six failures ARCHITECTURE names for this path -- malformed output,
schema-invalid output, a hallucinated skill, out-of-range integers, a timeout
and a late response -- plus the three properties of the request itself: A17's
order, A16's ``enable_thinking: false`` on every request with no ``guided_*``
anywhere, and a retry that appends without dropping the image.
"""

from __future__ import annotations

import asyncio
import json
import sys
from collections.abc import AsyncIterator, Mapping, Sequence
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.box import (  # noqa: E402
    FIND_SCHEMA,
    SKILL_CALL_FLAT_SCHEMA,
    SKILL_CALL_SCHEMA,
    BoxClient,
    BoxRejected,
    BoxTimeout,
    BoxUnavailable,
    ChatChunk,
    OpenAITransport,
)
from rover_contracts.config import BoxConfig  # noqa: E402
from rover_contracts.messages import Face, SkillName  # noqa: E402
from rover_contracts.observations import FindObservation  # noqa: E402
from rover_contracts.skills import CATALOG, SKILLS  # noqa: E402
from rover_contracts.worldstate import MotionBudget, WorldState  # noqa: E402

DRIVE = (
    '{"speech":"Heading over.","skill":"drive_for",'
    '"args":{"duration_ms":1000,"power_pct":15}}'
)
FOUND = (
    '{"kind":"find","present":true,"center_x_permille":610,"confidence":"medium",'
    '"description":"a red ceramic mug on a wooden table"}'
)
IMAGE = b"\xff\xd8\xff\xdbJPEG-ish\xff\xd9"

WORLD = WorldState(
    heading_deg=87,
    battery_pct=62,
    obstacle_ahead=False,
    front_range_cm=120,
    bumper=False,
    moving=False,
    power_cap_pct=20,
    allowed_skills=[skill.value for skill in SkillName],
    motion_budget_left=MotionBudget(seconds=9),
)


class StubTransport:
    """One scripted reply per call, with optional pacing."""

    def __init__(
        self,
        replies: Sequence[str],
        *,
        ttft_delay: float = 0.0,
        tail_delay: float = 0.0,
        raises: Exception | None = None,
        models_raise: bool = False,
    ) -> None:
        self._replies = list(replies)
        self._ttft = ttft_delay
        self._tail = tail_delay
        self._raises = raises
        self._models_raise = models_raise
        self.payloads: list[Mapping[str, Any]] = []
        self.closed = 0
        self.streams: list[AsyncIterator[ChatChunk]] = []

    def stream(self, payload: Mapping[str, Any]) -> AsyncIterator[ChatChunk]:
        # A strong reference, deliberately: an abandoned async generator is
        # otherwise closed by refcount GC the moment the caller's local goes
        # out of scope, which hides the leak this counter exists to catch.  In
        # production the leaked object is an httpx.Response holding a pooled
        # connection, and the pool has 100 slots.
        generator = self._stream(payload)
        self.streams.append(generator)
        return generator

    async def _stream(self, payload: Mapping[str, Any]) -> AsyncIterator[ChatChunk]:
        self.payloads.append(payload)
        if self._raises is not None:
            raise self._raises
        reply = self._replies.pop(0) if self._replies else ""
        try:
            await asyncio.sleep(self._ttft)
            yield ChatChunk(reply[:1])
            await asyncio.sleep(self._tail)
            yield ChatChunk(reply[1:], usage={"prompt_tokens": 700})
        finally:
            self.closed += 1

    async def models(self) -> tuple[str, ...]:
        if self._models_raise:
            raise ConnectionRefusedError("box is down")
        return ("rover-vlm",)


def client(transport: StubTransport, **config: Any) -> BoxClient:
    defaults = {"timeout_s": 2.0, "ttft_s": 1.0, "connect_s": 0.5}
    return BoxClient(BoxConfig(**(defaults | config)), transport)


def user_parts(payload: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    return [m for m in payload["messages"] if m["role"] == "user"][0]["content"]


# -- the request ------------------------------------------------------------


@pytest.mark.asyncio
async def test_the_prompt_is_built_in_a17s_order() -> None:
    transport = StubTransport([DRIVE])
    await client(transport).plan(
        world=WORLD, utterance="go to the table", image_jpeg=IMAGE
    )
    payload = transport.payloads[0]
    roles = [message["role"] for message in payload["messages"]]
    assert roles == ["system", "user"]
    parts = user_parts(payload)
    assert parts[0]["type"] == "image_url"
    assert parts[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert json.loads(parts[1]["text"])["heading_deg"] == 87
    assert parts[2]["text"] == "USER: go to the table"


@pytest.mark.asyncio
async def test_thinking_is_disabled_on_every_request_and_guided_is_never_used() -> None:
    transport = StubTransport([DRIVE, FOUND])
    box = client(transport)
    await box.plan(world=WORLD, utterance="go", image_jpeg=IMAGE)
    await box.observe("find", image_jpeg=IMAGE, target="red mug")
    for payload in transport.payloads:
        assert payload["extra_body"] == {
            "chat_template_kwargs": {"enable_thinking": False}
        }
        assert payload["stream"] is True
        assert payload["temperature"] == 0.0
        assert payload["top_p"] == 1.0
        assert "guided" not in json.dumps(payload)


@pytest.mark.asyncio
async def test_the_strict_profile_carries_the_oneof_schema() -> None:
    transport = StubTransport([DRIVE])
    await client(transport).plan(world=WORLD, utterance="go")
    schema = transport.payloads[0]["response_format"]["json_schema"]
    assert schema["strict"] is True
    assert schema["schema"] == SKILL_CALL_SCHEMA


@pytest.mark.asyncio
async def test_the_compat_profile_flattens_the_schema() -> None:
    transport = StubTransport([DRIVE])
    box = BoxClient(BoxConfig(), transport, compat=True)
    await box.plan(world=WORLD, utterance="go")
    schema = transport.payloads[0]["response_format"]["json_schema"]["schema"]
    assert schema == SKILL_CALL_FLAT_SCHEMA


@pytest.mark.asyncio
async def test_json_object_mode_asks_for_json_and_still_validates() -> None:
    transport = StubTransport([DRIVE])
    box = BoxClient(BoxConfig(structured_output_mode="json_object"), transport)
    plan = await box.plan(world=WORLD, utterance="go")
    assert transport.payloads[0]["response_format"] == {"type": "json_object"}
    assert plan.call.skill == "drive_for"


@pytest.mark.asyncio
async def test_a_planning_call_without_a_camera_still_works() -> None:
    transport = StubTransport([DRIVE])
    await client(transport).plan(world=WORLD, utterance="go")
    assert all(part["type"] != "image_url" for part in user_parts(transport.payloads[0]))


# -- the retry ---------------------------------------------------------------


@pytest.mark.asyncio
async def test_the_retry_appends_and_keeps_the_image() -> None:
    transport = StubTransport(["{not json", DRIVE])
    plan = await client(transport).plan(
        world=WORLD, utterance="go to the table", image_jpeg=IMAGE
    )
    assert plan.retried is True
    first, second = transport.payloads
    assert second["messages"][: len(first["messages"])] == first["messages"]
    assert [m["role"] for m in second["messages"]] == [
        "system",
        "user",
        "assistant",
        "user",
    ]
    assert second["messages"][-1]["content"].startswith("VALIDATOR: ")
    assert user_parts(second)[0]["type"] == "image_url"


@pytest.mark.asyncio
async def test_malformed_output_twice_is_a_rejection() -> None:
    transport = StubTransport(["{not json", "still not json"])
    with pytest.raises(BoxRejected) as caught:
        await client(transport).plan(world=WORLD, utterance="go")
    assert caught.value.reason == "malformed_json"
    assert len(transport.payloads) == 2


@pytest.mark.asyncio
async def test_a_hallucinated_skill_twice_is_a_rejection() -> None:
    line = '{"speech":"Off I go.","skill":"teleport","args":{}}'
    transport = StubTransport([line, line])
    with pytest.raises(BoxRejected) as caught:
        await client(transport).plan(world=WORLD, utterance="go")
    assert caught.value.reason == "unknown_skill"


@pytest.mark.asyncio
async def test_out_of_range_integers_are_told_to_the_model_then_rejected() -> None:
    line = (
        '{"speech":"","skill":"drive_for","args":{"duration_ms":4000,"power_pct":15}}'
    )
    transport = StubTransport([line, line])
    with pytest.raises(BoxRejected) as caught:
        await client(transport).plan(world=WORLD, utterance="go far")
    assert caught.value.reason == "out_of_range"
    complaint = transport.payloads[1]["messages"][-1]["content"]
    assert "duration_ms" in complaint


@pytest.mark.asyncio
async def test_a_schema_invalid_reply_is_repaired_by_the_retry() -> None:
    bad = (
        '{"speech":"","skill":"drive_for",'
        '"args":{"duration_ms":1000,"power_pct":15,"x":1}}'
    )
    transport = StubTransport([bad, DRIVE])
    plan = await client(transport).plan(world=WORLD, utterance="go")
    assert plan.retried is True
    assert plan.call.args.duration_ms == 1000


@pytest.mark.asyncio
async def test_an_empty_reply_is_a_rejection() -> None:
    transport = StubTransport(["", ""])
    with pytest.raises(BoxRejected):
        await client(transport).plan(world=WORLD, utterance="go")


# -- timeouts ----------------------------------------------------------------


@pytest.mark.asyncio
async def test_no_first_token_inside_ttft_is_a_timeout() -> None:
    transport = StubTransport([DRIVE], ttft_delay=0.3)
    with pytest.raises(BoxTimeout) as caught:
        await client(transport, ttft_s=0.05, timeout_s=1.0).plan(
            world=WORLD, utterance="go"
        )
    assert caught.value.reason == "ttft"
    # Closed on the deadline path, not left for the garbage collector: httpx
    # returns a pooled connection only when the response body is closed, so a
    # box that is up but slow otherwise leaks one connection per turn until
    # every later request blocks on an empty pool.
    assert transport.closed == 1


@pytest.mark.asyncio
async def test_a_late_response_is_abandoned_at_the_total_deadline() -> None:
    transport = StubTransport([DRIVE], tail_delay=0.4)
    with pytest.raises(BoxTimeout) as caught:
        await client(transport, ttft_s=0.2, timeout_s=0.1).plan(
            world=WORLD, utterance="go"
        )
    assert caught.value.reason == "total"
    assert transport.closed == 1


@pytest.mark.asyncio
async def test_a_transport_failure_is_unavailable_not_a_rejection() -> None:
    transport = StubTransport([DRIVE], raises=ConnectionRefusedError("no box"))
    with pytest.raises(BoxUnavailable):
        await client(transport).plan(world=WORLD, utterance="go")


@pytest.mark.asyncio
async def test_the_liveness_probe_answers_both_ways() -> None:
    assert await client(StubTransport([])).probe() is True
    assert await client(StubTransport([], models_raise=True)).probe() is False


# -- observations (A13) ------------------------------------------------------


@pytest.mark.asyncio
async def test_an_observation_is_validated_against_its_own_schema() -> None:
    reply = json.dumps(
        {
            "kind": "find",
            "present": True,
            "center_x_permille": 610,
            "confidence": "medium",
            "description": "a red ceramic mug on a wooden table",
        }
    )
    transport = StubTransport([reply])
    observation = await client(transport).observe(
        "find", image_jpeg=IMAGE, target="red mug"
    )
    assert isinstance(observation, FindObservation)
    assert observation.center_x_permille == 610
    sent = transport.payloads[0]["response_format"]["json_schema"]["schema"]
    assert sent == FIND_SCHEMA


@pytest.mark.asyncio
async def test_an_observation_that_invents_a_bearing_is_rejected() -> None:
    reply = json.dumps(
        {
            "kind": "find",
            "present": True,
            "center_x_permille": 610,
            "confidence": "medium",
            "description": "a mug",
            "bearing_deg": 12.5,
        }
    )
    with pytest.raises(BoxRejected):
        await client(StubTransport([reply])).observe(
            "find", image_jpeg=IMAGE, target="mug"
        )


@pytest.mark.asyncio
async def test_an_observation_can_never_carry_a_skill() -> None:
    for schema in (FIND_SCHEMA, json.loads(json.dumps(FIND_SCHEMA))):
        assert "skill" not in schema["properties"]
        assert schema["additionalProperties"] is False


# -- the schema cannot drift from the contracts ------------------------------


def test_every_schema_bound_matches_the_contract_model() -> None:
    """A11's integers, checked against the models robotd validates with."""
    for branch in SKILL_CALL_SCHEMA["oneOf"]:
        skill = branch["properties"]["skill"]["const"]
        model = SKILLS[skill].model_args
        assert model is not None
        contract = model.model_json_schema()
        args = branch["properties"]["args"]
        assert args["additionalProperties"] is False
        assert branch["required"] == ["speech", "skill", "args"]
        assert branch["properties"]["speech"]["maxLength"] == 160
        for name, spec in args["properties"].items():
            expected = contract["properties"][name]
            for key in ("minimum", "maximum", "maxLength"):
                if key in spec:
                    assert spec[key] == expected[key], f"{skill}.{name}.{key}"
            if spec.get("type") == "integer":
                assert expected["type"] == "integer"


def test_the_set_face_enum_matches_the_contract() -> None:
    branch = next(
        b
        for b in SKILL_CALL_SCHEMA["oneOf"]
        if b["properties"]["skill"]["const"] == "set_face"
    )
    assert branch["properties"]["args"]["properties"]["expr"]["enum"] == [
        face.value for face in Face
    ]


def test_the_schema_is_generated_from_the_catalog_in_order() -> None:
    """Seven branches, one per model skill, in the catalog's order, with every
    ``$ref`` inlined and no prose: a grammar back end sees a flat ``oneOf`` and
    nothing to resolve, and ``make schemas`` writes exactly this object."""
    consts = [b["properties"]["skill"]["const"] for b in SKILL_CALL_SCHEMA["oneOf"]]
    assert consts == [s.name for s in CATALOG if s.model_args is not None]
    assert len(consts) == 7 and set(consts) == {s.value for s in SkillName}
    rendered = json.dumps(SKILL_CALL_SCHEMA)
    assert "$ref" not in rendered and "$defs" not in rendered
    assert "title" not in rendered and "description" not in rendered
    for branch in SKILL_CALL_SCHEMA["oneOf"]:
        model = SKILLS[branch["properties"]["skill"]["const"]].model_args
        assert model is not None
        expected = model.model_json_schema()
        args = branch["properties"]["args"]
        assert args["required"] == list(expected.get("required", []))
        assert set(args["properties"]) == set(expected["properties"])


def test_the_drive_and_turn_branches_carry_the_open_loop_arguments() -> None:
    by_skill = {b["properties"]["skill"]["const"]: b for b in SKILL_CALL_SCHEMA["oneOf"]}
    drive = by_skill["drive_for"]["properties"]["args"]["properties"]
    assert drive == {
        "duration_ms": {"type": "integer", "minimum": 100, "maximum": 2000},
        "power_pct": {"type": "integer", "minimum": -30, "maximum": 30},
    }
    turn = by_skill["turn_to"]["properties"]["args"]["properties"]
    assert turn == {"heading_deg": {"type": "integer", "minimum": 0, "maximum": 359}}
    rendered = json.dumps(SKILL_CALL_SCHEMA)
    for retired in ("drive", "turn", "distance_cm", "speed_cms", "angle_deg", "rate_dps"):
        assert retired not in by_skill
        assert f'"{retired}"' not in rendered


# -- the transport releases its HTTP response -------------------------------


class _FakeStream:
    """An ``openai.AsyncStream``-shaped object that records being closed."""

    def __init__(self, chunks: Sequence[Any]) -> None:
        self._chunks = list(chunks)
        self.closed = False

    async def __aenter__(self) -> _FakeStream:
        return self

    async def __aexit__(self, *exc: object) -> None:
        self.closed = True

    def __aiter__(self) -> AsyncIterator[Any]:
        return self._iterate()

    async def _iterate(self) -> AsyncIterator[Any]:
        for chunk in self._chunks:
            yield chunk
        await asyncio.sleep(3600)  # a box that is up but slow


def _chunk(text: str) -> Any:
    delta = SimpleNamespace(content=text)
    return SimpleNamespace(choices=[SimpleNamespace(delta=delta)], usage=None)


@pytest.mark.asyncio
async def test_abandoning_the_stream_early_closes_the_http_response() -> None:
    """httpx returns a pooled connection only when the response body is closed,
    not on garbage collection.  AsyncOpenAI is built with max_retries=0 and the
    default 100-connection limit, so one leak per timed-out turn ends with every
    request blocking on an empty pool -- and it never recovers."""
    transport = OpenAITransport(BoxConfig())
    stream = _FakeStream([_chunk("{")])

    class _Completions:
        async def create(self, **_: Any) -> _FakeStream:
            return stream

    transport._client = SimpleNamespace(  # type: ignore[assignment]
        chat=SimpleNamespace(completions=_Completions())
    )
    chunks = transport.stream({"model": "rover-vlm", "messages": []}).__aiter__()
    first = await anext(chunks)
    assert first.text == "{"
    assert not stream.closed
    await chunks.aclose()
    assert stream.closed, "the HTTP response was left open"
