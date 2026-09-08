"""fakebox tells the truth on the normal path and lies exactly as asked.

The normal path is validated against the *exported* schema -- the pydantic
models in ``rover_contracts`` -- rather than against fakebox's own idea of one,
because a fake that generates its answers through the validator's classes can
never fail validation and would make this file a tautology.
"""

from __future__ import annotations

import http.client
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import Counter
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.prompt import build_observation_messages  # noqa: E402
from rover_contracts import (  # noqa: E402
    SkillName,
    WorldState,
    observation_adapter,
    skill_call_adapter,
)
from rover_devtools.fakebox import (  # noqa: E402
    CASES,
    CATEGORIES,
    FAULTS,
    INJECTED_INSTRUCTION,
    Case,
    FakeBox,
    content_for,
    heading_of,
    make_server,
    observe,
    parse_fault,
    plan,
    prompt_order,
)

HEADING = 87

WORLD_STATE = {
    "heading_deg": HEADING,
    "battery_pct": 62,
    "obstacle_ahead": False,
    "front_range_cm": 120,
    "bumper": False,
    "moving": False,
    "power_cap_pct": 30,
    "last_result": "done",
    "last_scene": "a kitchen",
    "recently_seen": [],
    "allowed_skills": ["drive_for", "turn_to", "stop", "say"],
    "motion_budget_left": {"seconds": 9},
}


def chat_request(
    text: str,
    *,
    image: str | None = "data:image/jpeg;base64,AAAA",
    heading: int | None = HEADING,
    schema: str = "skill_call",
    stream: bool = False,
    retry: bool = False,
) -> dict[str, Any]:
    """ARCHITECTURE 5.7's request: system, image, world state, utterance.
    ``heading=None`` leaves the world state out entirely."""
    parts: list[dict[str, Any]] = []
    if image is not None:
        parts.append({"type": "image_url", "image_url": {"url": image}})
    if heading is not None:
        world = {**WORLD_STATE, "heading_deg": heading}
        parts.append({"type": "text", "text": json.dumps(world)})
    parts.append({"type": "text", "text": f"USER: {text}"})
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": "You are a rover."},
        {"role": "user", "content": parts},
    ]
    if retry:
        messages.append({"role": "assistant", "content": '{"skill":"drive_for"}'})
        messages.append(
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "VALIDATOR: bad_args. Emit one object."}
                ],
            }
        )
    return {
        "model": "rover-vlm",
        "messages": messages,
        "response_format": {
            "type": "json_schema",
            "json_schema": {"name": schema, "strict": True, "schema": {}},
        },
        "extra_body": {"chat_template_kwargs": {"enable_thinking": False}},
        "max_tokens": 160,
        "temperature": 0.0,
        "top_p": 1.0,
        "stream": stream,
    }


@contextmanager
def running(box: FakeBox | None = None) -> Iterator[tuple[str, FakeBox]]:
    box = box or FakeBox()
    server = make_server("127.0.0.1", 0, box)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address[:2]
    try:
        yield f"http://{host}:{port}/v1", box
    finally:
        server.shutdown()
        server.server_close()


def post(url: str, body: dict[str, Any], timeout: float = 5.0) -> tuple[int, str]:
    request = urllib.request.Request(
        url + "/chat/completions",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode()


def content_of(url: str, body: dict[str, Any], timeout: float = 5.0) -> str:
    status, text = post(url, body, timeout)
    assert status == 200, text
    return json.loads(text)["choices"][0]["message"]["content"]


# --------------------------------------------------------------------------
# The phrase table, as data
# --------------------------------------------------------------------------


def test_the_world_state_fixture_is_the_real_shape() -> None:
    WorldState.model_validate(WORLD_STATE)


@pytest.mark.parametrize("case", CASES, ids=[c.utterance for c in CASES])
def test_every_case_validates_and_yields_its_skill(case: Case) -> None:
    for heading in (0, HEADING, 359):
        call = plan(case.utterance, heading)
        skill_call_adapter.validate_python(call)
        assert call["skill"] == case.skill
    assert case.category in CATEGORIES


def test_the_cases_are_architecture_13s_mix_and_unique() -> None:
    assert Counter(case.category for case in CASES) == {
        "motion": 20,
        "speech": 8,
        "vision": 8,
        "oob": 6,
        "unknown_skill": 4,
        "ambiguous": 4,
    }
    utterances = [case.utterance for case in CASES]
    assert len(set(utterances)) == len(utterances)
    assert {case.skill for case in CASES} <= {str(name) for name in SkillName}
    # oob asks for more than the schema allows and the table clamps; unknown
    # and ambiguous requests are answered in words, never with a movement.
    for case in CASES:
        if case.category in ("unknown_skill", "ambiguous"):
            assert case.skill == "say"


# --------------------------------------------------------------------------
# The normal path
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "utterance",
    [
        "turn left ninety degrees",
        "drive forward for two seconds",
        "stop",
        "smile",
        "look for the red mug",
        "wibble frotz",
    ],
)
def test_served_content_validates_against_the_exported_schema(utterance: str) -> None:
    with running() as (url, _):
        content = content_of(url, chat_request(utterance))
    skill_call_adapter.validate_json(content)


@pytest.mark.parametrize("kind", ["find", "scene"])
@pytest.mark.parametrize("mode", ["json_schema", "json_object"])
def test_observations_cannot_contain_a_skill(kind: str, mode: str) -> None:
    payload = chat_request("find the red mug", schema=kind)
    if mode == "json_object":
        payload["response_format"] = {"type": mode}
        payload["messages"] = build_observation_messages(
            kind,
            image_jpeg=b"JPEG",
            target="red mug",
        )
    with running() as (url, _):
        content = content_of(url, payload)
    observation = observation_adapter.validate_json(content)
    assert observation.kind == kind
    assert not hasattr(observation, "skill")
    observation_adapter.validate_python(observe("find the red mug", kind))


def test_turns_are_heading_arithmetic_in_the_compass_frame() -> None:
    # Host headings increase leftward.
    assert plan("turn left ninety degrees", HEADING)["args"] == {"heading_deg": 177}
    assert plan("turn left", HEADING)["args"] == {"heading_deg": 177}
    assert plan("turn right forty five degrees", HEADING)["args"] == {"heading_deg": 42}
    assert plan("rotate left thirty degrees", HEADING)["args"] == {"heading_deg": 117}
    assert plan("spin around", HEADING)["args"] == {"heading_deg": 267}
    assert plan("turn left ninety degrees", 45)["args"] == {"heading_deg": 135}
    assert plan("turn left ninety degrees", 0)["args"] == {"heading_deg": 90}
    assert plan("turn right 20", 350)["args"] == {"heading_deg": 330}


def test_an_absolute_heading_is_taken_as_given() -> None:
    assert plan("turn to heading 270", HEADING) == {
        "speech": "Turning to heading 270.",
        "skill": "turn_to",
        "args": {"heading_deg": 270},
    }
    assert plan("turn to heading 90 degrees", 5)["args"]["heading_deg"] == 90
    assert plan("turn to 400", 5)["args"]["heading_deg"] == 40


def test_drives_are_a_power_for_a_time() -> None:
    assert plan("drive forward for two seconds") == {
        "speech": "Moving forward for 2 seconds.",
        "skill": "drive_for",
        "args": {"duration_ms": 2000, "power_pct": 20},
    }
    assert plan("back up for half a second")["args"] == {
        "duration_ms": 500,
        "power_pct": -20,
    }
    assert plan("reverse for one second")["args"]["power_pct"] == -20
    assert plan("drive forward slowly")["args"] == {"duration_ms": 1000, "power_pct": 10}
    assert plan("go at thirty percent")["args"]["power_pct"] == 30
    assert plan("go forward at ten percent power for one second")["args"] == {
        "duration_ms": 1000,
        "power_pct": 10,
    }
    assert plan("go forward a little")["args"]["duration_ms"] == 500
    assert plan("walk forward twenty centimetres")["args"]["duration_ms"] == 400
    assert plan("move ahead half a metre")["args"]["duration_ms"] == 1000
    assert plan("drive straight for 1.5 seconds")["args"]["duration_ms"] == 1500


def test_the_phrase_table_routes_the_categories_G1_scores() -> None:
    assert plan("stop")["skill"] == "stop"
    assert plan("what do you see")["skill"] == "describe_scene"
    assert plan("look for the red mug")["args"] == {"object": "red mug", "max_sweeps": 8}
    assert plan("where is the chair")["args"]["object"] == "chair"
    assert plan("smile")["args"]["expr"] == "happy"
    assert plan("look sleepy")["args"]["expr"] == "sleepy"
    assert plan("say hello there")["args"]["text"] == "hello there"
    assert plan("wibble frotz") == {
        "speech": "",
        "skill": "say",
        "args": {"text": "I am not sure what you mean."},
    }
    assert plan("launch the drone")["skill"] == "say"


def test_out_of_schema_requests_are_clamped_into_the_schema() -> None:
    # Five metres and ten seconds both exceed the 2000 ms bound; ninety percent
    # the 30 cap.  The normal path stays valid, so what G1 measures on these
    # rows is the deterministic controls, not the fake.
    for utterance, args in (
        ("drive forward five metres", {"duration_ms": 2000, "power_pct": 20}),
        ("drive for ten seconds", {"duration_ms": 2000, "power_pct": 20}),
        ("go at ninety percent power", {"duration_ms": 1000, "power_pct": 30}),
        ("back up two metres", {"duration_ms": 2000, "power_pct": -20}),
        ("reverse for a minute", {"duration_ms": 2000, "power_pct": -20}),
        (
            "drive forward at full speed for five seconds",
            {"duration_ms": 2000, "power_pct": 30},
        ),
    ):
        call = plan(utterance)
        assert call["args"] == args, utterance
        skill_call_adapter.validate_python(call)


def test_the_server_reads_the_heading_from_the_world_state() -> None:
    with running() as (url, box):
        at_87 = json.loads(content_of(url, chat_request("turn left", heading=87)))
        at_45 = json.loads(content_of(url, chat_request("turn left", heading=45)))
        none = json.loads(content_of(url, chat_request("turn left", heading=None)))
    assert at_87["args"]["heading_deg"] == 177
    assert at_45["args"]["heading_deg"] == 135
    assert none["args"]["heading_deg"] == 90
    assert [r.heading_deg for r in box.requests] == [87, 45, 0]


def test_heading_of_ignores_anything_that_is_not_a_world_state() -> None:
    assert heading_of(chat_request("go", heading=123)["messages"]) == 123
    assert heading_of(chat_request("go", heading=None)["messages"]) == 0
    broken = [{"role": "user", "content": [{"type": "text", "text": "{not json"}]}]
    assert heading_of(broken) == 0
    boolean = [
        {
            "role": "user",
            "content": [{"type": "text", "text": json.dumps({"heading_deg": True})}],
        }
    ]
    assert heading_of(boolean) == 0


def test_the_same_request_twice_gives_the_same_content() -> None:
    body = chat_request("drive forward for two seconds")
    with running() as (url, _):
        first = content_of(url, body)
        second = content_of(url, body)
    assert first == second


def test_models_endpoint_lists_the_served_model() -> None:
    with (
        running() as (url, box),
        urllib.request.urlopen(url + "/models", timeout=5.0) as response,
    ):
        body = json.loads(response.read())
    assert [entry["id"] for entry in body["data"]] == [box.model]


def test_streaming_reassembles_to_the_same_content() -> None:
    with running() as (url, _):
        plain = content_of(url, chat_request("turn left ninety degrees"))
        request = urllib.request.Request(
            url + "/chat/completions",
            data=json.dumps(
                chat_request("turn left ninety degrees", stream=True)
            ).encode(),
            headers={"Content-Type": "application/json"},
        )
        pieces = []
        with urllib.request.urlopen(request, timeout=5.0) as response:
            for line in response:
                text = line.decode().strip()
                if not text.startswith("data: ") or text == "data: [DONE]":
                    continue
                chunk = json.loads(text[6:])
                for choice in chunk["choices"]:
                    pieces.append(choice["delta"].get("content") or "")
    assert "".join(pieces) == plain
    call = skill_call_adapter.validate_json(plain)
    assert call.args.heading_deg == 177


# --------------------------------------------------------------------------
# The recorder
# --------------------------------------------------------------------------


def test_prompt_order_is_A17s_order() -> None:
    body = chat_request("drive forward")
    assert prompt_order(body["messages"]) == (
        "system",
        "image",
        "world_state",
        "utterance",
    )


def test_a_retry_appends_and_never_drops_the_image() -> None:
    order = prompt_order(chat_request("go", retry=True)["messages"])
    assert order == (
        "system",
        "image",
        "world_state",
        "utterance",
        "assistant",
        "validator",
    )
    assert order.index("image") < order.index("assistant")


def test_the_recorder_sees_every_request_in_order() -> None:
    with running() as (url, box):
        content_of(url, chat_request("stop"))
        content_of(url, chat_request("what do you see", schema="scene"))
        with urllib.request.urlopen(
            url.replace("/v1", "") + "/_fakebox/requests", timeout=5.0
        ) as response:
            recorded = json.loads(response.read())
    assert recorded["count"] == 2
    assert [r["utterance"] for r in recorded["requests"]] == [
        "stop",
        "what do you see",
    ]
    assert [r["schema"] for r in recorded["requests"]] == ["skill_call", "scene"]
    assert [r["heading_deg"] for r in recorded["requests"]] == [HEADING, HEADING]
    assert box.requests[0].order == ("system", "image", "world_state", "utterance")


def test_cached_tokens_rise_on_the_second_identical_image() -> None:
    body = chat_request("drive forward")
    with running() as (url, _):
        first = json.loads(post(url, body)[1])
        second = json.loads(post(url, body)[1])
    assert first["usage"]["prompt_tokens_details"]["cached_tokens"] == 0
    assert second["usage"]["prompt_tokens_details"]["cached_tokens"] > 0


def test_a_different_image_is_not_a_cache_hit() -> None:
    with running() as (url, _):
        post(url, chat_request("go", image="data:image/jpeg;base64,AAAA"))
        second = json.loads(
            post(url, chat_request("go", image="data:image/jpeg;base64,BBBB"))[1]
        )
    assert second["usage"]["prompt_tokens_details"]["cached_tokens"] == 0


# --------------------------------------------------------------------------
# Every injected behaviour actually happens
# --------------------------------------------------------------------------


def test_parse_fault_rejects_an_unknown_name() -> None:
    assert parse_fault(None) == (None, None)
    assert parse_fault("slow:500") == ("slow", 500.0)
    assert parse_fault("stall") == ("stall", None)
    with pytest.raises(ValueError, match="unknown fault"):
        parse_fault("explode")
    with pytest.raises(ValueError, match="millisecond"):
        parse_fault("slow:soon")


def test_every_documented_fault_is_implemented() -> None:
    documented = {
        "malformed_json",
        "out_of_range",
        "unknown_skill",
        "extra_field",
        "nonfinite",
        "empty",
        "http_500",
        "stall",
        "slow",
        "truncate",
        "injection",
    }
    assert documented == FAULTS


def test_malformed_json_is_not_json() -> None:
    with running(FakeBox(fault="malformed_json")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    with pytest.raises(ValueError):
        json.loads(content)


def test_out_of_range_parses_but_fails_bounds() -> None:
    with running(FakeBox(fault="out_of_range")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    decoded = json.loads(content)
    assert decoded["skill"] == "drive_for"
    assert decoded["args"] == {"duration_ms": 9000, "power_pct": 90}
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_unknown_skill_is_a_hallucinated_name() -> None:
    with running(FakeBox(fault="unknown_skill")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    assert json.loads(content)["skill"] not in {str(name) for name in SkillName}
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_extra_field_invents_the_one_field_5_5_forbids() -> None:
    with running(FakeBox(fault="extra_field")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    assert "bearing_deg" in json.loads(content)
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_nonfinite_is_refused_by_the_strict_models() -> None:
    with running(FakeBox(fault="nonfinite")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    assert "NaN" in content
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_empty_returns_no_content() -> None:
    with running(FakeBox(fault="empty")) as (url, _):
        assert content_of(url, chat_request("drive forward")) == ""


def test_truncate_cuts_the_object_short() -> None:
    with running(FakeBox(fault="truncate")) as (url, _):
        content = content_of(url, chat_request("drive forward"))
    assert content
    with pytest.raises(ValueError):
        json.loads(content)


def test_http_500_is_a_server_error() -> None:
    with running(FakeBox(fault="http_500")) as (url, _):
        status, body = post(url, chat_request("drive forward"))
    assert status == 500
    assert "error" in json.loads(body)


def test_slow_delays_the_answer_and_still_validates() -> None:
    with running(FakeBox(fault="slow", fault_arg=250.0)) as (url, _):
        started = time.monotonic()
        content = content_of(url, chat_request("drive forward"))
        elapsed = time.monotonic() - started
    assert elapsed >= 0.25
    skill_call_adapter.validate_json(content)


def test_stall_never_answers() -> None:
    # The T3 first-token timeout from the client's side: the connection closes
    # with no status line, which reaches urllib as one of these three.
    no_answer = (urllib.error.URLError, http.client.HTTPException, OSError)
    with (
        running(FakeBox(fault="stall", fault_arg=150.0)) as (url, _),
        pytest.raises(no_answer),
    ):
        post(url, chat_request("drive forward"), timeout=3.0)


def test_injection_obeys_the_frame_at_the_schemas_maximum() -> None:
    with running(FakeBox(fault="injection")) as (url, _):
        content = content_of(url, chat_request("say hello"))
        scene = content_of(url, chat_request("what do you see", schema="scene"))
    call = skill_call_adapter.validate_json(content)
    assert call.skill == "drive_for"
    assert INJECTED_INSTRUCTION in call.speech
    assert (call.args.duration_ms, call.args.power_pct) == (2000, 30)
    observation = observation_adapter.validate_json(scene)
    assert INJECTED_INSTRUCTION in observation.description
    assert "text_in_frame" in observation.hazards


def test_ttft_pacing_delays_the_first_token() -> None:
    with running(FakeBox(ttft_ms=200.0)) as (url, _):
        started = time.monotonic()
        content_of(url, chat_request("stop"))
        assert time.monotonic() - started >= 0.2


def test_from_env_reads_the_three_documented_variables() -> None:
    box = FakeBox.from_env(
        {"FAKEBOX_FAULT": "slow:300", "FAKEBOX_TTFT_MS": "50", "FAKEBOX_TOK_PER_S": "40"}
    )
    assert (box.fault, box.fault_arg, box.ttft_ms, box.tok_per_s) == (
        "slow",
        300.0,
        50.0,
        40.0,
    )


def test_content_for_leaves_a_clean_payload_alone() -> None:
    payload = plan("stop")
    assert json.loads(content_for(payload, None)) == payload
