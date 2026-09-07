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
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts import observation_adapter, skill_call_adapter  # noqa: E402
from rover_devtools.fakebox import (  # noqa: E402
    FAULTS,
    INJECTED_INSTRUCTION,
    FakeBox,
    content_for,
    make_server,
    observe,
    parse_fault,
    plan,
    prompt_order,
)

WORLD_STATE = {
    "pose_cm": {"x": 142, "y": -30},
    "heading_deg": 87,
    "battery_pct": 62,
    "obstacle_ahead": False,
    "front_range_cm": 120,
    "front_at_max": False,
    "bumper": False,
    "moving": False,
    "speed_cap_cms": 30,
    "last_result": "done",
    "last_scene": "a kitchen",
    "recently_seen": [],
    "allowed_skills": ["drive", "turn", "stop", "say"],
    "motion_budget_left": {"path_cm": 110, "seconds": 9},
}

UTTERANCES = [
    "go to the table",
    "turn left ninety degrees",
    "drive forward 40 centimetres",
    "back up",
    "stop",
    "what do you see",
    "look for the red mug",
    "say hello there",
    "smile",
    "spin around",
    "wibble frotz",
]


def chat_request(
    text: str,
    *,
    image: str | None = "data:image/jpeg;base64,AAAA",
    schema: str = "skill_call",
    stream: bool = False,
    retry: bool = False,
) -> dict[str, Any]:
    """ARCHITECTURE 5.7's request: system, image, world state, utterance."""
    parts: list[dict[str, Any]] = []
    if image is not None:
        parts.append({"type": "image_url", "image_url": {"url": image}})
    parts.append({"type": "text", "text": json.dumps(WORLD_STATE)})
    parts.append({"type": "text", "text": f"USER: {text}"})
    messages: list[dict[str, Any]] = [
        {"role": "system", "content": "You are a rover."},
        {"role": "user", "content": parts},
    ]
    if retry:
        messages.append({"role": "assistant", "content": '{"skill":"drive"}'})
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
# The normal path
# --------------------------------------------------------------------------


@pytest.mark.parametrize("utterance", UTTERANCES)
def test_plan_validates_against_the_exported_schema(utterance: str) -> None:
    skill_call_adapter.validate_python(plan(utterance))


@pytest.mark.parametrize("utterance", UTTERANCES)
def test_served_content_validates_against_the_exported_schema(utterance: str) -> None:
    with running() as (url, _):
        content = content_of(url, chat_request(utterance))
    skill_call_adapter.validate_json(content)


@pytest.mark.parametrize("kind", ["find", "scene"])
def test_observations_cannot_contain_a_skill(kind: str) -> None:
    with running() as (url, _):
        content = content_of(url, chat_request("find the red mug", schema=kind))
    observation = observation_adapter.validate_json(content)
    assert observation.kind == kind
    assert not hasattr(observation, "skill")
    observation_adapter.validate_python(observe("find the red mug", kind))


def test_the_phrase_table_routes_the_categories_G1_scores() -> None:
    assert plan("stop")["skill"] == "stop"
    assert plan("turn left ninety degrees") == {
        "speech": "Turning left 90 degrees.",
        "skill": "turn",
        "args": {"angle_deg": 90, "rate_dps": 40},
    }
    assert plan("turn right 45 degrees")["args"]["angle_deg"] == -45
    assert plan("go to the table")["args"]["distance_cm"] == 40
    assert plan("back up 20 cm")["args"]["distance_cm"] == -20
    assert plan("what do you see")["skill"] == "describe_scene"
    assert plan("look for the red mug")["args"]["object"] == "red mug"
    assert plan("smile")["args"]["expr"] == "happy"
    assert plan("wibble frotz")["skill"] == "say"


def test_out_of_schema_requests_are_clamped_into_the_schema() -> None:
    # "5 metres" is 500 cm; the schema stops at 100, so the normal path stays
    # valid and the deterministic controls -- not the fake -- are what is tested.
    call = plan("drive forward 5 metres")
    assert call["args"]["distance_cm"] == 100
    skill_call_adapter.validate_python(call)


def test_the_same_request_twice_gives_the_same_content() -> None:
    body = chat_request("go to the table")
    with running() as (url, _):
        first = content_of(url, body)
        second = content_of(url, body)
    assert first == second


def test_models_endpoint_lists_the_served_model() -> None:
    with running() as (url, box), urllib.request.urlopen(
        url + "/models", timeout=5.0
    ) as response:
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
    skill_call_adapter.validate_json(plain)


# --------------------------------------------------------------------------
# The recorder
# --------------------------------------------------------------------------


def test_prompt_order_is_A17s_order() -> None:
    body = chat_request("go to the table")
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
        with urllib.request.urlopen(url.replace("/v1", "") + "/_fakebox/requests",
                                    timeout=5.0) as response:
            recorded = json.loads(response.read())
    assert recorded["count"] == 2
    assert [r["utterance"] for r in recorded["requests"]] == [
        "stop",
        "what do you see",
    ]
    assert [r["schema"] for r in recorded["requests"]] == ["skill_call", "scene"]
    assert box.requests[0].order == ("system", "image", "world_state", "utterance")


def test_cached_tokens_rise_on_the_second_identical_image() -> None:
    body = chat_request("go to the table")
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
        "malformed_json", "out_of_range", "unknown_skill", "extra_field",
        "nonfinite", "empty", "http_500", "stall", "slow", "truncate", "injection",
    }
    assert documented == FAULTS


def test_malformed_json_is_not_json() -> None:
    with running(FakeBox(fault="malformed_json")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    with pytest.raises(ValueError):
        json.loads(content)


def test_out_of_range_parses_but_fails_bounds() -> None:
    with running(FakeBox(fault="out_of_range")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    decoded = json.loads(content)
    assert decoded["args"]["distance_cm"] == 5000
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_unknown_skill_is_a_hallucinated_name() -> None:
    with running(FakeBox(fault="unknown_skill")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    assert json.loads(content)["skill"] not in {
        "drive", "turn", "stop", "say", "describe_scene", "find", "set_face"
    }
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_extra_field_invents_the_one_field_5_5_forbids() -> None:
    with running(FakeBox(fault="extra_field")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    assert "bearing_deg" in json.loads(content)
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_nonfinite_is_refused_by_the_strict_models() -> None:
    with running(FakeBox(fault="nonfinite")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    assert "NaN" in content
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json(content)


def test_empty_returns_no_content() -> None:
    with running(FakeBox(fault="empty")) as (url, _):
        assert content_of(url, chat_request("go to the table")) == ""


def test_truncate_cuts_the_object_short() -> None:
    with running(FakeBox(fault="truncate")) as (url, _):
        content = content_of(url, chat_request("go to the table"))
    assert content
    with pytest.raises(ValueError):
        json.loads(content)


def test_http_500_is_a_server_error() -> None:
    with running(FakeBox(fault="http_500")) as (url, _):
        status, body = post(url, chat_request("go to the table"))
    assert status == 500
    assert "error" in json.loads(body)


def test_slow_delays_the_answer_and_still_validates() -> None:
    with running(FakeBox(fault="slow", fault_arg=250.0)) as (url, _):
        started = time.monotonic()
        content = content_of(url, chat_request("go to the table"))
        elapsed = time.monotonic() - started
    assert elapsed >= 0.25
    skill_call_adapter.validate_json(content)


def test_stall_never_answers() -> None:
    # The T3 first-token timeout from the client's side: the connection closes
    # with no status line, which reaches urllib as one of these three.
    no_answer = (urllib.error.URLError, http.client.HTTPException, OSError)
    with running(FakeBox(fault="stall", fault_arg=150.0)) as (url, _), pytest.raises(
        no_answer
    ):
        post(url, chat_request("go to the table"), timeout=3.0)


def test_injection_obeys_the_frame_instead_of_the_utterance() -> None:
    with running(FakeBox(fault="injection")) as (url, _):
        content = content_of(url, chat_request("say hello"))
        scene = content_of(url, chat_request("what do you see", schema="scene"))
    call = skill_call_adapter.validate_json(content)
    assert call.skill == "drive"
    assert INJECTED_INSTRUCTION in call.speech
    assert call.args.distance_cm == 100
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
        {"FAKEBOX_FAULT": "slow:300", "FAKEBOX_TTFT_MS": "50",
         "FAKEBOX_TOK_PER_S": "40"}
    )
    assert (box.fault, box.fault_arg, box.ttft_ms, box.tok_per_s) == (
        "slow", 300.0, 50.0, 40.0
    )


def test_content_for_leaves_a_clean_payload_alone() -> None:
    payload = plan("stop")
    assert json.loads(content_for(payload, None)) == payload
