"""Every model accepts the architecture's own example and rejects each way of
getting it wrong.

The examples follow ARCHITECTURE 5.2, 5.3, 5.4, 5.5, 5.6 and 5.9 as amended by
ADR-0013 -- open loop: a drive is a power for a time, a turn is a heading closed
on the controller's fused yaw, and there is no pose -- with the elided ULIDs
filled in.  If one of these stops parsing, the documents and the code have
diverged.  ``config/robot.toml`` loading is covered in ``test_config.py``.
"""

from __future__ import annotations

import json
import math

import pytest
from hypothesis import given
from hypothesis import strategies as st
from pydantic import ValidationError
from rover_contracts import (
    BUS_SKILL_ARGS,
    CATALOG,
    MOTION_SKILLS,
    NON_MOTION_SKILLS,
    POWER_CAP,
    SKILLS,
    TWIST,
    TWIST_BOUNDS,
    ClearableFault,
    DriveForBusArgs,
    EventKind,
    FindObservation,
    FrameHeader,
    JsonlWriter,
    LimitsConfig,
    ResultReason,
    ResultStatus,
    SafetyConfig,
    SkillMessage,
    SkillName,
    StateMessage,
    TwistPayload,
    Unit,
    WelcomeMessage,
    WorldState,
    atomic_write_jsonl,
    brain_client_adapter,
    client_adapter,
    decode_ulid,
    goal_deadline_s,
    is_session_id,
    is_ulid,
    new_cmd_id,
    new_session_id,
    new_turn_id,
    new_ulid,
    observation_adapter,
    power_from_pct,
    read_jsonl,
    server_adapter,
    skill_call_adapter,
)

CMD_ID = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
TURN_ID = "01J9ZC7K000000000000000000"
T_UTC_NS = 1757260802230000000


# --------------------------------------------------------------------------
# ids
# --------------------------------------------------------------------------


def test_architecture_ulids_are_canonical():
    assert is_ulid(CMD_ID)
    assert is_ulid(TURN_ID)
    assert len(CMD_ID) == 26


@pytest.mark.parametrize(
    "bad",
    ["", "short", CMD_ID.lower(), CMD_ID + "X", "81J9ZC7K3QF2M8XR4V6T0YAHBD",
     "01J9ZC7K3QF2M8XR4V6T0YAHBI", 42, None],
)
def test_non_ulids_rejected(bad):
    assert not is_ulid(bad)


def test_ulid_is_monotonic_by_string_compare():
    ids = [new_ulid() for _ in range(2000)]
    assert ids == sorted(ids)
    assert len(set(ids)) == len(ids)


def test_ulid_decodes_to_a_plausible_timestamp():
    ts, rand = decode_ulid(new_turn_id())
    assert ts > 1_700_000_000_000
    assert 0 <= rand < (1 << 80)
    assert is_ulid(new_cmd_id())


def test_bus_session_ids_are_eight_hex_characters():
    session = new_session_id()
    assert is_session_id(session)
    assert not is_session_id("5F3C1A2B")
    assert not is_session_id("5f3c1a2")


# --------------------------------------------------------------------------
# SkillCall (ARCHITECTURE 5.4) -- what the model emits, in integers (A11)
# --------------------------------------------------------------------------

SKILL_CALLS = [
    {"speech": "Heading over.", "skill": "drive_for",
     "args": {"duration_ms": 1500, "power_pct": 20}},
    {"speech": "Turning.", "skill": "turn_to", "args": {"heading_deg": 270}},
    {"speech": "Stopping.", "skill": "stop", "args": {}},
    {"speech": "", "skill": "say",
     "args": {"text": "There is a mug on the table."}},
    {"speech": "Let me look.", "skill": "describe_scene", "args": {}},
    {"speech": "Looking for it.", "skill": "find",
     "args": {"object": "red mug", "max_sweeps": 8}},
    {"speech": "", "skill": "set_face", "args": {"expr": "happy"}},
]


@pytest.mark.parametrize("call", SKILL_CALLS, ids=[c["skill"] for c in SKILL_CALLS])
def test_documented_skill_calls_validate(call):
    parsed = skill_call_adapter.validate_python(call)
    assert parsed.skill == call["skill"]


def test_skill_call_has_seven_branches_and_speech_first():
    schema = skill_call_adapter.json_schema()
    assert len(schema["oneOf"]) == 7
    for branch in schema["oneOf"]:
        target = schema["$defs"][branch["$ref"].rsplit("/", 1)[-1]]
        assert list(target["properties"])[0] == "speech"
        assert target["additionalProperties"] is False
        assert set(target["required"]) == {"speech", "skill", "args"}


@pytest.mark.parametrize("power_pct", [-30, -1, 1, 30])
def test_drive_for_takes_both_directions_up_to_the_cap(power_pct):
    call = {"speech": "", "skill": "drive_for",
            "args": {"duration_ms": 100, "power_pct": power_pct}}
    assert skill_call_adapter.validate_python(call).args.power_pct == power_pct


@pytest.mark.parametrize("heading_deg", [0, 359])
def test_turn_to_takes_the_whole_compass(heading_deg):
    call = {"speech": "", "skill": "turn_to", "args": {"heading_deg": heading_deg}}
    assert skill_call_adapter.validate_python(call).args.heading_deg == heading_deg


def _drive(**args):
    return {"speech": "x", "skill": "drive_for",
            "args": {"duration_ms": 500, "power_pct": 20, **args}}


def _turn(**args):
    return {"speech": "x", "skill": "turn_to", "args": {"heading_deg": 90, **args}}


BAD_SKILL_CALLS = {
    "unknown_skill": {"speech": "x", "skill": "fly", "args": {}},
    "retired_drive": {"speech": "x", "skill": "drive",
                      "args": {"distance_cm": 40, "speed_cms": 15}},
    "retired_turn": {"speech": "x", "skill": "turn",
                     "args": {"angle_deg": 45, "rate_dps": 40}},
    "missing_arg": {"speech": "x", "skill": "drive_for", "args": {"duration_ms": 500}},
    "extra_arg": _drive(extra=1),
    "top_level_extra": {"speech": "x", "skill": "stop", "args": {}, "extra": 1},
    "duration_over": _drive(duration_ms=2001),
    "duration_under": _drive(duration_ms=99),
    "duration_float": _drive(duration_ms=500.0),
    "duration_nan": _drive(duration_ms=math.nan),
    "duration_inf": _drive(duration_ms=math.inf),
    "power_over": _drive(power_pct=31),
    "power_under": _drive(power_pct=-31),
    "power_zero": _drive(power_pct=0),
    "power_float": _drive(power_pct=20.0),
    "power_string": _drive(power_pct="20"),
    "power_bool": _drive(power_pct=True),
    "power_neg_inf": _drive(power_pct=-math.inf),
    "heading_360": _turn(heading_deg=360),
    "heading_negative": _turn(heading_deg=-1),
    "heading_float": _turn(heading_deg=90.0),
    "heading_string": _turn(heading_deg="90"),
    "heading_bool": _turn(heading_deg=True),
    "heading_nan": _turn(heading_deg=math.nan),
    "turn_args_on_drive": {"speech": "x", "skill": "drive_for",
                           "args": {"heading_deg": 90}},
    "drive_args_on_turn": {"speech": "x", "skill": "turn_to",
                           "args": {"duration_ms": 500, "power_pct": 20}},
    "speech_too_long": {"speech": "x" * 161, "skill": "stop", "args": {}},
    "missing_speech": {"skill": "stop", "args": {}},
    "stop_with_args": {"speech": "x", "skill": "stop", "args": {"now": True}},
    "unknown_face": {"speech": "x", "skill": "set_face", "args": {"expr": "smug"}},
    "nine_sweeps": {"speech": "x", "skill": "find",
                    "args": {"object": "mug", "max_sweeps": 9}},
    "zero_sweeps": {"speech": "x", "skill": "find",
                    "args": {"object": "mug", "max_sweeps": 0}},
    "empty_object": {"speech": "x", "skill": "find",
                     "args": {"object": "", "max_sweeps": 1}},
    "empty_say": {"speech": "x", "skill": "say", "args": {"text": ""}},
    "long_say": {"speech": "x", "skill": "say", "args": {"text": "x" * 241}},
}


@pytest.mark.parametrize("bad", BAD_SKILL_CALLS.values(), ids=list(BAD_SKILL_CALLS))
def test_bad_skill_calls_rejected(bad):
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_python(bad)


@pytest.mark.parametrize(
    "args",
    [{"distance_m": 0.4, "speed_mps": 0.15}, {"distance_cm": 40, "power_pct": 20},
     {"duration_ms": 500, "power_pct": 20, "speed_mps": 0.1}],
    ids=["metres", "centimetres", "smuggled_speed"],
)
def test_the_model_cannot_emit_a_distance(args):
    # ADR-0013: no encoder, so no distance anywhere in the catalog.
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_python(
            {"speech": "x", "skill": "drive_for", "args": args}
        )


# --------------------------------------------------------------------------
# robotd bus, client -> robotd (ARCHITECTURE 5.2)
# --------------------------------------------------------------------------

SKILL_MESSAGE = {
    "v": 1, "type": "skill", "source": "brain", "cmd_id": CMD_ID, "seq": 42,
    "turn_id": TURN_ID, "issued_mono_ns": 123456789012, "goal_ttl_ms": 2750,
    "skill": "drive_for", "args": {"duration_s": 1.5, "power": 0.20},
    "obs": {"frame_id": "cam-000917", "frame_mono_ns": 98764000000},
    "trace": {"model": "rover-vlm", "prompt_sha256": "ab12",
              "authorized_motion": True},
}

TURN_TO_MESSAGE = {
    **SKILL_MESSAGE, "skill": "turn_to", "goal_ttl_ms": 5000,
    "args": {"heading_deg": 270.0, "timeout_s": 4.0, "tolerance_deg": 5.0},
}

TWIST_MESSAGE = {
    "v": 1, "type": "twist", "source": "teleop", "cmd_id": CMD_ID, "seq": 901,
    "twist": {"lin": 0.15, "ang": -0.10},
}

CLIENT_MESSAGES = [
    {"v": 1, "type": "hello", "source": "brain", "pid": 1234,
     "caps": ["skill", "subscribe"]},
    {"v": 1, "type": "subscribe", "topics": ["state", "result", "event"],
     "state_hz": 10},
    {"v": 1, "type": "turn", "source": "brain", "turn_id": TURN_ID},
    {"v": 1, "type": "ping", "source": "brain"},
    SKILL_MESSAGE,
    TURN_TO_MESSAGE,
    TWIST_MESSAGE,
    {"v": 1, "type": "cancel", "source": "brain", "cmd_id": CMD_ID,
     "reason": "barge_in"},
    {"v": 1, "type": "stop", "source": "brain", "reason": "stop_word"},
    {"v": 1, "type": "estop", "source": "web", "reason": "user"},
    {"v": 1, "type": "clear", "source": "web", "faults": ["estop_sw"]},
]
CLIENT_IDS = [m["type"] + ("-" + m["skill"] if "skill" in m else "")
              for m in CLIENT_MESSAGES]


@pytest.mark.parametrize("message", CLIENT_MESSAGES, ids=CLIENT_IDS)
def test_documented_client_messages_validate(message):
    parsed = client_adapter.validate_python(message)
    assert parsed.type == message["type"]


def test_drive_for_goal_ttl_is_the_catalog_deadline():
    # brain computes goal_ttl_ms from goal_deadline_s; the example must agree.
    args = SKILL_MESSAGE["args"]
    assert SKILL_MESSAGE["goal_ttl_ms"] == goal_deadline_s(args["duration_s"]) * 1000


def test_stop_class_messages_stay_loose():
    # I-22: stop-class messages skip strict parsing, seq, replay, freshness,
    # fault and cooldown checks, and are never answered "rejected".  brain's
    # restart path sends a stop on connect, before any turn exists.
    bare = client_adapter.validate_python({"v": 1, "type": "stop", "source": "brain"})
    assert bare.reason is None and bare.seq is None
    assert bare.cmd_id is None and bare.turn_id is None
    dressed = client_adapter.validate_python(
        {"v": 1, "type": "stop", "source": "brain", "reason": "user",
         "cmd_id": CMD_ID, "turn_id": TURN_ID, "seq": 7}
    )
    assert dressed.seq == 7
    assert client_adapter.validate_python(
        {"v": 1, "type": "cancel", "source": "brain"}
    ).cmd_id is None
    assert client_adapter.validate_python(
        {"v": 1, "type": "estop", "source": "web"}
    ).reason is None


def test_stop_is_not_carried_as_a_skill():
    # brain translates the stop skill into the stop bus message, so it never
    # has to pass strict parsing to be recognised as a stop.
    assert "stop" not in BUS_SKILL_ARGS
    assert set(BUS_SKILL_ARGS) == {s.value for s in SkillName} - {"stop"}
    with pytest.raises(ValidationError):
        client_adapter.validate_python({**SKILL_MESSAGE, "skill": "stop", "args": {}})


BUS_ARGS_EXAMPLES = {
    "drive_for": {"duration_s": 0.1, "power": -0.1},
    "turn_to": {"heading_deg": 0.0},
    "say": {"text": "hello"},
    "describe_scene": {},
    "find": {"object": "mug", "max_sweeps": 1},
    "set_face": {"expr": "happy"},
}


@pytest.mark.parametrize("skill", list(BUS_SKILL_ARGS))
def test_skill_message_args_are_the_matching_bus_model(skill):
    message = SkillMessage.model_validate(
        {**SKILL_MESSAGE, "skill": skill, "args": BUS_ARGS_EXAMPLES[skill]}
    )
    assert type(message.args) is BUS_SKILL_ARGS[skill]


@pytest.mark.parametrize(
    ("skill", "wrong"),
    [("drive_for", {"heading_deg": 90.0}),
     ("turn_to", {"duration_s": 1.0, "power": 0.1}),
     ("say", {}),
     ("describe_scene", {"text": "x"}),
     ("find", {"expr": "happy"}),
     ("set_face", {"object": "mug", "max_sweeps": 1})],
    ids=lambda v: v if isinstance(v, str) else "",
)
def test_skill_message_args_must_match_the_skill(skill, wrong):
    with pytest.raises(ValidationError):
        SkillMessage.model_validate({**SKILL_MESSAGE, "skill": skill, "args": wrong})


def _bus_drive(**args):
    return {"args": {"duration_s": 1.0, "power": 0.2, **args}}


def _bus_turn(**args):
    return {"skill": "turn_to", "args": {"heading_deg": 90.0, **args}}


BAD_SKILL_MESSAGES = {
    "unknown_skill": {"skill": "fly"},
    "retired_skill": {"skill": "drive", "args": {"distance_m": 0.4, "speed_mps": 0.15}},
    "ttl_over": {"goal_ttl_ms": 5001},                                  # I-8
    "ttl_under": {"goal_ttl_ms": 99},
    "bad_source": {"source": "hacker"},
    "bad_cmd_id": {"cmd_id": "not-a-ulid"},
    "lowercase_turn_id": {"turn_id": CMD_ID.lower()},
    "negative_seq": {"seq": -1},
    "negative_mono": {"issued_mono_ns": -1},
    "power_over": _bus_drive(power=0.31),
    "power_under": _bus_drive(power=-0.31),
    "power_zero": _bus_drive(power=0.0),
    "power_nan": _bus_drive(power=math.nan),
    "power_inf": _bus_drive(power=math.inf),
    "power_string": _bus_drive(power="0.2"),
    "power_pct_on_bus": {"args": {"duration_s": 1.0, "power_pct": 20}},
    "duration_zero": _bus_drive(duration_s=0.0),
    "duration_over": _bus_drive(duration_s=2.001),
    "duration_negative": _bus_drive(duration_s=-1.0),
    "hidden_arg": _bus_drive(hidden=1),
    "empty_args": {"args": {}},
    "heading_360": _bus_turn(heading_deg=360.0),
    "heading_negative": _bus_turn(heading_deg=-0.001),
    "heading_nan": _bus_turn(heading_deg=math.nan),
    "timeout_over": _bus_turn(timeout_s=4.001),
    "timeout_zero": _bus_turn(timeout_s=0.0),
    "tolerance_under": _bus_turn(tolerance_deg=1.999),
    "tolerance_over": _bus_turn(tolerance_deg=20.001),
    "bad_frame_id": {"obs": {"frame_id": "cam-17", "frame_mono_ns": 1}},
    "unknown_trace_key": {"trace": {"model": "x", "unknown": 1}},
}


@pytest.mark.parametrize(
    "patch", BAD_SKILL_MESSAGES.values(), ids=list(BAD_SKILL_MESSAGES)
)
def test_bad_skill_messages_rejected(patch):
    with pytest.raises(ValidationError):
        client_adapter.validate_python({**SKILL_MESSAGE, **patch})


@pytest.mark.parametrize(
    "twist",
    [{"lin": 0.30, "ang": 0.30}, {"lin": -0.30, "ang": -0.30}, {"lin": 0.0, "ang": 0.0}],
    ids=["cap", "neg_cap", "zero"],
)
def test_twist_payload_accepts_the_whole_power_range(twist):
    parsed = TwistPayload.model_validate(twist)
    assert (parsed.lin, parsed.ang) == (twist["lin"], twist["ang"])


BAD_TWISTS = {
    "lin_over": {"lin": 0.31, "ang": 0.0},
    "lin_under": {"lin": -0.31, "ang": 0.0},
    "ang_over": {"lin": 0.0, "ang": 0.31},
    "ang_under": {"lin": 0.0, "ang": -0.31},
    "ang_neg_inf": {"lin": 0.0, "ang": -math.inf},
    "lin_nan": {"lin": math.nan, "ang": 0.0},
    "missing_ang": {"lin": 0.0},
    "extra": {"lin": 0.0, "ang": 0.0, "z": 1},
    "string": {"lin": "0.1", "ang": 0.0},
    "bool": {"lin": True, "ang": 0.0},
    "retired_units": {"linear_x_mps": 0.15, "angular_z_radps": 0.35},
}


@pytest.mark.parametrize("twist", BAD_TWISTS.values(), ids=list(BAD_TWISTS))
def test_bad_twists_rejected(twist):
    # I-8 covers twist as well as skill.
    with pytest.raises(ValidationError):
        TwistPayload.model_validate(twist)
    with pytest.raises(ValidationError):
        client_adapter.validate_python({**TWIST_MESSAGE, "twist": twist})


def test_clear_names_only_clearable_faults():
    assert {f.value for f in ClearableFault} == {
        "estop_sw", "obstacle_latched", "low_battery"
    }
    for faults in (["tof_stop"], [], ["heartbeat"]):
        with pytest.raises(ValidationError):
            client_adapter.validate_python(
                {"v": 1, "type": "clear", "source": "web", "faults": faults}
            )


# --------------------------------------------------------------------------
# robotd bus, robotd -> clients
# --------------------------------------------------------------------------

LIMITS = {
    "power_max": 0.30, "power_default": 0.20, "power_min": 0.08,
    "drive_for_max_s": 2.0, "turn_timeout_max_s": 4.0, "turn_tolerance_deg": 5.0,
    "turn_kp": 0.004, "goal_ttl_ms_max": 5000, "budget_motion_s": 12,
    "motion_cooldown_ms": 3000, "twist_power": 0.30, "twist_renew_ms": 200,
}
SAFETY = {
    "obs_max_age_ms": 5000, "heartbeat_ms": 300, "feedback_max_age_ms": 150,
    "tof_stop_mm": 250, "low_battery_v": 9.9, "require_patched_firmware": True,
}

WELCOME = {
    "v": 1, "type": "welcome", "session": "5f3c1a2b", "robotd_version": "0.2.0",
    "rover_fw": "bot-wr-1", "limits": LIMITS, "safety": SAFETY,
}

STATE = {
    "v": 1, "type": "state", "t_utc_ns": 1757260800120000000,
    "t_mono_ns": 98765432100,
    "rover": {"fw": "bot-wr-1", "hb_ok": True, "stop_flags": 0, "feedback_age_ms": 18,
              "cmd_left": 0.20, "cmd_right": 0.20, "heading_deg": 87.5,
              "yaw_rate_dps": 0.0, "roll_deg": 0.4, "pitch_deg": -1.2,
              "temp_c": 31.5, "clamp_count": 0, "motion": True},
    "twist": {"lin": 0.20, "ang": 0.0},
    "front_m": 1.204, "bumper": False, "estop_sw": False,
    "battery": {"pack_v": 11.62, "pct": 62},
    "active": {"cmd_id": CMD_ID, "skill": "drive_for", "source": "brain",
               "progress": 0.34, "deadline_in_ms": 1800},
    "budget": {"motion_s": 9.0},
    "ready": True, "reason": "",
}

RESULT_DONE = {
    "v": 1, "type": "result", "cmd_id": CMD_ID, "seq": 42, "status": "done",
    "reason": "", "detail": {"duration_ms": 1500}, "t_utc_ns": T_UTC_NS,
}
RESULT_TIMEOUT = {
    **RESULT_DONE, "status": "timeout",
    "detail": {"turned_deg": 84.0, "heading_error_deg": -6.0},
}
RESULT_CLAMPED = {
    **RESULT_DONE, "status": "accepted", "reason": "power_clamped",
    "detail": {"power_clamped_to": 0.20},
}

SERVER_MESSAGES = [
    WELCOME,
    STATE,
    RESULT_DONE,
    RESULT_TIMEOUT,
    RESULT_CLAMPED,
    {"v": 1, "type": "event", "kind": "tof_block", "fault": "obstacle_latched",
     "detail": {"tof_mm": 231}, "t_utc_ns": T_UTC_NS},
    {"v": 1, "type": "event", "kind": "rover_restart",
     "detail": {"fw": "bot-wr-1", "hb_ms": 300, "cap": 0.3}, "t_utc_ns": T_UTC_NS},
    {"v": 1, "type": "event", "kind": "unpatched_firmware",
     "detail": {"banner": False}, "t_utc_ns": T_UTC_NS},
    {"v": 1, "type": "error", "code": "bad_json", "detail": "not an object"},
]
SERVER_IDS = [f"{m['type']}-{i}" for i, m in enumerate(SERVER_MESSAGES)]


@pytest.mark.parametrize("message", SERVER_MESSAGES, ids=SERVER_IDS)
def test_documented_server_messages_validate(message):
    parsed = server_adapter.validate_python(message)
    assert parsed.type == message["type"]


def test_welcome_republishes_every_limits_and_safety_key():
    # A33: a gate asserts against what is running, not against the file.
    dumped = WelcomeMessage.model_validate(WELCOME).model_dump(mode="json")
    assert dumped["limits"].keys() == LimitsConfig.model_fields.keys()
    assert dumped["safety"].keys() == SafetyConfig.model_fields.keys()
    assert WELCOME["limits"].keys() == LimitsConfig.model_fields.keys()
    assert WELCOME["safety"].keys() == SafetyConfig.model_fields.keys()


def test_welcome_rover_fw_is_null_for_stock_firmware_or_no_link():
    welcome = WelcomeMessage.model_validate({**WELCOME, "rover_fw": None})
    assert welcome.rover_fw is None
    again = server_adapter.validate_json(server_adapter.dump_json(welcome))
    assert again == welcome


@pytest.mark.parametrize(
    "patch",
    [{"mcu_session": 40010}, {"rover_fw": "x" * 33}, {"session": "5F3C1A2B"},
     {"limits": {**LIMITS, "speed_mps": 0.3}},
     {"safety": {**SAFETY, "cliff_delta_mm": 80}}],
    ids=["retired_mcu_session", "fw_too_long", "uppercase_session",
         "retired_limit", "retired_safety_key"],
)
def test_bad_welcomes_rejected(patch):
    with pytest.raises(ValidationError):
        WelcomeMessage.model_validate({**WELCOME, **patch})


def test_state_round_trips_through_json():
    state = StateMessage.model_validate(STATE)
    again = server_adapter.validate_json(server_adapter.dump_json(state))
    assert again == state
    assert again.rover.heading_deg == 87.5
    assert json.loads(server_adapter.dump_json(state))["front_m"] == 1.204


def test_state_front_m_null_is_representable_and_stays_null():
    # I-16: null is "no reading", which is not a clear path; robotd owns that
    # rule, the contract only has to carry the distinction.
    state = StateMessage.model_validate({**STATE, "front_m": None})
    assert state.front_m is None
    assert json.loads(server_adapter.dump_json(state))["front_m"] is None


def test_state_with_no_active_command_no_firmware_and_no_yaw_rate():
    state = StateMessage.model_validate(
        {**STATE, "active": None,
         "rover": {**STATE["rover"], "fw": None, "yaw_rate_dps": None}}
    )
    assert state.active is None
    assert state.rover.fw is None and state.rover.yaw_rate_dps is None


@pytest.mark.parametrize(
    ("field", "value"),
    [("heading_deg", 180.0), ("heading_deg", -179.999), ("stop_flags", 31),
     ("cmd_left", -0.5), ("cmd_right", 0.5), ("clamp_count", 0xFFFF)],
)
def test_state_rover_edges_inside_the_range_are_accepted(field, value):
    rover = StateMessage.model_validate(
        {**STATE, "rover": {**STATE["rover"], field: value}}
    ).rover
    assert getattr(rover, field) == value


def _rover(**patch):
    return {"rover": {**STATE["rover"], **patch}}


BAD_STATES = {
    "stop_flags_32": _rover(stop_flags=32),
    "stop_flags_negative": _rover(stop_flags=-1),
    "stop_flags_float": _rover(stop_flags=1.0),
    "stop_flags_bool": _rover(stop_flags=True),
    "heading_minus_180": _rover(heading_deg=-180.0),
    "heading_over_180": _rover(heading_deg=180.001),
    "heading_360": _rover(heading_deg=360.0),
    "heading_nan": _rover(heading_deg=math.nan),
    "cmd_left_over": _rover(cmd_left=0.51),
    "cmd_right_under": _rover(cmd_right=-0.51),
    "clamp_count_over": _rover(clamp_count=0x10000),
    "clamp_count_negative": _rover(clamp_count=-1),
    "feedback_age_negative": _rover(feedback_age_ms=-1),
    "fw_too_long": _rover(fw="x" * 33),
    "rover_extra": _rover(session=40010),
    "twist_over": {"twist": {"lin": 0.31, "ang": 0.0}},
    "front_m_negative": {"front_m": -0.1},
    "front_m_string": {"front_m": "1.2"},
    "battery_pct_over": {"battery": {**STATE["battery"], "pct": 101}},
    "battery_negative": {"battery": {**STATE["battery"], "pack_v": -0.1}},
    "active_stop": {"active": {**STATE["active"], "skill": "stop"}},
    "active_twist": {"active": {**STATE["active"], "skill": "twist"}},
    "active_progress": {"active": {**STATE["active"], "progress": 1.5}},
    "budget_negative": {"budget": {"motion_s": -0.1}},
    "budget_retired": {"budget": {"path_m": 1.1, "motion_s": 9.0}},
    "ready_string": {"ready": "yes"},
    "retired_pose": {"pose": {"frame_id": "odom", "x_m": 1.4, "yaw_rad": 0.1}},
    "retired_mcu": {"mcu": {"state": "ARMED_MOVING"}},
    "retired_ranges": {"ranges_m": {"front": 1.2}},
    "retired_armed": {"armed": True},
    "unknown": {"unknown": 1},
}


@pytest.mark.parametrize("patch", BAD_STATES.values(), ids=list(BAD_STATES))
def test_bad_state_messages_rejected(patch):
    with pytest.raises(ValidationError):
        server_adapter.validate_python({**STATE, **patch})


def test_result_reason_is_a_closed_speakable_enum():
    reasons = {r.value for r in ResultReason}
    assert {"unpatched_firmware", "heading_unavailable", "feedback_stale",
            "power_clamped"} <= reasons
    assert not {"speed_clamped", "mcu_nack"} & reasons
    for bad in ({"reason": "because"}, {"reason": "speed_clamped"},
                {"reason": "mcu_nack"}, {"status": "fault"}):
        with pytest.raises(ValidationError):
            server_adapter.validate_python({**RESULT_DONE, **bad})


@pytest.mark.parametrize(
    "detail",
    [{"power_clamped_to": 0.31}, {"power_clamped_to": -0.1}, {"duration_ms": -1},
     {"traveled_m": 0.4}, {"odom_delta": {"x_m": 0.4}}, {"turned_deg": math.nan}],
    ids=["clamp_over_cap", "clamp_negative", "duration_negative",
         "retired_traveled_m", "retired_odom_delta", "nan"],
)
def test_bad_result_details_rejected(detail):
    with pytest.raises(ValidationError):
        server_adapter.validate_python({**RESULT_DONE, "detail": detail})


def test_event_kinds_are_the_rover_link_table():
    assert {k.value for k in EventKind} == {
        "link_up", "link_lost", "heartbeat_timeout", "heartbeat_recovered",
        "cap_clamp", "tof_block", "bumper", "low_battery", "unpatched_firmware",
        "rover_restart", "feedback_stale", "fault_cleared",
    }
    for retired in ("fault_set", "mcu_restart", "cliff"):
        with pytest.raises(ValidationError):
            server_adapter.validate_python({**SERVER_MESSAGES[5], "kind": retired})


def test_event_detail_rejects_non_finite_numbers():
    with pytest.raises(ValidationError):
        server_adapter.validate_python(
            {**SERVER_MESSAGES[5], "detail": {"tof_mm": math.nan}}
        )


# --------------------------------------------------------------------------
# frames.sock (5.3) and brain.sock (5.9) -- unchanged by ADR-0013
# --------------------------------------------------------------------------


def test_documented_frame_header_validates():
    header = FrameHeader.model_validate(
        {"v": 1, "frame_id": "cam-000917", "kind": "stream",
         "frame_mono_ns": 98764000000, "frame_wallclock_ns": 1757260799980000000,
         "w": 640, "h": 480, "fmt": "jpeg", "quality": 80, "bytes": 41233,
         "exposure_us": 8000, "gain": 2.4, "lux": 180}
    )
    assert header.kind == "stream"


@pytest.mark.parametrize(
    "patch",
    [{"kind": "video"}, {"frame_id": "917"}, {"fmt": "png"}, {"quality": 0},
     {"w": 0}, {"gain": math.inf}],
)
def test_bad_frame_headers_rejected(patch):
    base = {"v": 1, "frame_id": "cam-000917", "kind": "stream",
            "frame_mono_ns": 1, "frame_wallclock_ns": 1, "w": 640, "h": 480,
            "fmt": "jpeg", "quality": 80, "bytes": 1}
    with pytest.raises(ValidationError):
        FrameHeader.model_validate({**base, **patch})


BRAIN_MESSAGES = [
    {"v": 1, "type": "utterance", "source": "web",
     "text": "turn left ninety degrees", "confidence": None, "is_final": True,
     "mono_ns": 98765432100},
    {"v": 1, "type": "ptt_start", "source": "web"},
    {"v": 1, "type": "ptt_end", "source": "web"},
    {"v": 1, "type": "cancel", "source": "web"},
]


@pytest.mark.parametrize(
    "message", BRAIN_MESSAGES, ids=[m["type"] for m in BRAIN_MESSAGES]
)
def test_documented_brain_messages_validate(message):
    assert brain_client_adapter.validate_python(message).type == message["type"]


def test_null_confidence_is_representable():
    # ARCHITECTURE 7: a null confidence counts as authorized, which is what
    # keeps the whole Mac test plan (stt.backend="text") able to move.
    parsed = brain_client_adapter.validate_python(BRAIN_MESSAGES[0])
    assert parsed.confidence is None


def test_brain_source_set_differs_from_the_bus_source_set():
    with pytest.raises(ValidationError):
        brain_client_adapter.validate_python({**BRAIN_MESSAGES[1], "source": "brain"})


# --------------------------------------------------------------------------
# Observations (5.5) -- A13
# --------------------------------------------------------------------------

OBSERVATIONS = [
    {"kind": "find", "present": True, "center_x_permille": 610,
     "confidence": "medium",
     "description": "a red ceramic mug on a wooden table"},
    {"kind": "scene",
     "description": "a kitchen; table on the left, doorway ahead",
     "labels": ["table", "chair", "doorway"], "lighting": "normal",
     "hazards": ["clutter", "text_in_frame"]},
]


@pytest.mark.parametrize(
    "observation", OBSERVATIONS, ids=[o["kind"] for o in OBSERVATIONS]
)
def test_documented_observations_validate(observation):
    assert observation_adapter.validate_python(observation).kind == observation["kind"]


@pytest.mark.parametrize(
    "smuggled",
    [{"skill": "drive_for"}, {"args": {"duration_ms": 500, "power_pct": 20}},
     {"bearing_deg": 30.0}, {"heading_deg": 30}, {"distance_m": 1.2},
     {"range_cm": 120}, {"speech": "go"}],
)
def test_an_observation_cannot_carry_a_skill_angle_or_distance(smuggled):
    # A13 is structural, not a convention: extra="forbid" is what makes it so.
    for observation in OBSERVATIONS:
        with pytest.raises(ValidationError):
            observation_adapter.validate_python({**observation, **smuggled})


@pytest.mark.parametrize(
    "patch",
    [{"center_x_permille": 1001}, {"center_x_permille": -1},
     {"center_x_permille": 610.5}, {"confidence": "certain"},
     {"present": "yes"}, {"description": "x" * 241}],
)
def test_bad_find_observations_rejected(patch):
    with pytest.raises(ValidationError):
        FindObservation.model_validate({**OBSERVATIONS[0], **patch})


def test_find_observation_geometry_is_an_integer_permille():
    parsed = FindObservation.model_validate(OBSERVATIONS[0])
    assert isinstance(parsed.center_x_permille, int)
    assert "bearing_deg" not in parsed.model_dump()


# --------------------------------------------------------------------------
# WorldState (5.6, amended: heading and power cap, no pose)
# --------------------------------------------------------------------------

WORLD_STATE = {
    "heading_deg": 87, "battery_pct": 62, "obstacle_ahead": False,
    "front_range_cm": 120, "bumper": False, "moving": False, "power_cap_pct": 20,
    "last_result": "done",
    "last_scene": "a kitchen, table on the left, doorway ahead",
    "recently_seen": [{"label": "red mug", "heading_deg": 40, "age_s": 94}],
    "allowed_skills": ["drive_for", "turn_to", "stop", "say", "describe_scene",
                       "find", "set_face"],
    "motion_budget_left": {"seconds": 9},
}


def test_documented_world_state_validates_and_round_trips():
    world = WorldState.model_validate(WORLD_STATE)
    assert world.power_cap_pct == 20
    assert world.last_result is ResultStatus.DONE
    assert WorldState.model_validate_json(world.model_dump_json()) == world
    assert json.loads(world.model_dump_json()) == WORLD_STATE


def test_world_state_field_order_is_stable_for_the_prefix_cache():
    dumped = json.loads(WorldState.model_validate(WORLD_STATE).model_dump_json())
    assert list(dumped) == list(WORLD_STATE)


def test_world_state_carries_no_timestamp_seq_session_or_pose():
    # A17: nothing that changes every turn appears before the image.
    # ADR-0013: no encoder, so no pose.
    fields = set(WorldState.model_fields)
    assert not fields & {"t_utc_ns", "t_mono_ns", "seq", "session", "cmd_id",
                         "turn_id", "pose_cm", "pose", "speed_cap_cms",
                         "front_at_max"}


def test_world_state_front_range_null_is_unknown_not_clear():
    world = WorldState.model_validate({**WORLD_STATE, "front_range_cm": None})
    assert world.front_range_cm is None
    assert json.loads(world.model_dump_json())["front_range_cm"] is None
    # ... and the same for a dropped key.
    dropped = {k: v for k, v in WORLD_STATE.items() if k != "front_range_cm"}
    assert WorldState.model_validate(dropped).front_range_cm is None


@pytest.mark.parametrize("heading", [0, 359])
def test_recently_seen_heading_spans_the_compass(heading):
    world = WorldState.model_validate(
        {**WORLD_STATE,
         "recently_seen": [{"label": "mug", "heading_deg": heading, "age_s": 0}]}
    )
    assert world.recently_seen[0].heading_deg == heading


def _seen(**patch):
    return {"recently_seen": [{"label": "red mug", "heading_deg": 40, "age_s": 94,
                               **patch}]}


BAD_WORLD_STATES = {
    "battery_over": {"battery_pct": 101},
    "range_over": {"front_range_cm": 401},
    "range_negative": {"front_range_cm": -1},
    "range_float": {"front_range_cm": 120.5},
    "heading_360": {"heading_deg": 360},
    "heading_negative": {"heading_deg": -1},
    "heading_float": {"heading_deg": 87.0},
    "power_cap_under": {"power_cap_pct": 4},
    "power_cap_over": {"power_cap_pct": 31},
    "unknown_skill": {"allowed_skills": ["fly"]},
    "twist_as_skill": {"allowed_skills": ["twist"]},
    "no_skills": {"allowed_skills": []},
    "last_result_open": {"last_result": "finished"},
    "scene_too_long": {"last_scene": "x" * 241},
    "seen_heading_360": _seen(heading_deg=360),
    "seen_heading_negative": _seen(heading_deg=-1),
    "seen_retired_where": {
        "recently_seen": [{"label": "mug", "where_deg": 40, "age_s": 1}]
    },
    "seen_empty_label": _seen(label=""),
    "seen_too_many": {"recently_seen": [WORLD_STATE["recently_seen"][0]] * 9},
    "budget_negative": {"motion_budget_left": {"seconds": -1}},
    "budget_retired_path": {"motion_budget_left": {"path_cm": 110, "seconds": 9}},
    "retired_pose": {"pose_cm": {"x": 142, "y": -30}},
    "retired_speed_cap": {"speed_cap_cms": 30},
    "retired_front_at_max": {"front_at_max": False},
}


@pytest.mark.parametrize("patch", BAD_WORLD_STATES.values(), ids=list(BAD_WORLD_STATES))
def test_bad_world_states_rejected(patch):
    with pytest.raises(ValidationError):
        WorldState.model_validate({**WORLD_STATE, **patch})


# --------------------------------------------------------------------------
# The skill catalog (ARCHITECTURE 6 as amended)
# --------------------------------------------------------------------------


def test_catalog_is_the_seven_model_skills_and_twist_is_not_one():
    assert {spec.name for spec in CATALOG} == {s.value for s in SkillName}
    assert len(CATALOG) == 7
    assert TWIST == "twist" and TWIST not in SKILLS


def test_skills_index_derives_from_the_catalog():
    assert {spec.name: spec for spec in CATALOG} == SKILLS
    assert list(SKILLS) == [spec.name for spec in CATALOG]


def test_motion_split_matches_the_validator_table():
    assert {"drive_for", "turn_to", "find"} == MOTION_SKILLS
    assert {"stop", "say", "describe_scene", "set_face"} == NON_MOTION_SKILLS
    assert not MOTION_SKILLS & NON_MOTION_SKILLS
    assert set(SKILLS) == MOTION_SKILLS | NON_MOTION_SKILLS
    assert all(SKILLS[name].moves for name in MOTION_SKILLS)


def test_catalog_bus_args_are_the_skill_message_union():
    from_catalog = {spec.name: spec.bus_args for spec in CATALOG if spec.bus_args}
    assert from_catalog == BUS_SKILL_ARGS


def test_only_stop_lacks_bus_args_and_every_skill_has_model_args():
    assert SKILLS["stop"].bus_args is None
    assert all(spec.bus_args is not None for spec in CATALOG if spec.name != "stop")
    assert all(spec.model_args is not None for spec in CATALOG)


def test_executor_owners_are_the_documented_ones():
    owners = {spec.name: spec.executor for spec in CATALOG}
    assert owners == {
        "drive_for": "robotd", "turn_to": "robotd", "stop": "robotd",
        "say": "brain", "describe_scene": "brain", "find": "brain",
        "set_face": "brain",
    }


def test_every_bound_names_a_field_of_its_own_argument_model():
    for spec in CATALOG:
        for bound in spec.bounds:
            assert spec.bus_args is not None, (spec.name, bound.field)
            assert bound.field in spec.bus_args.model_fields, (spec.name, bound.field)
    assert {b.field for b in TWIST_BOUNDS} == set(TwistPayload.model_fields)


def test_no_bound_is_stated_in_metres():
    assert {u.value for u in Unit} == {"s", "power", "deg", "count", "chars"}


def test_firmware_capped_bounds_are_the_three_power_values():
    capped = {
        (spec.name, b.field): b for spec in CATALOG for b in spec.bounds if b.fw_cap
    }
    capped |= {("twist", b.field): b for b in TWIST_BOUNDS if b.fw_cap}
    assert set(capped) == {("drive_for", "power"), ("twist", "lin"), ("twist", "ang")}
    for bound in capped.values():
        assert bound.fw_cap == "BOT_POWER_CAP"
        assert bound.unit is Unit.POWER
        assert (bound.lo, bound.hi) == (-POWER_CAP, POWER_CAP)


# The bounds table and the argument models must say the same thing.

BASE_ARGS = {**BUS_ARGS_EXAMPLES, "twist": {"lin": 0.1, "ang": 0.1}}
BOUND_MODELS = {spec.name: spec.bus_args for spec in CATALOG if spec.bus_args}
BOUND_MODELS["twist"] = TwistPayload

EDGE_CASES = [(spec.name, bound) for spec in CATALOG for bound in spec.bounds]
EDGE_CASES += [("twist", bound) for bound in TWIST_BOUNDS]


def _value(bound, magnitude):
    return "x" * int(magnitude) if bound.unit is Unit.CHARS else magnitude


def _validates(skill, bound, magnitude):
    BOUND_MODELS[skill].model_validate(
        {**BASE_ARGS[skill], bound.field: _value(bound, magnitude)}
    )


@pytest.mark.parametrize(
    ("skill", "bound"), EDGE_CASES, ids=[f"{s}.{b.field}" for s, b in EDGE_CASES]
)
def test_each_bound_matches_its_argument_model(skill, bound):
    step = 1 if bound.unit in (Unit.CHARS, Unit.COUNT) else 0.001

    inside_hi = bound.hi - step if bound.hi_exclusive else bound.hi
    assert bound.contains(inside_hi)
    _validates(skill, bound, inside_hi)

    inside_lo = bound.lo + step if bound.lo_exclusive else bound.lo
    if bound.contains(inside_lo) and inside_lo != 0.0:  # 0.0 power is "use stop"
        _validates(skill, bound, inside_lo)

    for outside in (bound.hi + step, bound.lo - step):
        assert not bound.contains(outside)
        if bound.unit is Unit.CHARS and outside < 0:
            continue
        with pytest.raises(ValidationError):
            _validates(skill, bound, outside)

    if bound.hi_exclusive:
        assert not bound.contains(bound.hi)
        with pytest.raises(ValidationError):
            _validates(skill, bound, bound.hi)
    if bound.lo_exclusive:
        assert not bound.contains(bound.lo)
        with pytest.raises(ValidationError):
            _validates(skill, bound, bound.lo)


# --------------------------------------------------------------------------
# goal_deadline_s and power_from_pct: the two formulas brain and robotd share
# --------------------------------------------------------------------------


def test_goal_deadline_is_half_again_plus_half_a_second():
    assert goal_deadline_s(1.5) == 2.75
    assert goal_deadline_s(2.0) == 3.5
    assert goal_deadline_s(0.1) == pytest.approx(0.65)
    for bad in (0.0, -1.0):
        with pytest.raises(ValueError):
            goal_deadline_s(bad)


@given(st.floats(min_value=0.001, max_value=2.0))
def test_every_drive_for_deadline_fits_the_goal_ttl_bounds(duration_s):
    # brain sends goal_ttl_ms = round(goal_deadline_s * 1000); robotd checks it
    # against 100..5000 (T2).  Over drive_for's whole range it must land inside.
    goal_ttl_ms = round(goal_deadline_s(duration_s) * 1000)
    message = SkillMessage.model_validate(
        {**SKILL_MESSAGE, "goal_ttl_ms": goal_ttl_ms,
         "args": {"duration_s": duration_s, "power": 0.1}}
    )
    assert message.goal_ttl_ms == goal_ttl_ms


def test_power_from_pct_documented_values():
    assert power_from_pct(30) == 0.30
    assert power_from_pct(-30) == -0.30
    assert power_from_pct(20) == 0.20
    assert power_from_pct(8) == 0.08
    assert power_from_pct(1) == 0.01


@given(st.one_of(st.integers(min_value=-30, max_value=-1),
                 st.integers(min_value=1, max_value=30)))
def test_every_model_power_lands_inside_the_bus_bound(power_pct):
    power = power_from_pct(power_pct)
    assert abs(power) <= POWER_CAP
    assert round(power * 100) == power_pct
    assert DriveForBusArgs(duration_s=1.0, power=power).power == power


# --------------------------------------------------------------------------
# jsonl (A34)
# --------------------------------------------------------------------------


def test_jsonl_appends_and_reads_back(tmp_path):
    path = tmp_path / "logs" / "episode.jsonl"
    with JsonlWriter(path) as writer:
        writer.write({"left": 0.2, "right": 0.2, "heading_deg": 87.5})
        writer.write(WorldState.model_validate(WORLD_STATE))
    with JsonlWriter(path) as writer:
        writer.write({"left": 0.0, "right": 0.0, "heading_deg": 87.5})
    rows = list(read_jsonl(path))
    assert len(rows) == 3
    assert rows[0] == {"left": 0.2, "right": 0.2, "heading_deg": 87.5}
    assert rows[1]["heading_deg"] == 87
    assert "pose_cm" not in rows[1]


def test_jsonl_refuses_non_finite_numbers(tmp_path):
    path = tmp_path / "log.jsonl"
    with JsonlWriter(path) as writer:
        with pytest.raises(ValueError):
            writer.write({"v": math.nan})
        with pytest.raises(ValueError):
            writer.write({"v": math.inf})
        writer.write({"v": 1.0})
    assert list(read_jsonl(path)) == [{"v": 1.0}]


def test_jsonl_key_order_survives(tmp_path):
    path = tmp_path / "log.jsonl"
    with JsonlWriter(path) as writer:
        writer.write({"z": 1, "a": 2, "m": 3})
    assert list(json.loads(path.read_text())) == ["z", "a", "m"]


def test_atomic_write_replaces_the_whole_file(tmp_path):
    path = tmp_path / "scene.jsonl"
    atomic_write_jsonl(path, [{"i": i} for i in range(3)])
    atomic_write_jsonl(path, [{"i": 9}])
    assert list(read_jsonl(path)) == [{"i": 9}]
    assert not (tmp_path / "scene.jsonl.tmp").exists()


def test_atomic_write_leaves_the_old_file_intact_on_a_bad_record(tmp_path):
    path = tmp_path / "scene.jsonl"
    atomic_write_jsonl(path, [{"i": 1}])
    with pytest.raises(ValueError):
        atomic_write_jsonl(path, [{"i": 2}, {"i": math.nan}])
    assert list(read_jsonl(path)) == [{"i": 1}]


# --------------------------------------------------------------------------
# The JSON path, which is the one the sockets actually use
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("adapter", "message"),
    [(client_adapter, m) for m in CLIENT_MESSAGES]
    + [(server_adapter, m) for m in SERVER_MESSAGES]
    + [(brain_client_adapter, m) for m in BRAIN_MESSAGES],
    ids=[f"client-{i}" for i in CLIENT_IDS]
    + [f"server-{i}" for i in SERVER_IDS]
    + [f"brain-{m['type']}" for m in BRAIN_MESSAGES],
)
def test_documented_messages_survive_a_json_round_trip(adapter, message):
    parsed = adapter.validate_json(json.dumps(message))
    again = adapter.validate_json(adapter.dump_json(parsed))
    assert again == parsed


def test_ndjson_line_with_a_coerced_number_is_rejected():
    # A12 stage two: structured generation guarantees syntax, not values.
    line = json.dumps({**SKILL_MESSAGE, "goal_ttl_ms": "2750"})
    with pytest.raises(ValidationError):
        client_adapter.validate_json(line)


def test_skill_call_from_the_box_is_validated_as_json_text():
    parsed = skill_call_adapter.validate_json(json.dumps(SKILL_CALLS[0]))
    assert parsed.args.duration_ms == 1500 and parsed.args.power_pct == 20
    for bad in (
        '{"speech":"x","skill":"drive_for","args":{"duration_ms":1500,"power_pct":"20"}}',
        '{"speech":"x","skill":"drive_for","args":{"duration_ms":NaN,"power_pct":20}}',
        '{"speech":"x","skill":"turn_to","args":{"heading_deg":Infinity}}',
    ):
        with pytest.raises(ValidationError):
            skill_call_adapter.validate_json(bad)
