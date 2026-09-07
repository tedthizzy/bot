"""Every model accepts the architecture's own example and rejects each way of
getting it wrong.

The valid examples are copied from ARCHITECTURE 5.2, 5.3, 5.4, 5.5, 5.6, 5.8
and 5.9, with the elided ULIDs filled in.  If one of these stops parsing, the
document and the code have diverged.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts import (  # noqa: E402
    BUS_SKILL_ARGS,
    CATALOG,
    CEILINGS,
    LATCHED_FAULTS,
    MOTION_SKILLS,
    NON_MOTION_SKILLS,
    SKILLS,
    ClearableFault,
    ConfigError,
    EventCode,
    EventKind,
    Fault,
    FindObservation,
    FrameHeader,
    JsonlWriter,
    McuState,
    ResultStatus,
    SkillMessage,
    SkillName,
    Source,
    StateMessage,
    Unit,
    WelcomeMessage,
    WorldState,
    atomic_write_jsonl,
    brain_client_adapter,
    client_adapter,
    decode_ulid,
    is_session_id,
    is_ulid,
    load_config,
    new_cmd_id,
    new_session_id,
    new_turn_id,
    new_ulid,
    observation_adapter,
    read_jsonl,
    server_adapter,
    skill_call_adapter,
)
from rover_contracts.config import _SourceName  # noqa: E402

CMD_ID = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
TURN_ID = "01J9ZC7K000000000000000000"


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
# SkillCall (ARCHITECTURE 5.4)
# --------------------------------------------------------------------------

SKILL_CALLS = [
    {"speech": "Heading over.", "skill": "drive",
     "args": {"distance_cm": 40, "speed_cms": 15}},
    {"speech": "Turning left.", "skill": "turn",
     "args": {"angle_deg": 45, "rate_dps": 40}},
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


@pytest.mark.parametrize(
    "bad",
    [
        {"speech": "x", "skill": "fly", "args": {}},                       # unknown skill
        {"speech": "x", "skill": "drive", "args": {"distance_cm": 40}},    # missing arg
        # extra field, out of bounds, below the floor, a float where an integer
        # is required, a coerced string, an over-long speech, a missing argument,
        # an unknown face, A14's eight sweeps, args that should be empty, a
        # missing speech, and a top-level extra.
        {"speech": "x", "skill": "drive",
         "args": {"distance_cm": 40, "speed_cms": 15, "extra": 1}},
        {"speech": "x", "skill": "drive",
         "args": {"distance_cm": 200, "speed_cms": 15}},
        {"speech": "x", "skill": "drive",
         "args": {"distance_cm": 40, "speed_cms": 4}},
        {"speech": "x", "skill": "drive",
         "args": {"distance_cm": 0.4, "speed_cms": 15}},
        {"speech": "x", "skill": "drive",
         "args": {"distance_cm": "40", "speed_cms": 15}},
        {"speech": "x" * 161, "skill": "stop", "args": {}},
        {"speech": "x", "skill": "turn", "args": {"angle_deg": 45}},
        {"speech": "x", "skill": "set_face", "args": {"expr": "smug"}},
        {"speech": "x", "skill": "find",
         "args": {"object": "mug", "max_sweeps": 9}},
        {"speech": "x", "skill": "stop", "args": {"now": True}},
        {"skill": "stop", "args": {}},
        {"speech": "x", "skill": "drive", "args": {}, "extra": 1},
    ],
)
def test_bad_skill_calls_rejected(bad):
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_python(bad)


def test_the_model_cannot_emit_a_bearing_or_a_distance_in_metres():
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_python(
            {"speech": "x", "skill": "drive",
             "args": {"distance_m": 0.4, "speed_mps": 0.15}}
        )


# --------------------------------------------------------------------------
# robotd bus, client -> robotd (ARCHITECTURE 5.2)
# --------------------------------------------------------------------------

CLIENT_MESSAGES = [
    {"v": 1, "type": "hello", "source": "brain", "pid": 1234,
     "caps": ["skill", "subscribe"]},
    {"v": 1, "type": "subscribe", "topics": ["state", "result", "event"],
     "state_hz": 10},
    {"v": 1, "type": "turn", "source": "brain", "turn_id": TURN_ID},
    {"v": 1, "type": "ping", "source": "brain"},
    {"v": 1, "type": "skill", "source": "brain", "cmd_id": CMD_ID, "seq": 42,
     "turn_id": TURN_ID, "issued_mono_ns": 123456789012, "goal_ttl_ms": 5000,
     "skill": "drive", "args": {"distance_m": 0.40, "speed_mps": 0.15},
     "obs": {"frame_id": "cam-000917", "frame_mono_ns": 98764000000},
     "trace": {"model": "rover-vlm", "prompt_sha256": "ab12"}},
    {"v": 1, "type": "twist", "source": "teleop", "cmd_id": CMD_ID, "seq": 901,
     "twist": {"linear_x_mps": 0.15, "angular_z_radps": 0.35}},
    {"v": 1, "type": "cancel", "source": "brain", "cmd_id": CMD_ID,
     "reason": "barge_in"},
    {"v": 1, "type": "stop", "source": "brain", "reason": "stop_word"},
    {"v": 1, "type": "estop", "source": "web", "reason": "user"},
    {"v": 1, "type": "clear", "source": "web", "faults": ["estop_sw"]},
]


@pytest.mark.parametrize(
    "message", CLIENT_MESSAGES, ids=[m["type"] for m in CLIENT_MESSAGES]
)
def test_documented_client_messages_validate(message):
    parsed = client_adapter.validate_python(message)
    assert parsed.type == message["type"]


def test_stop_needs_no_seq_cmd_id_or_turn_id_but_tolerates_them():
    # I-22: stop-class messages skip strict parsing, seq, replay, freshness,
    # fault and cooldown checks, and are never answered "rejected".  brain's
    # restart path sends one on connect, before any turn exists.
    bare = client_adapter.validate_python(
        {"v": 1, "type": "stop", "source": "brain"}
    )
    assert bare.reason is None and bare.seq is None and bare.cmd_id is None
    dressed = client_adapter.validate_python(
        {"v": 1, "type": "stop", "source": "brain", "reason": "user",
         "cmd_id": CMD_ID, "turn_id": TURN_ID, "seq": 7}
    )
    assert dressed.seq == 7


def test_stop_is_not_carried_as_a_skill():
    # ARCHITECTURE 6: brain translates the stop skill into the stop bus message,
    # so it never has to pass strict parsing to be recognised as a stop.
    assert "stop" not in BUS_SKILL_ARGS
    with pytest.raises(ValidationError):
        client_adapter.validate_python(
            {**CLIENT_MESSAGES[4], "skill": "stop", "args": {}}
        )


@pytest.mark.parametrize(
    "patch",
    [
        {"skill": "fly"},
        {"goal_ttl_ms": 5001},                                   # I-8
        {"goal_ttl_ms": 99},
        {"source": "hacker"},
        {"cmd_id": "not-a-ulid"},
        {"turn_id": CMD_ID.lower()},
        {"seq": -1},
        {"issued_mono_ns": -1},
        {"args": {"distance_m": 1.5, "speed_mps": 0.15}},        # over drive_m
        {"args": {"distance_m": 0.4, "speed_mps": 0.31}},        # over speed_mps
        {"args": {"distance_m": 0.4, "speed_mps": 0.0}},         # 0 < v
        {"args": {"distance_m": 0.4, "speed_mps": float("nan")}},
        {"args": {"distance_m": 0.4, "speed_mps": float("inf")}},
        {"args": {"angle_deg": 45.0, "rate_dps": 40.0}},         # turn args on drive
        {"args": {"distance_m": 0.4, "speed_mps": 0.15, "hidden": 1}},
        {"obs": {"frame_id": "cam-17", "frame_mono_ns": 1}},
        {"trace": {"model": "x", "unknown": 1}},
    ],
)
def test_bad_skill_messages_rejected(patch):
    with pytest.raises(ValidationError):
        client_adapter.validate_python({**CLIENT_MESSAGES[4], **patch})


@pytest.mark.parametrize(
    "patch",
    [
        {"twist": {"linear_x_mps": 0.31, "angular_z_radps": 0.0}},
        {"twist": {"linear_x_mps": -0.31, "angular_z_radps": 0.0}},
        {"twist": {"linear_x_mps": 0.0, "angular_z_radps": 1.048}},
        {"twist": {"linear_x_mps": 0.0, "angular_z_radps": float("-inf")}},
        {"twist": {"linear_x_mps": 0.0}},
        {"twist": {"linear_x_mps": 0.0, "angular_z_radps": 0.0, "z": 1}},
    ],
)
def test_bad_twists_rejected(patch):
    # I-8 covers twist as well as skill.
    with pytest.raises(ValidationError):
        client_adapter.validate_python({**CLIENT_MESSAGES[5], **patch})


def test_clear_names_only_clearable_faults():
    with pytest.raises(ValidationError):
        client_adapter.validate_python(
            {"v": 1, "type": "clear", "source": "web", "faults": ["tof_stop"]}
        )
    with pytest.raises(ValidationError):
        client_adapter.validate_python(
            {"v": 1, "type": "clear", "source": "web", "faults": []}
        )


def test_clearable_faults_are_the_latched_class_plus_the_software_estop():
    latched = {bit.name.lower() for bit in Fault if LATCHED_FAULTS & bit}
    assert {f.value for f in ClearableFault} == latched | {"estop_sw"}


# --------------------------------------------------------------------------
# robotd bus, robotd -> clients
# --------------------------------------------------------------------------

WELCOME = {
    "v": 1, "type": "welcome", "session": "5f3c1a2b", "robotd_version": "0.1.0",
    "mcu_session": 40010,
    "limits": {"drive_m": 1.0, "speed_mps": 0.30, "speed_default_mps": 0.20,
               "turn_deg": 180.0, "rate_dps": 60.0, "accel_mps2": 0.5,
               "alpha_radps2": 1.0, "frame_ttl_ms": 300, "goal_ttl_ms_max": 5000,
               "budget_path_m": 1.5, "budget_motion_s": 12,
               "motion_cooldown_ms": 3000, "motion_idle_disarm_ms": 5000,
               "twist_linear_mps": 0.30, "twist_angular_radps": 1.047,
               "twist_renew_ms": 200},
    "safety": {"obs_max_age_ms": 5000, "link_alive_max_age_ms": 200,
               "tof_stop_mm": 250, "tof_slow_mm": 600, "slow_zone_w_mrad_s": 500,
               "tof_timing_budget_ms": 20, "tof_inter_period_ms": 30,
               "tof_poll_hz": 50, "cliff_baseline_mm": 98, "cliff_delta_mm": 80,
               "obstacle_escalate_s": 30, "r_pack_mohm": 65},
}

STATE = {
    "v": 1, "type": "state", "t_utc_ns": 1757260800120000000,
    "t_mono_ns": 98765432100,
    "mcu": {"state": "ARMED_MOVING", "fault": 0, "session": 40010, "age_ms": 18,
            "last_ack_seq": 3, "loop_late_pct": 0, "rx_drop": 0, "motion": True},
    "armed": True,
    "pose": {"frame_id": "odom", "x_m": 1.42, "y_m": -0.30, "yaw_rad": 1.518},
    "twist": {"linear_x_mps": 0.248, "angular_z_radps": 0.208},
    "wheels": {"left_ticks": 204411, "right_ticks": 203877, "ticks_per_rev": 2200,
               "wheel_radius_m": 0.045, "track_m": 0.150},
    "ranges_m": {"front": 1.204, "cliff": 0.098}, "front_at_max": False,
    "tof": {"front_l_ok": True, "front_r_ok": True},
    "bumper": False, "estop_hw": False, "estop_sw": False,
    "battery": {"pack_v": 11.62, "oc_v": 11.70, "current_a": 0.41, "pct": 62},
    "rails": {"servo": False},
    "active": {"cmd_id": CMD_ID, "skill": "drive", "source": "brain",
               "progress": 0.34, "deadline_in_ms": 3500},
    "budget": {"path_m": 1.1, "motion_s": 9.0},
    "ready": True, "reason": "",
}

SERVER_MESSAGES = [
    WELCOME,
    STATE,
    {"v": 1, "type": "result", "cmd_id": CMD_ID, "seq": 42, "status": "done",
     "reason": "", "detail": {"traveled_m": 0.40, "duration_ms": 2970,
                              "odom_delta": {"x_m": 0.40, "y_m": 0.01,
                                             "yaw_rad": 0.02}},
     "t_utc_ns": 1757260802230000000},
    {"v": 1, "type": "event", "kind": "fault_set", "fault": "tof_stop",
     "detail": {"range_m": 0.231}, "t_utc_ns": 1757260802230000000},
    {"v": 1, "type": "event", "kind": "mcu_restart",
     "detail": {"old_session": 40010, "new_session": 51882, "reset_reason": 8},
     "t_utc_ns": 1757260802230000000},
    {"v": 1, "type": "error", "code": "bad_frame", "detail": "crc"},
]


@pytest.mark.parametrize(
    "message", SERVER_MESSAGES, ids=[m["type"] + "-" + str(i)
                                     for i, m in enumerate(SERVER_MESSAGES)]
)
def test_documented_server_messages_validate(message):
    parsed = server_adapter.validate_python(message)
    assert parsed.type == message["type"]


def test_welcome_republishes_every_limits_and_safety_key():
    welcome = WelcomeMessage.model_validate(WELCOME)
    dumped = welcome.model_dump(mode="json")
    assert dumped["limits"].keys() == WELCOME["limits"].keys()
    assert dumped["safety"].keys() == WELCOME["safety"].keys()


def test_state_mcu_state_round_trips_as_its_name():
    state = StateMessage.model_validate(STATE)
    assert state.mcu.state is McuState.ARMED_MOVING
    assert state.model_dump(mode="json")["mcu"]["state"] == "ARMED_MOVING"


def test_state_renders_the_two_tof_sentinels_distinctly():
    # I-16: 65535 is blockage and publishes as null; 65534 is a clear path and
    # publishes as 6.0 with front_at_max, never as 65.534.
    stale = {**STATE, "ranges_m": {"front": None, "cliff": 0.098}}
    assert StateMessage.model_validate(stale).ranges_m.front is None
    at_max = {**STATE, "ranges_m": {"front": 6.0, "cliff": 0.098},
              "front_at_max": True}
    assert StateMessage.model_validate(at_max).front_at_max is True


def test_state_with_no_active_command():
    assert StateMessage.model_validate({**STATE, "active": None}).active is None


@pytest.mark.parametrize(
    "patch",
    [
        {"mcu": {**STATE["mcu"], "state": "SPINNING"}},
        {"battery": {**STATE["battery"], "pct": 101}},
        {"active": {**STATE["active"], "progress": 1.5}},
        {"active": {**STATE["active"], "skill": "stop"}},
        {"pose": {**STATE["pose"], "x_m": float("nan")}},
        {"pose": {**STATE["pose"], "frame_id": "map"}},
        {"ready": "yes"},
        {"unknown": 1},
    ],
)
def test_bad_state_messages_rejected(patch):
    with pytest.raises(ValidationError):
        server_adapter.validate_python({**STATE, **patch})


def test_result_reason_is_a_closed_speakable_enum():
    with pytest.raises(ValidationError):
        server_adapter.validate_python(
            {**SERVER_MESSAGES[2], "reason": "because"}
        )
    with pytest.raises(ValidationError):
        server_adapter.validate_python({**SERVER_MESSAGES[2], "status": "fault"})


def test_event_detail_rejects_non_finite_numbers():
    with pytest.raises(ValidationError):
        server_adapter.validate_python(
            {**SERVER_MESSAGES[3], "detail": {"range_m": float("nan")}}
        )


def test_event_kinds_cover_every_mcu_event_code():
    assert {code.name.lower() for code in EventCode} <= {k.value for k in EventKind}
    assert EventKind.MCU_RESTART.value == "mcu_restart"


# --------------------------------------------------------------------------
# frames.sock (5.3) and brain.sock (5.9)
# --------------------------------------------------------------------------


def test_documented_frame_header_validates():
    header = FrameHeader.model_validate(
        {"v": 1, "frame_id": "cam-000917", "kind": "stream",
         "frame_mono_ns": 98764000000, "frame_wallclock_ns": 1757260799980000000,
         "w": 640, "h": 480, "fmt": "jpeg", "quality": 80, "bytes": 41233,
         "exposure_us": 8000, "gain": 2.4, "lux": 180}
    )
    assert header.kind == "still" or header.kind == "stream"


@pytest.mark.parametrize(
    "patch",
    [{"kind": "video"}, {"frame_id": "917"}, {"fmt": "png"}, {"quality": 0},
     {"w": 0}, {"gain": float("inf")}],
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
    # ARCHITECTURE 7: a null confidence counts as authorized, which is what keeps
    # the whole Mac test plan (stt.backend="text") able to move.
    parsed = brain_client_adapter.validate_python(BRAIN_MESSAGES[0])
    assert parsed.confidence is None


def test_brain_source_set_differs_from_the_bus_source_set():
    with pytest.raises(ValidationError):
        brain_client_adapter.validate_python(
            {**BRAIN_MESSAGES[1], "source": "brain"}
        )


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
    [
        {"skill": "drive"},
        {"args": {"distance_cm": 100}},
        {"bearing_deg": 30.0},
        {"distance_m": 1.2},
        {"range_cm": 120},
        {"speech": "go"},
    ],
)
def test_an_observation_cannot_carry_a_skill_or_a_distance(smuggled):
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
# WorldState (5.6)
# --------------------------------------------------------------------------

WORLD_STATE = {
    "pose_cm": {"x": 142, "y": -30}, "heading_deg": 87, "battery_pct": 62,
    "obstacle_ahead": False, "front_range_cm": 120, "front_at_max": False,
    "bumper": False, "moving": False, "speed_cap_cms": 30,
    "last_result": "done",
    "last_scene": "a kitchen, table on the left, doorway ahead",
    "recently_seen": [{"label": "red mug", "where_deg": 40, "age_s": 94}],
    "allowed_skills": ["drive", "turn", "stop", "say", "describe_scene", "find",
                       "set_face"],
    "motion_budget_left": {"path_cm": 110, "seconds": 9},
}


def test_documented_world_state_validates():
    world = WorldState.model_validate(WORLD_STATE)
    assert world.speed_cap_cms == 30
    assert world.last_result is ResultStatus.DONE


def test_world_state_field_order_is_stable_for_the_prefix_cache():
    dumped = json.loads(WorldState.model_validate(WORLD_STATE).model_dump_json())
    assert list(dumped) == list(WORLD_STATE)


def test_world_state_carries_no_timestamp_seq_or_session():
    # A17: nothing that changes every turn appears before the image.
    fields = set(WorldState.model_fields)
    assert not fields & {"t_utc_ns", "t_mono_ns", "seq", "session", "cmd_id",
                         "turn_id"}


def test_speed_cap_must_agree_with_the_front_range():
    # ARCHITECTURE 5.6: 30 is only legal because front_range_cm is above the
    # 100 cm unlock threshold; the two fields must agree.
    with pytest.raises(ValidationError):
        WorldState.model_validate({**WORLD_STATE, "front_range_cm": 40})
    ok = WorldState.model_validate(
        {**WORLD_STATE, "front_range_cm": 40, "speed_cap_cms": 20}
    )
    assert ok.speed_cap_cms == 20
    at_max = WorldState.model_validate(
        {**WORLD_STATE, "front_range_cm": 600, "front_at_max": True}
    )
    assert at_max.speed_cap_cms == 30


@pytest.mark.parametrize(
    "patch",
    [{"battery_pct": 101}, {"front_range_cm": 6553}, {"heading_deg": 400},
     {"allowed_skills": ["fly"]}, {"allowed_skills": []},
     {"pose_cm": {"x": 1.42, "y": -30}}, {"speed_cap_cms": 31},
     {"last_result": "finished"}, {"motion_budget_left": {"path_cm": -1,
                                                          "seconds": 9}}],
)
def test_bad_world_states_rejected(patch):
    with pytest.raises(ValidationError):
        WorldState.model_validate({**WORLD_STATE, **patch})


# --------------------------------------------------------------------------
# The skill catalog (ARCHITECTURE 6)
# --------------------------------------------------------------------------


def test_catalog_covers_every_model_skill_plus_twist():
    assert {spec.name for spec in CATALOG} == {s.value for s in SkillName} | {"twist"}
    assert len(CATALOG) == 8


def test_motion_split_matches_the_validator_table():
    assert {"drive", "turn", "twist"} == MOTION_SKILLS
    assert {"say", "describe_scene", "set_face", "find", "stop"} <= NON_MOTION_SKILLS


def test_catalog_bus_args_match_the_skill_message_union():
    from_catalog = {
        spec.name: spec.bus_args
        for spec in CATALOG
        if spec.bus_args is not None and spec.name != "twist"
    }
    assert from_catalog == BUS_SKILL_ARGS


def test_stop_has_no_bus_args_and_twist_has_no_model_args():
    assert SKILLS["stop"].bus_args is None
    assert SKILLS["twist"].model_args is None


def test_every_bound_is_satisfied_by_its_own_argument_model():
    for spec in CATALOG:
        if spec.bus_args is None:
            continue
        fields = set(spec.bus_args.model_fields)
        for bound in spec.bounds:
            assert bound.field in fields, (spec.name, bound.field)


def test_executor_owners_are_the_documented_ones():
    assert SKILLS["drive"].executor == "robotd"
    assert SKILLS["turn"].executor == "robotd"
    assert SKILLS["twist"].executor == "robotd"
    assert SKILLS["say"].executor == "brain"
    assert SKILLS["describe_scene"].executor == "brain"
    assert SKILLS["find"].executor == "brain"
    assert SKILLS["set_face"].executor == "brain"


# --------------------------------------------------------------------------
# config (5.8) -- A33
# --------------------------------------------------------------------------

MINIMAL_TOML = """
[robot]
name = "rover"
[limits]
speed_mps = 0.30
turn_deg = 180
budget_motion_s = 12
[serial]
backend = "pty"
port = "./run/mcu.pty"
[safety]
tof_stop_mm = 250
"""


def write_config(tmp_path: Path, text: str = MINIMAL_TOML) -> Path:
    path = tmp_path / "robot.toml"
    path.write_text(text)
    return path


def test_config_loads_with_documented_defaults(tmp_path):
    config = load_config(write_config(tmp_path), env={})
    assert config.limits.speed_mps == 0.30
    assert config.limits.turn_deg == 180.0
    assert config.serial.backend == "pty"
    assert config.safety.tof_stop_mm == 250
    assert config.camera.hfov_deg == 83.0
    assert config.bus.allow_stream == []


def test_config_safety_hash_matches_the_boot_banner(tmp_path):
    config = load_config(write_config(tmp_path), env={})
    assert config.safety_hash() == 3381018647


def test_env_override_applies_and_is_typed(tmp_path):
    config = load_config(
        write_config(tmp_path),
        env={"ROVER__LIMITS__SPEED_MPS": "0.25",
             "ROVER__BUS__ALLOW_STREAM": '["teleop"]',
             "ROVER__CAMERA__BACKEND": '"fake"'},
    )
    assert config.limits.speed_mps == 0.25
    assert config.bus.allow_stream == ["teleop"]
    assert config.camera.backend == "fake"


def test_override_above_a_ceiling_refuses_to_start(tmp_path):
    # A33: ROVER__LIMITS__SPEED_MPS=3.0 must not take effect silently, and must
    # not be clamped either.
    with pytest.raises(ConfigError) as excinfo:
        load_config(write_config(tmp_path),
                    env={"ROVER__LIMITS__SPEED_MPS": "3.0"})
    assert "ceiling" in str(excinfo.value)
    assert "limits.speed_mps" in str(excinfo.value)


def test_file_value_above_a_ceiling_refuses_to_start(tmp_path):
    path = write_config(tmp_path, MINIMAL_TOML.replace(
        "budget_motion_s = 12", "budget_motion_s = 60"))
    with pytest.raises(ConfigError):
        load_config(path, env={})


def test_every_limits_and_safety_key_has_a_ceiling(tmp_path):
    config = load_config(write_config(tmp_path), env={})
    for section in ("limits", "safety"):
        for key in type(getattr(config, section)).model_fields:
            assert f"{section}.{key}" in CEILINGS, f"{section}.{key}"


def test_a_literal_secret_in_the_file_is_refused(tmp_path):
    path = write_config(tmp_path, MINIMAL_TOML + '\n[box]\napi_key = "sk-live"\n')
    with pytest.raises(ConfigError) as excinfo:
        load_config(path, env={})
    assert "secret" in str(excinfo.value)


def test_api_key_env_names_a_variable_and_is_allowed(tmp_path):
    path = write_config(
        tmp_path, MINIMAL_TOML + '\n[box]\napi_key_env = "ROVER_BOX_API_KEY"\n'
    )
    assert load_config(path, env={}).box.api_key_env == "ROVER_BOX_API_KEY"


def test_cmd_gate_must_be_tighter_than_link_alive(tmp_path):
    path = write_config(
        tmp_path,
        MINIMAL_TOML + "\ncmd_gate_max_age_ms = 200\n",
    )
    with pytest.raises(ConfigError):
        load_config(path, env={})


def test_renaming_the_robot_without_a_wake_model_is_refused(tmp_path):
    """5.8 calls [robot] name the wake-word identity.

    A33 logs every applied override at WARN, so a key nothing reads reports
    that the change took effect while the robot goes on listening for the old
    name.  The wake model is trained for exactly one name (open item 11), so
    the two have to agree or startup refuses.
    """
    pyopen = '\n[wake]\nbackend = "pyopen"\nmodel = "/data/models/wake/rover.tflite"\n'
    path = write_config(tmp_path, MINIMAL_TOML + pyopen)
    assert load_config(path, env={}).robot.name == "rover"
    with pytest.raises(ConfigError):
        load_config(path, env={"ROVER__ROBOT__NAME": "scout"})

    # A model named for the robot passes however it is decorated.
    hey = (
        '\n[wake]\nbackend = "pyopen"\n'
        'model = "/data/models/wake/hey_scout_v2.tflite"\n'
    )
    scout = write_config(tmp_path, MINIMAL_TOML + hey)
    assert load_config(scout, env={"ROVER__ROBOT__NAME": "scout"}).robot.name == "scout"

    # Every other wake backend is unaffected: the hotkey has no model at all.
    hotkey = write_config(tmp_path, MINIMAL_TOML + '\n[wake]\nbackend = "hotkey"\n')
    assert load_config(hotkey, env={"ROVER__ROBOT__NAME": "scout"}).robot.name == "scout"


def test_unknown_section_or_key_is_refused(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, MINIMAL_TOML + "\n[nonsense]\nx = 1\n"),
                    env={})
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, MINIMAL_TOML + "\nspeed_kph = 5\n"),
                    env={})


def test_malformed_override_name_is_refused(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path), env={"ROVER__SPEED": "1"})


def test_battery_table_must_be_a_table(tmp_path):
    path = write_config(
        tmp_path,
        MINIMAL_TOML + "\n[battery]\nocv_per_cell = [4.2, 3.9]\nsoc_pct = [100]\n",
    )
    with pytest.raises(ConfigError):
        load_config(path, env={})


def test_bus_source_names_match_the_message_source_enum():
    assert set(_SourceName.__args__) == {s.value for s in Source}


# --------------------------------------------------------------------------
# jsonl (A34)
# --------------------------------------------------------------------------


def test_jsonl_appends_and_reads_back(tmp_path):
    path = tmp_path / "logs" / "episode.jsonl"
    with JsonlWriter(path) as writer:
        writer.write({"x.vel": 0.1, "y.vel": 0.0, "theta.vel": 30.0})
        writer.write(WorldState.model_validate(WORLD_STATE))
    with JsonlWriter(path) as writer:
        writer.write({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
    rows = list(read_jsonl(path))
    assert len(rows) == 3
    assert rows[0] == {"x.vel": 0.1, "y.vel": 0.0, "theta.vel": 30.0}
    assert rows[1]["pose_cm"] == {"x": 142, "y": -30}


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
# Cross-module consistency
# --------------------------------------------------------------------------


def test_skill_message_args_union_is_exhaustive():
    for name, model in BUS_SKILL_ARGS.items():
        example = {
            "drive": {"distance_m": 0.1, "speed_mps": 0.1},
            "turn": {"angle_deg": 10.0, "rate_dps": 10.0},
            "say": {"text": "hello"},
            "describe_scene": {},
            "find": {"object": "mug", "max_sweeps": 1},
            "set_face": {"expr": "happy"},
        }[name]
        message = SkillMessage.model_validate(
            {**CLIENT_MESSAGES[4], "skill": name, "args": example}
        )
        assert type(message.args) is model


# --------------------------------------------------------------------------
# The JSON path, which is the one the sockets actually use
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("adapter", "message"),
    [(client_adapter, m) for m in CLIENT_MESSAGES]
    + [(server_adapter, m) for m in SERVER_MESSAGES]
    + [(brain_client_adapter, m) for m in BRAIN_MESSAGES],
    ids=[f"client-{m['type']}" for m in CLIENT_MESSAGES]
    + [f"server-{i}-{m['type']}" for i, m in enumerate(SERVER_MESSAGES)]
    + [f"brain-{m['type']}" for m in BRAIN_MESSAGES],
)
def test_documented_messages_survive_a_json_round_trip(adapter, message):
    parsed = adapter.validate_json(json.dumps(message))
    again = adapter.validate_json(adapter.dump_json(parsed))
    assert again == parsed


def test_ndjson_line_with_a_coerced_number_is_rejected():
    # A12 stage two: structured generation guarantees syntax, not values.
    line = json.dumps({**CLIENT_MESSAGES[4], "goal_ttl_ms": "5000"})
    with pytest.raises(ValidationError):
        client_adapter.validate_json(line)


def test_skill_call_from_the_box_is_validated_as_json_text():
    parsed = skill_call_adapter.validate_json(json.dumps(SKILL_CALLS[0]))
    assert parsed.args.distance_cm == 40
    with pytest.raises(ValidationError):
        skill_call_adapter.validate_json('{"speech":"x","skill":"drive",'
                                         '"args":{"distance_cm":40,"speed_cms":"15"}}')


# --------------------------------------------------------------------------
# The bounds table and the argument models must say the same thing
# --------------------------------------------------------------------------

BASE_ARGS = {
    "drive": {"distance_m": 0.1, "speed_mps": 0.1},
    "turn": {"angle_deg": 10.0, "rate_dps": 10.0},
    "say": {"text": "hello"},
    "describe_scene": {},
    "find": {"object": "mug", "max_sweeps": 1},
    "set_face": {"expr": "happy"},
    "twist": {"linear_x_mps": 0.1, "angular_z_radps": 0.1},
}

EDGE_CASES = [
    (spec.name, bound)
    for spec in CATALOG
    if spec.bus_args is not None
    for bound in spec.bounds
]


def _value(bound, magnitude):
    return "x" * int(magnitude) if bound.unit is Unit.CHARS else magnitude


@pytest.mark.parametrize(
    ("skill", "bound"), EDGE_CASES, ids=[f"{s}.{b.field}" for s, b in EDGE_CASES]
)
def test_each_bound_matches_its_argument_model(skill, bound):
    model = SKILLS[skill].bus_args
    step = 1 if bound.unit in (Unit.CHARS, Unit.COUNT) else 0.001

    inside = bound.hi if not bound.hi_exclusive else bound.hi - step
    assert bound.contains(inside)
    model.model_validate({**BASE_ARGS[skill], bound.field: _value(bound, inside)})

    outside = bound.hi + step
    assert not bound.contains(outside)
    with pytest.raises(ValidationError):
        model.model_validate({**BASE_ARGS[skill], bound.field: _value(bound, outside)})

    below = bound.lo - step
    assert not bound.contains(below)
    if bound.unit is not Unit.CHARS or below >= 0:
        with pytest.raises(ValidationError):
            model.model_validate(
                {**BASE_ARGS[skill], bound.field: _value(bound, below)}
            )
