"""The rover link protocol of ``docs/protocol.md``: what the host may send, and
what every byte the rover could send decodes to -- a value, never an exception,
because the caller is the loop that has to keep streaming zeros."""

from __future__ import annotations

import json
import math

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from rover_contracts.wave_proto import (
    FULL_SCALE,
    LINE_MAX_BYTES,
    POWER_CAP,
    Banner,
    Dropped,
    Feedback,
    Imu,
    StopFlag,
    Unknown,
    banner_request,
    coast,
    decode_line,
    echo,
    feedback_flow,
    feedback_interval,
    heartbeat,
    imu_request,
    oled,
    quiet,
    speed,
)

DECODED = (Feedback, Imu, Banner, Unknown, Dropped)

# Lines as the firmware writes them.  BANNER is verbatim from docs/protocol.md.
PATCHED = (
    b'{"T":1001,"L":0.2,"R":-0.2,"r":0.5,"p":-1.25,"y":-91.5,"temp":31.2,'
    b'"v":11.8,"hb":1,"st":2,"tf":420,"bp":0,"cc":3}\n'
)
STOCK = b'{"T":1001,"L":0,"R":0,"r":0,"p":0,"y":0,"temp":30,"v":11.5}\n'
BANNER = b'{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}\n'
IMU = (
    b'{"T":1002,"r":0.5,"p":-1.25,"y":12.0,"ax":0.01,"ay":-0.02,"az":9.81,'
    b'"gx":0.1,"gy":-0.2,"gz":30.5,"mx":1,"my":2,"mz":3,"temp":31.2}\n'
)

_DELETE = object()


def _with(base: bytes, **fields: object) -> bytes:
    """``base`` re-encoded with ``fields`` replaced; ``_DELETE`` removes a key."""
    obj = json.loads(base)
    for key, value in fields.items():
        if value is _DELETE:
            del obj[key]
        else:
            obj[key] = value
    return json.dumps(obj).encode()


# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------


def test_documented_constants():
    assert FULL_SCALE == 0.5
    assert POWER_CAP == 0.30
    assert LINE_MAX_BYTES == 512


def test_stop_flags_are_the_documented_bits():
    assert {f.name: f.value for f in StopFlag} == {
        "HEARTBEAT": 1, "TOF": 2, "BUMPER": 4, "LOWBAT": 8, "COAST": 16
    }
    assert StopFlag.TOF.blocks_forward and StopFlag.BUMPER.blocks_forward
    assert not StopFlag.TOF.blocks_all and not StopFlag.BUMPER.blocks_all
    assert StopFlag.LOWBAT.blocks_all and not StopFlag.LOWBAT.blocks_forward
    for flag in (StopFlag.HEARTBEAT, StopFlag.COAST, StopFlag(0)):
        assert not flag.blocks_forward and not flag.blocks_all
    everything = StopFlag(31)
    assert everything.blocks_forward and everything.blocks_all


# --------------------------------------------------------------------------
# Encoding, host -> rover
# --------------------------------------------------------------------------

ENCODED = {
    "speed": (speed(0.2, -0.1), {"T": 1, "L": 0.2, "R": -0.1}),
    "coast": (coast(), {"T": 115}),
    "heartbeat": (heartbeat(300), {"T": 136, "cmd": 300}),
    "feedback_on": (feedback_flow(True), {"T": 131, "cmd": 1}),
    "feedback_off": (feedback_flow(False), {"T": 131, "cmd": 0}),
    "feedback_interval": (feedback_interval(50), {"T": 142, "cmd": 50}),
    "echo_off": (echo(False), {"T": 143, "cmd": 0}),
    "echo_on": (echo(True), {"T": 143, "cmd": 1}),
    "quiet": (quiet(), {"T": 605, "cmd": 0}),
    "imu_request": (imu_request(), {"T": 126}),
    "banner_request": (banner_request(), {"T": 1007}),
    "oled": (oled(2, "hello"), {"T": 3, "lineNum": 2, "Text": "hello"}),
}


@pytest.mark.parametrize(("line", "expected"), ENCODED.values(), ids=list(ENCODED))
def test_every_encoder_writes_one_json_line(line, expected):
    assert isinstance(line, bytes)
    assert line.endswith(b"\n") and line.count(b"\n") == 1
    assert len(line) <= LINE_MAX_BYTES
    assert json.loads(line) == expected
    assert isinstance(json.loads(line)["T"], int)


@pytest.mark.parametrize(
    "line", [pair[0] for pair in ENCODED.values()], ids=list(ENCODED)
)
def test_host_lines_echoed_back_are_unknown_not_feedback(line):
    # Stock firmware echoes accepted commands until echo(False) lands; none of
    # them may be mistaken for something the rover said.
    assert isinstance(decode_line(line), Unknown)


def test_speed_line_is_the_documented_shape():
    assert speed(0.2, -0.1) == b'{"T":1,"L":0.2,"R":-0.1}\n'
    assert speed(0.0, 0.0) == b'{"T":1,"L":0.0,"R":0.0}\n'


@pytest.mark.parametrize(
    ("value", "expected"),
    [(0.30, 0.3), (0.31, 0.3), (1.0, 0.3), (FULL_SCALE, 0.3), (-0.31, -0.3),
     (-1e9, -0.3), (0.0, 0.0), (math.nan, 0.0), (math.inf, 0.0), (-math.inf, 0.0),
     (0.12345, 0.123), (-0.29999, -0.3), (0.2999, 0.3)],
)
def test_speed_clamps_to_the_cap_and_zeroes_non_finite(value, expected):
    left = json.loads(speed(value, 0.0))
    right = json.loads(speed(0.0, value))
    assert (left["L"], left["R"]) == (expected, 0.0)
    assert (right["L"], right["R"]) == (0.0, expected)


@given(st.floats(allow_nan=True, allow_infinity=True),
       st.floats(allow_nan=True, allow_infinity=True))
def test_nothing_speed_emits_exceeds_the_cap(left, right):
    line = json.loads(speed(left, right))
    for value in (line["L"], line["R"]):
        assert math.isfinite(value)
        assert abs(value) <= POWER_CAP


def test_heartbeat_and_interval_carry_integers():
    assert json.loads(heartbeat(300.0))["cmd"] == 300
    assert json.loads(feedback_interval(50.9))["cmd"] == 50


def test_oled_keeps_to_four_lines_and_ascii():
    assert json.loads(oled(5, "x"))["lineNum"] == 1        # 5 & 3
    assert len(json.loads(oled(0, "x" * 100))["Text"]) == 32
    line = oled(3, "café → \U0001f916")
    line.decode("ascii")                                    # escaped, not raw
    assert json.loads(line)["Text"] == "café → \U0001f916"


# --------------------------------------------------------------------------
# Decoding, rover -> host
# --------------------------------------------------------------------------


def test_patched_feedback_decodes_every_field():
    fb = decode_line(PATCHED)
    assert isinstance(fb, Feedback)
    assert (fb.left, fb.right) == (0.2, -0.2)
    assert (fb.roll_deg, fb.pitch_deg, fb.yaw_deg) == (0.5, -1.25, -91.5)
    assert (fb.temp_c, fb.bus_v) == (31.2, 11.8)
    assert fb.hb is True
    assert fb.st == StopFlag.TOF and isinstance(fb.st, StopFlag)
    assert fb.st.blocks_forward and not fb.st.blocks_all
    assert fb.tof_mm == 420 and fb.tof_valid
    assert fb.bumper is False
    assert fb.clamp_count == 3
    assert fb.patched


def test_stock_feedback_has_no_fork_fields():
    fb = decode_line(STOCK)
    assert isinstance(fb, Feedback)
    assert (fb.left, fb.right, fb.yaw_deg) == (0, 0, 0)
    assert (fb.temp_c, fb.bus_v) == (30, 11.5)
    assert fb.hb is None and fb.st is None and fb.tof_mm is None
    assert fb.bumper is None and fb.clamp_count is None
    assert not fb.patched
    assert not fb.tof_valid


@pytest.mark.parametrize("missing", ["hb", "st", "tf", "bp", "cc"])
def test_patched_needs_all_five_fork_fields(missing):
    absent = decode_line(_with(PATCHED, **{missing: _DELETE}))
    null = decode_line(_with(PATCHED, **{missing: None}))
    for fb in (absent, null):
        assert isinstance(fb, Feedback)
        assert not fb.patched


@pytest.mark.parametrize(
    ("st_value", "expected"),
    [(0, StopFlag(0)), (1, StopFlag.HEARTBEAT), (2, StopFlag.TOF), (4, StopFlag.BUMPER),
     (8, StopFlag.LOWBAT), (16, StopFlag.COAST), (6, StopFlag.TOF | StopFlag.BUMPER),
     (31, StopFlag(31)), (32, StopFlag(0)), (63, StopFlag(31))],
)
def test_stop_flags_parse_and_unknown_bits_are_masked(st_value, expected):
    fb = decode_line(_with(PATCHED, st=st_value))
    assert isinstance(fb, Feedback)
    assert fb.st == expected
    assert fb.patched


@pytest.mark.parametrize(
    ("tf", "valid"), [(420, True), (0, True), (-1, False), (-7, False)]
)
def test_tof_valid_is_a_non_negative_reading(tf, valid):
    fb = decode_line(_with(PATCHED, tf=tf))
    assert isinstance(fb, Feedback)
    assert fb.tof_mm == tf
    assert fb.tof_valid is valid


def test_hb_and_bumper_are_booleans_from_integers():
    fb = decode_line(_with(PATCHED, hb=0, bp=1))
    assert isinstance(fb, Feedback)
    assert fb.hb is False and fb.bumper is True


def test_banner_decodes_the_documented_line():
    banner = decode_line(BANNER)
    assert banner == Banner(fw="bot-wr-1", hb_ms=300, cap=0.3, proto=1)
    assert isinstance(banner.hb_ms, int) and isinstance(banner.proto, int)


def test_imu_decodes_the_documented_fields():
    imu = decode_line(IMU)
    assert isinstance(imu, Imu)
    assert (imu.roll_deg, imu.pitch_deg, imu.yaw_deg) == (0.5, -1.25, 12.0)
    assert imu.gyro_dps == (0.1, -0.2, 30.5)
    assert imu.accel == (0.01, -0.02, 9.81)
    assert imu.temp_c == 31.2


def test_str_input_decodes_like_bytes():
    assert decode_line(BANNER.decode()) == decode_line(BANNER)
    assert decode_line(PATCHED.decode()) == decode_line(PATCHED)


def test_a_line_at_the_size_limit_is_kept():
    prefix, suffix = b'{"T":42,"pad":"', b'"}\n'
    line = prefix + b"x" * (LINE_MAX_BYTES - len(prefix) - len(suffix)) + suffix
    assert len(line) == LINE_MAX_BYTES
    assert decode_line(line) == Unknown(42)
    assert decode_line(line + b"x") == Dropped("oversize")


DROPPED = {
    "garbage_text": (b"UGV started.\n", "not_json"),
    "invalid_utf8": (b"\xff\xfe\xfd", "not_json"),
    "half_a_line": (b'{"T":1001,"L":0.1', "not_json"),
    "array": (b"[1,2,3]\n", "not_object"),
    "number": (b"1001\n", "not_object"),
    "string": (b'"T"\n', "not_object"),
    "null": (b"null\n", "not_object"),
    "missing_T": (b'{"L":0.1,"R":0.1}\n', "no_type"),
    "boolean_T": (b'{"T":true}\n', "no_type"),
    "float_T": (b'{"T":1001.0}\n', "no_type"),
    "string_T": (b'{"T":"1001"}\n', "no_type"),
    "null_T": (b'{"T":null}\n', "no_type"),
    "empty": (b"", "empty"),
    "newline_only": (b"\r\n", "empty"),
    "oversize": (b'{"T":1001,"pad":"' + b"x" * LINE_MAX_BYTES + b'"}\n', "oversize"),
    "feedback_float_tf": (_with(PATCHED, tf=12.5), "bad_fields"),
    "feedback_string_yaw": (_with(PATCHED, y="12"), "bad_fields"),
    "feedback_bool_left": (_with(PATCHED, L=True), "bad_fields"),
    "feedback_nan": (_with(STOCK, y=math.nan), "bad_fields"),
    "feedback_inf": (_with(STOCK, v=math.inf), "bad_fields"),
    "feedback_missing_v": (_with(STOCK, v=_DELETE), "bad_fields"),
    "feedback_null_yaw": (_with(STOCK, y=None), "bad_fields"),
    "feedback_list_st": (_with(PATCHED, st=[2]), "bad_fields"),
    "banner_missing_cap": (_with(BANNER, cap=_DELETE), "bad_fields"),
    "banner_fw_not_string": (_with(BANNER, fw=1), "bad_fields"),
    "banner_fw_empty": (_with(BANNER, fw=""), "bad_fields"),
    "banner_fw_too_long": (_with(BANNER, fw="x" * 33), "bad_fields"),
    "banner_string_hb": (_with(BANNER, hb_ms="300"), "bad_fields"),
    "imu_missing_gz": (_with(IMU, gz=_DELETE), "bad_fields"),
}


@pytest.mark.parametrize(("raw", "reason"), DROPPED.values(), ids=list(DROPPED))
def test_malformed_lines_are_dropped_with_a_reason(raw, reason):
    assert decode_line(raw) == Dropped(reason)


@pytest.mark.parametrize(
    ("raw", "t"),
    [(b'{"T":1003}\n', 1003), (b'{"T":1}\n', 1), (b'{"T":0}\n', 0), (b'{"T":-5}\n', -5),
     (b'{"T":1005,"L":0.1}\n', 1005), (b'{"T":' + b"9" * 400 + b"}", int("9" * 400))],
    ids=["1003", "echo_of_speed", "zero", "negative", "with_fields", "huge"],
)
def test_well_formed_lines_with_other_types_are_unknown(raw, t):
    assert decode_line(raw) == Unknown(t)


# --------------------------------------------------------------------------
# Fuzz: decode_line never raises, whatever arrives
# --------------------------------------------------------------------------

# No per-example deadline: a slow CI runner must not turn a passing property
# into a flaky one.

JSON_SCALARS = st.one_of(
    st.none(), st.booleans(), st.integers(), st.floats(), st.text(max_size=40),
    st.lists(st.integers(), max_size=3),
)
FIELD_KEYS = ["L", "R", "r", "p", "y", "temp", "v", "hb", "st", "tf", "bp", "cc",
              "fw", "hb_ms", "cap", "proto", "ax", "ay", "az", "gx", "gy", "gz",
              "mx", "my", "mz"]
TYPE_VALUES = st.one_of(
    st.sampled_from([1, 1001, 1002, 1006, 1007]), st.integers(), st.booleans(),
    st.none(), st.floats(), st.text(max_size=8),
)


def _check(result) -> None:
    assert isinstance(result, DECODED)
    if isinstance(result, Feedback):
        fork = (result.hb, result.st, result.tof_mm, result.bumper, result.clamp_count)
        assert result.patched == (None not in fork)
        assert result.tof_valid == (result.tof_mm is not None and result.tof_mm >= 0)
        assert result.st is None or 0 <= int(result.st) <= 31


@settings(max_examples=500, deadline=None)
@given(st.binary(max_size=LINE_MAX_BYTES + 32))
def test_random_bytes_never_raise(raw):
    _check(decode_line(raw))


@settings(max_examples=300, deadline=None)
@given(st.text(max_size=LINE_MAX_BYTES + 32))
def test_random_text_never_raises(text):
    _check(decode_line(text))


@settings(max_examples=500, deadline=None)
@given(st.fixed_dictionaries(
    {"T": TYPE_VALUES}, optional={key: JSON_SCALARS for key in FIELD_KEYS}
))
def test_random_json_objects_never_raise(obj):
    _check(decode_line(json.dumps(obj).encode()))
    _check(decode_line(json.dumps(obj) + "\n"))
