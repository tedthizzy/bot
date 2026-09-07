"""The ARCHITECTURE 5.1 line protocol, against the shared golden vectors."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.serial_codec import (  # noqa: E402
    ADVISORY_FAULTS,
    DOWN_FRAME_TYPES,
    FRAME_TYPES,
    LATCHED_FAULTS,
    MAX_LINE_BYTES,
    OBSTACLE_FAULTS,
    PROTO_VER,
    UP_FRAME_TYPES,
    AckReason,
    ArmFrame,
    CtrlFlag,
    DecodeErr,
    DecodeOk,
    DisarmFrame,
    Fault,
    FrameReader,
    HelloFrame,
    McuState,
    SessionGuard,
    TelemetryFrame,
    VelocityFrame,
    ack_type_code,
    ack_type_letter,
    crc16_ccitt_false,
    decode_line,
    encode_frame,
    fault_mask,
    fault_names,
    frame_ttl_ok,
    safety_hash,
    seq_is_newer,
)

VECTORS_PATH = Path(__file__).resolve().parents[1] / "contract" / "serial_vectors.jsonl"
VECTORS = [json.loads(line) for line in VECTORS_PATH.read_text().splitlines() if line]
FRAMES = [v for v in VECTORS if v["kind"] == "frame"]
REJECTS = [v for v in VECTORS if v["kind"] == "reject"]


def _ids(rows):
    return [row["name"] for row in rows]


# --------------------------------------------------------------------------
# The algorithm itself
# --------------------------------------------------------------------------


def test_crc_check_value():
    probe = next(v for v in VECTORS if v["kind"] == "crc")
    assert crc16_ccitt_false(probe["input"].encode()) == probe["crc_u16"]
    assert crc16_ccitt_false(b"123456789") == 0x29B1


def test_safety_hash_matches_the_boot_banner():
    row = next(v for v in VECTORS if v["kind"] == "safety_hash")
    assert safety_hash(*row["values"]) == row["crc32"]
    banner = next(v for v in FRAMES if v["type"] == "B")
    assert banner["fields"]["safety_hash"] == row["crc32"]


# --------------------------------------------------------------------------
# Golden vectors
# --------------------------------------------------------------------------


@pytest.mark.parametrize("vector", FRAMES, ids=_ids(FRAMES))
def test_golden_vector_decodes_with_the_documented_fields(vector):
    result = decode_line(vector["line"].encode())
    assert isinstance(result, DecodeOk), result
    frame = result.frame
    assert vector["type"] == type(frame).TYPE
    assert frame.seq == vector["seq"]
    assert frame.session == vector["session"]
    for name, value in vector["fields"].items():
        assert getattr(frame, name) == value, name


@pytest.mark.parametrize("vector", FRAMES, ids=_ids(FRAMES))
def test_golden_vector_round_trips_byte_exactly(vector):
    result = decode_line(vector["line"].encode())
    assert encode_frame(result.frame).decode() == vector["line"]


@pytest.mark.parametrize("vector", FRAMES, ids=_ids(FRAMES))
def test_golden_vector_crc_and_direction(vector):
    assert f"{crc16_ccitt_false(vector['body'].encode()):04X}" == vector["crc"]
    expected = "down" if vector["type"] in DOWN_FRAME_TYPES else "up"
    assert vector["dir"] == expected


@pytest.mark.parametrize("vector", REJECTS, ids=_ids(REJECTS))
def test_reject_vector_fails_for_exactly_its_stated_reason(vector):
    result = decode_line(vector["line"].encode())
    assert isinstance(result, DecodeErr), vector["name"]
    assert result.reason.name.lower() == vector["reason"]
    assert int(result.reason) == vector["reason_code"]


def test_every_frame_type_has_a_golden_vector():
    covered = {v["type"] for v in FRAMES}
    missing = set(FRAME_TYPES) - covered
    # O (pong) is diagnostic-only and ARCHITECTURE 5.1 gives no vector for it.
    # P has one because it is where the u64 boundary is pinned: the C decoder
    # holds every field in an int64_t, so the two codecs have to agree on 2**63.
    assert missing == {"O"}


def test_telemetry_field_order_is_the_documented_one():
    names = [spec.name for spec in TelemetryFrame.FIELDS]
    assert names == [
        "mcu_us", "ack_seq", "state", "ctrl_flags", "fault",
        "left_ticks", "right_ticks", "v_meas_mm_s", "w_meas_mrad_s",
        "v_cmd_mm_s", "w_cmd_mrad_s", "vbat_mv", "imotor_ma",
        "tof_front_mm", "tof_cliff_mm", "sensor_age_ms", "loop_late_pct",
        "rx_drop", "gyro_z_mrad_s", "rails", "motion",
    ]


def test_obstacle_telemetry_decodes_to_the_documented_meaning():
    vector = next(v for v in FRAMES if v["name"] == "t_obstacle")
    frame = decode_line(vector["line"].encode()).frame
    assert Fault(frame.fault) is Fault.TOF_STOP
    assert frame.state == McuState.ARMED_IDLE  # obstacle class does not latch
    flags = CtrlFlag(frame.ctrl_flags)
    assert CtrlFlag.TOF_CLEAR not in flags
    assert CtrlFlag.IN_SLOW_ZONE in flags
    assert frame.tof_front_mm == 231


def test_clamped_velocity_is_decoded_not_rejected():
    # A cap is a robotd/MCU rule, not a codec rule: 900 mm/s is a legal i16 and
    # the wire carries it so the MCU can clamp it and raise CAP_CLAMPED.
    vector = next(v for v in FRAMES if v["name"] == "v_over_cap")
    frame = decode_line(vector["line"].encode()).frame
    assert frame.v_mm_s == 900


# --------------------------------------------------------------------------
# Grammar rules
# --------------------------------------------------------------------------


def test_line_without_dollar_is_rejected():
    result = decode_line(b"V,2,3,40010,250,210,300,0*97E7")
    assert result.reason is AckReason.BAD_LENGTH


def test_short_crc_is_rejected():
    result = decode_line(b"$D,2,8,40010*DFC")
    assert result.reason is AckReason.BAD_LENGTH


def test_hello_may_carry_the_wildcard_session():
    result = decode_line(b"$H,2,1,0,3735928559*B2C8")
    assert isinstance(result, DecodeOk)
    assert result.frame.session == 0


def test_encode_refuses_the_wildcard_session_outside_hello():
    with pytest.raises(ValueError):
        encode_frame(DisarmFrame(seq=1, session=0))


def test_encode_refuses_a_value_outside_its_wire_type():
    with pytest.raises(ValueError):
        encode_frame(VelocityFrame(1, 40010, 40000, 0, 300, 0))
    with pytest.raises(ValueError):
        encode_frame(ArmFrame(70000, 40010, 1))


def test_version_mismatch_is_not_negotiated():
    body = b"H,3,1,0,1"
    line = b"$" + body + b"*" + f"{crc16_ccitt_false(body):04X}".encode()
    assert decode_line(line).reason is AckReason.UNSUPPORTED_VERSION
    assert PROTO_VER == 2


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------


def test_leading_newline_after_port_open_is_legal():
    reader = FrameReader()
    results = reader.feed(b"\n$D,2,8,40010*DFCC\n")
    assert len(results) == 1
    assert isinstance(results[0], DecodeOk)
    assert reader.counters.dropped == 0


def test_over_length_line_is_dropped_to_the_next_newline_and_counted():
    vector = next(v for v in REJECTS if v["name"] == "over_length_line")
    reader = FrameReader()
    results = reader.feed(vector["line"].encode() + b"\n$D,2,8,40010*DFCC\n")
    assert [type(r).__name__ for r in results] == ["DecodeErr", "DecodeOk"]
    assert results[0].reason is AckReason.BAD_LENGTH
    assert reader.counters.overlong == 1
    assert reader.counters.ok == 1


def test_over_length_line_arriving_in_pieces_is_counted_once():
    reader = FrameReader()
    reader.feed(b"$" + b"1" * (MAX_LINE_BYTES + 50))
    reader.feed(b"1" * 500)
    results = reader.feed(b"\n$D,2,8,40010*DFCC\n")
    assert reader.counters.overlong == 1
    assert any(isinstance(r, DecodeOk) for r in results)


def test_reader_resynchronises_after_garbage():
    reader = FrameReader()
    results = reader.feed(b"\xff\xfe garbage\n$D,2,8,40010*DFCC\n")
    assert isinstance(results[0], DecodeErr)
    assert isinstance(results[1], DecodeOk)
    assert reader.counters.dropped == 1


def test_split_frame_is_reassembled():
    reader = FrameReader()
    assert reader.feed(b"$D,2,8,") == []
    results = reader.feed(b"40010*DFCC\n")
    assert isinstance(results[0], DecodeOk)


def test_counters_track_each_failure_class():
    reader = FrameReader()
    for vector in REJECTS:
        if vector["name"] == "over_length_line":
            continue
        reader.feed(vector["line"].encode() + b"\n")
    assert reader.counters.bad_crc == 2
    # Two: the version mismatch, plus the over-long field list, whose reason
    # must be the version check and not the field-count cap (both codecs).
    assert reader.counters.unsupported_version == 2
    assert reader.counters.unknown_type == 1
    assert reader.counters.bad_session == 1
    assert reader.counters.ok == 0


# --------------------------------------------------------------------------
# Session and sequence (A8)
# --------------------------------------------------------------------------


def test_seq_comparison_is_int16_of_the_difference():
    assert seq_is_newer(2, 1)
    assert not seq_is_newer(1, 1)
    assert not seq_is_newer(1, 2)
    assert seq_is_newer(0, 65535)  # wraps forward
    assert not seq_is_newer(65535, 0)
    assert seq_is_newer(32767, 0)
    assert not seq_is_newer(32768, 0)  # exactly int16 min: not newer


def test_stale_sequence_is_refused_and_last_seq_never_moves_backwards():
    guard = SessionGuard(session=40010, last_seq=5)
    assert guard.accept(VelocityFrame(6, 40010, 0, 0, 300, 0)) is AckReason.NONE
    assert guard.last_seq == 6
    assert guard.accept(VelocityFrame(6, 40010, 0, 0, 300, 0)) is AckReason.STALE_SEQ
    assert guard.accept(VelocityFrame(3, 40010, 0, 0, 300, 0)) is AckReason.STALE_SEQ
    assert guard.last_seq == 6


def test_foreign_session_is_refused():
    guard = SessionGuard(session=40010, last_seq=1)
    assert guard.check(VelocityFrame(2, 51882, 0, 0, 300, 0)) is AckReason.BAD_SESSION


def test_hello_wildcard_is_accepted_whatever_session_is_held():
    guard = SessionGuard(session=40010, last_seq=1)
    assert guard.check(HelloFrame(2, 0, 1)) is AckReason.NONE


def test_unlearned_guard_accepts_any_session():
    guard = SessionGuard()
    assert guard.accept(VelocityFrame(1, 51882, 0, 0, 300, 0)) is AckReason.NONE


# --------------------------------------------------------------------------
# Frame TTL, faults, ack types
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("ttl", "ok"),
    [(0, True), (49, False), (50, True), (300, True), (500, True), (501, False),
     (5000, False)],
)
def test_frame_ttl_range(ttl, ok):
    assert frame_ttl_ok(ttl) is ok


def test_fault_classes_partition_the_bitmask():
    every = 0
    for bit in Fault:
        every |= int(bit)
    assert int(ADVISORY_FAULTS) | int(OBSTACLE_FAULTS) | int(LATCHED_FAULTS) == every
    assert int(ADVISORY_FAULTS) & int(OBSTACLE_FAULTS) == 0
    assert int(ADVISORY_FAULTS) & int(LATCHED_FAULTS) == 0
    assert int(OBSTACLE_FAULTS) & int(LATCHED_FAULTS) == 0


def test_obstacle_class_is_exactly_the_self_clearing_four():
    assert set(fault_names(OBSTACLE_FAULTS)) == {
        "bumper", "tof_stop", "tof_stale", "cliff"
    }


def test_fault_names_round_trip():
    mask = int(Fault.OVERCURRENT | Fault.STALL)
    assert fault_names(mask) == ("overcurrent", "stall")
    assert fault_mask(("overcurrent", "stall")) == mask
    clear = next(v for v in FRAMES if v["name"] == "c_clear_latched")
    assert clear["fields"]["mask"] == mask


def test_ack_type_codes_are_the_ascii_letters():
    assert ack_type_code("V") == 86
    assert ack_type_code("A") == 65
    assert ack_type_code("S") == 83
    assert ack_type_code("C") == 67
    assert ack_type_code("D") == 68
    assert ack_type_letter(86) == "V"


def test_frame_directions_match_the_architecture():
    assert {"H", "A", "D", "V", "S", "C", "P"} == DOWN_FRAME_TYPES
    assert {"B", "T", "K", "E", "O"} == UP_FRAME_TYPES


# --------------------------------------------------------------------------
# Fuzz: a malformed input returns an error value and never raises
# --------------------------------------------------------------------------


@settings(max_examples=500, deadline=None)
@given(st.binary(max_size=400))
def test_decode_line_never_raises(data):
    result = decode_line(data)
    assert isinstance(result, DecodeOk | DecodeErr)


@settings(max_examples=300, deadline=None)
@given(st.lists(st.binary(max_size=300), max_size=12))
def test_reader_never_raises(chunks):
    reader = FrameReader()
    for chunk in chunks:
        for result in reader.feed(chunk):
            assert isinstance(result, DecodeOk | DecodeErr)


@settings(max_examples=300, deadline=None)
@given(
    st.sampled_from(sorted(FRAME_TYPES)),
    st.integers(min_value=0, max_value=0xFFFF),
    st.integers(min_value=1, max_value=0xFFFF),
)
def test_every_encodable_frame_round_trips(type_letter, seq, session):
    cls = FRAME_TYPES[type_letter]
    values = [spec.lo for spec in cls.FIELDS]
    frame = cls(seq, session, *values)
    line = encode_frame(frame)
    assert len(line) <= MAX_LINE_BYTES + 1
    result = decode_line(line)
    assert isinstance(result, DecodeOk)
    assert result.frame == frame
