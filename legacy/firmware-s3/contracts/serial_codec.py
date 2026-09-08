"""The Pi<->MCU line protocol of ARCHITECTURE 5.1.

``frame ::= "$" body "*" CRC "\\n"`` with
``body ::= TYPE "," VER "," SEQ "," SESS [ "," FIELD ]*``.  Every field is a
signed decimal integer or an unsigned uppercase hex word; there are no floats,
no quoted strings and no spaces, which is what makes ``\\n`` a deterministic
resynchronisation point (A5).

Everything here is pure: :func:`decode_line` and :class:`FrameReader` return an
error value for malformed input and never raise into a caller's loop, so a
corrupted UART byte cannot take down the 50 Hz telemetry task.  The golden
vectors in ``tests/contract/serial_vectors.jsonl`` are the shared contract with
the C firmware core.
"""

from __future__ import annotations

import zlib
from collections.abc import Iterable
from dataclasses import dataclass
from dataclasses import fields as dataclass_fields
from enum import IntEnum, IntFlag
from typing import ClassVar, Final

__all__ = [
    "ADVISORY_FAULTS",
    "AckFrame",
    "AckReason",
    "AckResult",
    "ArmFrame",
    "BootFrame",
    "CapBit",
    "ClearFaultFrame",
    "CtrlFlag",
    "DOWN_FRAME_TYPES",
    "DecodeErr",
    "DecodeOk",
    "DecodeResult",
    "DisarmFrame",
    "EventCode",
    "EventFrame",
    "FRAME_TTL_IMMEDIATE",
    "FRAME_TTL_MAX_MS",
    "FRAME_TTL_MIN_MS",
    "FRAME_TYPES",
    "Fault",
    "FieldSpec",
    "Frame",
    "FrameReader",
    "HelloFrame",
    "LATCHED_FAULTS",
    "MAX_LINE_BYTES",
    "McuState",
    "OBSTACLE_FAULTS",
    "PROTO_VER",
    "PingFrame",
    "PongFrame",
    "RxCounters",
    "SAFETY_HASH_KEYS",
    "SESSION_WILDCARD",
    "SessionGuard",
    "StopFrame",
    "TOF_ERROR_MM",
    "TOF_NO_TARGET_MM",
    "TelemetryFrame",
    "UP_FRAME_TYPES",
    "VFlag",
    "VelocityFrame",
    "ack_type_code",
    "ack_type_letter",
    "crc16_ccitt_false",
    "decode_line",
    "encode_frame",
    "fault_mask",
    "fault_names",
    "frame_ttl_ok",
    "safety_hash",
    "seq_is_newer",
]

PROTO_VER: Final = 2
"""``VER``; a mismatch is rejected with reason 6, there is no negotiation."""

MAX_LINE_BYTES: Final = 200
"""Longest legal line excluding the terminating newline; longer is dropped."""

SESSION_WILDCARD: Final = 0
"""``SESS`` 0.  Only ``H`` may send it; the MCU never mints it."""

FRAME_TTL_IMMEDIATE: Final = 0
FRAME_TTL_MIN_MS: Final = 50
FRAME_TTL_MAX_MS: Final = 500

TOF_NO_TARGET_MM: Final = 65534
"""No target within range: a clear path, not a fault (I-16)."""

TOF_ERROR_MM: Final = 65535
"""Sensor/I2C error or a sample older than 200 ms: forward refused."""

SAFETY_HASH_KEYS: Final = (
    "tof_stop_mm",
    "tof_slow_mm",
    "slow_zone_w_mrad_s",
    "tof_timing_budget_ms",
    "tof_inter_period_ms",
    "tof_poll_hz",
    "cliff_delta_mm",
)
"""The seven compiled safety constants, in the fixed order ``B.safety_hash``
is computed over."""

_U8 = (0, 0xFF)
_U16 = (0, 0xFFFF)
_U32 = (0, 0xFFFFFFFF)
# INT64_MAX, not 2**64-1: the C codec holds every decoded field in an int64_t,
# so a value above this is rejected there. One grammar, not two -- the boundary
# is pinned by a pair of golden vectors.
_U64 = (0, 0x7FFFFFFFFFFFFFFF)
_I16 = (-0x8000, 0x7FFF)
_I32 = (-0x80000000, 0x7FFFFFFF)


class AckResult(IntEnum):
    """``K.result``."""

    OK = 0
    REJECT = 1
    CLAMPED = 2


class AckReason(IntEnum):
    """``K.reason``, and the reason a decode or a session check failed."""

    NONE = 0
    BAD_SESSION = 1
    STALE_SEQ = 2
    BAD_CRC = 3
    BAD_LENGTH = 4
    UNKNOWN_TYPE = 5
    UNSUPPORTED_VERSION = 6
    NOT_ARMED = 7
    FAULT_LATCHED = 8
    ESTOP_ASSERTED = 9
    BUMPER_CLOSED = 10
    TOF_BLOCKED = 11
    UNDERVOLTAGE = 12
    CAP_EXCEEDED = 13
    FRAME_TTL_OUT_OF_RANGE = 14
    SENSORS_STALE = 15
    ARM_DENIED_MOVING = 16


class McuState(IntEnum):
    """``T.state``.  ``.name`` is also the string the robotd bus publishes."""

    BOOT = 0
    DISARMED = 1
    ARMED_IDLE = 2
    ARMED_MOVING = 3
    FAULT = 4
    ESTOP = 5


class CtrlFlag(IntFlag):
    """``T.ctrl_flags``; bits 10-15 are reserved and must be 0."""

    TTL_OK = 1 << 0
    ESTOP_RELEASED = 1 << 1
    BUMPER_CLEAR = 1 << 2
    TOF_CLEAR = 1 << 3
    PWM_ENABLED = 1 << 4
    IN_SLOW_ZONE = 1 << 5
    CAL_VALID = 1 << 6
    DEBUG_BUILD = 1 << 7
    TOF_FL_OK = 1 << 8
    TOF_FR_OK = 1 << 9


class Fault(IntFlag):
    """``T.fault``; bits 21-31 are reserved.

    ``OBSTACLE_LATCHED`` at bit 20 is what escalation raises: 5.1 moves an
    obstacle-class cause *into the latched class*, and a bit no ``C`` can name
    is not that.
    """

    TTL = 0x1
    ESTOP = 0x2
    BUMPER = 0x4
    TOF_STOP = 0x8
    TOF_STALE = 0x10
    CLIFF = 0x20
    OVERCURRENT = 0x40
    STALL = 0x80
    UNDERVOLT_W = 0x100
    UNDERVOLT_S = 0x200
    UNDERVOLT_D = 0x400
    DRIVER_FAULT = 0x800
    ENC_IMPLAUS = 0x1000
    LOOP_OVERRUN = 0x2000
    LINK_CRC = 0x4000
    SESSION = 0x8000
    CAP_CLAMPED = 0x10000
    WDT_REBOOT = 0x20000
    BROWNOUT = 0x40000
    DRIVER_HOT = 0x80000
    OBSTACLE_LATCHED = 0x100000


ADVISORY_FAULTS: Final = (
    Fault.TTL | Fault.UNDERVOLT_W | Fault.SESSION | Fault.CAP_CLAMPED
)
"""Set and cleared freely; robotd's readiness gate ignores this class."""

OBSTACLE_FAULTS: Final = Fault.TOF_STOP | Fault.TOF_STALE | Fault.BUMPER | Fault.CLIFF
"""Blocking but self-clearing; the MCU is the sole clearer of these bits."""

LATCHED_FAULTS: Final = Fault(
    sum(bit for bit in Fault) & ~int(ADVISORY_FAULTS) & ~int(OBSTACLE_FAULTS)
)
"""Enters FAULT, refuses ``V`` with reason 8, needs an explicit ``C``."""


class EventCode(IntEnum):
    """``E.event``.  The text table lives on the Pi; the numbers live here."""

    ARM_OK = 1
    ARM_DENIED = 2
    TTL_EXPIRED = 3
    TTL_RECOVERED = 4
    FAULT_SET = 5
    FAULT_CLEARED = 6
    CAP_CLAMP = 7
    WDT_REBOOT = 8
    BROWNOUT = 9
    I2C_ERROR = 10
    TOF_STATUS = 11
    SESSION_RESET = 12
    LOOP_OVERRUN = 13
    STALL = 14
    CAL_STORED = 15


class VFlag(IntFlag):
    """``V.flags``.  Every bit is narrowing-only (I-4); b2-7 reserved = 0."""

    REQUIRE_SLOW_ZONE_STOP = 1 << 0
    SERVO_RAIL_EN = 1 << 1


class CapBit(IntFlag):
    """``B.caps`` feature bitmask."""

    DEBUG_BUILD = 1 << 0
    CLIFF_SENSOR = 1 << 1
    IMU = 1 << 2
    INA = 1 << 3
    SERVO_RAIL = 1 << 4


def fault_names(mask: int) -> tuple[str, ...]:
    """Lowercase bus names for every fault bit set in ``mask``, low bit first."""
    return tuple(bit.name.lower() for bit in Fault if mask & bit)


def fault_mask(names: Iterable[str]) -> int:
    """Inverse of :func:`fault_names`."""
    mask = 0
    for name in names:
        mask |= int(Fault[name.upper()])
    return mask


def ack_type_code(letter: str) -> int:
    """``K.ack_type``: the decimal ASCII code of the acked type letter."""
    if len(letter) != 1 or not ("A" <= letter <= "Z"):
        raise ValueError(f"not a frame type letter: {letter!r}")
    return ord(letter)


def ack_type_letter(code: int) -> str:
    """Inverse of :func:`ack_type_code`."""
    if not 0x41 <= code <= 0x5A:
        raise ValueError(f"not a frame type code: {code!r}")
    return chr(code)


def frame_ttl_ok(frame_ttl_ms: int) -> bool:
    """``frame_ttl_ms`` is the one field clamping does not apply to: any value
    other than 0 or 50..500 is rejected with reason 14 (ARCHITECTURE 5.1)."""
    return frame_ttl_ms == FRAME_TTL_IMMEDIATE or (
        FRAME_TTL_MIN_MS <= frame_ttl_ms <= FRAME_TTL_MAX_MS
    )


def safety_hash(
    tof_stop_mm: int,
    tof_slow_mm: int,
    slow_zone_w_mrad_s: int,
    tof_timing_budget_ms: int,
    tof_inter_period_ms: int,
    tof_poll_hz: int,
    cliff_delta_mm: int,
) -> int:
    """CRC-32 over the seven compiled safety constants, in the fixed order of
    :data:`SAFETY_HASH_KEYS`, joined by commas.  Equals ``B.safety_hash``."""
    body = ",".join(
        str(int(value))
        for value in (
            tof_stop_mm,
            tof_slow_mm,
            slow_zone_w_mrad_s,
            tof_timing_budget_ms,
            tof_inter_period_ms,
            tof_poll_hz,
            cliff_delta_mm,
        )
    )
    return zlib.crc32(body.encode("ascii"))


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor-out."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def seq_is_newer(seq: int, last: int) -> bool:
    """``(int16)(seq - last) > 0`` on uint16 counters."""
    delta = (seq - last) & 0xFFFF
    if delta >= 0x8000:
        delta -= 0x10000
    return delta > 0


@dataclass(frozen=True, slots=True)
class FieldSpec:
    """One wire field: its name, its integer range, and how it is written.

    ``hex_width`` is ``None`` for a decimal field, ``0`` for minimal-width
    uppercase hex and ``n`` for hex zero-padded to ``n`` digits.  The widths are
    what the golden vectors use, so encode is byte-exact against them.
    """

    name: str
    lo: int
    hi: int
    hex_width: int | None = None

    def render(self, value: int) -> str:
        if not self.lo <= value <= self.hi:
            raise ValueError(
                f"{self.name} must be {self.lo}..{self.hi}, got {value!r}"
            )
        if self.hex_width is None:
            return str(value)
        return f"{value:0{self.hex_width}X}"


class FrameBase:
    """Shared class-level wire description; ``seq`` and ``session`` come first
    in every frame body, right after ``TYPE`` and ``VER``."""

    __slots__ = ()

    TYPE: ClassVar[str]
    FIELDS: ClassVar[tuple[FieldSpec, ...]]
    DOWN: ClassVar[bool]

    seq: int
    session: int


@dataclass(frozen=True, slots=True)
class HelloFrame(FrameBase):
    """``H`` hello.  The one frame that may carry the wildcard session 0."""

    TYPE: ClassVar[str] = "H"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (FieldSpec("host_boot_id", *_U32),)

    seq: int
    session: int
    host_boot_id: int


@dataclass(frozen=True, slots=True)
class ArmFrame(FrameBase):
    """``A`` arm.  ``nonce`` comes back in ``K.echo`` so a duplicate is visible."""

    TYPE: ClassVar[str] = "A"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (FieldSpec("nonce", *_U32),)

    seq: int
    session: int
    nonce: int


@dataclass(frozen=True, slots=True)
class DisarmFrame(FrameBase):
    """``D`` disarm."""

    TYPE: ClassVar[str] = "D"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = ()

    seq: int
    session: int


@dataclass(frozen=True, slots=True)
class VelocityFrame(FrameBase):
    """``V`` velocity at 20 Hz.  Body velocity, REP-103 (A6)."""

    TYPE: ClassVar[str] = "V"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("v_mm_s", *_I16),
        FieldSpec("w_mrad_s", *_I16),
        FieldSpec("frame_ttl_ms", *_U16),
        FieldSpec("flags", *_U8),
    )

    seq: int
    session: int
    v_mm_s: int
    w_mrad_s: int
    frame_ttl_ms: int
    flags: int


@dataclass(frozen=True, slots=True)
class StopFrame(FrameBase):
    """``S`` stop; mode 0 brake, 1 coast."""

    TYPE: ClassVar[str] = "S"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (FieldSpec("mode", *_U8),)

    seq: int
    session: int
    mode: int


@dataclass(frozen=True, slots=True)
class ClearFaultFrame(FrameBase):
    """``C`` clear_fault, naming the latched bits to clear."""

    TYPE: ClassVar[str] = "C"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (FieldSpec("mask", *_U32, hex_width=4),)

    seq: int
    session: int
    mask: int


@dataclass(frozen=True, slots=True)
class PingFrame(FrameBase):
    """``P`` ping.  ``pi_mono_us`` is an opaque token, never read as a time."""

    TYPE: ClassVar[str] = "P"
    DOWN: ClassVar[bool] = True
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (FieldSpec("pi_mono_us", *_U64),)

    seq: int
    session: int
    pi_mono_us: int


@dataclass(frozen=True, slots=True)
class BootFrame(FrameBase):
    """``B`` boot banner, at reset and 1 Hz until the first valid ``H``."""

    TYPE: ClassVar[str] = "B"
    DOWN: ClassVar[bool] = False
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("fw_ver", *_U32),
        FieldSpec("proto_ver", *_U8),
        FieldSpec("caps", *_U16, hex_width=4),
        FieldSpec("reset_reason", *_U8),
        FieldSpec("safety_hash", *_U32),
    )

    seq: int
    session: int
    fw_ver: int
    proto_ver: int
    caps: int
    reset_reason: int
    safety_hash: int


@dataclass(frozen=True, slots=True)
class TelemetryFrame(FrameBase):
    """``T`` telemetry at 50 Hz from boot, whether or not an ``H`` has arrived."""

    TYPE: ClassVar[str] = "T"
    DOWN: ClassVar[bool] = False
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("mcu_us", *_U64),
        FieldSpec("ack_seq", *_U16),
        FieldSpec("state", *_U8),
        FieldSpec("ctrl_flags", *_U16, hex_width=0),
        FieldSpec("fault", *_U32, hex_width=0),
        FieldSpec("left_ticks", *_I32),
        FieldSpec("right_ticks", *_I32),
        FieldSpec("v_meas_mm_s", *_I16),
        FieldSpec("w_meas_mrad_s", *_I16),
        FieldSpec("v_cmd_mm_s", *_I16),
        FieldSpec("w_cmd_mrad_s", *_I16),
        FieldSpec("vbat_mv", *_U16),
        FieldSpec("imotor_ma", *_I16),
        FieldSpec("tof_front_mm", *_U16),
        FieldSpec("tof_cliff_mm", *_U16),
        FieldSpec("sensor_age_ms", *_U8),
        FieldSpec("loop_late_pct", *_U8),
        FieldSpec("rx_drop", *_U16),
        FieldSpec("gyro_z_mrad_s", *_I16),
        FieldSpec("rails", *_U8),
        FieldSpec("motion", *_U8),
    )

    seq: int
    session: int
    mcu_us: int
    ack_seq: int
    state: int
    ctrl_flags: int
    fault: int
    left_ticks: int
    right_ticks: int
    v_meas_mm_s: int
    w_meas_mrad_s: int
    v_cmd_mm_s: int
    w_cmd_mrad_s: int
    vbat_mv: int
    imotor_ma: int
    tof_front_mm: int
    tof_cliff_mm: int
    sensor_age_ms: int
    loop_late_pct: int
    rx_drop: int
    gyro_z_mrad_s: int
    rails: int
    motion: int


@dataclass(frozen=True, slots=True)
class AckFrame(FrameBase):
    """``K`` ack.  ``V`` is not acked except the rate-limited clamp advisory."""

    TYPE: ClassVar[str] = "K"
    DOWN: ClassVar[bool] = False
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("ack_type", *_U8),
        FieldSpec("ack_seq", *_U16),
        FieldSpec("result", *_U8),
        FieldSpec("reason", *_U8),
        FieldSpec("echo", *_U32),
    )

    seq: int
    session: int
    ack_type: int
    ack_seq: int
    result: int
    reason: int
    echo: int


@dataclass(frozen=True, slots=True)
class EventFrame(FrameBase):
    """``E`` event; replaces the deleted free-text ``L`` log frame (A5)."""

    TYPE: ClassVar[str] = "E"
    DOWN: ClassVar[bool] = False
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("event", *_U8),
        FieldSpec("arg", *_I32),
        FieldSpec("mcu_us", *_U64),
    )

    seq: int
    session: int
    event: int
    arg: int
    mcu_us: int


@dataclass(frozen=True, slots=True)
class PongFrame(FrameBase):
    """``O`` pong; echoes ``pi_mono_us`` unmodified (principle 5, I-17)."""

    TYPE: ClassVar[str] = "O"
    DOWN: ClassVar[bool] = False
    FIELDS: ClassVar[tuple[FieldSpec, ...]] = (
        FieldSpec("echo_pi_mono_us", *_U64),
        FieldSpec("mcu_us", *_U64),
    )

    seq: int
    session: int
    echo_pi_mono_us: int
    mcu_us: int


Frame = (
    HelloFrame
    | ArmFrame
    | DisarmFrame
    | VelocityFrame
    | StopFrame
    | ClearFaultFrame
    | PingFrame
    | BootFrame
    | TelemetryFrame
    | AckFrame
    | EventFrame
    | PongFrame
)

FRAME_TYPES: Final[dict[str, type[FrameBase]]] = {
    cls.TYPE: cls
    for cls in (
        HelloFrame,
        ArmFrame,
        DisarmFrame,
        VelocityFrame,
        StopFrame,
        ClearFaultFrame,
        PingFrame,
        BootFrame,
        TelemetryFrame,
        AckFrame,
        EventFrame,
        PongFrame,
    )
}

DOWN_FRAME_TYPES: Final = frozenset(k for k, v in FRAME_TYPES.items() if v.DOWN)
UP_FRAME_TYPES: Final = frozenset(k for k, v in FRAME_TYPES.items() if not v.DOWN)

for _cls in FRAME_TYPES.values():
    _declared = tuple(f.name for f in dataclass_fields(_cls))  # type: ignore[arg-type]
    _expected = ("seq", "session", *(s.name for s in _cls.FIELDS))
    if _declared != _expected:
        raise RuntimeError(  # pragma: no cover - import-time consistency guard
            f"{_cls.__name__} fields {_declared} do not match its wire order {_expected}"
        )
del _cls, _declared, _expected


@dataclass(frozen=True, slots=True)
class DecodeOk:
    """A frame that passed length, CRC, type, version and grammar checks."""

    frame: Frame

    ok: ClassVar[bool] = True


@dataclass(frozen=True, slots=True)
class DecodeErr:
    """A line that did not.  Drop it, count it, and do not renew any TTL (I-2)."""

    reason: AckReason
    detail: str
    raw: bytes

    ok: ClassVar[bool] = False


DecodeResult = DecodeOk | DecodeErr

_HEX_DIGITS = frozenset(b"0123456789ABCDEF")
_DEC_DIGITS = frozenset(b"0123456789")


def _parse_dec(token: bytes) -> int | None:
    body = token[1:] if token[:1] == b"-" else token
    if not body or not _DEC_DIGITS.issuperset(body):
        return None
    return int(token)


def _parse_hex(token: bytes) -> int | None:
    if not 1 <= len(token) <= 8 or not _HEX_DIGITS.issuperset(token):
        return None
    return int(token, 16)


def encode_frame(frame: FrameBase) -> bytes:
    """Render a frame as ``$body*CRC\\n``.

    Raises ``ValueError`` for a value outside its wire type, for a session of 0
    on anything but ``H``, or for a seq outside uint16 -- an encoder fault is
    the caller's bug, not a wire event.
    """
    cls = type(frame)
    if not 0 <= frame.seq <= 0xFFFF:
        raise ValueError(f"seq must be 0..65535, got {frame.seq!r}")
    if not 0 <= frame.session <= 0xFFFF:
        raise ValueError(f"session must be 0..65535, got {frame.session!r}")
    if frame.session == SESSION_WILDCARD and cls.TYPE != HelloFrame.TYPE:
        raise ValueError(f"session 0 is the H wildcard, not legal on {cls.TYPE}")
    parts = [cls.TYPE, str(PROTO_VER), str(frame.seq), str(frame.session)]
    for spec in cls.FIELDS:
        parts.append(spec.render(getattr(frame, spec.name)))
    body = ",".join(parts).encode("ascii")
    return b"$" + body + b"*" + f"{crc16_ccitt_false(body):04X}".encode("ascii") + b"\n"


def decode_line(line: bytes) -> DecodeResult:
    """Decode one line, with or without its trailing newline.

    Checks run in the order the wire forces: framing and length, then CRC,
    then type, version, session and the field grammar.  Nothing raises.
    """
    if line.endswith(b"\n"):
        line = line[:-1]
    if line.endswith(b"\r"):
        line = line[:-1]
    if len(line) > MAX_LINE_BYTES:
        return DecodeErr(AckReason.BAD_LENGTH, "over-length line", line)
    if not line.startswith(b"$"):
        return DecodeErr(AckReason.BAD_LENGTH, "no '$' delimiter", line)
    body, star, crc_text = line[1:].rpartition(b"*")
    if not star:
        return DecodeErr(AckReason.BAD_LENGTH, "no '*' delimiter", line)
    if len(crc_text) != 4 or not _HEX_DIGITS.issuperset(crc_text):
        return DecodeErr(AckReason.BAD_LENGTH, "malformed CRC field", line)
    if int(crc_text, 16) != crc16_ccitt_false(body):
        return DecodeErr(AckReason.BAD_CRC, "CRC mismatch", line)

    parts = body.split(b",")
    if len(parts) < 4:
        return DecodeErr(AckReason.BAD_LENGTH, "short header", line)
    cls = FRAME_TYPES.get(parts[0].decode("ascii", "replace"))
    if cls is None:
        return DecodeErr(AckReason.UNKNOWN_TYPE, "unknown frame type", line)
    ver = _parse_dec(parts[1])
    if ver != PROTO_VER:
        return DecodeErr(AckReason.UNSUPPORTED_VERSION, "version mismatch", line)
    seq = _parse_dec(parts[2])
    session = _parse_dec(parts[3])
    if seq is None or not 0 <= seq <= 0xFFFF:
        return DecodeErr(AckReason.BAD_LENGTH, "malformed seq", line)
    if session is None or not 0 <= session <= 0xFFFF:
        return DecodeErr(AckReason.BAD_LENGTH, "malformed session", line)
    if session == SESSION_WILDCARD and cls.TYPE != HelloFrame.TYPE:
        return DecodeErr(AckReason.BAD_SESSION, "session 0 outside H", line)

    specs = cls.FIELDS
    if len(parts) - 4 != len(specs):
        return DecodeErr(AckReason.BAD_LENGTH, "field count mismatch", line)
    values: list[int] = []
    for spec, token in zip(specs, parts[4:], strict=True):
        value = _parse_hex(token) if spec.hex_width is not None else _parse_dec(token)
        if value is None or not spec.lo <= value <= spec.hi:
            return DecodeErr(AckReason.BAD_LENGTH, f"malformed field {spec.name}", line)
        values.append(value)
    return DecodeOk(cls(seq, session, *values))  # type: ignore[arg-type,call-arg]


@dataclass
class RxCounters:
    """What ``T.rx_drop`` and robotd's link stats are built from."""

    ok: int = 0
    bad_crc: int = 0
    bad_length: int = 0
    unknown_type: int = 0
    unsupported_version: int = 0
    bad_session: int = 0
    stale_seq: int = 0
    """Sequence staleness is a drop like the rest (A8: "any frame failing CRC,
    seq, session, length, type or version is dropped, counted, and does not
    renew the TTL").  It is counted here rather than only inside
    :class:`SessionGuard` because ``rover_rx_dropped()`` in
    ``firmware/core/rover_codec.c`` includes it, and the two sides have to sum
    the same wire events or ``T.rx_drop`` and robotd's link stats cannot be
    compared."""
    overlong: int = 0

    @property
    def dropped(self) -> int:
        return (
            self.bad_crc
            + self.bad_length
            + self.unknown_type
            + self.unsupported_version
            + self.bad_session
            + self.stale_seq
        )


_COUNTER_FOR_REASON = {
    AckReason.BAD_CRC: "bad_crc",
    AckReason.BAD_LENGTH: "bad_length",
    AckReason.UNKNOWN_TYPE: "unknown_type",
    AckReason.UNSUPPORTED_VERSION: "unsupported_version",
    AckReason.BAD_SESSION: "bad_session",
    AckReason.STALE_SEQ: "stale_seq",
}


class FrameReader:
    """Newline framing over a byte stream.

    A leading newline is legal and is sent once after every port open, so an
    empty line is skipped silently.  A line longer than ``max_line`` is dropped
    to the next newline and counted, exactly once, whatever its length.
    """

    def __init__(self, max_line: int = MAX_LINE_BYTES) -> None:
        self._max_line = max_line
        self._buf = bytearray()
        self._skipping = False
        self.counters = RxCounters()

    def feed(self, data: bytes) -> list[DecodeResult]:
        """Consume bytes; return every complete line's result, in order."""
        out: list[DecodeResult] = []
        self._buf += data
        while True:
            index = self._buf.find(b"\n")
            if index < 0:
                if len(self._buf) > self._max_line:
                    self._buf.clear()
                    if not self._skipping:
                        self._skipping = True
                        out.append(self._overlong(b""))
                break
            line = bytes(self._buf[:index])
            del self._buf[: index + 1]
            if self._skipping:
                self._skipping = False
                continue
            if not line or line == b"\r":
                continue
            if len(line) > self._max_line:
                out.append(self._overlong(line))
                continue
            result = decode_line(line)
            if isinstance(result, DecodeOk):
                self.counters.ok += 1
            else:
                self._count(result.reason)
            out.append(result)
        return out

    def _overlong(self, line: bytes) -> DecodeErr:
        self.counters.overlong += 1
        self.counters.bad_length += 1
        return DecodeErr(AckReason.BAD_LENGTH, "over-length line", line)

    def note_drop(self, reason: AckReason) -> None:
        """Count a frame dropped after decoding -- session or sequence.

        The reader decodes; ``SessionGuard`` decides.  Both outcomes are drops
        under A8, and ``rover_rx_dropped()`` sums both, so the caller that runs
        the guard reports back here rather than leaving the two sides counting
        different sets of events.
        """
        self._count(reason)

    def _count(self, reason: AckReason) -> None:
        name = _COUNTER_FOR_REASON.get(reason)
        if name is not None:
            setattr(self.counters, name, getattr(self.counters, name) + 1)


@dataclass
class SessionGuard:
    """Session and sequence acceptance for one direction (A8).

    ``session`` 0 means "not learned yet" and accepts any frame's session.
    ``last_seq`` never moves backwards, so a replay is always stale.  ``H``
    carrying the wildcard session 0 is accepted whatever session is held.
    """

    session: int = SESSION_WILDCARD
    last_seq: int | None = None

    def check(self, frame: FrameBase) -> AckReason:
        """``AckReason.NONE`` if the frame is acceptable; the reason if not."""
        wildcard_hello = (
            type(frame).TYPE == HelloFrame.TYPE and frame.session == SESSION_WILDCARD
        )
        if (
            self.session != SESSION_WILDCARD
            and frame.session != self.session
            and not wildcard_hello
        ):
            return AckReason.BAD_SESSION
        if self.last_seq is not None and not seq_is_newer(frame.seq, self.last_seq):
            return AckReason.STALE_SEQ
        return AckReason.NONE

    def accept(self, frame: FrameBase) -> AckReason:
        """:meth:`check`, and on success advance ``last_seq``."""
        reason = self.check(frame)
        if reason is AckReason.NONE:
            self.last_seq = frame.seq
        return reason
