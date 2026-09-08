"""The rover link protocol of ``docs/protocol.md``: Waveshare JSON lines.

One JSON object per ``\\n``-terminated line, no checksum, 115200 baud.  This
module is pure: bytes and dicts in, typed values out, no I/O.  A malformed line
is a value (:class:`Dropped`), never an exception, because the caller is the
control loop and a control loop that raises on the wire is a control loop that
stops sending zeros.

The fork's added fields (``hb``, ``st``, ``tf``, ``bp``, ``cc``) are what tell
the host it is talking to patched firmware.  Their absence is meaningful and is
preserved as ``None`` rather than defaulted, so the host can refuse motion
against stock firmware (see ``Feedback.patched``).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import IntFlag
from typing import Final

__all__ = [
    "Banner",
    "Dropped",
    "FULL_SCALE",
    "Feedback",
    "Imu",
    "LINE_MAX_BYTES",
    "POWER_CAP",
    "StopFlag",
    "Unknown",
    "banner_request",
    "coast",
    "decode_line",
    "echo",
    "feedback_flow",
    "feedback_interval",
    "heartbeat",
    "imu_request",
    "oled",
    "quiet",
    "speed",
]

FULL_SCALE: Final = 0.5
"""Waveshare's full-scale speed value; the firmware multiplies by 512 into an
8-bit duty, so 0.5 is 100 percent."""

POWER_CAP: Final = 0.30
"""The cap the firmware fork compiles in.  Sixty percent duty.  The host clamps
to its own ``[limits] power_max`` first, which must not exceed this."""

LINE_MAX_BYTES: Final = 512
"""Longer lines are dropped whole, both directions."""

T_SPEED: Final = 1
T_OLED: Final = 3
T_COAST: Final = 115
T_IMU_REQUEST: Final = 126
T_BASE_REQUEST: Final = 130
T_FEEDBACK_FLOW: Final = 131
T_HEARTBEAT: Final = 136
T_FEEDBACK_INTERVAL: Final = 142
T_ECHO: Final = 143
T_INFO_PRINT: Final = 605
T_FEEDBACK: Final = 1001
T_IMU: Final = 1002
T_BANNER: Final = 1006
T_BANNER_REQUEST: Final = 1007


class StopFlag(IntFlag):
    """``st`` in the fork's feedback."""

    HEARTBEAT = 1
    TOF = 2
    BUMPER = 4
    LOWBAT = 8
    COAST = 16

    @property
    def blocks_forward(self) -> bool:
        return bool(self & (StopFlag.TOF | StopFlag.BUMPER))

    @property
    def blocks_all(self) -> bool:
        return bool(self & StopFlag.LOWBAT)


# --------------------------------------------------------------------------
# Encoding, host -> rover
# --------------------------------------------------------------------------


def _line(obj: dict[str, object]) -> bytes:
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii")


def speed(left: float, right: float) -> bytes:
    """``T:1``.  Values are clamped to ``POWER_CAP`` here as well as in the
    firmware, and non-finite values become zero: nothing this function emits
    can exceed the cap, whatever the caller computed."""
    return _line({"T": T_SPEED, "L": _clamp(left), "R": _clamp(right)})


def _clamp(value: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return round(max(-POWER_CAP, min(POWER_CAP, float(value))), 3)


def coast() -> bytes:
    """``T:115``: all H-bridge inputs low."""
    return _line({"T": T_COAST})


def heartbeat(ms: int) -> bytes:
    """``T:136``.  The fork ignores values above its compiled default."""
    return _line({"T": T_HEARTBEAT, "cmd": int(ms)})


def feedback_flow(on: bool) -> bytes:
    return _line({"T": T_FEEDBACK_FLOW, "cmd": 1 if on else 0})


def feedback_interval(ms: int) -> bytes:
    return _line({"T": T_FEEDBACK_INTERVAL, "cmd": int(ms)})


def echo(on: bool) -> bytes:
    return _line({"T": T_ECHO, "cmd": 1 if on else 0})


def quiet() -> bytes:
    """``T:605 cmd 0``: no human-readable prints on the control line."""
    return _line({"T": T_INFO_PRINT, "cmd": 0})


def imu_request() -> bytes:
    return _line({"T": T_IMU_REQUEST})


def banner_request() -> bytes:
    return _line({"T": T_BANNER_REQUEST})


def oled(line: int, text: str) -> bytes:
    """``T:3``.  Cosmetic; the display has four lines of about 20 characters."""
    return _line({"T": T_OLED, "lineNum": int(line) & 3, "Text": text[:32]})


# --------------------------------------------------------------------------
# Decoding, rover -> host
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Feedback:
    """``T:1001``.  The fork's fields are ``None`` when the firmware is stock."""

    left: float
    right: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    temp_c: float
    bus_v: float
    hb: bool | None
    st: StopFlag | None
    tof_mm: int | None
    bumper: bool | None
    clamp_count: int | None

    @property
    def patched(self) -> bool:
        """True when every fork field is present.  Stock firmware is refused."""
        return None not in (self.hb, self.st, self.tof_mm, self.bumper, self.clamp_count)

    @property
    def tof_valid(self) -> bool:
        return self.tof_mm is not None and self.tof_mm >= 0


@dataclass(frozen=True, slots=True)
class Imu:
    """``T:1002``."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    gyro_dps: tuple[float, float, float]
    accel: tuple[float, float, float]
    temp_c: float


@dataclass(frozen=True, slots=True)
class Banner:
    """``T:1006``: what the fork compiled in.  The host compares ``hb_ms`` and
    ``cap`` with its configuration and refuses to move if they disagree."""

    fw: str
    hb_ms: int
    cap: float
    proto: int


@dataclass(frozen=True, slots=True)
class Unknown:
    """A well-formed line with a ``T`` this document does not define."""

    t: int


@dataclass(frozen=True, slots=True)
class Dropped:
    """Not JSON, not an object, no integer ``T``, or over the size limit."""

    reason: str


Decoded = Feedback | Imu | Banner | Unknown | Dropped


def decode_line(raw: bytes | str) -> Decoded:
    """Decode one line.  Never raises."""
    if isinstance(raw, str):
        raw = raw.encode("utf-8", "replace")
    if len(raw) > LINE_MAX_BYTES:
        return Dropped("oversize")
    text = raw.strip()
    if not text:
        return Dropped("empty")
    try:
        obj = json.loads(text)
    except (ValueError, UnicodeDecodeError):
        return Dropped("not_json")
    if not isinstance(obj, dict):
        return Dropped("not_object")
    t = obj.get("T")
    if isinstance(t, bool) or not isinstance(t, int):
        return Dropped("no_type")
    try:
        if t == T_FEEDBACK:
            return _feedback(obj)
        if t == T_IMU:
            return _imu(obj)
        if t == T_BANNER:
            return _banner(obj)
    except (KeyError, TypeError, ValueError, OverflowError):
        return Dropped("bad_fields")
    return Unknown(t)


def _num(obj: dict[str, object], key: str) -> float:
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(key)
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(key)
    return result


def _opt_int(obj: dict[str, object], key: str) -> int | None:
    value = obj.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(key)
    if isinstance(value, float) and not value.is_integer():
        raise ValueError(key)
    return int(value)


def _opt_bool(obj: dict[str, object], key: str) -> bool | None:
    value = _opt_int(obj, key)
    return None if value is None else value != 0


def _feedback(obj: dict[str, object]) -> Feedback:
    st = _opt_int(obj, "st")
    return Feedback(
        left=_num(obj, "L"),
        right=_num(obj, "R"),
        roll_deg=_num(obj, "r"),
        pitch_deg=_num(obj, "p"),
        yaw_deg=_num(obj, "y"),
        temp_c=_num(obj, "temp"),
        bus_v=_num(obj, "v"),
        hb=_opt_bool(obj, "hb"),
        st=None if st is None else StopFlag(st & 0x1F),
        tof_mm=_opt_int(obj, "tf"),
        bumper=_opt_bool(obj, "bp"),
        clamp_count=_opt_int(obj, "cc"),
    )


def _imu(obj: dict[str, object]) -> Imu:
    return Imu(
        roll_deg=_num(obj, "r"),
        pitch_deg=_num(obj, "p"),
        yaw_deg=_num(obj, "y"),
        gyro_dps=(_num(obj, "gx"), _num(obj, "gy"), _num(obj, "gz")),
        accel=(_num(obj, "ax"), _num(obj, "ay"), _num(obj, "az")),
        temp_c=_num(obj, "temp"),
    )


def _banner(obj: dict[str, object]) -> Banner:
    fw = obj["fw"]
    if not isinstance(fw, str) or not fw or len(fw) > 32:
        raise ValueError("fw")
    return Banner(
        fw=fw,
        hb_ms=int(_num(obj, "hb_ms")),
        cap=_num(obj, "cap"),
        proto=int(_num(obj, "proto")),
    )
