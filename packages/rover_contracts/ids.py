"""ULIDs for ``cmd_id`` and ``turn_id``, plus the robotd bus session id.

ARCHITECTURE 5.2 makes ``turn_id`` "monotonic by ULID compare, so an
out-of-order ``turn`` cannot roll the id back", which a plain random ULID does
not give within a single millisecond.  ``new_ulid`` is therefore the monotonic
factory: inside one millisecond the 80-bit randomness is incremented rather
than redrawn, so lexicographic order is generation order.

Canonical form only: 26 uppercase Crockford base32 characters, first character
``0``-``7`` so the 48-bit timestamp cannot overflow.  Pure stdlib.
"""

from __future__ import annotations

import secrets
import threading
import time

__all__ = [
    "SESSION_ID_PATTERN",
    "ULID_LENGTH",
    "ULID_PATTERN",
    "decode_ulid",
    "encode_ulid",
    "is_session_id",
    "is_ulid",
    "new_cmd_id",
    "new_session_id",
    "new_turn_id",
    "new_ulid",
    "ulid_timestamp_ms",
]

_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"
_DECODE = {c: i for i, c in enumerate(_ALPHABET)}

ULID_LENGTH = 26
ULID_PATTERN = r"^[0-7][0-9A-HJKMNP-TV-Z]{25}$"
"""Anchored regex for a canonical ULID, for pydantic ``Field(pattern=...)``."""

SESSION_ID_PATTERN = r"^[0-9a-f]{8}$"
"""Anchored regex for a robotd bus session id (ARCHITECTURE 5.2 ``welcome``)."""

_TIME_CHARS = 10
_RAND_CHARS = 16
_MAX_MS = (1 << 48) - 1
_MAX_RAND = (1 << 80) - 1

_lock = threading.Lock()
_last_ms = -1
_last_rand = 0


def _b32(value: int, length: int) -> str:
    out = [""] * length
    for i in range(length - 1, -1, -1):
        out[i] = _ALPHABET[value & 0x1F]
        value >>= 5
    return "".join(out)


def _unb32(text: str) -> int:
    value = 0
    for char in text:
        digit = _DECODE.get(char)
        if digit is None:
            raise ValueError(f"not a Crockford base32 character: {char!r}")
        value = (value << 5) | digit
    return value


def encode_ulid(timestamp_ms: int, randomness: int) -> str:
    """Build a canonical ULID from its 48-bit time and 80-bit random halves."""
    if not 0 <= timestamp_ms <= _MAX_MS:
        raise ValueError(f"timestamp_ms must be 0..{_MAX_MS}, got {timestamp_ms!r}")
    if not 0 <= randomness <= _MAX_RAND:
        raise ValueError(f"randomness must be 0..{_MAX_RAND}, got {randomness!r}")
    return _b32(timestamp_ms, _TIME_CHARS) + _b32(randomness, _RAND_CHARS)


def new_ulid() -> str:
    """A ULID that sorts after every ULID this process has already returned."""
    global _last_ms, _last_rand
    with _lock:
        now_ms = int(time.time() * 1000)
        if now_ms > _last_ms:
            _last_ms = now_ms
            _last_rand = secrets.randbits(80)
        elif _last_rand < _MAX_RAND:
            _last_rand += 1
        else:
            _last_ms += 1
            _last_rand = secrets.randbits(80)
        return encode_ulid(_last_ms, _last_rand)


def is_ulid(value: object) -> bool:
    """True for a canonical 26-character uppercase ULID."""
    if not isinstance(value, str) or len(value) != ULID_LENGTH:
        return False
    if value[0] not in "01234567":
        return False
    return all(char in _DECODE for char in value)


def decode_ulid(value: str) -> tuple[int, int]:
    """Split a ULID into ``(timestamp_ms, randomness)``."""
    if not is_ulid(value):
        raise ValueError(f"not a canonical ULID: {value!r}")
    return _unb32(value[:_TIME_CHARS]), _unb32(value[_TIME_CHARS:])


def ulid_timestamp_ms(value: str) -> int:
    """Unix milliseconds encoded in a ULID's first ten characters."""
    return decode_ulid(value)[0]


def new_turn_id() -> str:
    """A ``turn_id`` for one user instruction (ARCHITECTURE 7)."""
    return new_ulid()


def new_cmd_id() -> str:
    """A ``cmd_id`` for one dispatched command (ARCHITECTURE 5.2)."""
    return new_ulid()


def new_session_id() -> str:
    """A robotd bus session id: eight lowercase hex characters."""
    return secrets.token_hex(4)


def is_session_id(value: object) -> bool:
    """True for a canonical eight-character lowercase hex bus session id."""
    return (
        isinstance(value, str)
        and len(value) == 8
        and all(char in "0123456789abcdef" for char in value)
    )
