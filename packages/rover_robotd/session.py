"""Client sessions, connection-bound identity, and instruction identity.

ARCHITECTURE 4.2: ``source`` is **bound to the connection at hello**, together
with the peer's uid and pid, because unbound the field is self-declared and the
allow-list is not an isolation boundary -- all four units sit in group ``rover``
on one 0660 socket, so a compromised peer could claim ``"source":"web"`` and
take the top arbitration slot.  A second ``hello`` on one connection is refused
and any later message whose ``source`` differs from the bound value is rejected
``source_not_allowed``.

Also here: the last-64 ``(source, cmd_id)`` replay window of I-12, and the
``current_turn_id`` of I-11, which advances on a new utterance, a stop, a bus
disconnect or an e-stop and never rolls back.
"""

from __future__ import annotations

import logging
import socket
import struct
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Final

from rover_contracts.ids import new_session_id
from rover_contracts.messages import (
    BusCap,
    HelloMessage,
    Source,
    SubscribeMessage,
    SubscribeTopic,
)
from rover_contracts.serial_codec import seq_is_newer

__all__ = [
    "REPLAY_WINDOW",
    "ClientSession",
    "PeerCredentials",
    "ReplayWindow",
    "TurnTracker",
    "peer_credentials",
    "resolve_uid",
]

log = logging.getLogger("rover.robotd.session")

REPLAY_WINDOW: Final = 64
"""ARCHITECTURE 7: the last 64 ``(source, cmd_id)`` pairs."""


@dataclass(frozen=True, slots=True)
class PeerCredentials:
    """What ``SO_PEERCRED`` said about the other end of the socket."""

    uid: int | None = None
    gid: int | None = None
    pid: int | None = None

    @property
    def known(self) -> bool:
        return self.uid is not None


def peer_credentials(sock: socket.socket) -> PeerCredentials:
    """Read the peer's uid/gid/pid, or return an empty record.

    ``SO_PEERCRED`` is a kernel capability, not a platform branch: where the
    socket layer offers it the uid check of 4.2 is enforced, and where it does
    not the binding still holds -- ``source`` is still fixed at ``hello`` and
    still checked at every dispatch -- but the uid row is skipped and logged.
    macOS has no equivalent that reports a pid, which is why the Mac
    simulation runs without it.
    """
    option = getattr(socket, "SO_PEERCRED", None)
    if option is None:
        return PeerCredentials()
    try:
        raw = sock.getsockopt(socket.SOL_SOCKET, option, struct.calcsize("3i"))
    except OSError:  # pragma: no cover - kernel refused a supported option
        return PeerCredentials()
    pid, uid, gid = struct.unpack("3i", raw)
    return PeerCredentials(uid=uid, gid=gid, pid=pid)


def resolve_uid(user: str) -> int | None:
    """The numeric uid of a configured unit user, or ``None`` if it is absent.

    The four per-unit users exist on the Pi and not on a development machine,
    so an unresolvable name disables the uid row rather than refusing every
    connection.
    """
    import pwd

    try:
        return pwd.getpwnam(user).pw_uid
    except KeyError:
        return None


class ReplayWindow:
    """The last :data:`REPLAY_WINDOW` ``(source, cmd_id)`` pairs (I-12)."""

    def __init__(self, size: int = REPLAY_WINDOW) -> None:
        self._size = size
        self._seen: OrderedDict[tuple[str, str], None] = OrderedDict()

    def seen(self, source: Source, cmd_id: str) -> bool:
        return (str(source), cmd_id) in self._seen

    def remember(self, source: Source, cmd_id: str) -> None:
        key = (str(source), cmd_id)
        self._seen[key] = None
        self._seen.move_to_end(key)
        while len(self._seen) > self._size:
            self._seen.popitem(last=False)


class TurnTracker:
    """``current_turn_id``: monotonic by ULID compare, so an out-of-order
    ``turn`` cannot roll the id back (ARCHITECTURE 5.2)."""

    def __init__(self) -> None:
        self.current: str | None = None

    def adopt(self, turn_id: str) -> bool:
        """Take ``turn_id`` as current if it is newer.  ``True`` if it moved."""
        if self.current is not None and turn_id <= self.current:
            return False
        self.current = turn_id
        return True

    def is_current(self, turn_id: str) -> bool:
        return self.current is not None and turn_id == self.current


@dataclass(slots=True)
class ClientSession:
    """One bus connection's identity and per-source sequence expectation."""

    session_id: str = field(default_factory=new_session_id)
    peer: PeerCredentials = field(default_factory=PeerCredentials)
    source: Source | None = None
    caps: frozenset[BusCap] = frozenset()
    topics: frozenset[SubscribeTopic] = frozenset()
    state_hz: int = 10
    pid: int | None = None
    last_seq: int | None = None
    last_ping_mono_ns: int = 0

    @property
    def bound(self) -> bool:
        return self.source is not None

    def bind(self, hello: HelloMessage, *, expected_uid: int | None) -> None:
        """Bind ``source`` to this connection.  Raises on a second ``hello``,
        or on a peer uid that is not the unit ``[bus] source_uids`` names."""
        if self.bound:
            raise ValueError("a second hello on one connection is refused")
        if (
            expected_uid is not None
            and self.peer.known
            and self.peer.uid != expected_uid
        ):
            raise ValueError(
                f"peer uid {self.peer.uid} may not claim source {hello.source!r}"
            )
        if expected_uid is None or not self.peer.known:
            log.warning(
                "binding source %s without a uid check (peer credentials %s, "
                "configured unit user %s)",
                hello.source,
                "unavailable" if not self.peer.known else "known",
                "unresolved" if expected_uid is None else expected_uid,
            )
        self.source = hello.source
        self.caps = frozenset(hello.caps)
        self.pid = hello.pid

    def subscribe(self, message: SubscribeMessage) -> None:
        self.topics = frozenset(message.topics)
        self.state_hz = message.state_hz

    def seq_ok(self, seq: int) -> bool:
        """``seq`` strictly increasing per ``(source, client-session)``.

        The counter is a ``uint16`` on the wire and an unbounded integer here;
        comparison goes through the codec's wrap-safe rule for both, so a
        client that wraps at 65535 is not locked out.
        """
        if self.last_seq is None:
            return True
        if seq > 0xFFFF or self.last_seq > 0xFFFF:
            return seq > self.last_seq
        return seq_is_newer(seq, self.last_seq)

    def note_seq(self, seq: int) -> None:
        self.last_seq = seq
