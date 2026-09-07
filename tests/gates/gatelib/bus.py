"""A blocking NDJSON client for ``robotd.sock`` and ``brain.sock``.

The gates are the adversary of the bus, not a well-behaved client: G4 has to
send a second ``hello``, a mismatched ``source``, a duplicate ``cmd_id`` and a
line that is not JSON at all, and then assert what came back.  So this speaks
the wire directly rather than reusing a component's client, and it never raises
on a malformed reply -- a gate scores what arrived.

Everything is blocking with an explicit deadline.  A gate that hangs is a gate
that lies about a timeout.
"""

from __future__ import annotations

import contextlib
import json
import os
import socket
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from rover_contracts import (
    HelloMessage,
    PingMessage,
    SubscribeMessage,
    WelcomeMessage,
    new_cmd_id,
    new_turn_id,
    to_json_line,
)

__all__ = ["BusClient", "MAX_LINE", "new_cmd_id", "new_turn_id"]

MAX_LINE = 64 * 1024
"""ARCHITECTURE 5.2: NDJSON, at most 64 KiB per line."""


class BusClient:
    """One connection to a NDJSON Unix socket.

    ``send`` takes a pydantic model, a mapping or raw bytes, so a gate can put a
    deliberately malformed line on the wire without building a model for it.
    """

    PING_HZ = 5.0
    """4.2: every client owning an active command pings at 5 Hz, and a 400 ms
    gap aborts that command.  A gate that skips this measures the ping gap
    instead of the rule it came to test."""

    def __init__(self, path: str | Path, timeout: float = 2.0) -> None:
        self.path = str(path)
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect(self.path)
        self._buf = bytearray()
        self._send_lock = threading.Lock()
        self._pinger: threading.Thread | None = None
        self._stop_pings = threading.Event()
        self.welcome: WelcomeMessage | None = None

    # -- lifecycle ------------------------------------------------------

    def close(self) -> None:
        self.stop_pings()
        with contextlib.suppress(OSError):
            self.sock.close()

    def start_pings(self, source: str) -> None:
        """Ping at 5 Hz until :meth:`close`, as 4.2 requires of any client that
        may own a command."""
        if self._pinger is not None:
            return
        self._stop_pings.clear()

        def loop() -> None:
            while not self._stop_pings.wait(1.0 / self.PING_HZ):
                with contextlib.suppress(OSError):
                    self.ping(source)

        self._pinger = threading.Thread(target=loop, daemon=True, name="gate-ping")
        self._pinger.start()

    def stop_pings(self) -> None:
        self._stop_pings.set()
        if self._pinger is not None:
            self._pinger.join(timeout=1.0)
            self._pinger = None

    def __enter__(self) -> BusClient:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def hello(
        self, source: str, caps: list[str], *, subscribe: list[str] | None = None
    ) -> WelcomeMessage | None:
        """Say ``hello``, read the ``welcome``, optionally subscribe."""
        self.send(HelloMessage(source=source, pid=os.getpid(), caps=caps))
        message = self.expect("welcome", deadline=self.timeout)
        if message is not None:
            try:
                self.welcome = WelcomeMessage.model_validate(message)
            except Exception:  # noqa: BLE001 - a bad welcome is data, not a crash
                self.welcome = None
        if subscribe:
            self.send(SubscribeMessage(topics=subscribe))
        self.start_pings(source)
        return self.welcome

    def ping(self, source: str) -> None:
        self.send(PingMessage(source=source))

    # -- wire -----------------------------------------------------------

    def send(self, message: Any) -> None:
        """Write one line.  ``bytes`` go out verbatim, newline appended."""
        if isinstance(message, bytes):
            payload = message if message.endswith(b"\n") else message + b"\n"
        elif isinstance(message, str):
            payload = message.encode("utf-8") + b"\n"
        else:
            payload = to_json_line(message).encode("utf-8")
        with self._send_lock:
            self.sock.sendall(payload)

    def read(self, deadline: float = 1.0) -> dict[str, Any] | None:
        """The next decodable line, or ``None`` on timeout or EOF."""
        end = time.monotonic() + deadline
        while True:
            index = self._buf.find(b"\n")
            if index >= 0:
                line = bytes(self._buf[:index])
                del self._buf[: index + 1]
                if not line.strip():
                    continue
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(65536)
            except TimeoutError:
                return None
            except OSError:
                return None
            if not chunk:
                return None
            self._buf += chunk

    def drain(self, seconds: float) -> list[dict[str, Any]]:
        """Everything that arrives in a window."""
        end = time.monotonic() + seconds
        out: list[dict[str, Any]] = []
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return out
            message = self.read(remaining)
            if message is None:
                return out
            out.append(message)

    def expect(
        self, *types: str, deadline: float = 2.0, cmd_id: str | None = None
    ) -> dict[str, Any] | None:
        """The next message of any of these types, optionally for one ``cmd_id``.

        More than one type because a refusal's envelope depends on the message
        refused: 5.2 gives ``result`` a pattern-bound ``cmd_id``, so a ``clear``
        -- which carries none -- can only be refused as an ``error``.
        """
        wanted = set(types)
        end = time.monotonic() + deadline
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            message = self.read(remaining)
            if message is None:
                return None
            if message.get("type") not in wanted:
                continue
            if cmd_id is not None and message.get("cmd_id") != cmd_id:
                continue
            return message

    def states(self, seconds: float) -> Iterator[dict[str, Any]]:
        """Every ``state`` published in a window."""
        for message in self.drain(seconds):
            if message.get("type") == "state":
                yield message
