"""The robotd bus: NDJSON over a Unix socket (A10, ARCHITECTURE 5.2).

The socket is ``0660`` and group ``rover``, because a filesystem permission is
how "only robotd writes the port" becomes "only group ``rover`` can command
motion".  The mode is set by the umask at bind time rather than by a later
``chmod``, so there is no window in which the socket is world-writable.

Publishing never blocks the control loop.  Each connection owns a small queue
and a pump task; :meth:`BusConnection.send` only appends and sets an event.
``state`` is **newest-only** -- a slow subscriber gets the latest snapshot and
never a backlog of stale ones -- while ``result``, ``event``, ``welcome`` and
``error`` are reliable, because ZMQ-style conflation applied to a ``result``
would lose the answer to a command (A10).  A reliable queue that overflows drops
its *oldest* entry and counts it; it never applies backpressure upwards.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import os
import shutil
from collections import deque
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Any, Final

from pydantic import BaseModel
from rover_contracts.jsonl import to_json_line
from rover_contracts.messages import ErrorMessage, SubscribeTopic

from rover_robotd.clients import ClientSession, PeerCredentials, peer_credentials

__all__ = ["MAX_LINE_BYTES", "Bus", "BusConnection"]

log = logging.getLogger("rover.robotd.bus")

MAX_LINE_BYTES: Final = 64 * 1024
"""ARCHITECTURE 5.2: NDJSON, at most 64 KiB per line."""

_DEFAULT_QUEUE: Final = 256

OnMessage = Callable[["BusConnection", dict[str, Any]], Awaitable[None]]
OnConnection = Callable[["BusConnection"], Awaitable[None]]


class BusConnection:
    """One client, its bound session, and its outbound queues."""

    def __init__(
        self,
        session: ClientSession,
        writer: asyncio.StreamWriter,
        *,
        max_queued: int = _DEFAULT_QUEUE,
    ) -> None:
        self.session = session
        self.writer = writer
        self.dropped = 0
        self._max_queued = max_queued
        self._reliable: deque[bytes] = deque()
        self._state: bytes | None = None
        self._wake = asyncio.Event()
        self._closed = False

    # -- outbound -----------------------------------------------------------

    def send(self, message: BaseModel) -> None:
        """Queue a reliable message.  Returns immediately, always."""
        if self._closed:
            return
        self._reliable.append(to_json_line(message).encode("utf-8"))
        while len(self._reliable) > self._max_queued:
            self._reliable.popleft()
            self.dropped += 1
        self._wake.set()

    def send_state(self, message: BaseModel) -> None:
        """Queue a newest-only message: a later one replaces an unsent earlier."""
        if self._closed:
            return
        self._state = to_json_line(message).encode("utf-8")
        self._wake.set()

    def wants(self, topic: SubscribeTopic) -> bool:
        return topic in self.session.topics

    async def pump(self) -> None:
        """Drain the queues onto the socket until the connection closes."""
        while not self._closed:
            if not self._reliable and self._state is None:
                self._wake.clear()
                await self._wake.wait()
                continue
            chunks: list[bytes] = []
            while self._reliable:
                chunks.append(self._reliable.popleft())
            if self._state is not None:
                chunks.append(self._state)
                self._state = None
            self.writer.write(b"".join(chunks))
            try:
                await self.writer.drain()
            except (ConnectionError, OSError):
                self._closed = True
                return

    def close(self) -> None:
        self._closed = True
        self._wake.set()
        with contextlib.suppress(ConnectionError, OSError):  # pragma: no cover
            self.writer.close()


class Bus:
    """The listening socket and the set of live connections."""

    def __init__(
        self,
        path: str | Path,
        *,
        on_message: OnMessage,
        on_connect: OnConnection | None = None,
        on_disconnect: OnConnection | None = None,
        group: str = "rover",
        mode: int = 0o660,
    ) -> None:
        self.path = Path(path)
        self.group = group
        self.mode = mode
        self.connections: set[BusConnection] = set()
        self._on_message = on_message
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists() or self.path.is_symlink():
            self.path.unlink()
        previous = os.umask(0o777 & ~self.mode)
        try:
            self._server = await asyncio.start_unix_server(
                self._serve, path=str(self.path), limit=MAX_LINE_BYTES
            )
        finally:
            os.umask(previous)
        try:
            shutil.chown(self.path, group=self.group)
        except (LookupError, PermissionError, OSError) as exc:
            log.warning(
                "cannot set group %r on %s (%s); the socket is mode %o and "
                "owned by this user only",
                self.group,
                self.path,
                exc,
                self.mode,
            )
        log.info("bus listening on %s", self.path)

    async def close(self) -> None:
        for connection in list(self.connections):
            connection.close()
        self.connections.clear()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        self.path.unlink(missing_ok=True)

    # -- publishing ---------------------------------------------------------

    def broadcast(self, message: BaseModel, topic: SubscribeTopic) -> None:
        for connection in self.connections:
            if connection.wants(topic):
                connection.send(message)

    def broadcast_state(self, message: BaseModel) -> None:
        for connection in self.connections:
            if connection.wants(SubscribeTopic.STATE):
                connection.send_state(message)

    # -- serving ------------------------------------------------------------

    async def _serve(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        sock = writer.get_extra_info("socket")
        peer = peer_credentials(sock) if sock is not None else PeerCredentials()
        connection = BusConnection(ClientSession(peer=peer), writer)
        self.connections.add(connection)
        pump = asyncio.create_task(connection.pump())
        try:
            if self._on_connect is not None:
                await self._on_connect(connection)
            await self._read_lines(connection, reader)
        finally:
            connection.close()
            pump.cancel()
            self.connections.discard(connection)
            if self._on_disconnect is not None:
                await self._on_disconnect(connection)
            with contextlib.suppress(ConnectionError, OSError):  # pragma: no cover
                await writer.wait_closed()

    async def _read_lines(
        self, connection: BusConnection, reader: asyncio.StreamReader
    ) -> None:
        while True:
            try:
                line = await reader.readline()
            except (ValueError, asyncio.LimitOverrunError):
                connection.send(
                    ErrorMessage(
                        code="line_too_long", detail=f"max {MAX_LINE_BYTES} bytes"
                    )
                )
                return
            except (ConnectionError, OSError):
                return
            if not line:
                return
            stripped = line.strip()
            if not stripped:
                continue
            try:
                payload = json.loads(stripped)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                connection.send(ErrorMessage(code="bad_json", detail=str(exc)[:256]))
                continue
            if not isinstance(payload, dict):
                connection.send(
                    ErrorMessage(code="bad_json", detail="a message must be an object")
                )
                continue
            await self._on_message(connection, payload)
