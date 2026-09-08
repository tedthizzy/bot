"""``brain.sock`` -- the third socket, and the only way anything reaches the
conversational layer (ARCHITECTURE 5.9).

robotd MUST NOT hold conversational state (4.2) and its message set carries
neither an utterance nor a face expression, so without this server ``rover-web``'s
text box, its PTT button, ``roverctl utter`` and ``set_face`` have no path at
all.  brain is the only listener; ``rover_web`` and ``roverctl`` are its clients.

Inbound is one of ``utterance``, ``ptt_start``, ``ptt_end``, ``cancel``;
outbound is ``face`` and ``fsm``, fanned to every connected client.  A line that
does not validate costs that line only -- a malformed utterance must not be able
to drop the socket the STOP-adjacent text box speaks through.

The socket is ``0660`` group ``rover`` (A10), set by the umask at bind time so
there is no window in which it is world-writable.  Writes never block the FSM:
each connection owns a bounded queue whose oldest entry is dropped on overflow.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import os
import shutil
from collections import deque
from collections.abc import Callable
from pathlib import Path
from typing import Final

from pydantic import ValidationError
from rover_contracts.jsonl import to_json_line
from rover_contracts.messages import (
    BrainCancelMessage,
    BrainClientMessage,
    BrainServerMessage,
    BrainSkillRequest,
    ResultMessage,
    UtteranceSource,
    brain_client_adapter,
)

__all__ = ["MAX_LINE_BYTES", "BrainBus"]

log = logging.getLogger("rover.brain.bus")

MAX_LINE_BYTES: Final = 64 * 1024
"""The same NDJSON line cap the robotd bus carries (5.2)."""

_MAX_QUEUED: Final = 64

OnMessage = Callable[[BrainClientMessage], None]


class _Client:
    """One connected page or CLI, and the lines waiting to reach it."""

    def __init__(self, writer: asyncio.StreamWriter) -> None:
        self.writer = writer
        self.dropped = 0
        self._queue: deque[bytes] = deque()
        self._wake = asyncio.Event()
        self._closed = False
        self.requests: dict[str, UtteranceSource] = {}

    def send(self, line: bytes) -> None:
        if self._closed:
            return
        self._queue.append(line)
        while len(self._queue) > _MAX_QUEUED:
            self._queue.popleft()
            self.dropped += 1
        self._wake.set()

    async def pump(self) -> None:
        while not self._closed:
            if not self._queue:
                self._wake.clear()
                await self._wake.wait()
                continue
            chunks = [self._queue.popleft() for _ in range(len(self._queue))]
            self.writer.write(b"".join(chunks))
            try:
                await self.writer.drain()
            except (ConnectionError, OSError):
                self._closed = True
                return

    def close(self) -> None:
        self._closed = True
        self._wake.set()
        with contextlib.suppress(ConnectionError, OSError):
            self.writer.close()


class BrainBus:
    """The listening socket brain serves on ``[bus] brain_sock``."""

    def __init__(
        self,
        path: str | Path,
        on_message: OnMessage,
        *,
        group: str = "rover",
        mode: int = 0o660,
    ) -> None:
        self.path = Path(path)
        self.group = group
        self.mode = mode
        self._on_message = on_message
        self._clients: set[_Client] = set()
        self._server: asyncio.AbstractServer | None = None

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def start(self) -> None:
        """Bind and listen.  Idempotent for a bus that is already up."""
        if self._server is not None:
            return
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
                "cannot set group %r on %s (%s); the socket is mode %o and owned "
                "by this user only",
                self.group,
                self.path,
                exc,
                self.mode,
            )
        log.info("brain.sock listening on %s", self.path)

    async def serve(self) -> None:
        """Serve until cancelled.  This is what ``BrainApp.run`` awaits."""
        await self.start()
        try:
            await asyncio.Event().wait()
        finally:
            await self.close()

    async def close(self) -> None:
        for client in list(self._clients):
            client.close()
        self._clients.clear()
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(ConnectionError, OSError):
                await self._server.wait_closed()
            self._server = None
        self.path.unlink(missing_ok=True)

    async def broadcast(self, message: BrainServerMessage) -> None:
        """Fan a ``face`` or ``fsm`` message out to every client."""
        line = to_json_line(message).encode("utf-8")
        for client in self._clients:
            if isinstance(message, ResultMessage):
                client.requests.pop(message.cmd_id, None)
            client.send(line)

    # -- serving ------------------------------------------------------------

    async def _serve(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        client = _Client(writer)
        self._clients.add(client)
        pump = asyncio.create_task(client.pump())
        try:
            await self._read_lines(reader, client)
        finally:
            for request_id, source in client.requests.items():
                self._on_message(BrainCancelMessage(source=source, request_id=request_id))
            client.close()
            pump.cancel()
            self._clients.discard(client)
            with contextlib.suppress(ConnectionError, OSError):
                await writer.wait_closed()

    async def _read_lines(
        self, reader: asyncio.StreamReader, client: _Client | None = None
    ) -> None:
        while True:
            try:
                line = await reader.readline()
            except (ValueError, asyncio.LimitOverrunError):
                log.warning("brain.sock: line over %d bytes, dropped", MAX_LINE_BYTES)
                return
            except (ConnectionError, OSError):
                return
            if not line:
                return
            stripped = line.strip()
            if not stripped:
                continue
            try:
                message = brain_client_adapter.validate_json(stripped)
            except (ValidationError, UnicodeDecodeError, json.JSONDecodeError) as exc:
                count = exc.error_count() if isinstance(exc, ValidationError) else 1
                log.warning("brain.sock: undecodable line, %d errors", count)
                continue
            if client is not None and isinstance(message, BrainSkillRequest):
                client.requests[message.request_id] = message.source
            self._on_message(message)
