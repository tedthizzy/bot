"""brain's side of ``robotd.sock``, against a socket that goes away.

robotd is ``Restart=always`` and is the process most likely to be restarted --
a crash, a config reload, a deploy.  A client that connects once and never
re-dials loses every motion skill the moment that happens, and loses it
silently: wake word, text and PTT all still appear to work.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import shutil
import sys
import tempfile
import time
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.main import RobotdClient  # noqa: E402
from rover_contracts.messages import (  # noqa: E402
    DriveForBusArgs,
    ResultStatus,
    SkillMessage,
    Source,
)

TURN = "01J9ZC7K000000000000000000"
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"


class FakeRobotd:
    """One NDJSON listener that records greetings and can drop a connection."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.greetings: list[list[dict[str, Any]]] = []
        self.commands: list[dict[str, Any]] = []
        self._writers: list[asyncio.StreamWriter] = []
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_unix_server(self._serve, str(self.path))

    async def _serve(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._writers.append(writer)
        greeting: list[dict[str, Any]] = []
        self.greetings.append(greeting)
        with contextlib.suppress(ConnectionError, OSError):
            while line := await reader.readline():
                message = json.loads(line)
                if message.get("type") in {"hello", "subscribe", "stop"}:
                    greeting.append(message)
                if message.get("type") == "skill":
                    self.commands.append(message)

    def drop(self) -> None:
        """What a robotd restart looks like from here: the socket closes."""
        for writer in self._writers:
            with contextlib.suppress(OSError, ConnectionError):
                writer.close()
        self._writers.clear()

    def answer(self, cmd_id: str, status: str) -> None:
        line = json.dumps(
            {
                "v": 1,
                "type": "result",
                "cmd_id": cmd_id,
                "status": status,
                "reason": "",
                "t_utc_ns": time.time_ns(),
            }
        ).encode() + b"\n"
        for writer in self._writers:
            with contextlib.suppress(OSError, ConnectionError):
                writer.write(line)

    async def stop(self) -> None:
        self.drop()
        if self._server is not None:
            self._server.close()
            # Bounded: from 3.12 wait_closed() also waits for every handler
            # coroutine, and one of them is parked on readline() for a client
            # that is about to notice the drop.
            with contextlib.suppress(Exception):
                await asyncio.wait_for(self._server.wait_closed(), 1.0)


@contextlib.asynccontextmanager
async def wired() -> AsyncIterator[tuple[RobotdClient, FakeRobotd, asyncio.Task[None]]]:
    # A short directory: an AF_UNIX path is capped near 104 bytes.
    run_dir = Path(tempfile.mkdtemp(prefix="br"))
    server = FakeRobotd(run_dir / "robotd.sock")
    await server.start()
    client = RobotdClient(str(server.path), reconnect_s=0.05)
    task = asyncio.create_task(client.serve())
    try:
        assert await client.wait_connected(3.0)
        yield client, server, task
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await server.stop()
        shutil.rmtree(run_dir, ignore_errors=True)


async def until(predicate: Any, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("condition not reached within the timeout")


def skill() -> SkillMessage:
    return SkillMessage(
        source=Source.BRAIN,
        cmd_id=CMD,
        seq=1,
        turn_id=TURN,
        issued_mono_ns=time.monotonic_ns(),
        goal_ttl_ms=2000,
        skill="drive_for",
        args=DriveForBusArgs(duration_s=1.0, power=0.15),
    )


@pytest.mark.timeout(30)
def test_the_greeting_is_resent_on_every_reconnect() -> None:
    """4.3's restart contract is per connection, not per process: hello,
    subscribe, then stop before anything else can move."""

    async def scenario() -> None:
        async with wired() as (client, server, _task):
            await until(lambda: server.greetings and len(server.greetings[0]) == 3)
            assert [m["type"] for m in server.greetings[0]] == [
                "hello",
                "subscribe",
                "stop",
            ]
            server.drop()
            await until(lambda: len(server.greetings) == 2)
            await until(
                lambda: len(server.greetings) == 2
                and len(server.greetings[1]) == 3
            )
            assert [m["type"] for m in server.greetings[1]] == [
                "hello",
                "subscribe",
                "stop",
            ]
            assert client.connected

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_command_in_flight_when_the_bus_drops_is_answered_not_left_hanging() -> None:
    """Without this the execute task dies on ConnectionResetError, no Executed
    event is ever posted, and the FSM sits in EXECUTING until its state timeout
    -- for this command and every one after it."""

    async def scenario() -> None:
        async with wired() as (client, server, _task):
            run = asyncio.create_task(client.run(skill()))
            await until(lambda: bool(server.commands))
            server.drop()
            result = await asyncio.wait_for(run, 5.0)
            assert result.status is ResultStatus.ABORTED
            assert result.cmd_id == CMD

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_dispatch_onto_a_dead_socket_returns_a_result() -> None:
    async def scenario() -> None:
        async with wired() as (client, server, _task):
            # Wait for the accept: Server.close() stops listening but does not
            # close a connection the handler has not picked up yet.
            await until(lambda: bool(server.greetings) and bool(server._writers))
            await server.stop()
            await until(lambda: not client.connected, timeout=5.0)
            result = await asyncio.wait_for(client.run(skill()), 5.0)
            assert result.status is ResultStatus.ABORTED

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_the_ping_rate_comes_from_the_bus_config() -> None:
    """``[bus] client_ping_hz`` is read, not a constant beside a configurable
    ``client_ping_gap_ms``."""
    client = RobotdClient("/nonexistent.sock", ping_hz=2.0)
    assert client._ping_hz == 2.0
    assert RobotdClient("/nonexistent.sock")._ping_hz == 5.0
