"""The robotd bus and the daemon's dispatch, over a real Unix socket.

The MCU is deliberately absent here -- ``[serial] port`` names a path that does
not exist, so the link retries ``open()`` forever and telemetry is never fresh.
That is the state I-22 cares about: ``estop``, ``stop`` and ``cancel`` must be
accepted in **every** state and are never answered ``rejected``.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import shutil
import stat
import sys
import tempfile
import time
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "packages"))

from rover_contracts.config import (  # noqa: E402
    BusConfig,
    LogConfig,
    RobotConfig,
    SerialConfig,
)
from rover_contracts.ids import new_turn_id  # noqa: E402
from rover_contracts.messages import ResultStatus, SubscribeTopic  # noqa: E402
from rover_robotd.bus import BusConnection  # noqa: E402
from rover_robotd.main import Robotd  # noqa: E402
from rover_robotd.session import ClientSession  # noqa: E402

TURN = "01J9ZC7K000000000000000000"
OLD_TURN = "01J9ZC7J000000000000000000"
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"


def _config(run_dir: Path, log_dir: Path) -> RobotConfig:
    """``./run`` is where ARCHITECTURE 5.2 puts the Mac sockets, and a Unix
    socket path has ~104 bytes to live in -- pytest's own tmp_path does not
    fit, so only the logs go there."""
    return RobotConfig(
        bus=BusConfig(sock=str(run_dir / "robotd.sock")),
        serial=SerialConfig(
            backend="pty", port=str(run_dir / "no-such.pty"), open_retry_ms=100
        ),
        log=LogConfig(dir=str(log_dir)),
    )


class Client:
    """One NDJSON connection, with a small inbox."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader = reader
        self.writer = writer

    async def send(self, **message: Any) -> None:
        self.writer.write(json.dumps({"v": 1, **message}).encode() + b"\n")
        await self.writer.drain()

    async def recv(self, timeout: float = 3.0) -> dict[str, Any]:
        line = await asyncio.wait_for(self.reader.readline(), timeout)
        assert line, "the server closed the connection"
        return json.loads(line)

    async def recv_type(self, wanted: str, timeout: float = 3.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = await self.recv(timeout=max(0.1, deadline - time.monotonic()))
            if message.get("type") == wanted:
                return message
        raise AssertionError(f"no {wanted!r} message within {timeout} s")

    async def hello(self, source: str = "brain", caps: tuple[str, ...] = ("skill",)):
        await self.send(type="hello", source=source, pid=1234, caps=list(caps))
        return await self.recv_type("welcome")

    async def close(self) -> None:
        self.writer.close()
        with contextlib.suppress(ConnectionError, OSError):
            await self.writer.wait_closed()


@contextlib.asynccontextmanager
async def daemon(tmp: Path) -> AsyncIterator[Robotd]:
    (REPO / "run").mkdir(exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(dir=REPO / "run", prefix="t"))
    robotd = Robotd(_config(run_dir, tmp / "logs"))
    task = asyncio.create_task(robotd.run())
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not robotd.bus.path.exists():
            if task.done():
                task.result()  # re-raise whatever stopped the daemon
            await asyncio.sleep(0.01)
        assert robotd.bus.path.exists(), "the bus socket was never created"
        yield robotd
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        shutil.rmtree(run_dir, ignore_errors=True)


@contextlib.asynccontextmanager
async def client(robotd: Robotd) -> AsyncIterator[Client]:
    reader, writer = await asyncio.open_unix_connection(str(robotd.bus.path))
    connection = Client(reader, writer)
    try:
        yield connection
    finally:
        await connection.close()


async def until(predicate: Any, timeout: float = 5.0) -> None:
    """Poll ``predicate`` until it holds: the effects below are asynchronous."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("condition not reached within the timeout")


def run(scenario: Any, tmp: Path) -> None:
    asyncio.run(scenario(tmp))


# ---------------------------------------------------------------------------
# The socket itself (A10)
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_the_socket_is_group_readable_and_not_world_accessible(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd:
            mode = stat.S_IMODE(robotd.bus.path.stat().st_mode)
            assert mode == 0o660, f"the bus must be 0660, got {mode:o}"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_hello_binds_the_source_and_answers_with_welcome(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            welcome = await brain.hello()
            assert welcome["limits"]["speed_mps"] == 0.30
            assert welcome["safety"]["tof_stop_mm"] == 250
            assert welcome["limits"]["budget_path_m"] == 1.5

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_second_hello_on_one_connection_is_refused(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            await brain.send(type="hello", source="web", pid=1, caps=["skill"])
            error = await brain.recv_type("error")
            assert error["code"] == "hello_refused"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_malformed_line_is_answered_and_the_connection_survives(
    tmp_path: Path,
) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            brain.writer.write(b"{not json}\n")
            await brain.writer.drain()
            error = await brain.recv_type("error")
            assert error["code"] == "bad_json"
            welcome = await brain.hello()
            assert welcome["type"] == "welcome"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_subscriber_receives_state(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as web:
            await web.hello(source="web", caps=("subscribe",))
            await web.send(type="subscribe", topics=["state"], state_hz=10)
            state = await web.recv_type("state")
            assert state["ready"] is False
            assert state["reason"] == "mcu_link_down"
            assert state["armed"] is False
            assert state["pose"]["frame_id"] == "odom"

    run(scenario, tmp_path)


# ---------------------------------------------------------------------------
# I-22: stop-class is never validated away
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
@pytest.mark.parametrize("kind", ["stop", "estop", "cancel"])
def test_stop_class_is_never_answered_rejected(tmp_path: Path, kind: str) -> None:
    """Fired with the link down and telemetry age unbounded."""

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as web:
            await web.hello(source="web", caps=("skill", "subscribe"))
            await web.send(type="subscribe", topics=["result", "event"])
            await web.send(type=kind, source="web", reason="user")
            await asyncio.sleep(0.2)
            with contextlib.suppress(TimeoutError):
                while True:
                    message = await web.recv(timeout=0.2)
                    assert message.get("status") != str(ResultStatus.REJECTED)

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_stop_missing_every_optional_field_is_still_a_stop(tmp_path: Path) -> None:
    """brain's restart path sends one on connect, before any turn exists."""

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            before = robotd.turns.current
            await brain.send(type="stop", source="brain")
            await until(lambda: robotd.turns.current != before)

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_stop_before_hello_is_still_honoured(tmp_path: Path) -> None:
    """The connection is unbound, so the source comes from the raw field; the
    allow-list still applies, and nothing is ever answered ``rejected``."""

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as anonymous:
            before = robotd.turns.current
            await anonymous.send(type="stop", source="web")
            await until(lambda: robotd.turns.current != before)

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_stop_from_a_source_that_may_not_command_is_ignored(tmp_path: Path) -> None:
    """``teleop`` is not in the default ``allow_sources``; rover-web's STOP
    button is its ``web`` session, which is."""

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as anonymous:
            before = robotd.turns.current
            await anonymous.send(type="stop", source="teleop")
            await asyncio.sleep(0.3)
            assert robotd.turns.current == before
            with pytest.raises(TimeoutError):
                await anonymous.recv(timeout=0.3)

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_estop_latches_persists_and_blocks_every_later_skill(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as web:
            await web.hello(source="web", caps=("skill",))
            await web.send(type="estop", source="web", reason="user")
            await until(lambda: robotd.estop_sw)
            await until(robotd.estop_path.exists)

            await web.send(
                type="turn", source="web", turn_id=robotd.turns.current or TURN
            )
            await web.send(
                type="skill",
                source="web",
                cmd_id=CMD,
                seq=1,
                turn_id=robotd.turns.current,
                issued_mono_ns=1,
                goal_ttl_ms=5000,
                skill="drive",
                args={"distance_m": 0.4, "speed_mps": 0.15},
                obs={"frame_id": "cam-000001", "frame_mono_ns": time.monotonic_ns()},
            )
            result = await web.recv_type("result")
            assert result["status"] == "rejected"
            assert result["reason"] == "estop_active"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_clear_from_web_lifts_the_latch_and_brain_may_not(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as web, client(robotd) as brain:
            await web.hello(source="web", caps=("skill",))
            await brain.hello(source="brain", caps=("skill",))
            await web.send(type="estop", source="web")
            await until(lambda: robotd.estop_sw)

            await brain.send(type="clear", source="brain", faults=["estop_sw"])
            error = await brain.recv_type("error")
            assert error["code"] == "source_not_allowed"
            assert robotd.estop_sw, "brain must not be able to clear a stop authority"

            await web.send(type="clear", source="web", faults=["estop_sw"])
            await until(lambda: not robotd.estop_sw)
            await until(lambda: not robotd.estop_path.exists())
            assert not robotd.link.armed, "a clear must not re-arm"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_stale_turn_is_rejected_over_the_wire(tmp_path: Path) -> None:
    """I-11's bare-turn-boundary path, end to end.

    The skill here is non-motion, because with no controller attached a motion
    skill is refused by the telemetry row of 4.2 first -- that row sits above
    ``turn_id`` in the document's order.  The motion case with a live link is
    covered in ``test_robotd_validator.py``.
    """

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await asyncio.sleep(0.1)
            await brain.send(
                type="skill",
                source="brain",
                cmd_id=CMD,
                seq=1,
                turn_id=OLD_TURN,
                issued_mono_ns=1,
                goal_ttl_ms=1000,
                skill="say",
                args={"text": "a late answer to a superseded instruction"},
            )
            result = await brain.recv_type("result")
            assert result["status"] == "rejected"
            assert result["reason"] == "stale_turn"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_turn_boundary_never_rolls_back(tmp_path: Path) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await until(lambda: robotd.turns.current == TURN)
            await brain.send(type="turn", source="brain", turn_id=OLD_TURN)
            await asyncio.sleep(0.2)
            assert robotd.turns.current == TURN

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_non_motion_skill_is_accepted_with_the_link_down(tmp_path: Path) -> None:
    """say and describe_scene must stay permitted when motion is not."""

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await asyncio.sleep(0.1)
            await brain.send(
                type="skill",
                source="brain",
                cmd_id=CMD,
                seq=1,
                turn_id=TURN,
                issued_mono_ns=1,
                goal_ttl_ms=1000,
                skill="say",
                args={"text": "there is a mug on the table"},
            )
            result = await brain.recv_type("result")
            assert result["status"] == "accepted"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_a_motion_skill_is_refused_while_the_controller_is_absent(
    tmp_path: Path,
) -> None:
    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await asyncio.sleep(0.1)
            await brain.send(
                type="skill",
                source="brain",
                cmd_id=CMD,
                seq=1,
                turn_id=TURN,
                issued_mono_ns=1,
                goal_ttl_ms=5000,
                skill="drive",
                args={"distance_m": 0.4, "speed_mps": 0.15},
                obs={"frame_id": "cam-000001", "frame_mono_ns": time.monotonic_ns()},
            )
            result = await brain.recv_type("result")
            assert result["status"] == "rejected"
            assert result["reason"] == "not_ready"

    run(scenario, tmp_path)


@pytest.mark.timeout(30)
def test_the_estop_latch_survives_a_restart(tmp_path: Path) -> None:
    """A crash-restart must not silently clear a stop authority."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    config = _config(run_dir, tmp_path / "logs")
    first = Robotd(config)
    assert not first.estop_sw
    first._persist_estop()  # noqa: SLF001 - what an estop does, without a socket
    assert Robotd(config).estop_sw, "estop_sw must be read back at boot"
    first.estop_path.unlink()
    assert not Robotd(config).estop_sw


@pytest.mark.timeout(60)
def test_a_hostile_message_costs_that_message_and_nothing_else(
    tmp_path: Path,
) -> None:
    """I-8's fuzz corpus lands on this path: no case may kill the connection
    or the daemon, and no stop-class case may be answered ``rejected``."""
    corpus: list[dict[str, Any]] = [
        {"type": "skill"},
        {"type": "skill", "skill": "fly", "args": {}},
        {"type": "skill", "cmd_id": "not-a-ulid", "skill": "drive", "args": {}},
        {"type": "skill", "cmd_id": 7, "skill": "drive", "args": {"distance_m": "x"}},
        {"type": "twist", "twist": {"linear_x_mps": 9e9, "angular_z_radps": 0}},
        {"type": "twist", "cmd_id": None, "seq": -1, "twist": {}},
        {"type": "turn", "turn_id": ""},
        {"type": "clear", "faults": []},
        {"type": "clear", "source": "web", "faults": ["not_a_fault"]},
        {"type": "hello", "source": "nobody", "pid": 0, "caps": []},
        {"type": "subscribe", "topics": ["nonsense"]},
        {"type": "stop", "source": "web", "seq": "x", "cmd_id": 12},
        {"type": "estop", "source": 5},
        {"type": "cancel", "source": "web", "cmd_id": {"a": 1}},
        {"type": "nonsense"},
        {"type": 42},
        {},
    ]

    async def scenario(tmp: Path) -> None:
        async with daemon(tmp) as robotd, client(robotd) as brain:
            await brain.hello(source="web", caps=("skill", "subscribe"))
            await brain.send(type="subscribe", topics=["result", "event"])
            for case in corpus:
                await brain.send(**case)
            await asyncio.sleep(0.5)
            with contextlib.suppress(TimeoutError):
                while True:
                    message = await brain.recv(timeout=0.2)
                    if message.get("type") == "result":
                        assert message["cmd_id"] != "not-a-ulid"
            # the daemon and the connection are both still alive.  The id has
            # to be freshly minted: a stop-class case above advanced the
            # boundary, and a turn_id never rolls back.
            fresh = new_turn_id()
            await brain.send(type="ping", source="web")
            await brain.send(type="turn", source="web", turn_id=fresh)
            await until(lambda: robotd.turns.current == fresh)

    run(scenario, tmp_path)


# ---------------------------------------------------------------------------
# Backpressure (A10)
# ---------------------------------------------------------------------------


class _NullWriter:
    def write(self, data: bytes) -> None:  # pragma: no cover - never drained here
        pass

    async def drain(self) -> None:  # pragma: no cover
        pass

    def close(self) -> None:
        pass


def _connection(max_queued: int = 4) -> BusConnection:
    return BusConnection(ClientSession(), _NullWriter(), max_queued=max_queued)  # type: ignore[arg-type]


def test_state_is_newest_only(tmp_path: Path) -> None:
    async def scenario() -> None:
        from rover_contracts.messages import ErrorMessage

        connection = _connection()
        for index in range(10):
            connection.send_state(ErrorMessage(code="state", detail=str(index)))
        assert connection._state is not None  # noqa: SLF001 - the one-slot queue
        assert b'"9"' in connection._state  # noqa: SLF001

    asyncio.run(scenario())


def test_a_reliable_queue_drops_its_oldest_rather_than_blocking(
    tmp_path: Path,
) -> None:
    async def scenario() -> None:
        from rover_contracts.messages import ErrorMessage

        connection = _connection(max_queued=4)
        for index in range(10):
            connection.send(ErrorMessage(code="result", detail=str(index)))
        assert connection.dropped == 6
        assert len(connection._reliable) == 4  # noqa: SLF001

    asyncio.run(scenario())


def test_a_connection_only_receives_topics_it_subscribed_to() -> None:
    connection = _connection()
    assert not connection.wants(SubscribeTopic.STATE)
    connection.session.topics = frozenset({SubscribeTopic.RESULT})
    assert connection.wants(SubscribeTopic.RESULT)
    assert not connection.wants(SubscribeTopic.EVENT)
