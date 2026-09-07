"""rover-web: the pages serve, STOP posts the right message, oversized uploads
are refused, and the app comes up with robotd and brain absent
(ARCHITECTURE 4.5, I-22)."""

from __future__ import annotations

import asyncio
import contextlib
import json
import sys
import tempfile
from collections.abc import AsyncIterator
from pathlib import Path

import aiohttp
import pytest
from aiohttp import web

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.config import BusConfig, RobotConfig  # noqa: E402
from rover_contracts.messages import (  # noqa: E402
    ClearableFault,
    EstopMessage,
    HelloMessage,
    StopMessage,
    SubscribeMessage,
    TwistMessage,
    UtteranceMessage,
    brain_client_adapter,
    client_adapter,
)
from rover_web.app import (  # noqa: E402
    MAX_BODY_BYTES,
    STATIC_DIR,
    TeleopStream,
    create_app,
)

pytestmark = pytest.mark.asyncio


# --------------------------------------------------------------------------
# harness
# --------------------------------------------------------------------------


class FakePeer:
    """A stand-in robotd or brain: records NDJSON lines, can push some back."""

    def __init__(self, path: str | Path) -> None:
        self.path = str(path)
        self.lines: list[str] = []
        self._server: asyncio.AbstractServer | None = None
        self._writers: list[asyncio.StreamWriter] = []

    async def start(self) -> None:
        self._server = await asyncio.start_unix_server(self._handle, path=self.path)

    async def _handle(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._writers.append(writer)
        try:
            with contextlib.suppress(OSError, ConnectionError, ValueError):
                async for line in reader:
                    self.lines.append(line.decode().rstrip("\n"))
        finally:
            # EOF leaves a StreamReaderProtocol half-open, and the server's
            # active count with it, so wait_closed() would never return.
            writer.close()

    async def push(self, payload: dict[str, object]) -> None:
        for writer in self._writers:
            writer.write(json.dumps(payload).encode() + b"\n")
            await writer.drain()

    async def close(self) -> None:
        for writer in self._writers:
            writer.close()
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(self._server.wait_closed(), 2.0)

    def typed(self, kind: str) -> list[dict[str, object]]:
        return [json.loads(line) for line in self.lines if f'"type":"{kind}"' in line]


async def _wait(predicate, timeout: float = 3.0) -> None:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.01)
    raise AssertionError("timed out")


def _config(tmp: Path, *, allow_teleop: bool = False) -> RobotConfig:
    return RobotConfig(
        bus=BusConfig(
            sock=str(tmp / "robotd.sock"),
            frames_sock=str(tmp / "frames.sock"),
            brain_sock=str(tmp / "brain.sock"),
            allow_stream=["teleop"] if allow_teleop else [],
        )
    )


@contextlib.asynccontextmanager
async def _serving(config: RobotConfig) -> AsyncIterator[aiohttp.ClientSession]:
    app = create_app(config)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    port = runner.addresses[0][1]
    session = aiohttp.ClientSession(base_url=f"http://127.0.0.1:{port}")
    try:
        yield session
    finally:
        await session.close()
        await runner.cleanup()


@contextlib.contextmanager
def _tmpdir():
    # Unix socket paths are capped near 104 bytes on macOS.
    with tempfile.TemporaryDirectory(prefix="rw") as name:
        yield Path(name)


# --------------------------------------------------------------------------
# pages
# --------------------------------------------------------------------------


async def test_every_page_serves_and_carries_the_stop_form():
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            for route in ("/", "/teleop", "/ptt"):
                response = await session.get(route)
                assert response.status == 200
                body = await response.text()
                assert 'class="stop"' in body
                assert 'action="/estop"' in body
            for asset in ("/static/rover.css", "/static/rover.js"):
                assert (await session.get(asset)).status == 200


async def test_no_page_loads_an_external_asset():
    """MUST NOT reach the network: the rover runs with no route off the LAN."""
    for path in sorted(STATIC_DIR.iterdir()):
        text = path.read_text()
        assert "http://" not in text
        assert "https://" not in text
        assert 'src="//' not in text


# --------------------------------------------------------------------------
# the stop authorities
# --------------------------------------------------------------------------


async def test_stop_button_posts_an_estop_from_the_web_source():
    with _tmpdir() as tmp:
        robotd = FakePeer(tmp / "robotd.sock")
        await robotd.start()
        async with _serving(_config(tmp)) as session:
            await _wait(lambda: len(robotd.lines) >= 2)
            hello = client_adapter.validate_json(robotd.lines[0])
            subscribe = client_adapter.validate_json(robotd.lines[1])
            assert isinstance(hello, HelloMessage)
            assert hello.source == "web"
            assert isinstance(subscribe, SubscribeMessage)

            response = await session.post("/estop")
            assert response.status == 200
            assert await response.json() == {"ok": True, "delivered": True}

            await _wait(lambda: len(robotd.lines) >= 3)
            estop = client_adapter.validate_json(robotd.lines[2])
            assert isinstance(estop, EstopMessage)
            assert estop.type == "estop"
            assert estop.source == "web"
        await robotd.close()


async def test_stop_and_clear_also_reach_robotd():
    with _tmpdir() as tmp:
        robotd = FakePeer(tmp / "robotd.sock")
        await robotd.start()
        async with _serving(_config(tmp)) as session:
            await _wait(lambda: len(robotd.lines) >= 2)
            assert (await session.post("/stop")).status == 200
            assert (await session.post("/clear")).status == 200
            await _wait(lambda: len(robotd.lines) >= 4)
            stop = client_adapter.validate_json(robotd.lines[2])
            clear = json.loads(robotd.lines[3])
            assert isinstance(stop, StopMessage)
            assert clear["type"] == "clear" and clear["faults"] == ["estop_sw"]
        await robotd.close()


async def test_clear_names_the_latched_fault_the_page_asked_for():
    """I-20's designed event latches WDT_REBOOT, not estop_sw.  With no body
    form, the page's only recovery control cleared the wrong bit and the rover
    stayed dead to voice, web and teleop."""
    with _tmpdir() as tmp:
        robotd = FakePeer(tmp / "robotd.sock")
        await robotd.start()
        async with _serving(_config(tmp)) as session:
            await _wait(lambda: len(robotd.lines) >= 2)
            response = await session.post(
                "/clear", json={"faults": ["wdt_reboot", "brownout"]}
            )
            assert response.status == 200
            assert (await response.json())["faults"] == ["wdt_reboot", "brownout"]
            await _wait(lambda: len(robotd.lines) >= 3)
            clear = json.loads(robotd.lines[2])
            assert clear["type"] == "clear"
            assert clear["source"] == "web"
            assert clear["faults"] == ["wdt_reboot", "brownout"]

            # An unknown name is a 400, not a silently dropped clear.
            bad = await session.post("/clear", json={"faults": ["nonsense"]})
            assert bad.status == 400
            empty = await session.post("/clear", json={"faults": []})
            assert empty.status == 400
        await robotd.close()


async def test_status_publishes_the_latched_fault_bit_table():
    """The face page decodes state.mcu.fault with this table rather than
    restating the 5.1 class in JavaScript."""
    from rover_contracts.serial_codec import LATCHED_FAULTS, Fault

    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            table = (await (await session.get("/status")).json())["latched_faults"]
    assert table["wdt_reboot"] == int(Fault.WDT_REBOOT)
    assert table["obstacle_latched"] == int(Fault.OBSTACLE_LATCHED)
    assert "tof_stop" not in table  # obstacle class: the MCU is the sole clearer
    assert "ttl" not in table  # advisory
    assert sum(table.values()) == int(LATCHED_FAULTS)
    # Every name the panel can offer is a ClearableFault robotd will accept.
    assert set(table) <= {str(f) for f in ClearableFault}


async def test_stop_button_answers_200_with_robotd_dead():
    """It must report honestly, not fail: the button is an authority, not a
    dependency, and the page has to say the hardware button is the fallback."""
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            response = await session.post("/estop")
            assert response.status == 200
            assert await response.json() == {"ok": True, "delivered": False}


# --------------------------------------------------------------------------
# brain.sock
# --------------------------------------------------------------------------


async def test_text_box_emits_an_utterance_on_brain_sock():
    with _tmpdir() as tmp:
        brain = FakePeer(tmp / "brain.sock")
        await brain.start()
        async with _serving(_config(tmp)) as session:
            response = await session.post(
                "/utter", json={"text": "turn left ninety degrees"}
            )
            assert response.status == 200
            assert (await response.json())["delivered"] is True
            await _wait(lambda: bool(brain.lines))
            message = brain_client_adapter.validate_json(brain.lines[0])
            assert isinstance(message, UtteranceMessage)
            assert message.source == "web"
            assert message.text == "turn left ninety degrees"
            assert message.is_final is True
            assert message.confidence is None  # null counts as authorized
        await brain.close()


async def test_a_face_message_reaches_the_browser_verbatim():
    with _tmpdir() as tmp:
        brain = FakePeer(tmp / "brain.sock")
        await brain.start()
        async with (
            _serving(_config(tmp)) as session,
            session.ws_connect("/ws") as ws,
        ):
            await _wait(lambda: bool(brain._writers))
            await brain.push({"v": 1, "type": "face", "expr": "happy"})
            message = await asyncio.wait_for(ws.receive_json(), 3.0)
            assert message == {"v": 1, "type": "face", "expr": "happy"}
        await brain.close()


# --------------------------------------------------------------------------
# uploads
# --------------------------------------------------------------------------


async def test_an_oversized_upload_is_refused():
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            response = await session.post(
                "/utter", data=b"x" * (MAX_BODY_BYTES + 1024)
            )
            assert response.status == 413


async def test_an_overlong_utterance_is_refused():
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            assert (await session.post("/utter", json={"text": "x" * 501})).status == 400
            assert (await session.post("/utter", json={"text": "  "})).status == 400
            assert (await session.post("/utter", data=b"not json")).status == 400


# --------------------------------------------------------------------------
# teleop
# --------------------------------------------------------------------------


class _StubClient:
    def __init__(self) -> None:
        self.sent: list[TwistMessage] = []
        self._seq = 0

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def send(self, message: TwistMessage) -> bool:
        self.sent.append(message)
        return True


async def test_stale_browser_input_sends_one_zero_then_stops():
    client = _StubClient()
    stream = TeleopStream(
        client,
        linear_mps=0.30,
        angular_radps=1.047,
        input_max_age_ms=60,
        hz=500.0,
    )
    stream.update(0.5, 1.0)
    assert stream.streaming
    await asyncio.sleep(0.25)
    assert not stream.streaming

    first, last = client.sent[0], client.sent[-1]
    assert (first.twist.linear_x_mps, first.twist.angular_z_radps) == (0.30, 0.5235)
    assert (last.twist.linear_x_mps, last.twist.angular_z_radps) == (0.0, 0.0)
    zeros = [
        m
        for m in client.sent
        if m.twist.linear_x_mps == m.twist.angular_z_radps == 0.0
    ]
    assert len(zeros) == 1
    assert [m.seq for m in client.sent] == sorted(m.seq for m in client.sent)

    before = len(client.sent)
    await stream.close()
    assert len(client.sent) == before  # a stopped stream sends nothing more


async def test_closing_the_socket_is_an_immediate_zero():
    client = _StubClient()
    stream = TeleopStream(
        client, linear_mps=0.30, angular_radps=1.047, input_max_age_ms=5000
    )
    stream.update(1.0, 1.0)
    await stream.close()
    assert not stream.streaming
    assert client.sent[-1].twist.linear_x_mps == 0.0
    assert client.sent[-1].twist.angular_z_radps == 0.0


async def test_a_browser_cannot_name_an_out_of_bounds_twist():
    with _tmpdir() as tmp:
        robotd = FakePeer(tmp / "robotd.sock")
        await robotd.start()
        async with (
            _serving(_config(tmp, allow_teleop=True)) as session,
            session.ws_connect("/ws") as ws,
        ):
            await _wait(lambda: len(robotd.typed("hello")) == 2)
            await ws.send_json({"type": "twist", "x": 9.0, "y": -9.0})
            await _wait(lambda: bool(robotd.typed("twist")))
            twist = client_adapter.validate_json(json.dumps(robotd.typed("twist")[0]))
            assert isinstance(twist, TwistMessage)
            assert twist.source == "teleop"
            assert twist.twist.linear_x_mps == -0.30
            assert twist.twist.angular_z_radps == 1.047
        await robotd.close()


async def test_teleop_answers_source_not_allowed_until_allow_stream_is_set():
    with _tmpdir() as tmp:
        async with (
            _serving(_config(tmp)) as session,
            session.ws_connect("/ws") as ws,
        ):
            await ws.send_json({"type": "twist", "x": 0.0, "y": 0.5})
            message = await asyncio.wait_for(ws.receive_json(), 3.0)
            assert message == {"type": "error", "code": "source_not_allowed"}


# --------------------------------------------------------------------------
# the rover behaves identically with rover-web's peers dead
# --------------------------------------------------------------------------


async def test_the_app_starts_with_robotd_and_brain_absent():
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            assert (await session.get("/")).status == 200
            status = await (await session.get("/status")).json()
            assert status["robotd"] is False
            assert status["brain"] is False
            assert status["teleop_allowed"] is False
            assert status["teleop"] is False
            assert status["latched_faults"]["wdt_reboot"] == 0x20000
            assert (await session.post("/utter", json={"text": "hello"})).status == 200


async def test_a_late_robotd_is_picked_up_by_the_reconnect_loop():
    with _tmpdir() as tmp:
        async with _serving(_config(tmp)) as session:
            robotd = FakePeer(tmp / "robotd.sock")
            await robotd.start()
            await _wait(lambda: len(robotd.lines) >= 2, timeout=5.0)
            assert (await session.post("/estop")).status == 200
            await _wait(lambda: any('"type":"estop"' in x for x in robotd.lines))
            await robotd.close()
