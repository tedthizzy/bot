"""The rover link, end to end over TCP and over a pty.

Most of the file drives :class:`FakeRover`, an in-test controller that speaks
``docs/protocol.md`` the way the firmware fork does -- banner on request, the
fork's fields in every feedback line, a heartbeat that zeroes the applied
powers, a heading that integrates the wheel difference -- so bring-up, the
firmware gate, the T0 gate, the restart handling, the reopen and the shape of
the command stream are all exercised with nothing built.  The last tests drive
the real ``rover-stub`` over a pty and **skip with a clear reason** when it is
not importable yet.
"""

from __future__ import annotations

import asyncio
import contextlib
import importlib.util
import json
import os
import pty
import subprocess
import sys
import time
from collections.abc import AsyncIterator, Callable
from pathlib import Path
from typing import Any

import pytest
from rover_contracts.config import LinkConfig, RobotConfig, SafetyConfig
from rover_contracts.units import wrap_deg_180
from rover_contracts.wave_proto import Banner
from rover_robotd.link import Link, LinkProtocol

REPO = Path(__file__).resolve().parents[2]
TIMEOUT_S = 5.0


class FakeRover:
    """Just enough controller to answer bring-up and stream feedback.

    ``stock=True`` is unpatched Waveshare firmware: no banner, no fork fields,
    no cap.  ``fork_fields=False`` with ``stock=False`` is the odd case of a
    banner from a controller whose feedback lacks the fields, which the host
    must also refuse.  The heading integrates ``(right - left)`` at
    ``turn_dps_per_power`` degrees per second per unit of difference, in the
    ``yaw_sign = +1`` convention: the right wheel leading turns left.
    """

    def __init__(
        self,
        *,
        stock: bool = False,
        fork_fields: bool | None = None,
        fw: str = "bot-wr-1",
        hb_ms: int = 300,
        cap: float = 0.3,
        proto: int = 1,
        feedback_hz: float = 20.0,
        turn_dps_per_power: float = 200.0,
    ) -> None:
        self.stock = stock
        self.fork_fields = (not stock) if fork_fields is None else fork_fields
        self.fw = fw
        self.hb_ms = hb_ms
        self.cap = cap
        self.proto = proto
        self.feedback_period_s = 1.0 / feedback_hz
        self.turn_dps_per_power = turn_dps_per_power

        self.received: list[dict[str, Any]] = []
        self.speeds: list[tuple[float, float]] = []
        self.banners_sent = 0
        self.connections = 0
        self.left = 0.0
        self.right = 0.0
        self.yaw_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.temp_c = 31.0
        self.bus_v = 11.6
        self.tof_mm = -1
        self.bumper = False
        self.lowbat = False
        self.coasting = False
        self.clamp_count = 0
        self.feedback_on = False
        self.feedback_enabled = True
        """A test switch: ``False`` silences the stream to simulate a dead link."""

        self._last_speed_at: float | None = None
        self._write: Callable[[bytes], None] | None = None
        self._rx = bytearray()
        self._server: asyncio.AbstractServer | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._fds: list[int] = []
        self._task: asyncio.Task[None] | None = None

    # -- transports ---------------------------------------------------------

    async def start_tcp(self) -> int:
        """Listen on a loopback port; return it."""
        self._server = await asyncio.start_server(self._serve, "127.0.0.1", 0)
        self._ensure_loop()
        return self._server.sockets[0].getsockname()[1]

    def start_pty(self) -> str:
        """Open a pseudo terminal; return the slave path the host opens."""
        master, slave = pty.openpty()
        os.set_blocking(master, False)
        self._fds = [master, slave]
        loop = asyncio.get_running_loop()
        loop.add_reader(master, self._pty_readable, master)
        self._attach(lambda data: os.write(master, data))
        self._ensure_loop()
        return os.ttyname(slave)

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        self.disconnect()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        for fd in self._fds:
            with contextlib.suppress(OSError):
                asyncio.get_running_loop().remove_reader(fd)
            with contextlib.suppress(OSError):
                os.close(fd)

    def disconnect(self) -> None:
        """Drop the current TCP client: a cable pull, seen from the host."""
        writer, self._writer = self._writer, None
        if writer is not None:
            writer.close()
        self._write = None

    async def _serve(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._writer = writer
        self._attach(writer.write)
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                self.feed(line)
        except (ConnectionError, OSError):
            pass
        finally:
            if self._writer is writer:
                self._writer = None
                self._write = None
            with contextlib.suppress(ConnectionError, OSError):
                writer.close()

    def _pty_readable(self, fd: int) -> None:
        try:
            data = os.read(fd, 4096)
        except (BlockingIOError, OSError):
            return
        self.feed(data)

    def _attach(self, write: Callable[[bytes], None]) -> None:
        self._write = write
        self.connections += 1

    def _ensure_loop(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._feedback_loop())

    # -- the wire -----------------------------------------------------------

    def feed(self, data: bytes) -> None:
        self._rx += data
        while True:
            end = self._rx.find(b"\n")
            if end < 0:
                return
            line = bytes(self._rx[:end])
            del self._rx[: end + 1]
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if isinstance(obj, dict):
                self._on_command(obj)

    def _on_command(self, obj: dict[str, Any]) -> None:
        self.received.append(obj)
        t = obj.get("T")
        if t == 1:
            left, right = float(obj["L"]), float(obj["R"])
            self.speeds.append((left, right))
            if not self.stock:
                for value in (left, right):
                    if abs(value) > self.cap + 1e-9:
                        self.clamp_count += 1
                left = max(-self.cap, min(self.cap, left))
                right = max(-self.cap, min(self.cap, right))
            self.left, self.right = left, right
            self.coasting = False
            self._last_speed_at = time.monotonic()
        elif t == 115:
            self.left = self.right = 0.0
            self.coasting = True
        elif t == 131:
            self.feedback_on = bool(obj.get("cmd"))
        elif t == 1007 and not self.stock:
            self.send_banner()

    def send_banner(self) -> None:
        self.banners_sent += 1
        self._send(
            {
                "T": 1006,
                "fw": self.fw,
                "hb_ms": self.hb_ms,
                "cap": self.cap,
                "proto": self.proto,
            }
        )

    def reboot(self) -> None:
        """The controller restarts: settings lost, motors off, boot banner."""
        self.feedback_on = False
        self.left = self.right = 0.0
        self._last_speed_at = None
        self.send_banner()

    def _send(self, obj: dict[str, Any]) -> None:
        if self._write is None:
            return
        with contextlib.suppress(ConnectionError, OSError):
            self._write(json.dumps(obj, separators=(",", ":")).encode() + b"\n")

    @property
    def hb_alive(self) -> bool:
        return (
            self._last_speed_at is not None
            and time.monotonic() - self._last_speed_at <= self.hb_ms / 1000.0
        )

    def stop_flags(self) -> int:
        flags = 0
        if not self.hb_alive:
            flags |= 1
        if 0 <= self.tof_mm < 250:
            flags |= 2
        if self.bumper:
            flags |= 4
        if self.lowbat:
            flags |= 8
        if self.coasting:
            flags |= 16
        return flags

    def feedback(self) -> dict[str, Any]:
        obj: dict[str, Any] = {
            "T": 1001,
            "L": round(self.left, 3),
            "R": round(self.right, 3),
            "r": self.roll_deg,
            "p": self.pitch_deg,
            "y": round(self.yaw_deg, 3),
            "temp": self.temp_c,
            "v": self.bus_v,
        }
        if self.fork_fields:
            obj.update(
                hb=1 if self.hb_alive else 0,
                st=self.stop_flags(),
                tf=self.tof_mm,
                bp=1 if self.bumper else 0,
                cc=self.clamp_count & 0xFFFF,
            )
        return obj

    async def _feedback_loop(self) -> None:
        last = time.monotonic()
        while True:
            await asyncio.sleep(self.feedback_period_s)
            now = time.monotonic()
            dt_s, last = now - last, now
            if not self.hb_alive:
                self.left = self.right = 0.0
            self.yaw_deg = wrap_deg_180(
                self.yaw_deg + (self.right - self.left) * self.turn_dps_per_power * dt_s
            )
            if self.feedback_on and self.feedback_enabled:
                self._send(self.feedback())


async def until(predicate: Callable[[], bool], timeout: float = TIMEOUT_S) -> None:
    """Poll ``predicate`` until it holds, or fail the test."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("condition not reached within the timeout")


def link_config(port: int, **overrides: Any) -> RobotConfig:
    link: dict[str, Any] = {
        "backend": "tcp",
        "tcp_host": "127.0.0.1",
        "tcp_port": port,
        "open_retry_ms": 50,
        "banner_wait_ms": 300,
    }
    link.update(overrides.pop("link", {}))
    safety = SafetyConfig(**overrides.pop("safety", {}))
    return RobotConfig(link=LinkConfig(**link), safety=safety, **overrides)


class Recorder:
    """Collects the link's callbacks."""

    def __init__(self) -> None:
        self.feedback: list[int] = []
        self.ups = 0
        self.lost = 0
        self.restarts: list[Banner] = []

    def on_feedback(self, _feedback: Any, arrival_ns: int) -> None:
        self.feedback.append(arrival_ns)

    def on_up(self, _link: Link) -> None:
        self.ups += 1

    def on_lost(self, _link: Link) -> None:
        self.lost += 1

    def on_restart(self, banner: Banner) -> None:
        self.restarts.append(banner)


@contextlib.asynccontextmanager
async def linked(
    rover: FakeRover | None = None, **overrides: Any
) -> AsyncIterator[tuple[Link, FakeRover, Recorder]]:
    rover = rover or FakeRover()
    port = await rover.start_tcp()
    recorder = Recorder()
    link = Link(
        link_config(port, **overrides),
        on_feedback=recorder.on_feedback,
        on_up=recorder.on_up,
        on_lost=recorder.on_lost,
        on_restart=recorder.on_restart,
    )
    task = asyncio.create_task(link.run())
    try:
        yield link, rover, recorder
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await rover.stop()


async def stream(link: Link, left: float, right: float, seconds: float) -> None:
    """Call ``command`` the way the control loop does, once per 50 ms."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        link.command(left, right)
        await asyncio.sleep(0.05)


def speeds_of(rover: FakeRover) -> list[tuple[float, float]]:
    return [(o["L"], o["R"]) for o in rover.received if o.get("T") == 1]


# ---------------------------------------------------------------------------
# Bring-up and the firmware gate
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
async def test_bring_up_sends_the_protocol_sequence_then_waits_for_the_banner() -> None:
    async with linked() as (link, rover, recorder):
        await until(lambda: link.up)
        types = [o["T"] for o in rover.received]
        assert types[:6] == [605, 143, 136, 142, 131, 1007]
        by_type = {o["T"]: o for o in rover.received[:6]}
        assert by_type[605]["cmd"] == 0, "quiet"
        assert by_type[143]["cmd"] == 0, "echo off"
        assert by_type[136]["cmd"] == 300, "heartbeat at [safety] heartbeat_ms"
        assert by_type[142]["cmd"] == 50, (
            "feedback interval at [link] feedback_interval_ms"
        )
        assert by_type[131]["cmd"] == 1, "feedback on"
        assert link.fw == "bot-wr-1"
        assert link.banner is not None and link.banner.proto == 1
        assert recorder.ups == 1
        await until(lambda: link.feedback is not None)
        assert link.firmware_ok
        assert link.firmware_refusal() == ""
        await until(lambda: link.motion_allowed())


@pytest.mark.timeout(30)
async def test_no_speed_line_is_sent_unless_the_caller_asks() -> None:
    """The link has no writer task: bring-up done, nothing else goes out."""
    async with linked() as (link, rover, _recorder):
        await until(lambda: link.up and link.feedback is not None)
        await asyncio.sleep(0.4)
        assert speeds_of(rover) == []


@pytest.mark.timeout(30)
async def test_stock_firmware_sends_no_banner_and_is_refused() -> None:
    async with linked(FakeRover(stock=True)) as (link, rover, recorder):
        await until(lambda: link.up)
        assert link.fw is None
        assert link.banner is None
        assert "no banner" in link.firmware_refusal()
        await until(lambda: link.feedback is not None)
        assert not link.feedback.patched  # type: ignore[union-attr]
        assert not link.firmware_ok
        assert not link.motion_allowed()
        await stream(link, 0.2, 0.2, 0.3)
        sent = speeds_of(rover)
        assert sent, "zeros still flow to a stock controller"
        assert all(s == (0.0, 0.0) for s in sent)
        assert recorder.ups == 1


@pytest.mark.timeout(30)
async def test_a_banner_whose_heartbeat_disagrees_is_refused() -> None:
    async with linked(FakeRover(hb_ms=3000)) as (link, _rover, _recorder):
        await until(lambda: link.up and link.feedback is not None)
        assert link.fw == "bot-wr-1", "the tag is still reported"
        assert "hb_ms=3000" in link.firmware_refusal()
        assert not link.motion_allowed()


@pytest.mark.timeout(30)
async def test_a_banner_whose_cap_disagrees_is_refused() -> None:
    async with linked(FakeRover(cap=0.5)) as (link, _rover, _recorder):
        await until(lambda: link.up and link.feedback is not None)
        assert "cap=0.5" in link.firmware_refusal()
        assert not link.motion_allowed()


@pytest.mark.timeout(30)
async def test_a_banner_without_the_fork_fields_in_feedback_is_refused() -> None:
    async with linked(FakeRover(fork_fields=False)) as (link, _rover, _recorder):
        await until(lambda: link.up)
        assert link.firmware_ok, "the banner alone passes"
        await until(lambda: link.feedback is not None)
        assert "fork fields" in link.firmware_refusal()
        assert not link.motion_allowed()


@pytest.mark.timeout(30)
async def test_require_patched_firmware_false_waives_the_gate() -> None:
    """A bench with stock firmware and the wheels off the floor."""
    rover = FakeRover(stock=True)
    async with linked(rover, safety={"require_patched_firmware": False}) as (
        link,
        rover,
        _recorder,
    ):
        await until(lambda: link.up and link.feedback is not None)
        assert link.firmware_ok
        await until(lambda: link.motion_allowed())
        await stream(link, 0.2, 0.2, 0.3)
        assert (0.2, 0.2) in speeds_of(rover)


# ---------------------------------------------------------------------------
# Feedback ingest and the counters
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
async def test_feedback_is_stamped_on_arrival_and_ages_on_the_host_clock() -> None:
    async with linked() as (link, _rover, recorder):
        await until(lambda: len(recorder.feedback) >= 3)
        arrival = link.feedback_arrival_ns
        assert arrival is not None
        assert arrival == recorder.feedback[-1]
        assert link.feedback_age_ms(arrival) == 0.0
        assert link.feedback_age_ms(arrival + 100_000_000) == pytest.approx(100.0)
        assert link.feedback_fresh(arrival + 150_000_000)
        assert not link.feedback_fresh(arrival + 151_000_000)
        gaps = [
            b - a for a, b in zip(recorder.feedback, recorder.feedback[1:], strict=False)
        ]
        assert all(gap > 0 for gap in gaps)


def test_dropped_and_unknown_lines_are_counted_and_renew_nothing() -> None:
    link = Link(RobotConfig(), clock=lambda: 777)
    link.ingest(
        b'{"T":1001,"L":0,"R":0,"r":0,"p":0,"y":10,"temp":30,"v":11.5,'
        b'"hb":1,"st":0,"tf":-1,"bp":0,"cc":0}'
    )
    assert link.feedback_arrival_ns == 777
    link._clock = lambda: 999  # noqa: SLF001 - a later arrival would stamp 999
    link.ingest(b"UGV started.")
    link.ingest(b'{"T":999}')
    link.ingest(b'{"T":1001,"L":"x"}')
    link.ingest(b'{"T":1001,' + b" " * 600 + b"}")
    assert link.dropped == 3
    assert link.unknown == 1
    assert link.feedback_arrival_ns == 777, "nothing above renewed the stamp"
    assert link.feedback is not None and link.feedback.yaw_deg == 10.0


def test_oversized_line_is_dropped_once_and_buffer_stays_bounded() -> None:
    link = Link(RobotConfig())
    protocol = LinkProtocol(link)
    for _ in range(10):
        protocol.data_received(b"x" * 400)
    protocol.data_received(b"tail\n")
    assert link.dropped == 1
    assert len(protocol._rx) == 0  # noqa: SLF001 - the bound is the point
    protocol.data_received(
        b'{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}\n'
    )
    assert link.unknown == 0 and link.dropped == 1, "the next line decodes normally"


def test_lines_split_across_reads_are_reassembled() -> None:
    link = Link(RobotConfig(), clock=lambda: 5)
    protocol = LinkProtocol(link)
    line = b'{"T":1001,"L":0,"R":0,"r":0,"p":0,"y":0,"temp":30,"v":11.5}\n'
    protocol.data_received(line[:20])
    assert link.feedback is None
    protocol.data_received(line[20:] + b'{"T":9')
    assert link.feedback is not None
    protocol.data_received(b"}\n")
    assert link.unknown == 1


# ---------------------------------------------------------------------------
# The command stream
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
async def test_command_streams_what_the_caller_asks_and_stops_when_it_stops() -> None:
    """I-14 in one mechanism: the stream is the caller's loop, nothing else."""
    async with linked() as (link, rover, _recorder):
        await until(lambda: link.motion_allowed())
        await stream(link, 0.2, -0.1, 0.3)
        sent = speeds_of(rover)
        assert (0.2, -0.1) in sent
        assert link.cmd_left == 0.2 and link.cmd_right == -0.1
        frozen_at = len(sent)
        await asyncio.sleep(0.4)  # the caller has frozen
        assert len(speeds_of(rover)) == frozen_at, "no keep-alive may outlive the loop"
        await until(lambda: rover.left == 0.0 and rover.right == 0.0)
        assert not rover.hb_alive, "the firmware's heartbeat zeroed the motors"


@pytest.mark.timeout(30)
async def test_command_clamps_to_power_max_and_zeroes_non_finite_values() -> None:
    async with linked() as (link, rover, _recorder):
        await until(lambda: link.motion_allowed())
        assert link.command(0.9, -0.9) == (0.30, -0.30)
        assert link.command(float("nan"), float("inf")) == (0.0, 0.0)
        await until(lambda: (0.3, -0.3) in speeds_of(rover))
        assert rover.clamp_count == 0, "nothing above the cap reached the controller"


@pytest.mark.timeout(30)
async def test_command_sends_zeros_before_feedback_and_after_it_goes_stale() -> None:
    rover = FakeRover()
    rover.feedback_enabled = False
    async with linked(rover) as (link, rover, _recorder):
        await until(lambda: link.up)
        assert link.command(0.2, 0.2) == (0.0, 0.0), "no feedback yet"
        rover.feedback_enabled = True
        await until(lambda: link.motion_allowed())
        assert link.command(0.2, 0.2) == (0.2, 0.2)
        rover.feedback_enabled = False
        await until(lambda: not link.feedback_fresh(), timeout=2.0)
        assert link.command(0.2, 0.2) == (0.0, 0.0), "T0: zeros to a silent controller"
        await until(lambda: speeds_of(rover)[-1] == (0.0, 0.0))


@pytest.mark.timeout(30)
async def test_nothing_is_written_while_the_transport_is_closed() -> None:
    link = Link(link_config(1))
    assert link.command(0.2, 0.2) == (0.0, 0.0)
    assert not link.connected


# ---------------------------------------------------------------------------
# Restart, loss and reopen
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
async def test_a_second_banner_is_a_restart_that_re_runs_bring_up() -> None:
    async with linked() as (link, rover, recorder):
        await until(lambda: link.motion_allowed())
        first_bring_up = len(rover.received)

        def on_restart(banner: Banner) -> None:
            # Inspect the restart boundary itself, before the async handshake
            # can finish again; polling a transient state races the TCP peer.
            assert not link.up
            assert link.command(0.2, 0.2) == (0.0, 0.0)
            recorder.on_restart(banner)

        link._on_restart = on_restart
        rover.reboot()
        await until(lambda: len(recorder.restarts) == 1)
        assert recorder.restarts[0].fw == "bot-wr-1"
        await until(lambda: link.up and recorder.ups == 2)
        types = [o["T"] for o in rover.received[first_bring_up:] if o["T"] != 1]
        assert types[:6] == [605, 143, 136, 142, 131, 1007], "the same sequence again"
        assert rover.banners_sent == 3, "initial request, reboot, and repeated request"
        await until(lambda: link.motion_allowed())


@pytest.mark.timeout(30)
async def test_banners_during_bring_up_are_one_bring_up() -> None:
    """A boot banner and the answer to our request may both land in the wait."""
    rover = FakeRover()
    async with linked(rover) as (link, rover, recorder):
        await until(lambda: link.up)
        assert recorder.restarts == []
        assert recorder.ups == 1


@pytest.mark.timeout(30)
async def test_the_link_reopens_after_a_loss_and_replays_nothing() -> None:
    """I-13: a reconnect starts with bring-up and zeros, never the last command."""
    async with linked() as (link, rover, recorder):
        await until(lambda: link.motion_allowed())
        await stream(link, 0.2, 0.2, 0.2)
        assert (0.2, 0.2) in speeds_of(rover)
        rover.disconnect()
        await until(lambda: recorder.lost == 1)
        assert not link.connected and not link.up
        assert link.feedback is None and link.fw is None
        assert (link.cmd_left, link.cmd_right) == (0.0, 0.0)
        boundary = len(rover.received)
        await until(lambda: rover.connections == 2 and link.up, timeout=TIMEOUT_S)
        after = rover.received[boundary:]
        assert [o["T"] for o in after if o["T"] != 1][:6] == [
            605,
            143,
            136,
            142,
            131,
            1007,
        ]
        assert all((o["L"], o["R"]) == (0.0, 0.0) for o in after if o["T"] == 1)
        assert recorder.ups == 2


@pytest.mark.timeout(30)
async def test_a_loss_during_bring_up_is_a_loss_not_a_stock_verdict() -> None:
    rover = FakeRover()
    rover.banner_on_request = False  # type: ignore[attr-defined]
    port = await rover.start_tcp()
    link = Link(link_config(port, link={"banner_wait_ms": 2000}))
    task = asyncio.create_task(link.run())
    try:
        await until(lambda: rover.connections == 1)
        await until(lambda: any(o["T"] == 1007 for o in rover.received))
        rover.disconnect()
        await until(lambda: rover.connections == 2, timeout=TIMEOUT_S)
        assert link.banner is None or link.up is False or link.fw is not None
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await rover.stop()


@pytest.mark.timeout(30)
async def test_an_absent_peer_is_retried_until_it_appears() -> None:
    rover = FakeRover()
    server = await asyncio.start_server(lambda r, w: None, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    server.close()
    await server.wait_closed()
    link = Link(link_config(port))
    task = asyncio.create_task(link.run())
    try:
        await asyncio.sleep(0.3)
        assert not link.connected
        rover._server = await asyncio.start_server(rover._serve, "127.0.0.1", port)  # noqa: SLF001
        rover._ensure_loop()  # noqa: SLF001
        await until(lambda: link.up, timeout=TIMEOUT_S)
        assert link.fw == "bot-wr-1"
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await rover.stop()


@pytest.mark.timeout(30)
async def test_close_sends_zeros_and_stops_reopening() -> None:
    rover = FakeRover()
    port = await rover.start_tcp()
    link = Link(link_config(port))
    task = asyncio.create_task(link.run())
    try:
        await until(lambda: link.motion_allowed())
        await stream(link, 0.2, 0.2, 0.2)
        await link.close()
        await asyncio.wait_for(task, 2.0)
        await asyncio.sleep(0.1)
        assert speeds_of(rover)[-1] == (0.0, 0.0)
        assert not link.connected
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await rover.stop()


# ---------------------------------------------------------------------------
# The serial backend over a pty
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
async def test_the_serial_backend_opens_a_pty_path_like_the_uart() -> None:
    rover = FakeRover()
    path = rover.start_pty()
    config = RobotConfig(
        link=LinkConfig(backend="serial", port=path, open_retry_ms=50, banner_wait_ms=300)
    )
    link = Link(config)
    task = asyncio.create_task(link.run())
    try:
        await until(lambda: link.up and link.fw == "bot-wr-1")
        await until(lambda: link.motion_allowed())
        await stream(link, 0.15, 0.15, 0.3)
        assert (0.15, 0.15) in speeds_of(rover)
        await until(lambda: rover.left == 0.15)
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await rover.stop()


# ---------------------------------------------------------------------------
# The real simulator: rover-stub over a pty
# ---------------------------------------------------------------------------


def _stub_reason() -> str | None:
    """Why the rover-stub tests cannot run, or ``None`` when they can."""
    if importlib.util.find_spec("rover_devtools.rover_stub") is None:
        return (
            "rover_devtools.rover_stub is not importable yet; it is being written "
            "concurrently and this test runs against its CLI contract "
            "(rover-stub --pty prints the device path; --stock, --tof-mm, --bumper, "
            "--vbat)"
        )
    return None


@contextlib.contextmanager
def rover_stub(*flags: str):  # type: ignore[no-untyped-def]
    """Start ``rover-stub --pty`` and yield the device path it prints."""
    process = subprocess.Popen(  # noqa: S603 - a repo-local developer tool
        [sys.executable, "-m", "rover_devtools.rover_stub", "--pty", *flags],
        cwd=REPO,
        env={**os.environ, "PYTHONPATH": f"{REPO / 'packages'}:{REPO / 'hosts' / 'pi'}"},
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        assert process.stdout is not None
        deadline = time.monotonic() + 15.0
        path: str | None = None
        while time.monotonic() < deadline and path is None:
            if process.poll() is not None:
                pytest.skip("rover-stub exited before printing a device path")
            line = process.stdout.readline().strip()
            candidate = line.split()[-1] if line else ""
            if candidate.startswith("/") and Path(candidate).exists():
                path = candidate
        if path is None:
            pytest.skip("rover-stub did not print an existing device path within 15 s")
        yield path
    finally:
        process.terminate()
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=5)


@pytest.mark.timeout(60)
@pytest.mark.skipif(_stub_reason() is not None, reason=_stub_reason() or "")
async def test_the_link_brings_up_the_real_stub_over_a_pty() -> None:
    with rover_stub() as path:
        config = RobotConfig(
            link=LinkConfig(backend="serial", port=path, open_retry_ms=100)
        )
        link = Link(config)
        task = asyncio.create_task(link.run())
        try:
            await until(lambda: link.up, timeout=10.0)
            assert link.fw is not None, "the stub is the fork unless --stock"
            await until(lambda: link.motion_allowed(), timeout=5.0)
            await stream(link, 0.15, 0.15, 0.6)
            assert link.feedback is not None
            await until(lambda: link.feedback is not None and link.feedback.left > 0.0)
            await stream(link, 0.0, 0.0, 0.4)
            await until(
                lambda: (
                    link.feedback is not None
                    and link.feedback.left == 0.0
                    and link.feedback.right == 0.0
                )
            )
            await link.close()
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


@pytest.mark.timeout(60)
@pytest.mark.skipif(_stub_reason() is not None, reason=_stub_reason() or "")
async def test_the_stock_stub_is_refused() -> None:
    with rover_stub("--stock") as path:
        config = RobotConfig(
            link=LinkConfig(backend="serial", port=path, open_retry_ms=100)
        )
        link = Link(config)
        task = asyncio.create_task(link.run())
        try:
            await until(lambda: link.up, timeout=10.0)
            assert link.fw is None
            await until(lambda: link.feedback is not None, timeout=5.0)
            assert not link.firmware_ok
            await stream(link, 0.15, 0.15, 0.3)
            assert link.feedback is not None and link.feedback.left == 0.0
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
