"""rover-stub speaks ``docs/protocol.md`` and nothing else.

The protocol tests drive :class:`RoverStub` in-process on a fake clock, so the
300 ms heartbeat is asserted in microseconds and the integrated heading is
compared with its closed form rather than with a stopwatch.  Two tests use
real transports: TCP clients sharing one rover, and the CLI on a pseudo
terminal opened the way robotd opens ``/dev/serial0``.
"""

from __future__ import annotations

import asyncio
import importlib
import importlib.util
import json
import math
import os
import signal
import stat
import subprocess
import sys
import time
import tty
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

REPO = Path(__file__).resolve().parents[2]
PACKAGES = REPO / "packages"
MODULE_FILE = PACKAGES / "rover_devtools" / "rover_stub.py"
sys.path.insert(0, str(PACKAGES))


def _import_stub() -> tuple[Any, bool]:
    """``rover_devtools.rover_stub``, or the same file loaded on its own.

    The stub is pure stdlib by design, but ``rover_devtools/__init__.py``
    imports ``rover_contracts.config``, and that import is broken while the
    repository is restructured (``rover_contracts.serial_codec`` now lives
    under ``legacy/``).  The fallback keeps this file's verdict about the stub
    itself; the flag decides how the pty test spawns the CLI.
    """
    try:
        return importlib.import_module("rover_devtools.rover_stub"), True
    except ImportError:
        spec = importlib.util.spec_from_file_location(
            "rover_stub_standalone", MODULE_FILE
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module, False


rover_stub, PACKAGE_IMPORTS = _import_stub()
STUB_COMMAND = (
    [sys.executable, "-m", "rover_devtools.rover_stub"]
    if PACKAGE_IMPORTS
    else [sys.executable, str(MODULE_FILE)]
)

Params = rover_stub.Params
RoverStub = rover_stub.RoverStub
StubServer = rover_stub.StubServer
LineSplitter = rover_stub.LineSplitter
StopFlag = rover_stub.StopFlag
BANNER: bytes = rover_stub.BANNER
STOCK_FIELDS: tuple[str, ...] = rover_stub.STOCK_FIELDS
FORK_FIELDS: tuple[str, ...] = rover_stub.FORK_FIELDS
FEEDBACK = 1001
IMU = 1002
BANNER_T = 1006

pytestmark = pytest.mark.timeout(60)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def cmd(t: int, **fields: Any) -> bytes:
    return (json.dumps({"T": t, **fields}) + "\n").encode()


def objects(lines: list[bytes]) -> list[dict[str, Any]]:
    """The JSON objects among ``lines``; text lines are left out."""
    found: list[dict[str, Any]] = []
    for line in lines:
        try:
            obj = json.loads(line)
        except ValueError:
            continue
        if isinstance(obj, dict):
            found.append(obj)
    return found


def of_type(lines: list[bytes], t: int) -> list[dict[str, Any]]:
    return [obj for obj in objects(lines) if obj.get("T") == t]


def expected_turn(
    diff: float, seconds: float, tau: float = 0.15, gain: float = 150.0
) -> float:
    """Closed form of the lagged differential's integral from rest."""
    return gain * diff * (seconds - tau * (1.0 - math.exp(-seconds / tau)))


class Bench:
    """A stub on a fake clock.  ``run`` advances in 10 ms ticks; with ``speed``
    the host streams that ``T:1`` at 20 Hz, the way robotd does.  Every line
    the rover sent is kept in ``trace`` with the time it went out."""

    def __init__(self, **params: Any) -> None:
        self.stub = RoverStub(Params(**params))
        self.now = 0.0
        self.trace: list[tuple[float, bytes]] = []
        self.boot: list[bytes] = self._keep(self.stub.start(self.now))

    def _keep(self, lines: list[bytes]) -> list[bytes]:
        self.trace += [(self.now, line) for line in lines]
        return lines

    def send(self, t: int, **fields: Any) -> list[bytes]:
        return self._keep(self.stub.feed(cmd(t, **fields), self.now))

    def raw(self, line: bytes) -> list[bytes]:
        return self._keep(self.stub.feed(line, self.now))

    def run(
        self, seconds: float, speed: tuple[float, float] | None = None
    ) -> list[bytes]:
        produced: list[bytes] = []
        next_command = self.now
        for _ in range(round(seconds / 0.01)):
            if speed is not None and self.now >= next_command - 1e-9:
                produced += self.send(1, L=speed[0], R=speed[1])
                next_command = round(next_command + 0.05, 6)
            self.now = round(self.now + 0.01, 6)
            produced += self._keep(self.stub.tick(self.now))
        return produced

    def feedback(self, lines: list[bytes]) -> list[dict[str, Any]]:
        return of_type(lines, FEEDBACK)

    def applied(self) -> tuple[float, float]:
        return self.stub.applied_l, self.stub.applied_r


# ---------------------------------------------------------------------------
# Banner
# ---------------------------------------------------------------------------


def test_the_banner_is_the_protocol_line_verbatim_at_boot_and_on_request() -> None:
    bench = Bench()
    assert bench.boot == [b'{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}\n']
    assert bench.send(1007) == [BANNER]
    assert bench.boot[0] == BANNER


def test_stock_firmware_has_no_banner_and_ignores_the_request() -> None:
    bench = Bench(stock=True)
    assert b"UGV started.\n" in bench.boot
    assert objects(bench.boot) == []
    assert bench.send(1007) == []
    assert bench.stub.counters.unknown == 1
    assert of_type(bench.run(1.0), BANNER_T) == []


def test_banner_delay_holds_the_banner_back() -> None:
    bench = Bench(banner_delay_ms=200)
    assert bench.boot == []
    assert of_type(bench.run(0.15), BANNER_T) == []
    assert of_type(bench.run(0.10), BANNER_T) == [json.loads(BANNER)]


# ---------------------------------------------------------------------------
# Feedback
# ---------------------------------------------------------------------------


def test_feedback_starts_on_t131_and_carries_every_field() -> None:
    bench = Bench()
    assert bench.feedback(bench.run(0.3)) == []
    bench.send(131, cmd=1)
    lines = bench.feedback(bench.run(1.0))
    assert abs(len(lines) - 20) <= 1
    for obj in lines:
        assert set(obj) == {"T", *STOCK_FIELDS, *FORK_FIELDS}
    first = lines[0]
    assert (first["L"], first["R"], first["r"], first["p"]) == (0, 0, 0, 0)
    assert (first["v"], first["tf"], first["bp"], first["cc"]) == (11.4, -1, 0, 0)
    bench.send(131, cmd=0)
    assert bench.feedback(bench.run(0.5)) == []


def test_stock_feedback_has_only_the_stock_fields() -> None:
    bench = Bench(stock=True)
    bench.send(131, cmd=1)
    lines = bench.feedback(bench.run(0.3))
    assert lines
    for obj in lines:
        assert set(obj) == {"T", *STOCK_FIELDS}


def test_t142_sets_the_feedback_interval() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    bench.send(142, cmd=100)
    assert abs(len(bench.feedback(bench.run(1.0))) - 10) <= 1


def test_t130_and_t126_are_one_shots() -> None:
    bench = Bench()
    assert len(of_type(bench.send(130), FEEDBACK)) == 1
    imu = of_type(bench.send(126), IMU)
    assert len(imu) == 1
    assert set(imu[0]) == {
        "T", "r", "p", "y", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "temp"
    }
    assert objects(bench.run(0.3)) == []


# ---------------------------------------------------------------------------
# The cap
# ---------------------------------------------------------------------------


def test_a_speed_over_the_cap_is_applied_as_the_cap_and_counted() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    bench.send(1, L=0.9, R=-0.9)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert (fb["L"], fb["R"]) == (0.3, -0.3)
    assert fb["cc"] == 2
    bench.send(1, L=0.3, R=0.1)  # at the cap is not over it
    assert bench.stub.clamp_count == 2


def test_stock_firmware_does_not_clamp() -> None:
    bench = Bench(stock=True)
    bench.send(131, cmd=1)
    bench.send(1, L=0.9, R=0.9)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert (fb["L"], fb["R"]) == (0.9, 0.9)
    assert "cc" not in fb


# ---------------------------------------------------------------------------
# The heartbeat
# ---------------------------------------------------------------------------


def test_a_stopped_speed_stream_zeroes_the_motors_within_300ms() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    alive = bench.feedback(bench.run(0.5, speed=(0.2, 0.2)))
    assert alive
    for obj in alive:
        assert (obj["hb"], obj["st"] & 1, obj["L"]) == (1, 0, 0.2)
    last_command = 0.45
    bench.run(0.4)
    expired = [
        (t, obj)
        for t, line in bench.trace
        for obj in of_type([line], FEEDBACK)
        if t > last_command and obj["hb"] == 0
    ]
    assert expired, "hb never dropped"
    t_first, first = expired[0]
    assert last_command + 0.3 <= t_first <= last_command + 0.3 + 0.05 + 0.01 + 1e-9
    assert (first["st"] & 1, first["L"], first["R"]) == (1, 0, 0)
    assert bench.applied() == (0.0, 0.0)
    bench.send(1, L=0.1, R=0.1)  # the next speed command clears both
    fb = bench.feedback(bench.run(0.06))[-1]
    assert (fb["hb"], fb["st"] & 1, fb["L"]) == (1, 0, 0.1)


def test_a_rover_that_never_hears_the_host_reads_hb_0_after_300ms() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    early = bench.feedback(bench.run(0.29))
    assert early and all(obj["hb"] == 1 for obj in early)
    late = bench.feedback(bench.run(0.1))
    assert late[-1]["hb"] == 0 and late[-1]["st"] & 1


def test_stock_firmware_takes_3000ms_to_notice_a_stopped_stream() -> None:
    bench = Bench(stock=True)
    bench.send(143, cmd=0)
    bench.send(131, cmd=1)
    bench.run(0.5, speed=(0.9, 0.9))  # last command at 0.45
    bench.run(2.9)  # 2.95 s after it
    assert bench.applied() == (0.9, 0.9)
    bench.run(0.2)  # 3.15 s after it
    assert bench.applied() == (0.0, 0.0)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert fb["L"] == 0 and "hb" not in fb


def test_t136_only_lowers_the_heartbeat() -> None:
    bench = Bench()
    bench.send(136, cmd=500)
    assert bench.stub.hb_ms == 300
    bench.send(136, cmd=0)
    bench.send(136, cmd="soon")
    assert bench.stub.hb_ms == 300
    bench.send(136, cmd=200)
    assert bench.stub.hb_ms == 200
    bench.send(1, L=0.2, R=0.2)
    bench.run(0.19)
    assert bench.applied() == (0.2, 0.2)
    bench.run(0.02)
    assert bench.applied() == (0.0, 0.0)
    assert bench.stub.stop_flags & StopFlag.HEARTBEAT

    stock = Bench(stock=True)
    stock.send(136, cmd=500)
    assert stock.stub.hb_ms == 500


# ---------------------------------------------------------------------------
# Stop flags
# ---------------------------------------------------------------------------


def test_tof_blocks_forward_but_passes_reverse_and_rotation() -> None:
    bench = Bench(tof_mm=200)
    bench.send(131, cmd=1)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert fb["st"] & 2 and fb["tf"] == 200
    bench.send(1, L=0.2, R=0.2)
    assert bench.applied() == (0.0, 0.0)
    bench.send(1, L=-0.2, R=-0.2)
    assert bench.applied() == (-0.2, -0.2)
    bench.send(1, L=-0.2, R=0.2)
    assert bench.applied() == (-0.2, 0.2)
    bench.send(1, L=0.2, R=0.0)
    assert bench.applied() == (0.2, 0.0)
    bench.stub.params.tof_mm = 300  # outside the 250 mm stop zone
    bench.send(1, L=0.2, R=0.2)
    assert bench.applied() == (0.2, 0.2)
    bench.stub.params.tof_mm = 100  # an obstacle mid-drive stops forward motion
    bench.run(0.01)
    assert bench.applied() == (0.0, 0.0)


def test_bumper_blocks_forward_and_low_battery_refuses_everything() -> None:
    bench = Bench(bumper=True)
    bench.send(131, cmd=1)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert fb["st"] & 4 and fb["bp"] == 1
    bench.send(1, L=0.2, R=0.2)
    assert bench.applied() == (0.0, 0.0)
    bench.send(1, L=-0.2, R=-0.2)
    assert bench.applied() == (-0.2, -0.2)

    low = Bench(vbat=9.5)
    low.send(131, cmd=1)
    fb = low.feedback(low.run(0.05))[-1]
    assert fb["st"] & 8 and fb["v"] == 9.5
    low.send(1, L=-0.2, R=0.2)
    assert low.applied() == (0.0, 0.0)


def test_stock_firmware_never_blocks() -> None:
    bench = Bench(stock=True, tof_mm=200, bumper=True, vbat=9.0)
    bench.send(1, L=0.2, R=0.2)
    assert bench.applied() == (0.2, 0.2)


def test_coast_sets_bit_4_until_the_next_speed_command() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    bench.send(1, L=0.2, R=0.2)
    bench.send(115)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert (fb["st"] & 16, fb["L"], fb["R"]) == (16, 0, 0)
    bench.send(1, L=0.2, R=0.2)
    fb = bench.feedback(bench.run(0.05))[-1]
    assert (fb["st"] & 16, fb["L"]) == (0, 0.2)


# ---------------------------------------------------------------------------
# The plant
# ---------------------------------------------------------------------------


def test_heading_integrates_the_lagged_differential() -> None:
    bench = Bench()
    bench.send(131, cmd=1)
    lines = bench.run(1.0, speed=(-0.2, 0.2))
    expected = expected_turn(0.4, 1.0)
    assert 50 < expected < 52
    assert bench.stub.plant.heading_deg == pytest.approx(expected, abs=0.01)
    assert bench.feedback(lines)[-1]["y"] == pytest.approx(expected, abs=0.01)
    assert bench.stub.plant.x_m == pytest.approx(0.0, abs=1e-9)


def test_yaw_sign_reverses_the_turn() -> None:
    bench = Bench(yaw_sign=-1)
    bench.run(1.0, speed=(-0.2, 0.2))
    expected = -expected_turn(0.4, 1.0)
    assert bench.stub.plant.heading_deg == pytest.approx(expected, abs=0.01)


def test_heading_wraps_and_position_tracks_without_lag() -> None:
    bench = Bench(tau_s=0.0)
    bench.run(2.5, speed=(-0.3, 0.3))  # 90 deg/s for 2.5 s is 225 degrees
    assert bench.stub.plant.heading_deg == pytest.approx(-135.0, abs=0.01)
    straight = Bench(tau_s=0.0)
    straight.run(1.0, speed=(0.3, 0.3))  # 1.2 m/s per unit of power
    assert straight.stub.plant.x_m == pytest.approx(0.36, abs=1e-6)
    assert straight.stub.plant.y_m == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize(
    ("angle", "wrapped"),
    [(0, 0), (180, 180), (-180, 180), (190, -170), (-190, 170), (540, 180), (359, -1)],
)
def test_wrap_deg_lands_in_the_protocol_interval(angle: float, wrapped: float) -> None:
    assert rover_stub.wrap_deg(angle) == pytest.approx(wrapped)


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------


def test_garbage_and_oversize_lines_are_counted_not_raised() -> None:
    bench = Bench()
    bench.raw(b"\xff\xfe not json at all")
    bench.raw(b"x" * 600)
    bench.raw(b"[1, 2, 3]")
    bench.raw(b'{"T": "1"}')
    bench.raw(b'{"T": true}')
    bench.raw(b'{"L": 0.1}')
    bench.raw(b'{"T": 999}')
    bench.raw(b"")
    c = bench.stub.counters
    assert (c.not_json, c.oversize, c.not_object, c.no_type, c.unknown) == (1, 1, 1, 3, 1)
    assert (c.dropped, c.received, c.accepted) == (7, 8, 0)
    assert bench.send(1007) == [BANNER]
    assert bench.raw(b'{"T":1007}\r') == [BANNER]


def test_the_line_splitter_handles_fragments_crlf_and_oversize() -> None:
    splitter = LineSplitter(limit=16)
    assert splitter.feed(b'{"T":10') == []
    assert splitter.feed(b'07}\r\n{"T":130}\n') == [b'{"T":1007}\r', b'{"T":130}']
    assert splitter.feed(b"x" * 40 + b"\n" + b'{"T":1}\n') == [b"x" * 17, b'{"T":1}']


def test_configuration_commands_are_accepted_and_oled_is_cosmetic() -> None:
    bench = Bench()
    assert bench.send(605, cmd=0) == []
    assert bench.stub.info_print is False
    assert bench.send(3, lineNum=0, Text="hello") == []
    assert bench.stub.counters.accepted == 2


# ---------------------------------------------------------------------------
# Fault injection
# ---------------------------------------------------------------------------


def test_latency_delays_a_command() -> None:
    bench = Bench(latency_ms=100)
    bench.send(1, L=0.2, R=0.2)
    bench.run(0.05)
    assert bench.applied() == (0.0, 0.0)
    bench.run(0.06)
    assert bench.applied() == (0.2, 0.2)


def test_drop_every_drops_the_nth_line() -> None:
    bench = Bench(drop_every=3)
    for left in (0.1, 0.2, 0.3):
        bench.send(1, L=left, R=0.0)
    assert bench.applied() == (0.2, 0.0)
    assert bench.stub.counters.injected_drops == 1


def test_garbage_emits_a_non_json_line_every_second() -> None:
    bench = Bench(garbage=True)
    lines = bench.run(2.5)
    text = [line for line in lines if not line.startswith(b"{")]
    assert len(text) == 2
    for line in text:
        with pytest.raises(ValueError):
            json.loads(line)


def test_freeze_stops_feedback_and_keeps_the_motors_running() -> None:
    bench = Bench(freeze_after_s=1.0)
    bench.send(131, cmd=1)
    bench.run(1.5, speed=(0.2, 0.2))
    times = [t for t, line in bench.trace if of_type([line], FEEDBACK)]
    assert times and max(times) < 1.0
    bench.run(1.0)  # a live watchdog would have fired by now
    assert bench.applied() == (0.2, 0.2) and bench.stub.hb_alive
    assert bench.run(0.5) == []


def test_stock_echoes_accepted_commands_until_told_not_to() -> None:
    bench = Bench(stock=True)
    assert bench.send(131, cmd=0) == [b'{"T":131,"cmd":0}\n']
    assert bench.raw(b"junk") == []
    assert bench.send(143, cmd=0) == [b'{"T":143,"cmd":0}\n']  # the last echo
    assert bench.send(131, cmd=0) == []
    fork = Bench()
    assert fork.send(131, cmd=0) == []
    fork.send(143, cmd=1)
    assert fork.send(605, cmd=0) == [b'{"T":605,"cmd":0}\n']


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------


async def read_object(
    reader: asyncio.StreamReader,
    t: int,
    accept: Callable[[dict[str, Any]], bool] = lambda obj: True,
    timeout: float = 3.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        assert remaining > 0, f"no T:{t} line within {timeout} s"
        raw = await asyncio.wait_for(reader.readline(), remaining)
        assert raw, "the stub closed the connection"
        try:
            obj = json.loads(raw)
        except ValueError:
            continue
        if isinstance(obj, dict) and obj.get("T") == t and accept(obj):
            return obj


async def test_tcp_clients_share_one_rover() -> None:
    server = StubServer(RoverStub(Params()), tcp_port=0)
    await server.start()
    assert server.tcp_port
    clients: list[tuple[asyncio.StreamReader, asyncio.StreamWriter]] = []
    try:
        for _ in range(2):
            clients.append(await asyncio.open_connection("127.0.0.1", server.tcp_port))
        (reader_a, writer_a), (reader_b, writer_b) = clients
        writer_a.write(cmd(131, cmd=1))
        await writer_a.drain()
        for reader in (reader_a, reader_b):
            fb = await read_object(reader, FEEDBACK)
            assert set(fb) == {"T", *STOCK_FIELDS, *FORK_FIELDS}
        writer_a.write(cmd(1, L=0.9, R=0.9))
        await writer_a.drain()
        fb = await read_object(reader_b, FEEDBACK, lambda obj: obj["L"] == 0.3)
        assert (fb["cc"], fb["hb"]) == (2, 1)
        writer_b.write(cmd(1007))
        await writer_b.drain()
        assert await read_object(reader_a, BANNER_T) == json.loads(BANNER)
        # One speed command, then silence: 300 ms later the motors are zero.
        fb = await read_object(
            reader_a, FEEDBACK, lambda obj: obj["hb"] == 0, timeout=1.0
        )
        assert (fb["L"], fb["R"], fb["st"] & 1) == (0, 0, 1)
    finally:
        for _reader, writer in clients:
            writer.close()
        await server.stop()


def _read_object_from_fd(fd: int, t: int, timeout: float) -> dict[str, Any]:
    buffer = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            buffer += os.read(fd, 4096)
        except BlockingIOError:
            time.sleep(0.01)
            continue
        *complete, buffer = buffer.split(b"\n")
        for line in complete:
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if isinstance(obj, dict) and obj.get("T") == t:
                return obj
    raise AssertionError(f"no T:{t} line on the pty within {timeout} s")


def test_the_pty_transport_prints_a_usable_device_path_and_answers() -> None:
    process = subprocess.Popen(  # noqa: S603 - a repo-local developer tool
        [*STUB_COMMAND, "--pty"],
        cwd=REPO,
        env={**os.environ, "PYTHONPATH": str(PACKAGES)},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    fd = -1
    try:
        assert process.stdout is not None and process.stderr is not None
        path = process.stdout.readline().decode().strip()
        assert path.startswith("/dev/"), (path, process.stderr.read())
        assert stat.S_ISCHR(os.stat(path).st_mode)
        # Exactly how robotd opens /dev/serial0.
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(fd)
        os.write(fd, cmd(1007))
        assert _read_object_from_fd(fd, BANNER_T, 5.0) == json.loads(BANNER)
        os.write(fd, cmd(131, cmd=1))
        fb = _read_object_from_fd(fd, FEEDBACK, 5.0)
        assert set(fb) == {"T", *STOCK_FIELDS, *FORK_FIELDS}
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=10) == 0
    finally:
        if fd >= 0:
            os.close(fd)
        if process.poll() is None:
            process.kill()
            process.wait()


def test_the_cli_needs_a_transport() -> None:
    with pytest.raises(SystemExit) as caught:
        rover_stub.main([])
    assert caught.value.code == 2
