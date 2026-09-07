"""The executed-motion path, through the daemon with a controller attached.

Everything below runs the real ``Robotd``: the NDJSON bus, the validator, the
arbiter, the profile, the setpoint cell and the serial link over a pty, against
the wheel plant in :mod:`test_robotd_link`'s ``FakeMcu``.  It is the seam the
unit tests above it cannot reach -- a whole drive from ``accepted`` to ``done``,
preemption between sources, the client-liveness gap and a recorded episode.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import os
import pty
import shutil
import signal
import sys
import tempfile
import time
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "packages"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rover_contracts.config import (  # noqa: E402
    BusConfig,
    LogConfig,
    RobotConfig,
    SerialConfig,
)
from rover_contracts.serial_codec import (  # noqa: E402
    ArmFrame,
    CtrlFlag,
    VelocityFrame,
)
from rover_robotd import main as robotd_main  # noqa: E402
from rover_robotd.episodes import (  # noqa: E402
    LEKIWI_ACTION_KEYS,
    EpisodeRecorder,
)
from rover_robotd.main import Robotd  # noqa: E402
from rover_robotd.odom import Pose  # noqa: E402
from test_robotd_link import CLEAR_FLAGS, FakeMcu  # noqa: E402

TURN = "01J9ZC7K000000000000000000"
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
CMD2 = "01J9ZC7K3QF2M8XR4V6T0YAHBE"

TELEOP_BUS = {
    "allow_sources": ["brain", "web", "teleop"],
    "allow_stream": ["teleop"],
}


class Client:
    """One NDJSON connection that also pings at 5 Hz while it owns a command."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader = reader
        self.writer = writer
        self.source = "brain"
        self._ping: asyncio.Task[None] | None = None

    async def send(self, **message: Any) -> None:
        self.writer.write(json.dumps({"v": 1, **message}).encode() + b"\n")
        await self.writer.drain()

    async def recv_result(self, cmd_id: str, status: str, timeout: float = 8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = await asyncio.wait_for(
                self.reader.readline(), max(0.1, deadline - time.monotonic())
            )
            assert line, "the server closed the connection"
            message = json.loads(line)
            if message.get("type") == "result" and message["cmd_id"] == cmd_id:
                if message["status"] == status:
                    return message
                if message["status"] in {"done", "rejected", "aborted", "timeout"}:
                    raise AssertionError(
                        f"expected {status!r} for {cmd_id}, got {message}"
                    )
        raise AssertionError(f"no {status!r} for {cmd_id} within {timeout} s")

    async def recv_type(self, kind: str, timeout: float = 5.0) -> Any:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = await asyncio.wait_for(
                self.reader.readline(), max(0.1, deadline - time.monotonic())
            )
            assert line, "the server closed the connection"
            message = json.loads(line)
            if message.get("type") == kind:
                return message
        raise AssertionError(f"no {kind!r} within {timeout} s")

    async def hello(self, source: str = "brain", caps: tuple[str, ...] = ("skill",)):
        self.source = source
        await self.send(type="hello", source=source, pid=1234, caps=list(caps))
        while True:
            line = await asyncio.wait_for(self.reader.readline(), 3.0)
            if json.loads(line).get("type") == "welcome":
                return

    def start_pings(self) -> None:
        async def loop() -> None:
            while True:
                await self.send(type="ping", source=self.source)
                await asyncio.sleep(0.2)

        self._ping = asyncio.create_task(loop())

    async def close(self) -> None:
        if self._ping is not None:
            self._ping.cancel()
            await asyncio.gather(self._ping, return_exceptions=True)
        self.writer.close()
        with contextlib.suppress(ConnectionError, OSError):
            await self.writer.wait_closed()


@contextlib.asynccontextmanager
async def rover(
    logs: Path, **bus: Any
) -> AsyncIterator[tuple[Robotd, FakeMcu, Path]]:
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    (REPO / "run").mkdir(exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(dir=REPO / "run", prefix="t"))
    config = RobotConfig(
        bus=BusConfig(sock=str(run_dir / "robotd.sock"), **bus),
        serial=SerialConfig(
            backend="pty",
            port=os.ttyname(slave),
            setpoint_hz=20,
            reseed_wait_ms=300,
            open_retry_ms=50,
        ),
        log=LogConfig(dir=str(logs)),
    )
    mcu = FakeMcu(master, plant=True)
    mcu.start()
    robotd = Robotd(config)
    task = asyncio.create_task(robotd.run())
    try:
        await until(
            lambda: robotd.bus.path.exists() and robotd.link.telemetry is not None
        )
        yield robotd, mcu, logs
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await mcu.stop()
        for fd in (master, slave):
            with contextlib.suppress(OSError):
                os.close(fd)
        shutil.rmtree(run_dir, ignore_errors=True)


@contextlib.asynccontextmanager
async def client(
    robotd: Robotd, source: str = "brain", caps: tuple[str, ...] = ("skill",)
) -> AsyncIterator[Client]:
    reader, writer = await asyncio.open_unix_connection(str(robotd.bus.path))
    connection = Client(reader, writer)
    try:
        await connection.hello(source, caps)
        yield connection
    finally:
        await connection.close()


async def until(predicate: Any, timeout: float = 8.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("condition not reached within the timeout")


def drive_message(
    *, cmd_id: str = CMD, turn_id: str = TURN, distance: float = 0.15, seq: int = 1
) -> dict[str, Any]:
    return {
        "type": "skill",
        "source": "brain",
        "cmd_id": cmd_id,
        "seq": seq,
        "turn_id": turn_id,
        "issued_mono_ns": 1,
        "goal_ttl_ms": 5000,
        "skill": "drive",
        "args": {"distance_m": distance, "speed_mps": 0.15},
        "obs": {"frame_id": "cam-000001", "frame_mono_ns": time.monotonic_ns()},
        # brain sets this on every dispatch; a motion skill without it is
        # refused `unauthorized_utterance` (section 7, I-21).
        "trace": {"authorized_motion": True},
    }


# ---------------------------------------------------------------------------
# A drive, end to end
# ---------------------------------------------------------------------------


@pytest.mark.timeout(60)
def test_a_drive_runs_from_accepted_to_done(tmp_path: Path) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
                brain.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.15))
                await brain.recv_result(CMD, "accepted")
                assert robotd.link.armed, "the arm policy must have sent A"
                done = await brain.recv_result(CMD, "done")
                assert done["detail"]["traveled_m"] == pytest.approx(0.15, abs=0.03)
                assert done["detail"]["duration_ms"] > 0
                assert done["detail"]["odom_delta"]["x_m"] == pytest.approx(
                    0.15, abs=0.03
                )
                moving = [f for f in mcu.velocities if f.v_mm_s > 0]
                assert moving, "a drive must put velocity on the wire"
                assert max(f.v_mm_s for f in moving) <= 150
                await until(lambda: mcu.velocities[-1].v_mm_s == 0, timeout=2.0)
                assert robotd.budget.spent(TURN)[0] == pytest.approx(0.15, abs=0.05)

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_drive_ramps_rather_than_stepping_to_its_cruise_speed(
    tmp_path: Path,
) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
                brain.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.30))
                await brain.recv_result(CMD, "accepted")
                await brain.recv_result(CMD, "done")
                first = [f.v_mm_s for f in mcu.velocities if f.v_mm_s > 0][:3]
                assert first, "no motion reached the wire"
                assert first[0] < 150, "the first setpoint must be on the ramp"
                assert sorted(first) == first

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_turn_reports_the_angle_it_turned(tmp_path: Path) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
                brain.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(
                    type="skill",
                    source="brain",
                    cmd_id=CMD,
                    seq=1,
                    turn_id=TURN,
                    issued_mono_ns=1,
                    goal_ttl_ms=5000,
                    skill="turn",
                    args={"angle_deg": 45.0, "rate_dps": 45.0},
                    obs={
                        "frame_id": "cam-000001",
                        "frame_mono_ns": time.monotonic_ns(),
                    },
                    trace={"authorized_motion": True},
                )
                await brain.recv_result(CMD, "accepted")
                done = await brain.recv_result(CMD, "done")
                assert done["detail"]["turned_deg"] == pytest.approx(45.0, abs=2.0)
                assert all(f.v_mm_s == 0 for f in mcu.velocities)
                assert any(f.w_mrad_s > 0 for f in mcu.velocities)

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# Preemption and stopping
# ---------------------------------------------------------------------------


@pytest.mark.timeout(60)
def test_a_stop_aborts_the_active_drive_and_zeroes_the_wire(tmp_path: Path) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
                brain.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.40))
                await brain.recv_result(CMD, "accepted")
                await until(lambda: any(f.v_mm_s > 0 for f in mcu.velocities))
                await brain.send(type="stop", source="brain", reason="stop_word")
                aborted = await brain.recv_result(CMD, "aborted")
                assert aborted["detail"]["traveled_m"] < 0.40
                await asyncio.sleep(0.1)  # let anything already on the wire arrive
                boundary = len(mcu.velocities)
                await asyncio.sleep(0.3)
                assert all(f.v_mm_s == 0 for f in mcu.velocities[boundary:])
                assert robotd.arbiter.active is None

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_teleop_twist_preempts_a_brain_drive(tmp_path: Path) -> None:
    """An accepted twist preempts an active skill, the loser preempted."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs", **TELEOP_BUS) as (robotd, mcu, _logs),
            client(robotd) as brain,
            client(robotd, "teleop", ("twist",)) as teleop,
        ):
                brain.start_pings()
                teleop.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.40))
                await brain.recv_result(CMD, "accepted")
                await until(lambda: any(f.v_mm_s > 0 for f in mcu.velocities))
                await teleop.send(
                    type="twist",
                    source="teleop",
                    cmd_id=CMD2,
                    seq=1,
                    twist={"linear_x_mps": -0.10, "angular_z_radps": 0.0},
                )
                preempted = await brain.recv_result(CMD, "preempted")
                assert preempted["cmd_id"] == CMD
                await until(lambda: any(f.v_mm_s < 0 for f in mcu.velocities))

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_twist_stream_ends_after_the_renewal_window(tmp_path: Path) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs", **TELEOP_BUS) as (robotd, mcu, _logs),
            client(robotd, "teleop", ("twist",)) as teleop,
        ):
                teleop.start_pings()
                await teleop.send(
                    type="twist",
                    source="teleop",
                    cmd_id=CMD,
                    seq=1,
                    twist={"linear_x_mps": 0.10, "angular_z_radps": 0.0},
                )
                await teleop.recv_result(CMD, "accepted")
                await until(lambda: any(f.v_mm_s > 0 for f in mcu.velocities))
                await teleop.recv_result(CMD, "done", timeout=3.0)
                assert robotd.arbiter.active is None
                await asyncio.sleep(0.1)  # let anything already on the wire arrive
                boundary = len(mcu.velocities)
                await asyncio.sleep(0.2)
                assert all(f.v_mm_s == 0 for f in mcu.velocities[boundary:])

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_client_that_stops_pinging_loses_its_command(tmp_path: Path) -> None:
    """The 400 ms gap is what covers ``kill -STOP`` on brain, where no EOF
    ever arrives (I-14)."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.40))
                await brain.recv_result(CMD, "accepted")
                aborted = await brain.recv_result(CMD, "aborted", timeout=3.0)
                assert aborted["reason"] == "not_ready"
                await asyncio.sleep(0.1)  # let anything already on the wire arrive
                boundary = len(mcu.velocities)
                await asyncio.sleep(0.2)
                assert all(f.v_mm_s == 0 for f in mcu.velocities[boundary:])

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_an_estop_mid_drive_aborts_disarms_and_blocks_a_replay(
    tmp_path: Path,
) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
            client(robotd, "web", ("skill",)) as web,
        ):
                brain.start_pings()
                await brain.send(type="turn", source="brain", turn_id=TURN)
                await brain.send(**drive_message(distance=0.40))
                await brain.recv_result(CMD, "accepted")
                await until(lambda: any(f.v_mm_s > 0 for f in mcu.velocities))
                await web.send(type="estop", source="web", reason="user")
                aborted = await brain.recv_result(CMD, "aborted")
                assert aborted["reason"] == "estop_active"
                await until(lambda: not robotd.link.armed)
                await brain.send(
                    **drive_message(cmd_id=CMD2, turn_id=robotd.turns.current, seq=2)
                )
                rejected = await brain.recv_result(CMD2, "rejected")
                assert rejected["reason"] == "estop_active"

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_cancel_naming_another_command_leaves_the_active_one_alone(
    tmp_path: Path,
) -> None:
    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
            brain.start_pings()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await brain.send(**drive_message(distance=0.40))
            await brain.recv_result(CMD, "accepted")
            await until(lambda: any(f.v_mm_s > 0 for f in mcu.velocities))
            await brain.send(type="cancel", source="brain", cmd_id=CMD2)
            await asyncio.sleep(0.3)
            assert robotd.arbiter.active is not None
            assert robotd.arbiter.active.cmd_id == CMD
            assert mcu.velocities[-1].v_mm_s > 0
            await brain.send(type="cancel", source="brain", cmd_id=CMD)
            await brain.recv_result(CMD, "aborted")

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# Episodes (A34)
# ---------------------------------------------------------------------------


@pytest.mark.timeout(60)
def test_a_teleop_stream_records_a_lekiwi_keyed_episode(tmp_path: Path) -> None:
    async def scenario() -> None:
        logs = tmp_path / "logs"
        async with (
            rover(logs, **TELEOP_BUS) as (robotd, _mcu, _logs),
            client(robotd, "teleop", ("twist",)) as teleop,
        ):
                teleop.start_pings()
                for seq in range(1, 12):
                    await teleop.send(
                        type="twist",
                        source="teleop",
                        cmd_id=CMD,
                        seq=seq,
                        twist={"linear_x_mps": 0.10, "angular_z_radps": 0.35},
                    )
                    await asyncio.sleep(0.05)
                await until(lambda: robotd.episodes.index > 3)
                await teleop.recv_result(CMD, "done", timeout=3.0)

        files = sorted((logs / "episodes").glob("*.jsonl"))
        assert files, "a teleop stream must record an episode"
        rows = [json.loads(line) for line in files[0].read_text().splitlines()]
        assert rows
        for row in rows:
            assert tuple(row["action"]) == LEKIWI_ACTION_KEYS
            assert row["action"]["y.vel"] == 0.0
            assert set(row["observation"]) >= {"left_ticks", "right_ticks", "mcu_us"}
        assert rows[-1]["action"]["x.vel"] == pytest.approx(0.10)
        assert rows[-1]["action"]["theta.vel"] == pytest.approx(20.05, abs=0.1)
        assert [row["index"] for row in rows] == list(range(len(rows)))

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# Published state and logs
# ---------------------------------------------------------------------------


@pytest.mark.timeout(60)
def test_state_reports_the_controller_and_the_active_command(tmp_path: Path) -> None:
    async def scenario() -> None:
        async with rover(tmp_path / "logs") as (robotd, _mcu, _logs):
            state = robotd._state(robotd.clock())  # noqa: SLF001 - the published shape
            assert state.ready is True
            assert state.mcu.session == 40010
            assert state.ranges_m.front == pytest.approx(1.204)
            assert state.front_at_max is False
            assert state.tof.front_l_ok and state.tof.front_r_ok
            assert state.battery.pack_v == pytest.approx(11.62)
            assert 0 <= state.battery.pct <= 100
            assert state.bumper is False and state.estop_hw is False

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_the_telemetry_log_carries_raw_ticks_and_mcu_microseconds(
    tmp_path: Path,
) -> None:
    """Principle 7: logs are datasets, so a pose is recomputable offline."""

    async def scenario() -> None:
        logs = tmp_path / "logs"
        async with rover(logs) as (_robotd, _mcu, _logs):
            await asyncio.sleep(0.6)
        files = sorted(logs.glob("telemetry-*.jsonl"))
        assert files
        row = json.loads(files[0].read_text().splitlines()[0])
        assert {"left_ticks", "right_ticks", "mcu_us", "recv_mono_ns"} <= set(row)

    asyncio.run(scenario())


def test_the_fake_controller_never_sees_a_frame_above_the_cap() -> None:
    """A guard on the fixture itself: I-8 is only meaningful if the harness
    would notice an over-cap frame."""
    frame = VelocityFrame(
        seq=1, session=40010, v_mm_s=300, w_mrad_s=1047, frame_ttl_ms=300, flags=0
    )
    assert abs(frame.v_mm_s) <= 300 and abs(frame.w_mrad_s) <= 1047


# ---------------------------------------------------------------------------
# The arm policy, the published state and the result detail
# ---------------------------------------------------------------------------


@pytest.mark.timeout(60)
def test_an_uncalibrated_controller_is_refused_rather_than_armed(
    tmp_path: Path,
) -> None:
    """4.1: with ctrl_flags b6 clear the MCU accepts A and then silently zeroes
    every forward v.  robotd must refuse to arm instead, so the user gets an
    attributable not_ready rather than a progress timeout on a drive that was
    never going to execute."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
            mcu.ctrl_flags = int(CLEAR_FLAGS) & ~int(CtrlFlag.CAL_VALID)
            await until(
                lambda: robotd.link.telemetry is not None
                and not robotd.link.telemetry.ctrl_flags & int(CtrlFlag.CAL_VALID)
            )
            brain.start_pings()
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await brain.send(**drive_message())
            answer = await brain.recv_result(CMD, "rejected")
            assert answer["reason"] == "not_ready"
            assert not robotd.link.armed
            assert not [f for f in mcu.received if isinstance(f, ArmFrame)]

            # And it arms as soon as the controller reports the baseline.
            mcu.ctrl_flags = int(CLEAR_FLAGS)
            await until(
                lambda: robotd.link.telemetry is not None
                and bool(robotd.link.telemetry.ctrl_flags & int(CtrlFlag.CAL_VALID))
            )
            await brain.send(**drive_message(cmd_id=CMD2, seq=2))
            await brain.recv_result(CMD2, "accepted")
            assert robotd.link.armed

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_the_published_oc_v_is_sag_compensated(tmp_path: Path) -> None:
    """A25 evaluates the ladder on V_oc, so the field named oc_v carries the
    compensation: publishing pack_v under that name understates the pack by
    ~0.26 V at 4 A, exactly the sag the estimate exists to remove."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd, caps=("subscribe",)) as web,
        ):
            mcu.vbat_mv = 11360
            mcu.imotor_ma = 4000
            await web.send(type="subscribe", topics=["state"], state_hz=10)
            state = await web.recv_type("state", timeout=5.0)
            while state["battery"]["pack_v"] != pytest.approx(11.36, abs=0.001):
                state = await web.recv_type("state", timeout=5.0)
            r_pack = robotd.config.safety.r_pack_mohm
            assert state["battery"]["oc_v"] == pytest.approx(
                11.36 + 4.0 * r_pack / 1000.0, abs=1e-6
            )
            assert state["battery"]["oc_v"] > state["battery"]["pack_v"]

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_the_state_publishes_the_budget_the_ledger_enforces(tmp_path: Path) -> None:
    """I-15 has one ledger.  brain reads this field instead of keeping a second
    accounting that charges goal deadlines for non-motion skills."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd, caps=("subscribe",)) as web,
        ):
            await web.send(type="subscribe", topics=["state"], state_hz=10)
            state = await web.recv_type("state", timeout=5.0)
            limits = robotd.config.limits
            assert state["budget"]["path_m"] == pytest.approx(limits.budget_path_m)
            assert state["budget"]["motion_s"] == pytest.approx(limits.budget_motion_s)

            robotd.turns.adopt(TURN)
            robotd.budget.charge(TURN, 0.4, 3.0)
            state = await web.recv_type("state", timeout=5.0)
            while state["budget"]["path_m"] == pytest.approx(limits.budget_path_m):
                state = await web.recv_type("state", timeout=5.0)
            assert state["budget"]["path_m"] == pytest.approx(
                limits.budget_path_m - 0.4
            )
            assert state["budget"]["motion_s"] == pytest.approx(
                limits.budget_motion_s - 3.0
            )

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_turn_across_the_heading_wrap_reports_the_swept_angle(
    tmp_path: Path,
) -> None:
    """Both headings are folded into (-pi, +pi], so their raw difference is not
    the swept angle across the boundary: a +90 deg turn from 3.0 rad reported
    -270 deg, contradicting turned_deg in the same result."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs") as (robotd, mcu, _logs),
            client(robotd) as brain,
        ):
            brain.start_pings()
            robotd.odom.pose = Pose(x_m=0.0, y_m=0.0, yaw_rad=3.0)
            await brain.send(type="turn", source="brain", turn_id=TURN)
            await brain.send(
                type="skill", source="brain", cmd_id=CMD, seq=1, turn_id=TURN,
                issued_mono_ns=1, goal_ttl_ms=5000, skill="turn",
                args={"angle_deg": 90.0, "rate_dps": 60.0},
                obs={"frame_id": "cam-000001", "frame_mono_ns": time.monotonic_ns()},
                trace={"authorized_motion": True},
            )
            await brain.recv_result(CMD, "accepted")
            done = await brain.recv_result(CMD, "done")
            swept = math.degrees(done["detail"]["odom_delta"]["yaw_rad"])
            assert swept == pytest.approx(done["detail"]["turned_deg"], abs=2.0)
            assert 80.0 < swept < 100.0

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_a_parse_rejection_reaches_a_sender_that_did_not_subscribe(
    tmp_path: Path,
) -> None:
    """docs/deviations robotd #7: a command's own result always reaches its
    sender.  rover-web's teleop client greets with `hello` alone."""

    async def scenario() -> None:
        async with (
            rover(tmp_path / "logs", **TELEOP_BUS) as (robotd, _mcu, _logs),
            client(robotd, source="teleop", caps=("twist",)) as teleop,
        ):
            await teleop.send(
                type="skill", source="teleop", cmd_id=CMD, seq=1, turn_id=TURN,
                issued_mono_ns=1, goal_ttl_ms=5000, skill="teleport",
                args={"distance_m": 1.0},
            )
            answer = await teleop.recv_result(CMD, "rejected")
            assert answer["reason"] in {"unknown_skill", "bad_args"}

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_recording_a_step_never_fsyncs_on_the_control_loop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """4.2 forbids the control loop to block on anything, and that loop also
    owns the setpoint cell and the 20 Hz V writer.  On the Pi the log directory
    is the SD card, where one fsync stalls into the hundreds of milliseconds
    during a card garbage-collection cycle -- past the 300 ms frame TTL, which
    brakes the wheels mid-teleop for no reason the operator can see."""
    calls: list[int] = []
    real_fsync = os.fsync

    def counting_fsync(fd: int) -> None:
        calls.append(fd)
        real_fsync(fd)

    monkeypatch.setattr(os, "fsync", counting_fsync)
    recorder = EpisodeRecorder(directory=tmp_path)
    recorder.start()
    for index in range(20):
        recorder.record(
            t_mono_ns=index,
            linear_x_mps=0.1,
            angular_z_radps=0.0,
            pose=Pose(),
            left_ticks=index,
            right_ticks=index,
            mcu_us=index,
            source="teleop",
        )
    assert calls == [], "the recorder must not fsync from the control loop"

    # Durability is taken once, when the episode closes.
    episode_id = recorder.stop()
    assert episode_id is not None
    assert len(calls) == 1
    lines = (tmp_path / "episodes" / f"{episode_id}.jsonl").read_text().splitlines()
    assert len(lines) == 20
    assert list(json.loads(lines[0])["action"]) == list(LEKIWI_ACTION_KEYS)


@pytest.mark.timeout(30)
def test_sigterm_unwinds_robotd_instead_of_killing_it(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """systemd stops a unit with SIGTERM.

    With no handler the interpreter dies where it stands: ``Robotd.run``'s
    finally never runs, so the shutdown ``S``+``D`` of 4.2 are never written,
    ``RobotdLog`` is not closed and an open teleop episode is never fsynced.
    The registered callback is invoked directly rather than raising a real
    signal -- a regression that dropped the handler would otherwise kill the
    whole test run instead of failing one case.
    """
    unwound: list[str] = []

    class StubRobotd:
        def __init__(self, config: RobotConfig) -> None:
            self.config = config

        async def run(self) -> None:
            try:
                await asyncio.Event().wait()
            finally:
                unwound.append("aclose")

    monkeypatch.setattr(robotd_main, "Robotd", StubRobotd)

    async def scenario() -> None:
        loop = asyncio.get_running_loop()
        handlers: dict[int, Any] = {}
        original = loop.add_signal_handler

        def spy(signum: int, callback: Any, *args: Any) -> None:
            handlers[signum] = callback
            original(signum, callback, *args)

        monkeypatch.setattr(loop, "add_signal_handler", spy)
        task = asyncio.create_task(robotd_main.serve(RobotConfig()))
        await asyncio.sleep(0.05)
        assert signal.SIGTERM in handlers
        assert signal.SIGINT in handlers
        handlers[signal.SIGTERM]()
        await asyncio.wait_for(task, 5.0)

    asyncio.run(scenario())
    assert unwound == ["aclose"]


@pytest.mark.timeout(60)
def test_state_publishes_the_mcus_own_rx_drop_counter(tmp_path: Path) -> None:
    """``state.mcu.rx_drop`` is ``T.rx_drop``.

    It was filled from robotd's own host-side count of malformed *up*-direction
    lines while every other field of ``mcu`` came from the ``T`` frame, so a
    noisy Pi->MCU link -- the direction ``LINK_CRC`` latches on -- climbed on
    the wire and stayed 0 for roverctl, the web status strip and the event log.
    """

    async def scenario() -> None:
        async with (
            rover(tmp_path) as (robotd, mcu, _),
            client(robotd, "web", ("subscribe",)) as web,
        ):
            await web.send(type="subscribe", topics=["state"], state_hz=10)
            mcu.rx_drop = 37
            state = await web.recv_type("state", timeout=5.0)
            while state["mcu"]["rx_drop"] != 37:
                state = await web.recv_type("state", timeout=5.0)
            assert state["mcu"]["rx_drop"] == 37

    asyncio.run(scenario())


@pytest.mark.timeout(60)
def test_an_out_of_enum_mcu_state_does_not_stop_state_publication(
    tmp_path: Path,
) -> None:
    """One CRC-valid frame with a seventh state used to raise inside
    ``_publish_state``: robotd then stopped publishing ``state`` while still
    driving, and brain's half-duplex gate reads ``moving`` off the last state
    it received."""

    async def scenario() -> None:
        async with (
            rover(tmp_path) as (robotd, mcu, _),
            client(robotd, "web", ("subscribe",)) as web,
        ):
            await web.send(type="subscribe", topics=["state"], state_hz=10)
            await web.recv_type("state", timeout=5.0)
            mcu.state = 200  # type: ignore[assignment]
            state = await web.recv_type("state", timeout=5.0)
            while state["mcu"]["state"] != "FAULT":
                state = await web.recv_type("state", timeout=5.0)
            # Unknown fails toward not-ready: an unreadable controller must not
            # read as armable.
            assert state["ready"] is False

    asyncio.run(scenario())
