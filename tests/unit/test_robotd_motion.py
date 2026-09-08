"""Actual Robotd + Unix bus + TCP link; independent board double, no hardware.

The double records raw T=1 commands before clamping. Encoder distance, CRC,
arming and current-based sag compensation belong to the archived S3 system;
their counterparts are timed power, heading, firmware identity and pack voltage.
No assertion here claims physical stopping distance.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import os
import signal
import tempfile
import time
from typing import Any

import pytest
from rover_contracts.config import BusConfig, LogConfig, RobotConfig
from rover_robotd import main as robotd_main
from rover_robotd.episodes import ACTION_KEYS, EpisodeRecorder
from rover_robotd.main import Robotd
from test_robotd_link import FakeRover, link_config, speeds_of, until

pytestmark = pytest.mark.timeout(30)
TURN = "01J9ZC7K000000000000000000"
TURN2 = "01J9ZC7K000000000000000001"
CMD = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
CMD2 = "01J9ZC7K3QF2M8XR4V6T0YAHBE"


class Client:
    def __init__(self, reader, writer, source):
        self.reader, self.writer, self.source = reader, writer, source

    async def send(self, **message: Any):
        if message.get("type") != "subscribe":
            message.setdefault("source", self.source)
        self.writer.write(json.dumps({"v": 1, **message}).encode() + b"\n")
        await self.writer.drain()

    async def receive(self, kind, *, cmd_id=None, status=None):
        async with asyncio.timeout(6):
            while True:
                line = await self.reader.readline()
                assert line, "bus closed before the expected answer"
                message = json.loads(line)
                if message.get("type") != kind or (
                    cmd_id and message.get("cmd_id") != cmd_id
                ):
                    continue
                if status and message["status"] != status:
                    assert message["status"] not in {
                        "done",
                        "rejected",
                        "aborted",
                        "timeout",
                    }, message
                    continue
                return message

    async def result(self, status, cmd_id=CMD):
        return await self.receive("result", cmd_id=cmd_id, status=status)

    async def ping(self):
        while True:
            await self.send(type="ping")
            await asyncio.sleep(0.15)


@contextlib.asynccontextmanager
async def client(robotd, source="brain", *, pings=True):
    reader, writer = await asyncio.open_unix_connection(str(robotd.bus.path))
    connection = Client(reader, writer, source)
    task = None
    try:
        await connection.send(
            type="hello", pid=os.getpid(), caps=["skill", "twist", "subscribe"]
        )
        await connection.receive("welcome")
        if pings:
            task = asyncio.create_task(connection.ping())
        yield connection
    finally:
        if task:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        writer.close()
        with contextlib.suppress(ConnectionError, OSError):
            await writer.wait_closed()


@contextlib.asynccontextmanager
async def rover(logs, *, board=None, teleop=False):
    board = board or FakeRover()
    port = await board.start_tcp()
    with tempfile.TemporaryDirectory(prefix="rover-test-", dir="/tmp") as run_dir:
        bus = BusConfig(
            sock=f"{run_dir}/bus.sock",
            source_uids={},
            allow_sources=["brain", "web", "teleop"] if teleop else ["brain", "web"],
            allow_stream=["teleop"] if teleop else [],
        )
        robotd = Robotd(link_config(port, bus=bus, log=LogConfig(dir=str(logs))))
        task = asyncio.create_task(robotd.run())
        try:
            await until(
                lambda: robotd.bus.path.exists() and robotd.link.feedback is not None
            )
            if not board.stock:
                await until(lambda: robotd.ready)
            yield robotd, board
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await board.stop()


def drive_message(*, cmd_id=CMD, turn_id=TURN, seq=1, duration=0.6, power=0.2):
    now = time.monotonic_ns()
    return dict(
        type="skill",
        cmd_id=cmd_id,
        seq=seq,
        turn_id=turn_id,
        issued_mono_ns=now,
        goal_ttl_ms=5000,
        skill="drive_for",
        args=dict(duration_s=duration, power=power),
        obs=dict(frame_id="cam-000001", frame_mono_ns=now),
        trace=dict(authorized_motion=True),
    )


async def start(brain, board, **args):
    await brain.send(type="turn", turn_id=TURN)
    await brain.send(**drive_message(**args))
    await brain.result("accepted")
    await until(lambda: any(left or right for left, right in speeds_of(board)))


async def assert_quiet(board):
    # Allow one 50 ms period plus transport delivery, then require real zeros.
    await asyncio.sleep(0.1)
    boundary = len(speeds_of(board))
    await asyncio.sleep(0.2)
    frames = speeds_of(board)[boundary:]
    assert frames and all(pair == (0, 0) for pair in frames), frames


async def test_drive_completes_ramps_and_charges_only_streamed_motion(tmp_path):
    async with rover(tmp_path) as (robotd, board), client(robotd) as brain:
        await start(brain, board)
        assert robotd._state(robotd.clock()).active.cmd_id == CMD
        done = await brain.result("done")
        assert 550 <= done["detail"]["duration_ms"] <= 900
        moving = [left for left, right in speeds_of(board) if left > 0 and left == right]
        assert moving and 0 < moving[0] < 0.2
        assert max(moving) == pytest.approx(0.2)
        assert 0.4 <= robotd.budget.spent(TURN) <= 0.7
        await assert_quiet(board)


@pytest.mark.parametrize("initial,target,swept", [(0, 45, 45), (170, 260, 90)])
async def test_turn_reports_swept_heading_including_wrap(
    tmp_path, initial, target, swept
):
    board = FakeRover()
    board.yaw_deg = initial
    async with rover(tmp_path, board=board) as (robotd, board), client(robotd) as brain:
        message = drive_message()
        message.update(
            skill="turn_to", args=dict(heading_deg=float(target), timeout_s=4.0)
        )
        await brain.send(type="turn", turn_id=TURN)
        await brain.send(**message)
        await brain.result("accepted")
        done = await brain.result("done")
        assert done["detail"]["turned_deg"] == pytest.approx(swept, abs=6)
        assert abs(done["detail"]["heading_error_deg"]) <= 5
        assert any(left < 0 < right for left, right in speeds_of(board))
        assert all(left == -right for left, right in speeds_of(board))


def test_local_left_intent_reaches_positive_left_motor_profile():
    from rover_brain.router import route
    from rover_brain.validate import bus_args_for
    from rover_contracts.worldstate import WorldState
    from rover_robotd.profiles import TurnToProfile

    world = WorldState(
        heading_deg=0,
        battery_pct=80,
        obstacle_ahead=False,
        bumper=False,
        moving=False,
        power_cap_pct=20,
        allowed_skills=["turn_to"],
        motion_budget_left={"seconds": 12},
    )
    call = route("turn left", world)
    args = bus_args_for(call, RobotConfig().limits)
    assert args.heading_deg == 90
    limits = RobotConfig().limits
    profile = TurnToProfile(
        args.heading_deg,
        tolerance_deg=limits.turn_tolerance_deg,
        kp=limits.turn_kp,
        power_min=limits.power_min,
        power_max=limits.power_max,
    )
    profile.observe(0, 1)
    left, right = profile.command()
    assert left < 0 < right


@pytest.mark.parametrize(
    "action",
    [
        "stop",
        "estop",
        "cancel",
        "turn",
        "silence",
        "disconnect",
        "reboot",
        "stale_feedback",
    ],
)
async def test_motion_faults_stop_raw_wire_and_never_resume(tmp_path, action):
    async with (
        rover(tmp_path) as (robotd, board),
        client(robotd, pings=action != "silence") as brain,
    ):
        await start(brain, board, duration=2.0)
        if action == "turn":
            await brain.send(type="turn", turn_id=TURN2)
        elif action == "disconnect":
            board.disconnect()
        elif action == "reboot":
            board.reboot()
        elif action == "stale_feedback":
            board.feedback_enabled = False
        elif action != "silence":
            await brain.send(type=action, cmd_id=CMD, reason={"malformed": True})
        answer = await brain.result("preempted" if action == "turn" else "aborted")
        if action == "silence":
            assert answer["reason"] == "not_ready"
        elif action == "stale_feedback":
            assert answer["reason"] == "feedback_stale"
            board.feedback_enabled = True
        if action in {"disconnect", "reboot", "stale_feedback"}:
            await until(lambda: robotd.ready)
        assert robotd.arbiter.active is None
        await assert_quiet(board)
        if action == "estop":
            assert robotd.estop_path.exists()
            restarted = Robotd(robotd.config)
            assert restarted.estop_sw
            restarted.logs.close()
            await brain.send(
                **drive_message(cmd_id=CMD2, seq=2, turn_id=robotd.turns.current)
            )
            assert (await brain.result("rejected", CMD2))["reason"] == "estop_active"


async def test_cancel_other_command_does_not_cancel_active_motion(tmp_path):
    async with rover(tmp_path) as (robotd, board), client(robotd) as brain:
        await start(brain, board, duration=1.0)
        await brain.send(type="cancel", cmd_id=CMD2)
        await asyncio.sleep(0.15)
        assert robotd.arbiter.active.cmd_id == CMD
        assert speeds_of(board)[-1][0] > 0
        await brain.send(type="cancel", cmd_id=CMD)
        await brain.result("aborted")
        await assert_quiet(board)


@pytest.mark.parametrize(
    "fault,reason",
    [
        ("replay", "duplicate_cmd"),
        ("stale_turn", "stale_turn"),
        ("stale_obs", "obs_stale"),
        ("unauthorized", "unauthorized_utterance"),
        ("oversized", "bad_args"),
    ],
)
async def test_rejected_motion_has_positive_control_and_no_wire_output(
    tmp_path, fault, reason
):
    async with rover(tmp_path) as (robotd, board), client(robotd) as brain:
        await start(brain, board)
        await brain.result("done")
        message = drive_message(cmd_id=CMD2, seq=2)
        if fault == "replay":
            message["cmd_id"] = CMD
        elif fault == "stale_turn":
            await brain.send(type="turn", turn_id=TURN2)
        elif fault == "stale_obs":
            message["obs"]["frame_mono_ns"] -= 6_000_000_000
        elif fault == "unauthorized":
            message["trace"]["authorized_motion"] = False
        else:
            message["args"]["power"] = 1.0
        await brain.send(**message)
        rejected = await brain.result("rejected", message["cmd_id"])
        assert rejected["reason"] == reason
        await assert_quiet(board)


async def test_stock_firmware_is_refused_without_nonzero_command(tmp_path):
    async with (
        rover(tmp_path, board=FakeRover(stock=True)) as (robotd, board),
        client(robotd) as brain,
    ):
        await brain.send(type="turn", turn_id=TURN)
        await brain.send(**drive_message())
        assert (await brain.result("rejected"))["reason"] == "unpatched_firmware"
        await assert_quiet(board)


async def test_teleop_preempts_then_expires_and_records_power_episode(tmp_path):
    async with (
        rover(tmp_path, teleop=True) as (robotd, board),
        client(robotd) as brain,
        client(robotd, "teleop") as teleop,
    ):
        await start(brain, board, duration=2.0)
        for seq in range(1, 7):
            await teleop.send(
                type="twist", cmd_id=CMD2, seq=seq, twist={"lin": -0.1, "ang": 0.05}
            )
            await asyncio.sleep(0.05)
        await brain.result("preempted")
        await teleop.result("done", CMD2)
        assert any(left < 0 and right < 0 for left, right in speeds_of(board))
        await assert_quiet(board)
    rows = [
        json.loads(line)
        for line in next((tmp_path / "episodes").glob("*.jsonl")).read_text().splitlines()
    ]
    assert len(rows) >= 3
    assert [row["index"] for row in rows] == list(range(len(rows)))
    for row in rows:
        assert tuple(row["action"]) == ACTION_KEYS
        assert row["action"]["left.power"] == pytest.approx(-0.15)
        assert row["action"]["right.power"] == pytest.approx(-0.05)
        assert {"heading_deg", "left.applied", "right.applied", "bus_v"} <= row[
            "observation"
        ].keys()


async def test_state_and_logs_report_current_feedback_and_shared_budget(tmp_path):
    async with rover(tmp_path) as (robotd, board), client(robotd, "web") as web:
        await web.send(type="subscribe", topics=["state"], state_hz=10)
        board.bus_v, board.tof_mm, board.clamp_count = 11.36, 1204, 37
        robotd.turns.adopt(TURN)
        robotd.budget.charge(TURN, 3)
        async with asyncio.timeout(3):
            while True:
                state = await web.receive("state")
                if state["rover"]["clamp_count"] == 37:
                    break
        assert state["ready"]
        assert state["rover"]["fw"] == "bot-wr-1"
        assert state["front_m"] == pytest.approx(1.204)
        assert state["battery"]["pack_v"] == pytest.approx(11.36)
        assert 0 <= state["battery"]["pct"] <= 100
        assert state["budget"]["motion_s"] == robotd.budget.remaining(TURN)
        await asyncio.sleep(0.25)
    row = json.loads(next(tmp_path.glob("feedback-*.jsonl")).read_text().splitlines()[0])
    assert {
        "left",
        "right",
        "yaw_deg",
        "bus_v",
        "recv_mono_ns",
        "clamp_count",
    } <= row.keys()


async def test_unknown_feedback_does_not_break_state_and_low_battery_fails_closed(
    tmp_path,
):
    async with rover(tmp_path) as (robotd, board), client(robotd, "web") as web:
        await web.send(type="subscribe", topics=["state"], state_hz=10)
        board._send({"T": 987654, "future": True})
        board.lowbat = True
        async with asyncio.timeout(3):
            while True:
                state = await web.receive("state")
                if state["reason"] == "faulted":
                    break
        assert not state["ready"] and state["rover"]["stop_flags"] & 8


async def test_parse_rejection_reaches_unsubscribed_sender_and_connection_survives(
    tmp_path,
):
    async with rover(tmp_path) as (robotd, board), client(robotd) as brain:
        for args in (
            {"power": "fast"},
            {"duration_s": -1, "power": 0.2},
            {"duration_s": 1, "power": 1e300},
        ):
            message = drive_message()
            message["args"] = args
            await brain.send(**message)
            assert (await brain.result("rejected"))["reason"] == "bad_args"
        await start(brain, board)
        await brain.result("done")
        assert all(
            abs(value) <= robotd.config.limits.power_max
            for pair in speeds_of(board)
            for value in pair
        )
        assert board.clamp_count == 0, "board clamping must not hide a host violation"


def test_recording_does_not_fsync_until_episode_closes(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(os, "fsync", calls.append)
    recorder = EpisodeRecorder(directory=tmp_path)
    episode = recorder.start()
    for index in range(20):
        recorder.record(
            t_mono_ns=index,
            left=0.1,
            right=0.1,
            heading_deg=0,
            yaw_rate_dps=None,
            feedback=None,
            source="teleop",
        )
    assert calls == []
    recorder.stop()
    assert len(calls) == 1
    assert (
        len((tmp_path / "episodes" / f"{episode}.jsonl").read_text().splitlines()) == 20
    )


async def test_sigterm_unwinds_robotd(monkeypatch):
    unwound, handlers = [], {}

    class StubRobotd:
        def __init__(self, config):
            pass

        async def run(self):
            try:
                await asyncio.Event().wait()
            finally:
                unwound.append(True)

    monkeypatch.setattr(robotd_main, "Robotd", StubRobotd)
    loop = asyncio.get_running_loop()
    monkeypatch.setattr(
        loop,
        "add_signal_handler",
        lambda signum, callback: handlers.update({signum: callback}),
    )
    task = asyncio.create_task(robotd_main.serve(RobotConfig()))
    await until(lambda: signal.SIGTERM in handlers)
    assert signal.SIGINT in handlers
    handlers[signal.SIGTERM]()
    await asyncio.wait_for(task, 2)
    assert unwound == [True]
