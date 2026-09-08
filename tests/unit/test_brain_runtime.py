"""Instruction identity and provenance across the real brain effect handlers.

Adapters are in-memory fakes, except for one short-lived local socket. No
production services, audio, or hardware start.
"""

import asyncio
import grp
import os
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from rover_brain import fsm as F
from rover_brain import main as brain_main
from rover_brain.bus import BrainBus
from rover_brain.main import BrainApp, _BusTurns
from rover_brain.router import RouterDefaults
from rover_brain.scene import SceneRing
from rover_brain.skills_local import Still
from rover_contracts.config import RobotConfig
from rover_contracts.messages import (
    BrainCancelMessage,
    BrainSkillRequest,
    DriveForArgs,
    DriveForCall,
    ResultMessage,
    ResultReason,
    ResultStatus,
    SayArgs,
    SayCall,
    brain_client_adapter,
    brain_server_adapter,
)

TURN = "01J9ZC7K000000000000000001"
OTHER = "01J9ZC7K000000000000000002"


def drive() -> DriveForCall:
    return DriveForCall(
        speech="", skill="drive_for", args=DriveForArgs(duration_ms=100, power_pct=15)
    )


def frame(number: int = 1, *, age_ms: int = 0) -> Still:
    return Still(
        f"cam-{number:06d}", time.monotonic_ns() - age_ms * 1_000_000, b"jpeg", 640, 480
    )


def app_for(still: Still | None) -> BrainApp:
    app = BrainApp.__new__(BrainApp)
    app.config = RobotConfig()
    app.fsm = F.Fsm(mint_turn_id=lambda: TURN)
    app.frames = SimpleNamespace(latest=lambda: still)
    app.robotd = SimpleNamespace(
        state=None,
        next_seq=lambda: 1,
        stop=AsyncMock(),
        cancel=AsyncMock(),
        run=AsyncMock(
            return_value=ResultMessage(cmd_id=OTHER, status=ResultStatus.DONE, t_utc_ns=0)
        ),
    )
    app.scene = SceneRing()
    app.events = asyncio.Queue()
    app._tasks = {}
    app._active_cmd = None
    app._last_result = None
    app._bus = None
    app._router_defaults = RouterDefaults.from_limits(app.config.limits)
    return app


@pytest.mark.asyncio
async def test_instruction_keeps_image_and_permission_when_frame_and_fsm_change() -> None:
    original = frame()
    app = app_for(original)
    app.fsm.handle(F.Heard(text="move toward the mug"))

    async def plan(**kwargs):
        assert kwargs["image_jpeg"] is original.jpeg
        app.frames = SimpleNamespace(latest=lambda: frame(2))
        return SimpleNamespace(call=drive())

    app.box = SimpleNamespace(plan=plan)
    await app._plan(F.StartPlanning("move toward the mug", TURN, True))
    planned = app.events.get_nowait()
    assert planned.obs == original.obs
    effects = app.fsm.handle(planned)
    instruction = next(effect for effect in effects if isinstance(effect, F.Dispatch))
    app.fsm.authorized_motion = False
    app.fsm.turn_id = OTHER
    await app.dispatch_bus(instruction)
    sent = app.robotd.run.call_args.args[0]
    assert sent.turn_id == TURN
    assert sent.trace.authorized_motion is True
    assert sent.obs == original.obs


@pytest.mark.asyncio
@pytest.mark.parametrize("age_ms", [None, 60_000, -1000])
async def test_a_new_frame_cannot_launder_missing_stale_or_future_planning_provenance(
    age_ms,
) -> None:
    app = app_for(frame(2))
    obs = frame(age_ms=age_ms).obs if age_ms is not None else None
    with pytest.raises(LookupError):
        await app.dispatch_bus(F.Dispatch(drive(), TURN, True, obs))
    app.robotd.run.assert_not_called()


@pytest.mark.asyncio
async def test_find_turn_uses_its_observed_frame_and_original_instruction() -> None:
    app = app_for(frame(3))
    app.fsm.turn_id = OTHER
    app.fsm.authorized_motion = True
    observed = frame(2)
    motion = _BusTurns(app, F.Dispatch(drive(), TURN, False, frame().obs))
    await motion.turn_to(90, observation=observed)
    sent = app.robotd.run.call_args.args[0]
    assert sent.turn_id == TURN
    assert sent.trace.authorized_motion is False
    assert sent.obs == observed.obs


@pytest.mark.asyncio
async def test_dispatch_checks_instruction_permission_not_live_fsm_permission() -> None:
    app = app_for(frame())
    app.fsm.authorized_motion = True
    await app._execute(F.Dispatch(drive(), TURN, False, frame().obs))
    result = app.events.get_nowait()
    assert result.status is ResultStatus.REJECTED
    assert result.reason is ResultReason.UNAUTHORIZED_UTTERANCE
    app.robotd.run.assert_not_called()


@pytest.mark.asyncio
async def test_explicit_say_preserves_stop_as_spoken_data_and_reports_completion() -> (
    None
):
    app = app_for(None)
    app.local = SimpleNamespace(say=AsyncMock())
    app.robotd.run.return_value = ResultMessage(
        cmd_id=OTHER, status=ResultStatus.ACCEPTED, t_utc_ns=0
    )
    app._bus = SimpleNamespace(broadcast=AsyncMock())
    request = BrainSkillRequest(
        request_id=OTHER,
        call=SayCall(
            speech="", skill="say", args=SayArgs(text="please stop at the door")
        ),
    )
    assert brain_client_adapter.validate_json(request.model_dump_json()) == request
    app._on_bus_message(request)
    effects = app.fsm.handle(app.events.get_nowait())
    assert not any(
        isinstance(effect, (F.SendStop, F.StartPlanning)) for effect in effects
    )
    instruction = next(effect for effect in effects if isinstance(effect, F.Dispatch))
    await app._execute(instruction)
    app.local.say.assert_awaited_once_with("please stop at the door")
    effects = app.fsm.handle(app.events.get_nowait())
    report = next(effect for effect in effects if isinstance(effect, F.RequestResult))
    await app._perform(report)
    message = app._bus.broadcast.call_args.args[0]
    assert message.cmd_id == OTHER and message.status is ResultStatus.DONE
    assert brain_server_adapter.validate_json(message.model_dump_json()) == message


@pytest.mark.asyncio
async def test_stop_is_first_even_with_no_image_no_authorization_and_no_model() -> None:
    app = app_for(None)
    effects = app.fsm.handle(F.Heard(text="STOP", confidence=0.0))
    assert isinstance(effects[0], F.SendStop)
    assert not any(
        isinstance(effect, (F.AnnounceTurn, F.StartPlanning, F.PlayAck))
        for effect in effects
    )
    await app._perform(effects[0])
    app.robotd.stop.assert_awaited_once_with("utterance_stop")


@pytest.mark.asyncio
async def test_find_timeout_stops_robotd_before_reporting_timeout() -> None:
    from rover_contracts.messages import FindArgs, FindCall

    app = app_for(frame())
    app.local = SimpleNamespace(find=AsyncMock(side_effect=TimeoutError))
    call = FindCall(speech="", skill="find", args=FindArgs(object="mug", max_sweeps=8))
    await app._execute(F.Dispatch(call, TURN, True))
    app.robotd.stop.assert_awaited_once_with("find_deadline")
    assert app.events.get_nowait().status is ResultStatus.TIMEOUT


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "outcomes", [[False] * 3, [False, False, True, False, False, False]]
)
async def test_box_watch_counts_slow_probes_inside_cadence_and_resets_on_success(
    monkeypatch, outcomes
) -> None:
    app = app_for(None)
    app.config.box.health_probe_s = 0.02
    clock = SimpleNamespace(now=0.0)
    starts = []
    replies = iter(outcomes)

    async def advance(duration):
        clock.now += duration

    async def probe():
        starts.append(clock.now)
        await advance(app.config.box.health_probe_s)  # a full-duration timeout
        return next(replies)

    monkeypatch.setattr(
        brain_main,
        "asyncio",
        SimpleNamespace(
            get_running_loop=lambda: SimpleNamespace(time=lambda: clock.now),
            sleep=advance,
        ),
    )
    app.box = SimpleNamespace(probe=probe)
    await app._watch_box(TURN)
    assert starts == pytest.approx([i * 0.02 for i in range(len(outcomes))])
    assert clock.now == pytest.approx(len(outcomes) * 0.02)
    lost = app.events.get_nowait()
    assert isinstance(lost, F.BoxLost) and lost.turn_id == TURN
    assert app.events.empty()


@pytest.mark.asyncio
@pytest.mark.parametrize("during_probe", [False, True])
async def test_box_watch_cancellation_does_not_report_loss(during_probe) -> None:
    app = app_for(None)
    app.config.box.health_probe_s = 0.02
    started = asyncio.Event()
    cancelled = asyncio.Event()

    async def probe():
        started.set()
        try:
            if during_probe:
                await asyncio.sleep(0.02)
            return False
        except asyncio.CancelledError:
            cancelled.set()
            raise

    app.box = SimpleNamespace(probe=probe)
    task = asyncio.create_task(app._watch_box(TURN))
    await asyncio.wait_for(started.wait(), 1)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert cancelled.is_set() is during_probe
    assert app.events.empty()


@pytest.fixture
def socket_dir():
    # macOS limits AF_UNIX paths to 104 bytes; pytest's default root is longer.
    with TemporaryDirectory(prefix="brain-", dir="/tmp") as directory:
        yield Path(directory)


@pytest.mark.asyncio
@pytest.mark.parametrize("completed", [False, True])
async def test_disconnect_cancels_only_an_unfinished_owned_request(socket_dir, completed):
    incoming = asyncio.Queue()
    bus = BrainBus(
        socket_dir / "brain.sock",
        incoming.put_nowait,
        group=grp.getgrgid(os.getgid()).gr_name,
    )
    await bus.start()
    try:
        reader, writer = await asyncio.open_unix_connection(str(bus.path))
        request = BrainSkillRequest(
            request_id=TURN,
            call=SayCall(
                speech="", skill="say", args=SayArgs(text="stop is spoken data")
            ),
        )
        writer.write(request.model_dump_json().encode() + b"\n")
        await writer.drain()
        assert await asyncio.wait_for(incoming.get(), 1) == request
        if completed:
            await bus.broadcast(
                ResultMessage(cmd_id=TURN, status=ResultStatus.DONE, t_utc_ns=0)
            )
            await asyncio.wait_for(reader.readline(), 1)
        writer.close()
        await writer.wait_closed()
        if completed:
            # Let the server observe EOF and finish its connection callback.
            async with asyncio.timeout(1):
                while bus.client_count:
                    await asyncio.sleep(0)
            assert incoming.empty()
        else:
            cancel = await asyncio.wait_for(incoming.get(), 1)
            assert isinstance(cancel, BrainCancelMessage)
            assert cancel.request_id == TURN
    finally:
        await bus.close()
