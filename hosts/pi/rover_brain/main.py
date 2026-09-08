"""``rover-brain``: the composition root.

It owns nothing conversational that the other modules do not already own.  Its
job is to connect the three sockets, turn the FSM's actions into coroutines,
and put every result back on the FSM's queue:

* ``robotd.sock`` -- ``hello`` as ``brain``, ``subscribe``, then ``turn``,
  ``skill``, ``stop`` and a 5 Hz ``ping`` while it owns a command (4.2).
* ``frames.sock`` -- newest-only, and only a ``still`` may authorize motion.
* ``brain.sock`` -- served by :mod:`rover_brain.bus`, the only way an utterance
  or a PTT press reaches the conversational layer (5.9).

Restart contract (4.3): every pending turn is cancelled, a ``stop`` goes out on
connect, and the machine starts in IDLE.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
import time
from collections.abc import AsyncIterator, Coroutine, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

from rover_contracts.config import LimitsConfig, RobotConfig, SafetyConfig, load_config
from rover_contracts.ids import new_cmd_id
from rover_contracts.jsonl import to_json_line
from rover_contracts.messages import (
    BrainClientMessage,
    BrainServerMessage,
    BrainSkillRequest,
    BusCap,
    CancelMessage,
    Face,
    FaceMessage,
    FrameHeader,
    FsmMessage,
    FsmState,
    HelloMessage,
    PingMessage,
    PttEndMessage,
    PttStartMessage,
    ResultDetail,
    ResultMessage,
    ResultReason,
    ResultStatus,
    SkillCall,
    SkillMessage,
    Source,
    StateMessage,
    StopMessage,
    SubscribeMessage,
    SubscribeTopic,
    TurnMessage,
    TurnToArgs,
    TurnToCall,
    UtteranceMessage,
    WelcomeMessage,
    server_adapter,
)
from rover_contracts.skills import MOTION_SKILLS
from rover_contracts.units import round_half_away, wrap_deg_360
from rover_contracts.wave_proto import StopFlag
from rover_contracts.worldstate import MotionBudget, RecentlySeen, WorldState

from rover_brain import fsm as F
from rover_brain.audio.capture import SAMPLE_RATE, make_capture
from rover_brain.audio.stt import make_stt
from rover_brain.audio.tts import make_tts
from rover_brain.audio.vad import VadEvent, make_vad
from rover_brain.audio.wake import make_wake
from rover_brain.box import BoxClient, BoxError
from rover_brain.box_probe import load_caps
from rover_brain.bus import BrainBus
from rover_brain.filler import FillerPlayer
from rover_brain.prompt import build_world_state, system_sha256
from rover_brain.router import RouterDefaults, route
from rover_brain.scene import SceneRing
from rover_brain.skills_local import LocalSkills, Still
from rover_brain.validate import (
    MOTION_CAPABLE,
    ValidationFailure,
    crosses_bus,
    permit,
    power_clamped_to,
    refusal_reason,
    to_bus_message,
)

log = logging.getLogger("rover.brain")

_PING_HZ = 5.0
"""Fallback only; the running rate comes from ``[bus] client_ping_hz``.  A key
that looks configurable and is not is worse than a constant."""
_RECONNECT_S = 1.0
"""Backoff between robotd bus reconnect attempts, matching rover_web's."""
_ROBOTD_CONNECT_S = 5.0
"""How long startup waits for the first connection before carrying on."""


# --------------------------------------------------------------------------
# robotd bus client
# --------------------------------------------------------------------------


class RobotdClient:
    """brain's side of ``robotd.sock``.  One bound source, reconnecting for ever.

    robotd is ``Restart=always`` and is the process most likely to be
    restarted, and a connection that ends when it does takes every motion skill
    with it: the reader returns, the writer points at a dead transport, and the
    next ``drain()`` raises inside the FSM's execute task.  So the connection is
    a supervised loop, exactly as ``rover_web.LineClient`` is, and the 4.3
    restart contract -- ``hello``, ``subscribe``, then ``stop`` -- is re-sent on
    every reconnect rather than only on the first.
    """

    def __init__(
        self,
        path: str,
        *,
        reconnect_s: float = _RECONNECT_S,
        ping_hz: float = _PING_HZ,
    ) -> None:
        self._path = path
        self._reconnect_s = reconnect_s
        self._ping_hz = ping_hz if ping_hz > 0 else _PING_HZ
        self._writer: asyncio.StreamWriter | None = None
        self._seq = 0
        self._results: dict[str, asyncio.Future[ResultMessage]] = {}
        self._terminal_on_accept: set[str] = set()
        self._connected = asyncio.Event()
        self.state: StateMessage | None = None
        self.welcome: WelcomeMessage | None = None

    @property
    def connected(self) -> bool:
        writer = self._writer
        return writer is not None and not writer.is_closing()

    async def serve(self) -> None:
        """Connect, greet, read, and start over.  Runs for the process's life."""
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(self._path)
            except (OSError, ConnectionError) as exc:
                log.warning("robotd bus %s unreachable (%s)", self._path, exc)
                await asyncio.sleep(self._reconnect_s)
                continue
            self._writer = writer
            log.info("connected to robotd at %s", self._path)
            try:
                await self._greet()
                self._connected.set()
                await self._read(reader)
            except (OSError, ConnectionError) as exc:
                log.warning("robotd bus dropped: %s", exc)
            finally:
                self._connected.clear()
                self._abort_pending("robotd bus lost")
                self.state = None
                self.welcome = None
                await self._disconnect()
                log.info("robotd bus disconnected; retrying")
            await asyncio.sleep(self._reconnect_s)

    async def wait_connected(self, timeout: float) -> bool:
        """Wait for the first connection, so startup can report an absent bus."""
        try:
            await asyncio.wait_for(self._connected.wait(), timeout)
        except TimeoutError:
            return False
        return True

    async def _greet(self) -> None:
        self._seq = 0
        await self._send(
            HelloMessage(
                source=Source.BRAIN,
                pid=os.getpid(),
                caps=[BusCap.SKILL, BusCap.SUBSCRIBE],
            )
        )
        await self._send(
            SubscribeMessage(
                topics=[SubscribeTopic.STATE, SubscribeTopic.RESULT, SubscribeTopic.EVENT]
            )
        )
        # 4.3's restart contract: a stop goes out before anything else can.
        await self.stop("brain_restart")

    async def _disconnect(self) -> None:
        writer, self._writer = self._writer, None
        if writer is None:
            return
        writer.close()
        # CancelledError is deliberately not suppressed: swallowing it here
        # leaves the supervising loop running after task.cancel(), and the
        # shutdown that cancelled it then waits for a task that never ends.
        with contextlib.suppress(OSError, ConnectionError):
            await writer.wait_closed()

    def _abort_pending(self, detail: str) -> None:
        """Fail every in-flight command rather than let ``run()`` hang until the
        FSM's own state timeout with no result ever posted."""
        for cmd_id, future in list(self._results.items()):
            if not future.done():
                log.warning("aborting in-flight %s: %s", cmd_id, detail)
                future.set_result(
                    ResultMessage(
                        cmd_id=cmd_id,
                        status=ResultStatus.ABORTED,
                        reason=ResultReason.NOT_READY,
                        t_utc_ns=time.time_ns(),
                    )
                )

    async def _read(self, reader: asyncio.StreamReader) -> None:
        while line := await reader.readline():
            try:
                message = server_adapter.validate_json(line)
            except ValueError:
                log.warning("robotd sent a line brain could not parse")
                continue
            if isinstance(message, StateMessage):
                self.state = message
            elif isinstance(message, WelcomeMessage):
                self.welcome = message
            elif isinstance(message, ResultMessage):
                future = self._results.get(message.cmd_id)
                terminal = (
                    message.status is not ResultStatus.ACCEPTED
                    or message.cmd_id in self._terminal_on_accept
                )
                if future is not None and not future.done() and terminal:
                    future.set_result(message)

    async def _send(self, message: Any) -> None:
        if self._writer is None:
            raise ConnectionError("brain is not connected to robotd")
        self._writer.write(to_json_line(message).encode("utf-8"))
        await self._writer.drain()

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    async def announce_turn(self, turn_id: str) -> None:
        await self._send(TurnMessage(source=Source.BRAIN, turn_id=turn_id))

    async def stop(self, reason: str) -> None:
        """Stop-class: no seq, no cmd_id, never rejected (I-22)."""
        await self._send(StopMessage(source=Source.BRAIN, reason=reason[:64]))

    async def cancel(self, cmd_id: str | None, reason: str) -> None:
        """Stop-class as well: a cancel is never answered ``rejected``."""
        await self._send(
            CancelMessage(source=Source.BRAIN, cmd_id=cmd_id, reason=reason[:64])
        )

    async def run(self, message: SkillMessage) -> ResultMessage:
        """Dispatch one skill and wait for the answer robotd owns.

        For a motion skill that is the terminal ``done``/``aborted``: robotd is
        the executor and A31 forbids anyone else reporting the completion.  For
        a non-motion skill (ARCHITECTURE 6's executor column puts ``say`` and
        ``describe_scene`` in brain) robotd's whole involvement is the validation
        that ``accepted`` reports, and it sends nothing further -- so waiting for
        a second result would hang every one of them until the 20 s EXECUTING
        timeout.  ``accepted`` is terminal exactly where robotd is not the
        executor.
        """
        future: asyncio.Future[ResultMessage] = asyncio.get_running_loop().create_future()
        self._results[message.cmd_id] = future
        self._terminal_on_accept.discard(message.cmd_id)
        if message.skill not in MOTION_SKILLS:
            self._terminal_on_accept.add(message.cmd_id)
        try:
            try:
                await self._send(message)
            except (ConnectionError, OSError) as exc:
                # A dead socket must produce a result, not an unretrieved
                # exception that leaves the FSM in EXECUTING until it times out.
                log.warning("robotd bus unavailable for %s: %s", message.cmd_id, exc)
                return ResultMessage(
                    cmd_id=message.cmd_id,
                    seq=message.seq,
                    status=ResultStatus.ABORTED,
                    reason=ResultReason.NOT_READY,
                    t_utc_ns=time.time_ns(),
                )
            pinger = asyncio.create_task(self._ping_until(future))
            try:
                return await future
            finally:
                pinger.cancel()
        finally:
            self._results.pop(message.cmd_id, None)
            self._terminal_on_accept.discard(message.cmd_id)

    async def _ping_until(self, future: asyncio.Future[ResultMessage]) -> None:
        """4.2: every client owning an active command pings at 5 Hz, and a
        400 ms gap aborts it -- which is what covers ``kill -STOP`` on brain."""
        while not future.done():
            with contextlib.suppress(ConnectionError, OSError):
                await self._send(PingMessage(source=Source.BRAIN))
            await asyncio.sleep(1.0 / self._ping_hz)

    @property
    def moving(self) -> bool:
        return self.state is not None and self.state.rover.motion

    @property
    def heading_deg(self) -> float:
        """The fused yaw in the 0..360 frame the WorldState and ``turn_to`` use."""
        return 0.0 if self.state is None else wrap_deg_360(self.state.rover.heading_deg)


# --------------------------------------------------------------------------
# frames.sock
# --------------------------------------------------------------------------


class FrameSource:
    """Newest stills, with explicit main-plane capture for vision skills."""

    def __init__(self, path: str, *, max_age_ms: int, main_size: tuple[int, int]) -> None:
        self._path = path
        self._max_age_ns = max_age_ms * 1_000_000
        self._latest: Still | None = None
        self._main: Still | None = None
        self._main_size = main_size
        self._writer: asyncio.StreamWriter | None = None
        self._arrived = asyncio.Event()

    async def serve(self) -> None:
        while True:
            try:
                reader, self._writer = await asyncio.open_unix_connection(self._path)
                await self._read(reader)
            except (OSError, ValueError, asyncio.IncompleteReadError) as exc:
                log.warning("camera connection lost: %s", exc)
            finally:
                self._latest = self._main = None
                if self._writer is not None:
                    self._writer.close()
                    with contextlib.suppress(OSError):
                        await self._writer.wait_closed()
                    self._writer = None
            await asyncio.sleep(0.5)

    async def _read(self, reader: asyncio.StreamReader) -> None:
        while line := await reader.readline():
            # A bad header loses framing: its JPEG payload has unknown length.
            # Reconnect instead of guessing where the next header starts.
            header = FrameHeader.model_validate_json(line)
            if header.bytes > 16 * 1024 * 1024:
                raise ValueError("camera frame exceeds 16 MiB")
            payload = await reader.readexactly(header.bytes)
            if header.kind == "still":
                self._latest = Still(
                    header.frame_id, header.frame_mono_ns, payload, header.w, header.h
                )
                if (header.w, header.h) == self._main_size:
                    self._main = self._latest
                self._arrived.set()

    async def still(self, *, wide: bool = False) -> Still:
        """Use a fresh still; wide requests a new main-plane capture."""
        requested_ns = 0
        if wide:
            if self._writer is None:
                raise LookupError("camera is disconnected")
            requested_ns = time.monotonic_ns()
            self._writer.write(b'{"type":"still","plane":"main"}\n')
            await self._writer.drain()
        with contextlib.suppress(TimeoutError):
            async with asyncio.timeout(1.0):
                while True:
                    self._arrived.clear()
                    frame = self._main if wide else self._latest
                    if frame is not None:
                        age = time.monotonic_ns() - frame.frame_mono_ns
                        if (
                            0 <= age <= self._max_age_ns
                            and frame.frame_mono_ns >= requested_ns
                        ):
                            return frame
                    await self._arrived.wait()
        raise LookupError("no requested still within obs_max_age_ms")

    def latest(self) -> Still | None:
        """The newest fresh still, or ``None``: a planning call without an
        image is better than one with a stale one."""
        return self._fresh()

    def _fresh(self) -> Still | None:
        frame = self._latest
        if frame is None:
            return None
        age = time.monotonic_ns() - frame.frame_mono_ns
        return frame if 0 <= age <= self._max_age_ns else None


# --------------------------------------------------------------------------
# the application
# --------------------------------------------------------------------------


class BrainApp:
    """The FSM plus everything that turns its actions into behaviour."""

    def __init__(self, config: RobotConfig, *, root: Path = Path(".")) -> None:
        self.config = config
        self.events: asyncio.Queue[F.BrainEvent] = asyncio.Queue()
        self.fsm = F.Fsm(stt=config.stt, timeouts=F.Timeouts.from_config(config.box))
        self.robotd = RobotdClient(config.bus.sock, ping_hz=config.bus.client_ping_hz)
        main_size = (config.camera.main[0], config.camera.main[1])
        if config.camera.backend == "fake" and config.camera.fake_still:
            from rover_cam.fake_backend import jpeg_size

            # The fixture backend serves its JPEG verbatim on either plane.
            main_size = jpeg_size(Path(config.camera.fake_still).read_bytes())
        self.frames = FrameSource(
            config.bus.frames_sock,
            max_age_ms=config.safety.obs_max_age_ms,
            main_size=main_size,
        )
        self.scene = SceneRing(Path(config.log.dir) / "scene.jsonl")
        self.tts = make_tts(
            config.tts,
            is_moving=lambda: self.robotd.moving,
            output_device=config.audio.output_device,
        )
        self.filler = FillerPlayer(self.tts)
        self.capture = make_capture(config.audio)
        self.wake = make_wake(config.wake)
        self.vad = make_vad(config.vad, config.audio, sample_rate=SAMPLE_RATE)
        self.stt = make_stt(config.stt, sample_rate=SAMPLE_RATE)
        caps = load_caps(root / "box_caps.json")
        self.box = BoxClient(
            config.box, compat=caps is not None and not caps.supports_oneof
        )
        self.local = LocalSkills(
            tts=self.tts,
            box=self.box,
            stills=self.frames,
            scene=self.scene,
            face=self._publish_face,
            hfov_deg=config.camera.hfov_deg,
            heading_deg=lambda: self.robotd.heading_deg,
        )
        self._router_defaults = RouterDefaults.from_limits(config.limits)
        self._bus: BrainBus | None = None
        self._tasks: dict[str, asyncio.Task[Any]] = {}
        self._timer: asyncio.Task[None] | None = None
        self._active_cmd: str | None = None
        self._idle = asyncio.Event()
        self._idle.set()
        self._last_result: ResultStatus | None = None

    # -- wiring ------------------------------------------------------------

    async def run(self) -> None:
        """Connect everything and pump the event queue until cancelled."""
        self._bus = BrainBus(self.config.bus.brain_sock, self._on_bus_message)
        # 4.6: brain probes the box at boot.  It is also where the OpenAI
        # client's first-call cost is paid -- lazy submodule imports and the TLS
        # context -- because inside T3's watch on the first motion command that
        # stall exceeds robotd's 400 ms client-ping gap and aborts the goal.
        log.info("box probe at boot: %s", await self.box.probe())
        async with asyncio.TaskGroup() as group:
            group.create_task(self.robotd.serve())
            if not await self.robotd.wait_connected(_ROBOTD_CONNECT_S):
                log.warning(
                    "robotd bus not up yet; brain keeps retrying in the background"
                )
            group.create_task(self._bus.serve())
            group.create_task(self.frames.serve())
            group.create_task(self._pump())
            if self.config.audio.input == "wake":
                group.create_task(self._wake_loop())

    async def _pump(self) -> None:
        while True:
            event = await self.events.get()
            before = (self.fsm.state, self.fsm.turn_id)
            for action in self.fsm.handle(event):
                await self._perform(action)
            if before != (self.fsm.state, self.fsm.turn_id):
                self._arm_timer()

    def _on_bus_message(self, message: BrainClientMessage) -> None:
        """Everything that arrives on brain.sock, mapped onto one FSM event."""
        if isinstance(message, BrainSkillRequest):
            frame = self.frames.latest()
            self.post(
                F.RequestSkill(
                    request_id=message.request_id,
                    call=message.call,
                    obs=frame.obs if frame is not None else None,
                )
            )
        elif isinstance(message, UtteranceMessage):
            if message.is_final:
                log.info(
                    "utterance: %s",
                    json.dumps(
                        {
                            "source": str(message.source),
                            "text": message.text,
                            "confidence": message.confidence,
                        }
                    ),
                )
                self.post(F.Heard(text=message.text, confidence=message.confidence))
        elif isinstance(message, PttStartMessage):
            self.post(F.PttStart())
        elif isinstance(message, PttEndMessage):
            self.post(F.PttEnd())
        else:  # cancel
            if message.request_id is not None:
                self.post(F.CancelRequest(request_id=message.request_id))
            else:
                self.post(F.Stop(reason="bus_cancel"))

    def post(self, event: F.BrainEvent) -> None:
        self.events.put_nowait(event)

    async def _wake_loop(self) -> None:
        while True:
            await self._idle.wait()
            await self.capture.start()
            await self.wake.wait(self.capture.frames())
            self._idle.clear()
            self.post(F.Wake())

    # -- timers ------------------------------------------------------------

    def _arm_timer(self) -> None:
        """One timer per state entry.  ARCHITECTURE 7 puts a bound on every
        wait; a timer that fires late names the state it was armed for and the
        FSM drops it."""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        seconds = self.fsm.timeout_s
        if seconds is None:
            return
        state = self.fsm.state
        turn_id = self.fsm.turn_id

        async def fire() -> None:
            await asyncio.sleep(seconds)
            self.post(F.Timeout(state=state, turn_id=turn_id))

        self._timer = asyncio.create_task(fire())

    def _spawn(self, name: str, coro: Coroutine[Any, Any, None]) -> None:
        self._cancel_task(name)
        self._tasks[name] = asyncio.create_task(coro)

    def _cancel_task(self, name: str) -> None:
        task = self._tasks.pop(name, None)
        if task is not None and not task.done():
            task.cancel()

    # -- actions -----------------------------------------------------------

    async def _perform(self, action: F.Action) -> None:
        match action:
            case F.AnnounceTurn(turn_id=turn_id):
                await self.robotd.announce_turn(turn_id)
            case F.StartListening(listening_id=listening_id):
                await self.capture.start()
                self._spawn("listen", self._listen(listening_id))
            case F.StopListening():
                # The recogniser keeps running on what it already heard; the
                # frame stream ends, so it finishes on its own.
                await self.capture.stop()
            case F.CancelListening():
                self._cancel_task("listen")
                await self.capture.stop()
            case F.StartPlanning():
                self._spawn("plan", self._plan(action))
            case F.CancelPlan():
                self._cancel_task("plan")
            case F.PlayAck():
                await self.filler.ack()
            case F.PlayFiller():
                self.filler.start_waiting()
            case F.StopFiller():
                self.filler.stop()
            case F.Speak(text=text, speech_id=speech_id):
                self._spawn("speak", self._speak(text, speech_id))
            case F.CancelSpeech():
                self._cancel_task("speak")
            case F.Dispatch():
                self._spawn("execute", self._execute(action))
            case F.CancelDispatch(reason=reason):
                self._cancel_task("execute")
                await self.robotd.cancel(self._active_cmd, reason)
            case F.SendStop(reason=reason):
                self._cancel_task("execute")
                await self.robotd.stop(reason)
            case F.RequestResult(
                request_id=request_id, status=status, reason=reason, detail=detail
            ):
                await self._broadcast(
                    ResultMessage(
                        cmd_id=request_id,
                        status=status,
                        reason=reason,
                        detail=detail,
                        t_utc_ns=time.time_ns(),
                    )
                )
            case F.Publish(state=state):
                # The wake word may own the microphone only while nothing else
                # does: one reader of the ring at a time.
                if state is FsmState.IDLE:
                    self._idle.set()
                else:
                    self._idle.clear()
                await self._broadcast(FsmMessage(state=state))

    async def _listen(self, listening_id: str) -> None:
        """Feed the detector and the recogniser from one capture."""
        frames = self.capture.frames()

        async def tapped() -> AsyncIterator[bytes]:
            async for frame in frames:
                if self.vad.accept(frame) is VadEvent.SPEECH_END:
                    self.post(F.SpeechEnd(listening_id=listening_id))
                yield frame

        transcript = await self.stt.transcribe(tapped())
        if transcript.text:
            self.post(
                F.Heard(
                    text=transcript.text,
                    confidence=transcript.confidence,
                    listening_id=listening_id,
                )
            )

    async def _plan(self, instruction: F.StartPlanning) -> None:
        """The local router first, then the box.  A dead box still answers
        stop, forward, back, left, right, turn around and look (4.6)."""
        text, turn_id = instruction.text, instruction.turn_id
        world = self._world_state(instruction.authorized_motion)
        frame = self.frames.latest()
        obs = frame.obs if frame is not None else None
        local = route(text, world, self._router_defaults)
        if local is not None:
            self._log_plan(
                local, origin="router", authorized=instruction.authorized_motion
            )
            self.post(F.Planned(turn_id=turn_id, call=local, local=True, obs=obs))
            return
        try:
            plan = await self.box.plan(
                world=world,
                utterance=text,
                image_jpeg=frame.jpeg if frame is not None else None,
            )
        except BoxError as exc:
            log.warning("plan failed: %s (%s)", exc.detail, exc.reason)
            self.post(F.PlanFailed(turn_id=turn_id, reason=exc.reason[:64]))
            return
        self._log_plan(plan.call, origin="box", authorized=instruction.authorized_motion)
        self.post(F.Planned(turn_id=turn_id, call=plan.call, obs=obs))

    def _log_plan(self, call: SkillCall, *, origin: str, authorized: bool) -> None:
        """One line per proposed skill, after ``validate_output`` and before any
        dispatch, so `make sim` shows what the model asked for beside what the
        validator allowed."""
        log.info(
            "plan: %s",
            json.dumps(
                {
                    "origin": origin,
                    "skill": str(call.skill),
                    "args": call.args.model_dump(mode="json"),
                    "speech": call.speech,
                    "authorized_motion": authorized,
                }
            ),
        )

    async def _speak(self, text: str, speech_id: str) -> None:
        log.info("speak: %s", json.dumps({"text": text}))
        await self.tts.speak(text)
        self.post(F.Spoke(speech_id=speech_id))

    async def _execute(self, instruction: F.Dispatch) -> None:
        """Run one call and report exactly one :class:`~rover_brain.fsm.Executed`."""
        call, turn_id = instruction.call, instruction.turn_id
        status, reason = ResultStatus.DONE, ResultReason.NONE
        detail: ResultDetail | None = None
        watch: asyncio.Task[None] | None = None
        try:
            refusal = permit(
                call, authorized=instruction.authorized_motion, limits=self.config.limits
            )
            if refusal is None and call.skill in MOTION_CAPABLE:
                watch = asyncio.create_task(self._watch_box(turn_id))
            if call.skill == "stop":
                await self.robotd.stop("operator_stop")
            elif refusal is not None:
                status, reason = ResultStatus.REJECTED, refusal
            elif crosses_bus(call):
                result = await self.dispatch_bus(instruction)
                status, reason, detail = result.status, result.reason, result.detail
                if status is ResultStatus.ACCEPTED:
                    # Only a non-motion skill can still be `accepted` here --
                    # `run` waits for a terminal result on everything robotd
                    # executes.  ARCHITECTURE 6 puts `say` and `describe_scene`
                    # in brain, so robotd's `accepted` is its permission and
                    # brain owns the completion that follows.
                    status = ResultStatus.DONE
                if status is ResultStatus.DONE:
                    if call.skill == "say":
                        await self.local.say(call.args.text)
                    elif call.skill == "describe_scene":
                        await self.local.describe_scene()
            elif call.skill == "set_face":
                await self.local.set_face(Face(call.args.expr))
            elif call.skill == "find":
                outcome = await self.local.find(
                    call.args.object,
                    call.args.max_sweeps,
                    motion=_BusTurns(self, instruction),
                )
                reason = outcome.reason
                status = ResultStatus.DONE if outcome.found else ResultStatus.ABORTED
                await self.local.say(
                    f"I can see the {call.args.object}."
                    if outcome.found
                    else f"I couldn't find the {call.args.object}."
                )
        except TimeoutError:
            await self.robotd.stop("find_deadline")
            status, reason = ResultStatus.TIMEOUT, ResultReason.NONE
        except BoxError as exc:
            log.warning("execute failed: %s", exc.detail)
            status, reason = ResultStatus.ABORTED, ResultReason.BOX_LOST
        except LookupError:
            status, reason = ResultStatus.REJECTED, ResultReason.OBS_STALE
        except ValidationFailure as exc:
            # A call the configured limits will not carry -- a duration over
            # drive_for_max_s, a deadline over goal_ttl_ms_max -- is refused
            # here with a speakable reason rather than sent for robotd to refuse.
            log.warning("execute refused: %s", exc.detail)
            status, reason = ResultStatus.REJECTED, refusal_reason(exc)
        finally:
            if watch is not None:
                watch.cancel()
        self._last_result = status
        log.info(
            "executed: %s",
            json.dumps(
                {"skill": str(call.skill), "status": str(status), "reason": str(reason)}
            ),
        )
        self.post(
            F.Executed(turn_id=turn_id, status=status, reason=reason, detail=detail)
        )

    async def _watch_box(self, turn_id: str) -> None:
        """T3, and only while a brain-owned motion command is active, so it
        costs nothing at idle.

        Start probes on a monotonic cadence, including their execution time.
        The default three 0.8 s timeouts take about 2.4 s, not three sleeps
        plus three timeouts. Teleop never reaches this process.
        """
        failures = 0
        loop = asyncio.get_running_loop()
        next_probe = loop.time()
        while failures < self.config.box.health_probe_fails:
            await asyncio.sleep(max(0.0, next_probe - loop.time()))
            failures = 0 if await self.box.probe() else failures + 1
            next_probe += self.config.box.health_probe_s
        self.post(F.BoxLost(turn_id=turn_id))

    async def dispatch_bus(self, instruction: F.Dispatch) -> ResultMessage:
        call = instruction.call
        obs = instruction.obs
        if call.skill in MOTION_SKILLS:
            age_ns = time.monotonic_ns() - obs.frame_mono_ns if obs else -1
            if not 0 <= age_ns <= self.config.safety.obs_max_age_ms * 1_000_000:
                raise LookupError(
                    "planning observation is missing, stale, or from the future"
                )
        clamped = power_clamped_to(call, self.config.limits)
        message = to_bus_message(
            call,
            cmd_id=new_cmd_id(),
            turn_id=instruction.turn_id,
            seq=self.robotd.next_seq(),
            issued_mono_ns=time.monotonic_ns(),
            limits=self.config.limits,
            authorized=instruction.authorized_motion,
            obs=obs,
            model=self.config.box.model,
            prompt_sha256=system_sha256(),
        )
        self._active_cmd = message.cmd_id
        log.info(
            "dispatch: %s",
            json.dumps(
                {
                    "cmd_id": message.cmd_id,
                    "turn_id": message.turn_id,
                    "skill": str(message.skill),
                    "args": message.args.model_dump(mode="json"),
                    "goal_ttl_ms": message.goal_ttl_ms,
                    "power_clamped_to": clamped,
                    "obs": message.obs.frame_id if message.obs else None,
                }
            ),
        )
        try:
            result = await self.robotd.run(message)
        finally:
            if self._active_cmd == message.cmd_id:
                self._active_cmd = None
        if (
            clamped is not None
            and result.status is ResultStatus.DONE
            and result.reason is ResultReason.NONE
        ):
            # brain clamped before robotd saw the request, so robotd reports a
            # plain done; the completion still has to say the power was not
            # what the model asked for.
            result = result.model_copy(update={"reason": ResultReason.POWER_CLAMPED})
        log.info(
            "result: %s",
            json.dumps(
                {
                    "cmd_id": result.cmd_id,
                    "status": str(result.status),
                    "reason": str(result.reason),
                    "detail": (
                        result.detail.model_dump(mode="json", exclude_none=True)
                        if result.detail is not None
                        else None
                    ),
                }
            ),
        )
        return result

    async def _publish_face(self, expr: Face) -> None:
        await self._broadcast(FaceMessage(expr=expr))

    async def _broadcast(self, message: BrainServerMessage) -> None:
        if self._bus is not None:
            await self._bus.broadcast(message)

    # -- the world the model sees -----------------------------------------

    def _world_state(self, authorized: bool) -> WorldState:
        return world_from_state(
            self.robotd.state,
            limits=self.config.limits,
            safety=self.config.safety,
            allow_motion=authorized,
            last_result=self._last_result,
            last_scene=self.scene.last_scene,
            recently_seen=self.scene.recently_seen(),
        )


def world_from_state(
    state: StateMessage | None,
    *,
    limits: LimitsConfig,
    safety: SafetyConfig,
    allow_motion: bool,
    last_result: ResultStatus | None = None,
    last_scene: str = "",
    recently_seen: Sequence[RecentlySeen] = (),
) -> WorldState:
    """ARCHITECTURE 5.6's block, from robotd's last ``state`` message.

    ``heading_deg`` is the fused yaw folded into 0..359.  ``front_range_cm`` is
    ``front_m`` in whole centimetres, or ``None`` when robotd published none:
    the model is told that ``null`` is unknown, not clear (I-16).
    ``obstacle_ahead`` is what blocks forward motion at the controller -- the
    TOF or BUMPER stop flag, or a valid range inside ``[safety] tof_stop_mm``.
    The budget is robotd's own ledger (I-15), never a second accounting here;
    before the first state message the configured limit is the honest answer,
    and with no state at all the path ahead is not known to be clear.
    """
    heading, battery, bumper, moving = 0, 0, False, False
    front_cm: int | None = None
    obstacle = True
    seconds = limits.budget_motion_s
    if state is not None:
        heading = round_half_away(wrap_deg_360(state.rover.heading_deg)) % 360
        battery = state.battery.pct
        bumper = state.bumper
        moving = state.rover.motion
        seconds = state.budget.motion_s
        front_m = state.front_m
        if front_m is not None:
            front_cm = min(400, round_half_away(front_m * 100.0))
        blocked = StopFlag(state.rover.stop_flags).blocks_forward
        near = front_m is not None and front_m * 1000.0 < safety.tof_stop_mm
        obstacle = blocked or near
    return build_world_state(
        heading_deg=heading,
        battery_pct=battery,
        obstacle_ahead=obstacle,
        front_range_cm=front_cm,
        bumper=bumper,
        moving=moving,
        power_cap_pct=max(5, min(30, round(limits.power_default * 100))),
        budget=MotionBudget(seconds=int(seconds)),
        allow_motion=allow_motion,
        last_result=last_result,
        last_scene=last_scene,
        recently_seen=recently_seen,
    )


class _BusTurns:
    """``find``'s motion primitive: a ``turn_to`` skill on the current turn id,
    so every sweep after the first is exempt from ``motion_cooldown_ms``."""

    def __init__(self, app: BrainApp, instruction: F.Dispatch) -> None:
        self._app = app
        self._instruction = instruction

    async def turn_to(
        self, heading_deg: int, *, observation: Still
    ) -> tuple[ResultStatus, ResultReason]:
        call = TurnToCall(
            speech="",
            skill="turn_to",
            args=TurnToArgs(heading_deg=int(heading_deg) % 360),
        )
        result = await self._app.dispatch_bus(
            replace(self._instruction, call=call, obs=observation.obs)
        )
        return result.status, result.reason


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rover-brain", description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/robot.toml"))
    parser.add_argument("--log-level", default="info")
    args = parser.parse_args(argv)
    _configure_logging(args.log_level)
    config = load_config(args.config)
    log.info("brain up: %s", json.dumps({"prompt": system_sha256()[:12]}))
    app = BrainApp(config)
    try:
        asyncio.run(app.run())
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":  # pragma: no cover - the systemd entry point
    raise SystemExit(main())
