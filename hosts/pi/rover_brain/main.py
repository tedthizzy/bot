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
from pathlib import Path
from typing import Any, Protocol

from rover_contracts.config import LimitsConfig, RobotConfig, SafetyConfig, load_config
from rover_contracts.ids import new_cmd_id
from rover_contracts.jsonl import to_json_line
from rover_contracts.messages import (
    BrainClientMessage,
    BrainServerMessage,
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
    SkillName,
    SkillObs,
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
from rover_brain.filler import FillerPlayer
from rover_brain.prompt import build_world_state, system_sha256
from rover_brain.router import RouterDefaults, route
from rover_brain.scene import SceneRing
from rover_brain.skills_local import LocalSkills, Still
from rover_brain.validate import (
    MOTION_CAPABLE,
    ValidationFailure,
    crosses_bus,
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


class BrainSocket(Protocol):
    """What :mod:`rover_brain.bus` serves on ``brain.sock`` (5.9).

    It is constructed with the socket path and a callback taking one validated
    ``BrainClientMessage``; ``serve()`` runs until cancelled and
    ``broadcast()`` fans a ``face`` or ``fsm`` message out to every client.
    """

    async def serve(self) -> None: ...

    async def broadcast(self, message: BrainServerMessage) -> None: ...


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
    """Newest-only subscriber to ``frames.sock`` (5.3).

    v1 has no request channel to ``rover-cam`` -- the architecture defines
    none, and cam MUST NOT talk to robotd -- so brain reads the newest ``still``
    the camera has published and refuses to use one older than
    ``[safety] obs_max_age_ms``.  Age is measured on ``frame_mono_ns``, the
    only clock the freshness gate reads (I-23).
    """

    def __init__(self, path: str, *, max_age_ms: int) -> None:
        self._path = path
        self._max_age_ns = max_age_ms * 1_000_000
        self._latest: Still | None = None
        self._arrived = asyncio.Event()

    async def serve(self) -> None:
        while True:
            try:
                reader, _ = await asyncio.open_unix_connection(self._path)
            except OSError:
                await asyncio.sleep(0.5)
                continue
            with contextlib.suppress(OSError, asyncio.IncompleteReadError):
                await self._read(reader)

    async def _read(self, reader: asyncio.StreamReader) -> None:
        while line := await reader.readline():
            # A malformed header costs that frame and nothing else, the way
            # BrainBus._read_lines already treats a malformed line.  Pydantic's
            # ValidationError is a ValueError, not an OSError, so it used to
            # propagate out of serve() -- and serve() is a member of BrainApp's
            # TaskGroup, so one bad line from cam killed wake, VAD, STT, TTS and
            # the whole conversational path, then restart-looped into it.
            try:
                header = FrameHeader.model_validate_json(line)
                payload = await reader.readexactly(header.bytes)
            except ValueError:
                log.warning("dropping a frame with an unreadable header")
                continue
            if header.kind == "still":
                self._latest = Still(
                    header.frame_id, header.frame_mono_ns, payload, header.w, header.h
                )
                self._arrived.set()

    async def still(self, *, wide: bool = False) -> Still:
        """The newest fresh still, waiting briefly for one to arrive."""
        if self._fresh() is None:
            self._arrived.clear()
            with contextlib.suppress(TimeoutError):
                async with asyncio.timeout(1.0):
                    await self._arrived.wait()
        frame = self._fresh()
        if frame is None:
            raise LookupError("no still within obs_max_age_ms")
        return frame

    def latest(self) -> Still | None:
        """The newest fresh still, or ``None``: a planning call without an
        image is better than one with a stale one."""
        return self._fresh()

    def _fresh(self) -> Still | None:
        frame = self._latest
        if frame is None:
            return None
        age = time.monotonic_ns() - frame.frame_mono_ns
        return frame if age <= self._max_age_ns else None


# --------------------------------------------------------------------------
# the application
# --------------------------------------------------------------------------


class BrainApp:
    """The FSM plus everything that turns its actions into behaviour."""

    def __init__(self, config: RobotConfig, *, root: Path = Path(".")) -> None:
        self.config = config
        self.events: asyncio.Queue[F.BrainEvent] = asyncio.Queue()
        self.fsm = F.Fsm(
            stt=config.stt, timeouts=F.Timeouts.from_config(config.box)
        )
        self.robotd = RobotdClient(config.bus.sock, ping_hz=config.bus.client_ping_hz)
        self.frames = FrameSource(
            config.bus.frames_sock, max_age_ms=config.safety.obs_max_age_ms
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
            motion=_BusTurns(self),
            scene=self.scene,
            face=self._publish_face,
            hfov_deg=config.camera.hfov_deg,
            heading_deg=lambda: self.robotd.heading_deg,
        )
        self._router_defaults = RouterDefaults.from_limits(config.limits)
        self._bus: BrainSocket | None = None
        self._tasks: dict[str, asyncio.Task[Any]] = {}
        self._timer: asyncio.Task[None] | None = None
        self._active_cmd: str | None = None
        self._idle = asyncio.Event()
        self._idle.set()
        self._last_result: ResultStatus | None = None

    # -- wiring ------------------------------------------------------------

    async def run(self) -> None:
        """Connect everything and pump the event queue until cancelled."""
        from rover_brain.bus import BrainBus  # serves brain.sock (5.9)

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
            for action in self.fsm.handle(event):
                await self._perform(action)
            self._arm_timer()

    def _on_bus_message(self, message: BrainClientMessage) -> None:
        """Everything that arrives on brain.sock, mapped onto one FSM event."""
        if isinstance(message, UtteranceMessage):
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

        async def fire() -> None:
            await asyncio.sleep(seconds)
            self.post(F.Timeout(state=state))

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
            case F.StartListening():
                await self.capture.start()
                self._spawn("listen", self._listen())
            case F.StopListening():
                # The recogniser keeps running on what it already heard; the
                # frame stream ends, so it finishes on its own.
                await self.capture.stop()
            case F.CancelListening():
                self._cancel_task("listen")
                await self.capture.stop()
            case F.StartPlanning(text=text, turn_id=turn_id):
                self._spawn("plan", self._plan(text, turn_id))
            case F.CancelPlan():
                self._cancel_task("plan")
            case F.PlayAck():
                await self.filler.ack()
            case F.PlayFiller():
                self.filler.start_waiting()
            case F.StopFiller():
                self.filler.stop()
            case F.Speak(text=text):
                self._spawn("speak", self._speak(text))
            case F.CancelSpeech():
                self._cancel_task("speak")
            case F.Dispatch(call=call, turn_id=turn_id):
                self._spawn("execute", self._execute(call, turn_id))
            case F.CancelDispatch(reason=reason):
                self._cancel_task("execute")
                await self.robotd.cancel(self._active_cmd, reason)
            case F.SendStop(reason=reason):
                self._cancel_task("execute")
                await self.robotd.stop(reason)
            case F.Publish(state=state):
                # The wake word may own the microphone only while nothing else
                # does: one reader of the ring at a time.
                if state is FsmState.IDLE:
                    self._idle.set()
                else:
                    self._idle.clear()
                await self._broadcast(FsmMessage(state=state))

    async def _listen(self) -> None:
        """Feed the detector and the recogniser from one capture."""
        frames = self.capture.frames()

        async def tapped() -> AsyncIterator[bytes]:
            async for frame in frames:
                if self.vad.accept(frame) is VadEvent.SPEECH_END:
                    self.post(F.SpeechEnd())
                yield frame

        transcript = await self.stt.transcribe(tapped())
        if transcript.text:
            self.post(F.Heard(text=transcript.text, confidence=transcript.confidence))

    async def _plan(self, text: str, turn_id: str) -> None:
        """The local router first, then the box.  A dead box still answers
        stop, forward, back, left, right, turn around and look (4.6)."""
        world = self._world_state()
        local = route(text, world, self._router_defaults)
        if local is not None:
            self._log_plan(local, origin="router")
            self.post(F.Planned(turn_id=turn_id, call=local, local=True))
            return
        try:
            plan = await self.box.plan(
                world=world,
                utterance=text,
                image_jpeg=self._image(),
            )
        except BoxError as exc:
            log.warning("plan failed: %s (%s)", exc.detail, exc.reason)
            self.post(F.PlanFailed(turn_id=turn_id, reason=exc.reason[:64]))
            return
        self._log_plan(plan.call, origin="box")
        self.post(F.Planned(turn_id=turn_id, call=plan.call))

    def _log_plan(self, call: SkillCall, *, origin: str) -> None:
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
                    "authorized_motion": self.fsm.authorized_motion,
                }
            ),
        )

    async def _speak(self, text: str) -> None:
        log.info("speak: %s", json.dumps({"text": text}))
        await self.tts.speak(text)
        self.post(F.Spoke())

    async def _execute(self, call: SkillCall, turn_id: str) -> None:
        """Run one call and report exactly one :class:`~rover_brain.fsm.Executed`."""
        status, reason = ResultStatus.DONE, ResultReason.NONE
        detail: ResultDetail | None = None
        if call.skill in MOTION_CAPABLE:
            self._spawn("box_watch", self._watch_box())
        try:
            if crosses_bus(call):
                result = await self.dispatch_bus(call, turn_id)
                status, reason, detail = result.status, result.reason, result.detail
                if status is ResultStatus.ACCEPTED:
                    # Only a non-motion skill can still be `accepted` here --
                    # `run` waits for a terminal result on everything robotd
                    # executes.  ARCHITECTURE 6 puts `say` and `describe_scene`
                    # in brain, so robotd's `accepted` is its permission and
                    # brain owns the completion that follows.
                    status = ResultStatus.DONE
                if status is ResultStatus.DONE:
                    if call.skill == SkillName.SAY:
                        await self.local.say(call.args.text)
                    elif call.skill == SkillName.DESCRIBE_SCENE:
                        await self.local.describe_scene()
            elif call.skill == SkillName.SET_FACE:
                await self.local.set_face(Face(call.args.expr))
            elif call.skill == SkillName.FIND:
                outcome = await self.local.find(call.args.object, call.args.max_sweeps)
                reason = outcome.reason
                status = ResultStatus.DONE if outcome.found else ResultStatus.ABORTED
                await self.local.say(
                    f"I can see the {call.args.object}."
                    if outcome.found
                    else f"I couldn't find the {call.args.object}."
                )
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
            self._cancel_task("box_watch")
            self._active_cmd = None
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

    async def _watch_box(self) -> None:
        """T3, and only while a brain-owned motion command is active, so it
        costs nothing at idle.

        Three consecutive failures of a probe every ``health_probe_s`` is
        2.4 s, inside A20's 3 s.  Teleop is exempt by construction: it never
        reaches this process.
        """
        failures = 0
        while failures < self.config.box.health_probe_fails:
            await asyncio.sleep(self.config.box.health_probe_s)
            failures = 0 if await self.box.probe() else failures + 1
        self.post(F.BoxLost())

    async def dispatch_bus(self, call: SkillCall, turn_id: str) -> ResultMessage:
        frame = self.frames.latest()
        clamped = power_clamped_to(call, self.config.limits)
        message = to_bus_message(
            call,
            cmd_id=new_cmd_id(),
            turn_id=turn_id,
            seq=self.robotd.next_seq(),
            issued_mono_ns=time.monotonic_ns(),
            limits=self.config.limits,
            authorized=self.fsm.authorized_motion,
            obs=(
                SkillObs(frame_id=frame.frame_id, frame_mono_ns=frame.frame_mono_ns)
                if frame is not None
                else None
            ),
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
        result = await self.robotd.run(message)
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

    def _image(self) -> bytes | None:
        frame = self.frames.latest()
        return frame.jpeg if frame is not None else None

    def _world_state(self) -> WorldState:
        return world_from_state(
            self.robotd.state,
            limits=self.config.limits,
            safety=self.config.safety,
            allow_motion=self.fsm.authorized_motion,
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

    def __init__(self, app: BrainApp) -> None:
        self._app = app

    async def turn_to(self, heading_deg: int) -> tuple[ResultStatus, ResultReason]:
        turn_id = self._app.fsm.turn_id
        if turn_id is None:
            return ResultStatus.ABORTED, ResultReason.STALE_TURN
        call = TurnToCall(
            speech="",
            skill="turn_to",
            args=TurnToArgs(heading_deg=int(heading_deg) % 360),
        )
        result = await self._app.dispatch_bus(call, turn_id)
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
