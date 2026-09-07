"""The agent FSM of ARCHITECTURE 7.  Pure logic: events in, actions out.

Nothing here opens a socket, reads a clock or awaits anything.  A timeout is an
event like any other and carries the state it was armed for, so a timer that
fires after the machine has moved on is dropped rather than acted upon; the
runner arms one timer per state entry from :attr:`Fsm.timeout_s`.

The three properties this file exists to hold:

* **Speech and motion do not overlap.**  The intent sentence plays in
  SPEAKING_INTENT and the skill is not dispatched until it has finished (A31).
  That is structural here: :class:`Dispatch` is only ever emitted on a
  :class:`Spoke` event, never beside a :class:`Speak`.
* **A late response cannot start motion** (I-11).  Every event that could carry
  a plan or a result carries the ``turn_id`` it belongs to, and one that is not
  the current turn produces no actions at all.  A stop, a new utterance and a
  box loss each end the current turn immediately, which is what makes the
  in-flight response stale before it lands.
* **Completion speech is never spoken before the executor reports.**  The only
  path to :func:`~rover_brain.filler.completion_sentence` is an
  :class:`Executed` event.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from typing import Annotated, Final, Literal

from pydantic import Field
from rover_contracts.config import BoxConfig, SttConfig
from rover_contracts.ids import ULID_PATTERN, new_turn_id
from rover_contracts.messages import (
    FsmState,
    ResultReason,
    ResultStatus,
    SkillCall,
    SkillName,
    StrictModel,
    UtteranceSource,
)

from rover_brain.filler import APOLOGY, BOX_LOST, completion_sentence, reason_sentence
from rover_brain.validate import MOTION_CAPABLE, authorized_motion

__all__ = [
    "Action",
    "AnnounceTurn",
    "BoxLost",
    "BrainEvent",
    "CancelDispatch",
    "CancelListening",
    "CancelPlan",
    "CancelSpeech",
    "Dispatch",
    "Executed",
    "Fsm",
    "Heard",
    "PlanFailed",
    "Planned",
    "PlayAck",
    "PlayFiller",
    "Publish",
    "PttEnd",
    "PttStart",
    "SendStop",
    "Speak",
    "SpeechEnd",
    "SpeechKind",
    "Spoke",
    "StartListening",
    "Stop",
    "StopFiller",
    "StopListening",
    "StartPlanning",
    "Timeout",
    "Timeouts",
    "Wake",
]

Ulid = Annotated[str, Field(pattern=ULID_PATTERN)]
_Handler = Callable[["BrainEvent"], tuple["Action", ...]]

_BUSY: Final = frozenset(
    {
        FsmState.PLANNING,
        FsmState.SPEAKING_INTENT,
        FsmState.EXECUTING,
        FsmState.SPEAKING_RESULT,
    }
)
"""The states in which a new utterance supersedes what is already running."""


# --------------------------------------------------------------------------
# Events -- one asyncio.Queue of pydantic models (ARCHITECTURE 7)
# --------------------------------------------------------------------------


class Wake(StrictModel):
    """The wake word fired."""

    type: Literal["wake"] = "wake"


class PttStart(StrictModel):
    """The push-to-talk button went down."""

    type: Literal["ptt_start"] = "ptt_start"


class PttEnd(StrictModel):
    """The button came up: end of speech, without a detector."""

    type: Literal["ptt_end"] = "ptt_end"


class SpeechEnd(StrictModel):
    """The VAD found A27's 400 ms of trailing silence."""

    type: Literal["speech_end"] = "speech_end"


class Heard(StrictModel):
    """A final transcript, typed text, or ``roverctl utter`` (A30: one shape).

    ``confidence`` is ``None`` for every source that supplies none, and a
    ``None`` counts as authorized.
    """

    type: Literal["heard"] = "heard"
    text: str = Field(max_length=500)
    confidence: float | None = Field(default=None, ge=0.0, le=1.0)
    source: UtteranceSource = UtteranceSource.STT


class Planned(StrictModel):
    """A validated :class:`SkillCall`, from the box or from the local router."""

    type: Literal["planned"] = "planned"
    turn_id: Ulid
    call: SkillCall
    local: bool = False


class PlanFailed(StrictModel):
    """No usable plan: two schema failures, a timeout, or a dead box."""

    type: Literal["plan_failed"] = "plan_failed"
    turn_id: Ulid
    reason: str = Field(max_length=64)


class Spoke(StrictModel):
    """The sentence that was playing has finished."""

    type: Literal["spoke"] = "spoke"


class Executed(StrictModel):
    """robotd (or a local executor) reported an outcome for this turn."""

    type: Literal["executed"] = "executed"
    turn_id: Ulid
    status: ResultStatus
    reason: ResultReason = ResultReason.NONE


class Stop(StrictModel):
    """The stop word, a cancel on brain.sock, or an e-stop seen on the bus.

    I-22: this is never validated away and never refused, in any state.
    """

    type: Literal["stop"] = "stop"
    reason: str = Field(default="stop", max_length=64)


class BoxLost(StrictModel):
    """T3: three consecutive probe failures, inside A20's 3 s."""

    type: Literal["box_lost"] = "box_lost"


class Timeout(StrictModel):
    """A timer armed on entry to ``state`` expired."""

    type: Literal["timeout"] = "timeout"
    state: FsmState


BrainEvent = Annotated[
    Wake
    | PttStart
    | PttEnd
    | SpeechEnd
    | Heard
    | Planned
    | PlanFailed
    | Spoke
    | Executed
    | Stop
    | BoxLost
    | Timeout,
    Field(discriminator="type"),
]


# --------------------------------------------------------------------------
# Actions
# --------------------------------------------------------------------------


class SpeechKind(StrEnum):
    """Which tier of A31 a sentence belongs to."""

    INTENT = "intent"
    """Tier three: the model's own ``speech``, before dispatch."""
    RESULT = "result"
    """The fixed completion table, after the executor has reported."""


@dataclass(frozen=True, slots=True)
class AnnounceTurn:
    """Send ``turn`` to robotd.  Before any box call, always (I-11)."""

    turn_id: str


@dataclass(frozen=True, slots=True)
class StartListening:
    """Open the microphone."""


@dataclass(frozen=True, slots=True)
class StopListening:
    """Close the microphone.  The recogniser finishes what it already has:
    this is the normal end of an utterance, not an abandonment."""


@dataclass(frozen=True, slots=True)
class CancelListening:
    """Abandon the utterance: close the microphone *and* drop the transcript
    in progress, so a stop cannot come back as a new instruction."""


@dataclass(frozen=True, slots=True)
class StartPlanning:
    """Ask the router, then the box, for a plan for this turn."""

    text: str
    turn_id: str
    authorized_motion: bool


@dataclass(frozen=True, slots=True)
class CancelPlan:
    """Cancel the in-flight box task.  Not the safety property (7); the
    stale ``turn_id`` is."""


@dataclass(frozen=True, slots=True)
class PlayAck:
    """Tier one: the tone."""


@dataclass(frozen=True, slots=True)
class PlayFiller:
    """Tier two: arm the waiting line."""


@dataclass(frozen=True, slots=True)
class StopFiller:
    """Cancel the waiting line the instant the first sentence is ready."""


@dataclass(frozen=True, slots=True)
class Speak:
    """Say one sentence and report :class:`Spoke` when it has finished."""

    text: str
    kind: SpeechKind


@dataclass(frozen=True, slots=True)
class CancelSpeech:
    """Stop the sound now."""


@dataclass(frozen=True, slots=True)
class Dispatch:
    """Execute the call: robotd for a motion skill, brain for the rest."""

    call: SkillCall
    turn_id: str
    authorized_motion: bool


@dataclass(frozen=True, slots=True)
class CancelDispatch:
    """Abandon the active command."""

    reason: str


@dataclass(frozen=True, slots=True)
class SendStop:
    """The stop-class bus message (5.2), which is never rejected."""

    reason: str


@dataclass(frozen=True, slots=True)
class Publish:
    """Broadcast ``{"type":"fsm","state":...}`` on brain.sock."""

    state: FsmState


Action = (
    AnnounceTurn
    | StartListening
    | StopListening
    | CancelListening
    | StartPlanning
    | CancelPlan
    | PlayAck
    | PlayFiller
    | StopFiller
    | Speak
    | CancelSpeech
    | Dispatch
    | CancelDispatch
    | SendStop
    | Publish
)


# --------------------------------------------------------------------------
# Timeouts
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Timeouts:
    """One bound per waiting state -- ARCHITECTURE 7 puts ``asyncio.timeout()``
    on every wait.

    ``planning`` is T3's own total (``[box] timeout_s``).  ``executing`` is a
    brain-side backstop over T2, which robotd owns and reports; it is generous
    on purpose, because a goal that outlives it has already been stopped by
    robotd and the MCU.  ``listening``, ``transcribing`` and ``speaking`` are
    not architecture constants -- they exist so no wait is unbounded.
    """

    listening: float = 15.0
    transcribing: float = 8.0
    planning: float = 8.0
    executing: float = 20.0
    speaking: float = 30.0

    @classmethod
    def from_config(cls, box: BoxConfig) -> Timeouts:
        return cls(planning=box.timeout_s)


# --------------------------------------------------------------------------
# The machine
# --------------------------------------------------------------------------


class Fsm:
    """The seven states, and every transition between them.

    ``handle`` returns the actions the runner must take, in order.  It never
    raises for an event that does not apply to the current state: an
    unexpected event is dropped, because the sources are a microphone, a
    socket and a network, and none of them can be told to stop sending.
    """

    def __init__(
        self,
        *,
        stt: SttConfig | None = None,
        timeouts: Timeouts | None = None,
        mint_turn_id: Callable[[], str] = new_turn_id,
    ) -> None:
        stt = stt if stt is not None else SttConfig()
        self._min_chars = stt.min_chars
        self._min_confidence = stt.min_confidence
        self._timeouts = timeouts if timeouts is not None else Timeouts()
        self._mint = mint_turn_id
        self.state: FsmState = FsmState.IDLE
        self.turn_id: str | None = None
        self.authorized_motion = False
        self.pending: SkillCall | None = None

    # -- the runner's two questions ---------------------------------------

    @property
    def timeout_s(self) -> float | None:
        """How long the current state may last before a :class:`Timeout`."""
        return {
            FsmState.IDLE: None,
            FsmState.LISTENING: self._timeouts.listening,
            FsmState.TRANSCRIBING: self._timeouts.transcribing,
            FsmState.PLANNING: self._timeouts.planning,
            FsmState.SPEAKING_INTENT: self._timeouts.speaking,
            FsmState.EXECUTING: self._timeouts.executing,
            FsmState.SPEAKING_RESULT: self._timeouts.speaking,
        }[self.state]

    def handle(self, event: BrainEvent) -> tuple[Action, ...]:
        """Apply one event.  The state is updated before the actions return."""
        if isinstance(event, Stop):
            return self._on_stop(event.reason)
        if isinstance(event, Heard) and self.state in _BUSY:
            return self._supersede(event)
        if isinstance(event, BoxLost):
            return self._on_box_lost()
        if isinstance(event, Timeout):
            if event.state is not self.state:
                return ()  # a timer for a state we have already left
            return self._on_timeout()
        handler: _Handler = getattr(self, f"_in_{self.state.value.lower()}")
        return handler(event)

    # -- states ------------------------------------------------------------

    def _in_idle(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, (Wake, PttStart)):
            listening = self._enter(FsmState.LISTENING)
            return self._begin_turn() + (StartListening(), listening)
        if isinstance(event, Heard):
            return self._begin_turn() + self._plan(event)
        return ()

    def _in_listening(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, (SpeechEnd, PttEnd)):
            return (StopListening(), self._enter(FsmState.TRANSCRIBING))
        if isinstance(event, Heard):
            # Typed text can arrive while the microphone is open; it wins, and
            # whatever the recogniser had is dropped rather than queued.
            return (CancelListening(),) + self._plan(event)
        return ()

    def _in_transcribing(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, Heard):
            return self._plan(event)
        return ()

    def _in_planning(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, Planned):
            if not self._is_current(event.turn_id):
                return ()  # I-11: the turn moved on while the box was thinking
            refusal = self._refuse(event.call)
            if refusal is not None:
                return (StopFiller(),) + self._speak_result(reason_sentence(refusal))
            self.pending = event.call
            if event.call.skill == SkillName.STOP:
                # A stop is dispatched immediately and spoken afterwards: A31's
                # dispatch-after-speech rule exists to keep the recognizer live
                # while the wheels turn, and delaying a stop inverts it.
                return (
                    StopFiller(),
                    SendStop("model_stop"),
                ) + self._speak_result(event.call.speech or "Stopping.")
            if not event.call.speech:
                return (StopFiller(),) + self._dispatch(event.call)
            self.state = FsmState.SPEAKING_INTENT
            return (
                StopFiller(),
                Speak(event.call.speech, SpeechKind.INTENT),
                Publish(self.state),
            )
        if isinstance(event, PlanFailed):
            if not self._is_current(event.turn_id):
                return ()
            return (StopFiller(), SendStop(event.reason)) + self._speak_result(APOLOGY)
        return ()

    def _in_speaking_intent(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, Spoke) and self.pending is not None:
            return self._dispatch(self.pending)
        return ()

    def _in_executing(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, Executed):
            if not self._is_current(event.turn_id):
                return ()
            if event.status is ResultStatus.ACCEPTED:
                return ()  # robotd acknowledges first and reports later
            return self._speak_result(completion_sentence(event.status, event.reason))
        return ()

    def _in_speaking_result(self, event: BrainEvent) -> tuple[Action, ...]:
        if isinstance(event, Spoke):
            return self._to_idle()
        return ()

    # -- shared transitions -------------------------------------------------

    def _begin_turn(self) -> tuple[Action, ...]:
        """Mint the turn and announce it to robotd before any box call."""
        self.turn_id = self._mint()
        self.pending = None
        return (AnnounceTurn(self.turn_id),)

    def _plan(self, event: Heard) -> tuple[Action, ...]:
        assert self.turn_id is not None
        self.authorized_motion = authorized_motion(
            event.text,
            event.confidence,
            min_chars=self._min_chars,
            min_confidence=self._min_confidence,
        )
        self.state = FsmState.PLANNING
        return (
            PlayAck(),
            PlayFiller(),
            StartPlanning(event.text, self.turn_id, self.authorized_motion),
            Publish(self.state),
        )

    def _refuse(self, call: SkillCall) -> ResultReason | None:
        """A12 stage three, run here rather than before the network."""
        if call.skill in MOTION_CAPABLE and not self.authorized_motion:
            return ResultReason.UNAUTHORIZED_UTTERANCE
        return None

    def _dispatch(self, call: SkillCall) -> tuple[Action, ...]:
        assert self.turn_id is not None
        self.state = FsmState.EXECUTING
        return (
            Dispatch(call, self.turn_id, self.authorized_motion),
            Publish(self.state),
        )

    def _speak_result(self, sentence: str) -> tuple[Action, ...]:
        self.pending = None
        self.state = FsmState.SPEAKING_RESULT
        return (Speak(sentence, SpeechKind.RESULT), Publish(self.state))

    def _to_idle(self) -> tuple[Action, ...]:
        self.state = FsmState.IDLE
        self.turn_id = None
        self.pending = None
        self.authorized_motion = False
        return (Publish(self.state),)

    def _enter(self, state: FsmState) -> Action:
        self.state = state
        return Publish(state)

    def _is_current(self, turn_id: str) -> bool:
        return self.turn_id is not None and turn_id == self.turn_id

    # -- events every state answers ----------------------------------------

    def _on_stop(self, reason: str) -> tuple[Action, ...]:
        """I-22, and ARCHITECTURE 7's "a stop advances ``current_turn_id``".

        Dropping the turn id here is what makes an in-flight box response
        stale when it lands: cancelling the task is best effort, the stale id
        is the property.
        """
        actions: list[Action] = [SendStop(reason)]
        if self.state is FsmState.LISTENING:
            actions.append(CancelListening())
        if self.state is FsmState.PLANNING:
            actions += [CancelPlan(), StopFiller()]
        if self.state in (FsmState.SPEAKING_INTENT, FsmState.SPEAKING_RESULT):
            actions.append(CancelSpeech())
        if self.state is FsmState.EXECUTING:
            actions.append(CancelDispatch(reason))
        actions += self._to_idle()
        return tuple(actions)

    def _supersede(self, event: Heard) -> tuple[Action, ...]:
        """A new utterance advances the turn immediately (ARCHITECTURE 7).

        The ``turn`` message the new turn announces is what cancels the old
        turn's command at robotd; cancelling here only stops brain's own task.
        """
        actions: list[Action] = []
        if self.state is FsmState.PLANNING:
            actions += [CancelPlan(), StopFiller()]
        if self.state in (FsmState.SPEAKING_INTENT, FsmState.SPEAKING_RESULT):
            actions.append(CancelSpeech())
        if self.state is FsmState.EXECUTING:
            actions.append(CancelDispatch("superseded"))
        actions += self._begin_turn()
        actions += self._plan(event)
        return tuple(actions)

    def _on_box_lost(self) -> tuple[Action, ...]:
        """A20: box-link loss cancels an in-flight brain goal within 3 s.

        Teleop is exempt because it never reaches this machine, and the local
        router keeps working: only the states that are waiting on the box, or
        acting on something it said, are affected.
        """
        if self.state is FsmState.PLANNING:
            return (
                CancelPlan(),
                StopFiller(),
                SendStop("box_lost"),
            ) + self._speak_result(BOX_LOST)
        if self.state is FsmState.EXECUTING:
            return (
                CancelDispatch("box_lost"),
                SendStop("box_lost"),
            ) + self._speak_result(BOX_LOST)
        return ()

    def _on_timeout(self) -> tuple[Action, ...]:
        if self.state is FsmState.LISTENING:
            return (CancelListening(),) + self._to_idle()
        if self.state is FsmState.TRANSCRIBING:
            return self._speak_result(APOLOGY)
        if self.state is FsmState.PLANNING:
            # T3, or a box that never finished a sentence: second filler is
            # already playing, then the apology, then stop (ARCHITECTURE 7).
            return (
                CancelPlan(),
                StopFiller(),
                SendStop("plan_timeout"),
            ) + self._speak_result(APOLOGY)
        if self.state is FsmState.SPEAKING_INTENT:
            # The sentence never finished, so the skill was never dispatched.
            return (CancelSpeech(), SendStop("speech_timeout")) + self._to_idle()
        if self.state is FsmState.EXECUTING:
            return (
                CancelDispatch("deadline"),
                SendStop("goal_deadline"),
            ) + self._speak_result(
                completion_sentence(ResultStatus.TIMEOUT, ResultReason.NONE)
            )
        if self.state is FsmState.SPEAKING_RESULT:
            return (CancelSpeech(),) + self._to_idle()
        return ()
