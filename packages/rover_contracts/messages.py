"""Every message on every socket, as pydantic v2 models.

Covers the robotd bus (ARCHITECTURE 5.2), the SkillCall the model emits (5.4),
the ``frames.sock`` header (5.3) and ``brain.sock`` (5.9).

Every model is ``strict`` with ``extra="forbid"`` and ``allow_inf_nan=False``:
A12's second validation stage is only worth having if it rejects a coerced
string, an unexpected key and a NaN.  Bounds here are the architecture's
compiled ceilings, which config may lower but never raise (A33), so a value that
fails one of these is out of bounds however robotd is configured.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Final, Literal, TypeVar

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    TypeAdapter,
    model_validator,
)

from rover_contracts.config import LimitsConfig, SafetyConfig
from rover_contracts.ids import SESSION_ID_PATTERN, ULID_PATTERN

__all__ = [
    "DriveForArgs",
    "DriveForBusArgs",
    "DriveForCall",
    "StateRover",
    "TurnToArgs",
    "TurnToBusArgs",
    "TurnToCall",
    "BUS_SKILL_ARGS",
    "BrainCancelMessage",
    "BrainClientMessage",
    "BrainServerMessage",
    "BusCap",
    "CancelMessage",
    "ClearMessage",
    "ClearableFault",
    "ClientMessage",
    "DescribeSceneCall",
    "EnumValue",
    "ErrorMessage",
    "EstopMessage",
    "EventKind",
    "EventMessage",
    "Face",
    "FaceMessage",
    "FindArgs",
    "FindCall",
    "FrameHeader",
    "FrameKind",
    "FsmMessage",
    "FsmState",
    "HelloMessage",
    "NoArgs",
    "PingMessage",
    "PttEndMessage",
    "PttStartMessage",
    "ResultDetail",
    "ResultMessage",
    "ResultReason",
    "ResultStatus",
    "SayArgs",
    "SayBusArgs",
    "SayCall",
    "ServerMessage",
    "SetFaceArgs",
    "SetFaceCall",
    "SkillCall",
    "SkillMessage",
    "SkillName",
    "SkillObs",
    "SkillTrace",
    "Source",
    "StateActive",
    "StateBattery",
    "StateMessage",
    "StateTwist",
    "StopCall",
    "StopMessage",
    "StrictModel",
    "SubscribeMessage",
    "SubscribeTopic",
    "TurnMessage",
    "TwistMessage",
    "TwistPayload",
    "UtteranceMessage",
    "UtteranceSource",
    "WelcomeMessage",
    "brain_client_adapter",
    "brain_server_adapter",
    "client_adapter",
    "server_adapter",
    "skill_call_adapter",
]


class StrictModel(BaseModel):
    """The one model configuration this repo uses (A12 stage two)."""

    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


_E = TypeVar("_E", bound=StrEnum)

EnumValue = Annotated[_E, Field(strict=False)]
"""A closed enum that also accepts its own string value.

The models are strict so that ``"40"`` never becomes ``40``.  An enum's JSON
form *is* its value, though, so accepting ``"brain"`` for :class:`Source` is a
representation, not a coercion -- and without this every caller would have to
rebuild an enum member before validating a decoded NDJSON line.
"""


# --------------------------------------------------------------------------
# Closed enums
# --------------------------------------------------------------------------


class Source(StrEnum):
    """``source`` on the robotd bus, bound to the connection at ``hello``."""

    BRAIN = "brain"
    WEB = "web"
    TELEOP = "teleop"
    PHONE = "phone"


class UtteranceSource(StrEnum):
    """``source`` on ``brain.sock`` -- a different set from :class:`Source`."""

    WEB = "web"
    CLI = "cli"
    STT = "stt"


class BusCap(StrEnum):
    """What a client declares it will do, in ``hello.caps``."""

    SKILL = "skill"
    TWIST = "twist"
    SUBSCRIBE = "subscribe"


class SubscribeTopic(StrEnum):
    """Topics a client may subscribe to."""

    STATE = "state"
    RESULT = "result"
    EVENT = "event"


class SkillName(StrEnum):
    """The seven skills the model may emit (ARCHITECTURE 6, ADR-0013)."""

    DRIVE_FOR = "drive_for"
    TURN_TO = "turn_to"
    STOP = "stop"
    SAY = "say"
    DESCRIBE_SCENE = "describe_scene"
    FIND = "find"
    SET_FACE = "set_face"


class Face(StrEnum):
    """``set_face`` expressions, rendered by rover-web's face page."""

    NEUTRAL = "neutral"
    HAPPY = "happy"
    THINKING = "thinking"
    CONFUSED = "confused"
    ALERT = "alert"
    SLEEPY = "sleepy"


class ResultStatus(StrEnum):
    """``result.status``.  There is no ``fault``: a fault is ``aborted``."""

    ACCEPTED = "accepted"
    DONE = "done"
    REJECTED = "rejected"
    PREEMPTED = "preempted"
    ABORTED = "aborted"
    TIMEOUT = "timeout"


class ResultReason(StrEnum):
    """``result.reason``, a speakable enum.  ``NONE`` is the empty string."""

    NONE = ""
    UNKNOWN_SKILL = "unknown_skill"
    BAD_ARGS = "bad_args"
    OUT_OF_BOUNDS = "out_of_bounds"
    POWER_CLAMPED = "power_clamped"
    STALE_SEQ = "stale_seq"
    DUPLICATE_CMD = "duplicate_cmd"
    STALE_TURN = "stale_turn"
    GOAL_TTL_TOO_LONG = "goal_ttl_too_long"
    GOAL_TTL_TOO_SHORT = "goal_ttl_too_short"
    TTL_EXPIRED = "ttl_expired"
    NOT_READY = "not_ready"
    FAULTED = "faulted"
    OBSTACLE = "obstacle"
    BUDGET_EXCEEDED = "budget_exceeded"
    RATE_LIMITED = "rate_limited"
    OBS_STALE = "obs_stale"
    SOURCE_NOT_ALLOWED = "source_not_allowed"
    UNAUTHORIZED_UTTERANCE = "unauthorized_utterance"
    UNPATCHED_FIRMWARE = "unpatched_firmware"
    HEADING_UNAVAILABLE = "heading_unavailable"
    FEEDBACK_STALE = "feedback_stale"
    ESTOP_ACTIVE = "estop_active"
    BOX_LOST = "box_lost"


class EventKind(StrEnum):
    """``event.kind``: the Pi-side text table for the ``E`` codes of 5.1, plus
    the one event robotd raises on its own."""

    LINK_UP = "link_up"
    LINK_LOST = "link_lost"
    HEARTBEAT_TIMEOUT = "heartbeat_timeout"
    HEARTBEAT_RECOVERED = "heartbeat_recovered"
    CAP_CLAMP = "cap_clamp"
    TOF_BLOCK = "tof_block"
    BUMPER = "bumper"
    LOW_BATTERY = "low_battery"
    UNPATCHED_FIRMWARE = "unpatched_firmware"
    ROVER_RESTART = "rover_restart"
    FEEDBACK_STALE = "feedback_stale"
    FAULT_CLEARED = "fault_cleared"


class ClearableFault(StrEnum):
    """What a ``clear`` message may name: the latched fault class, plus the
    software e-stop latch robotd persists in ``/run/rover/estop``."""

    ESTOP_SW = "estop_sw"
    OBSTACLE_LATCHED = "obstacle_latched"
    LOW_BATTERY = "low_battery"


class FsmState(StrEnum):
    """The seven agent states of ARCHITECTURE 7."""

    IDLE = "IDLE"
    LISTENING = "LISTENING"
    TRANSCRIBING = "TRANSCRIBING"
    PLANNING = "PLANNING"
    SPEAKING_INTENT = "SPEAKING_INTENT"
    EXECUTING = "EXECUTING"
    SPEAKING_RESULT = "SPEAKING_RESULT"


class FrameKind(StrEnum):
    """``frames.sock`` header ``kind``.  Only a ``still`` may authorize motion."""

    STREAM = "stream"
    STILL = "still"


# --------------------------------------------------------------------------
# Shared field types
# --------------------------------------------------------------------------

Ulid = Annotated[str, Field(pattern=ULID_PATTERN)]
FrameId = Annotated[str, Field(pattern=r"^cam-\d{6}$")]
MonoNs = Annotated[int, Field(ge=0)]
Scalar = bool | int | float | str | None


# --------------------------------------------------------------------------
# Skill arguments -- what the model emits (integers, A11)
# --------------------------------------------------------------------------


class DriveForArgs(StrictModel):
    """``drive_for`` as the model writes it.  Open loop: a power for a time.

    ``power_pct`` is percent of Waveshare full scale, so 30 is the 0.30 cap the
    firmware compiles in; the sign is the direction.  Zero is not a drive."""

    duration_ms: int = Field(ge=100, le=2000)
    power_pct: int = Field(ge=-30, le=30)

    @model_validator(mode="after")
    def _nonzero(self) -> DriveForArgs:
        if self.power_pct == 0:
            raise ValueError("power_pct 0 is not a drive; use stop")
        return self


class TurnToArgs(StrictModel):
    """``turn_to`` as the model writes it: an absolute compass-style heading in
    whole degrees, 0..359, in the frame the WorldState's ``heading_deg`` uses.
    The model turns left 90 by asking for ``(heading - 90) mod 360``."""

    heading_deg: int = Field(ge=0, le=359)


class SayArgs(StrictModel):
    """``say`` as the model writes it."""

    text: str = Field(min_length=1, max_length=240)


class FindArgs(StrictModel):
    """``find``: a stationary scan of at most eight 45 degree sweeps (A14)."""

    object: str = Field(min_length=1, max_length=48)
    max_sweeps: int = Field(ge=1, le=8)


class SetFaceArgs(StrictModel):
    """``set_face``."""

    expr: EnumValue[Face]


class NoArgs(StrictModel):
    """``{}`` -- ``stop`` and ``describe_scene`` take no arguments."""


# --------------------------------------------------------------------------
# Skill arguments -- what crosses the robotd bus (SI)
# --------------------------------------------------------------------------


class DriveForBusArgs(StrictModel):
    """``drive_for`` on the bus.  ``power`` is in Waveshare units (full scale
    0.5); above ``[limits] power_max`` it is clamped, not rejected.  0.30 is the
    absolute bound: the firmware fork clamps there too, so robotd can never
    send more than the controller would apply."""

    duration_s: float = Field(gt=0.0, le=2.0)
    power: float = Field(ge=-0.30, le=0.30)

    @model_validator(mode="after")
    def _nonzero(self) -> DriveForBusArgs:
        if self.power == 0.0:
            raise ValueError("power 0 is not a drive; use stop")
        return self


class TurnToBusArgs(StrictModel):
    """``turn_to`` on the bus: an absolute heading, a deadline, a tolerance.
    robotd closes the loop on the rover's fused yaw; there is no encoder."""

    heading_deg: float = Field(ge=0.0, lt=360.0)
    timeout_s: float = Field(default=4.0, gt=0.0, le=4.0)
    tolerance_deg: float = Field(default=5.0, ge=2.0, le=20.0)


class SayBusArgs(StrictModel):
    """``say`` on the bus: this is where the 300-character bound lives."""

    text: str = Field(min_length=1, max_length=300)


BUS_SKILL_ARGS: Final[dict[str, type[StrictModel]]] = {
    "drive_for": DriveForBusArgs,
    "turn_to": TurnToBusArgs,
    "say": SayBusArgs,
    "describe_scene": NoArgs,
    "find": FindArgs,
    "set_face": SetFaceArgs,
}
"""The six skills that cross robotd as ``type:"skill"``.  ``stop`` is its own
message (I-22) and ``twist`` is its own message (A35)."""


# --------------------------------------------------------------------------
# SkillCall -- ARCHITECTURE 5.4, a oneOf over seven branches, speech first
# --------------------------------------------------------------------------

Speech = Annotated[str, Field(max_length=160)]
"""A31: the sentence that must finish playing before dispatch.  An over-length
value is truncated at the validator, never a reason to refuse a valid skill."""


class DriveForCall(StrictModel):
    speech: Speech
    skill: Literal["drive_for"]
    args: DriveForArgs


class TurnToCall(StrictModel):
    speech: Speech
    skill: Literal["turn_to"]
    args: TurnToArgs


class StopCall(StrictModel):
    speech: Speech
    skill: Literal["stop"]
    args: NoArgs


class SayCall(StrictModel):
    speech: Speech
    skill: Literal["say"]
    args: SayArgs


class DescribeSceneCall(StrictModel):
    speech: Speech
    skill: Literal["describe_scene"]
    args: NoArgs


class FindCall(StrictModel):
    speech: Speech
    skill: Literal["find"]
    args: FindArgs


class SetFaceCall(StrictModel):
    speech: Speech
    skill: Literal["set_face"]
    args: SetFaceArgs


SkillCall = Annotated[
    DriveForCall
    | TurnToCall
    | StopCall
    | SayCall
    | DescribeSceneCall
    | FindCall
    | SetFaceCall,
    Field(discriminator="skill"),
]

skill_call_adapter: Final = TypeAdapter(SkillCall)


# --------------------------------------------------------------------------
# Client -> robotd
# --------------------------------------------------------------------------


class HelloMessage(StrictModel):
    """Binds ``source`` to the connection; a second ``hello`` is refused."""

    v: Literal[1] = 1
    type: Literal["hello"] = "hello"
    source: EnumValue[Source]
    pid: int = Field(gt=0)
    caps: list[EnumValue[BusCap]] = Field(min_length=1)


class SubscribeMessage(StrictModel):
    """Topic subscription.  Carries no ``source``: it is already bound."""

    v: Literal[1] = 1
    type: Literal["subscribe"] = "subscribe"
    topics: list[EnumValue[SubscribeTopic]] = Field(min_length=1)
    state_hz: int = Field(default=10, ge=1, le=50)


class TurnMessage(StrictModel):
    """The bare turn boundary brain sends at wake/PTT/text, before any box
    call.  Without it a superseded plan could still drive (I-11)."""

    v: Literal[1] = 1
    type: Literal["turn"] = "turn"
    source: EnumValue[Source]
    turn_id: Ulid


class PingMessage(StrictModel):
    """5 Hz liveness from whichever client owns an active command."""

    v: Literal[1] = 1
    type: Literal["ping"] = "ping"
    source: EnumValue[Source]


class SkillObs(StrictModel):
    """The frame that justifies a motion skill; robotd gates on its age (I-23)."""

    frame_id: FrameId
    frame_mono_ns: MonoNs


class SkillTrace(StrictModel):
    """Provenance, asserted per trial at G1.  ``authorized_motion`` rides here."""

    model: str | None = Field(default=None, max_length=128)
    prompt_sha256: str | None = Field(default=None, max_length=64)
    authorized_motion: bool | None = None


class SkillMessage(StrictModel):
    """A dispatched skill.  ``issued_mono_ns`` is the sender's clock and is
    never compared against robotd's, which stamps its own ``recv_mono_ns``."""

    v: Literal[1] = 1
    type: Literal["skill"] = "skill"
    source: EnumValue[Source]
    cmd_id: Ulid
    seq: int = Field(ge=0)
    turn_id: Ulid
    issued_mono_ns: MonoNs
    goal_ttl_ms: int = Field(ge=100, le=5000)
    skill: Literal["drive_for", "turn_to", "say", "describe_scene", "find", "set_face"]
    args: DriveForBusArgs | TurnToBusArgs | SayBusArgs | FindArgs | SetFaceArgs | NoArgs
    obs: SkillObs | None = None
    trace: SkillTrace | None = None

    @model_validator(mode="after")
    def _args_match_skill(self) -> SkillMessage:
        expected = BUS_SKILL_ARGS[self.skill]
        if type(self.args) is not expected:
            raise ValueError(
                f"skill {self.skill!r} takes {expected.__name__}, "
                f"got {type(self.args).__name__}"
            )
        return self


class TwistPayload(StrictModel):
    """A35's streamed command, renewed every 200 ms or it lapses.  Open loop, so
    both axes are in power units: ``lin`` drives both sides, ``ang`` is added to
    the right and subtracted from the left, and each side is then clamped."""

    lin: float = Field(ge=-0.30, le=0.30)
    ang: float = Field(ge=-0.30, le=0.30)


class TwistMessage(StrictModel):
    """A stream is one arbitration unit; it has no goal TTL and no ``turn_id``."""

    v: Literal[1] = 1
    type: Literal["twist"] = "twist"
    source: EnumValue[Source]
    cmd_id: Ulid
    seq: int = Field(ge=0)
    twist: TwistPayload


class CancelMessage(StrictModel):
    """Stop-class: accepted from any allow-listed source in every state (I-22)."""

    v: Literal[1] = 1
    type: Literal["cancel"] = "cancel"
    source: EnumValue[Source]
    cmd_id: Ulid | None = None
    reason: str | None = Field(default=None, max_length=64)


class StopMessage(StrictModel):
    """Stop-class, and deliberately the loosest message on the bus: no required
    ``seq``, ``cmd_id`` or ``turn_id``, because brain's restart path sends one
    before any turn exists and because I-22 forbids ever answering it
    ``rejected``.  A parse failure here must never become a rejection."""

    v: Literal[1] = 1
    type: Literal["stop"] = "stop"
    source: EnumValue[Source]
    reason: str | None = Field(default=None, max_length=64)
    cmd_id: Ulid | None = None
    turn_id: Ulid | None = None
    seq: int | None = Field(default=None, ge=0)


class EstopMessage(StrictModel):
    """Stop-class.  The web STOP button is never behind the model or the
    validator, and is one of the three counted stop authorities (A29)."""

    v: Literal[1] = 1
    type: Literal["estop"] = "estop"
    source: EnumValue[Source]
    reason: str | None = Field(default=None, max_length=64)


class ClearMessage(StrictModel):
    """Not stop-class.  Restricted to ``[bus] clear_sources`` and does not
    re-arm: recovery from a stop authority must not re-enable motion."""

    v: Literal[1] = 1
    type: Literal["clear"] = "clear"
    source: EnumValue[Source]
    faults: list[EnumValue[ClearableFault]] = Field(min_length=1)


# --------------------------------------------------------------------------
# robotd -> clients
# --------------------------------------------------------------------------


class WelcomeMessage(StrictModel):
    """Republishes every ``[limits]`` and ``[safety]`` key, so a gate asserts
    against what is running rather than against the file (A33)."""

    v: Literal[1] = 1
    type: Literal["welcome"] = "welcome"
    session: str = Field(pattern=SESSION_ID_PATTERN)
    robotd_version: str = Field(min_length=1, max_length=32)
    rover_fw: str | None = Field(default=None, max_length=32)
    """The firmware fork's banner tag, or ``null`` while the link is down or the
    firmware is stock.  Motion is refused in both cases."""
    limits: LimitsConfig
    safety: SafetyConfig


class StateRover(StrictModel):
    """What the controller last reported, with robotd's arrival stamp.

    ``fw`` is ``null`` for stock firmware, and that alone makes ``ready``
    false: a controller without the fork's heartbeat and cap is not one robotd
    will drive.  ``heading_deg`` is the fused yaw in (-180, 180], sign already
    corrected by ``[link] yaw_sign``."""

    fw: str | None = Field(default=None, max_length=32)
    hb_ok: bool
    stop_flags: int = Field(ge=0, le=31)
    feedback_age_ms: int = Field(ge=0)
    cmd_left: float = Field(ge=-0.5, le=0.5)
    cmd_right: float = Field(ge=-0.5, le=0.5)
    heading_deg: float = Field(gt=-180.0, le=180.0)
    yaw_rate_dps: float | None = None
    roll_deg: float
    pitch_deg: float
    temp_c: float
    clamp_count: int = Field(ge=0, le=0xFFFF)
    motion: bool
    """Either applied side non-zero in the last feedback."""


class StateTwist(StrictModel):
    """The command robotd is currently streaming, in power units."""

    lin: float = Field(ge=-0.30, le=0.30)
    ang: float = Field(ge=-0.30, le=0.30)


class StateBattery(StrictModel):
    """From the board's INA219 bus voltage.  No current or open-circuit
    estimate: the board does not report them."""

    pack_v: float = Field(ge=0.0)
    pct: int = Field(ge=0, le=100)


class StateActive(StrictModel):
    cmd_id: Ulid
    skill: Literal["drive_for", "turn_to", "say", "describe_scene", "find", "set_face"]
    source: EnumValue[Source]
    progress: float = Field(ge=0.0, le=1.0)
    deadline_in_ms: int = Field(ge=0)


class StateBudget(StrictModel):
    """What I-15's per-instruction budget has left on the current instruction,
    from the one ledger that enforces it.  Open loop, so the budget is motion
    time: robotd charges every 20 ms slot in which it streamed a non-zero
    command, and a dispatch-time estimate booked elsewhere is not the budget
    robotd applies."""

    motion_s: float = Field(ge=0.0)


class StateMessage(StrictModel):
    """Published at ``[bus] state_hz`` and on any change of ``rover.hb_ok``,
    ``rover.stop_flags``, ``rover.fw``, ``active.cmd_id`` or ``ready``."""

    v: Literal[1] = 1
    type: Literal["state"] = "state"
    t_utc_ns: int = Field(ge=0)
    t_mono_ns: MonoNs
    rover: StateRover
    twist: StateTwist
    front_m: float | None = Field(default=None, ge=0.0)
    """Front range in metres, ``null`` when the sensor is absent or has no
    reading.  ``null`` is not a clear path (I-16)."""
    bumper: bool
    estop_sw: bool
    battery: StateBattery
    active: StateActive | None = None
    budget: StateBudget
    ready: bool
    reason: str = Field(default="", max_length=64)


class ResultDetail(StrictModel):
    """What a completed or clamped command reports back."""

    duration_ms: int | None = Field(default=None, ge=0)
    turned_deg: float | None = None
    heading_error_deg: float | None = None
    power_clamped_to: float | None = Field(default=None, ge=0.0, le=0.30)


class ResultMessage(StrictModel):
    """The outcome of one ``cmd_id``."""

    v: Literal[1] = 1
    type: Literal["result"] = "result"
    cmd_id: Ulid
    seq: int | None = Field(default=None, ge=0)
    status: EnumValue[ResultStatus]
    reason: EnumValue[ResultReason] = ResultReason.NONE
    detail: ResultDetail | None = None
    t_utc_ns: int = Field(ge=0)


class EventMessage(StrictModel):
    """A fault, an MCU restart, or any of the ``E`` codes of ARCHITECTURE 5.1."""

    v: Literal[1] = 1
    type: Literal["event"] = "event"
    kind: EnumValue[EventKind]
    fault: str | None = Field(default=None, max_length=32)
    detail: dict[str, Scalar] | None = None
    t_utc_ns: int = Field(ge=0)


class ErrorMessage(StrictModel):
    """A protocol-level complaint about the client's own bytes."""

    v: Literal[1] = 1
    type: Literal["error"] = "error"
    code: str = Field(min_length=1, max_length=64, pattern=r"^[a-z][a-z0-9_]*$")
    detail: str = Field(default="", max_length=256)


ClientMessage = Annotated[
    HelloMessage
    | SubscribeMessage
    | TurnMessage
    | PingMessage
    | SkillMessage
    | TwistMessage
    | CancelMessage
    | StopMessage
    | EstopMessage
    | ClearMessage,
    Field(discriminator="type"),
]

ServerMessage = Annotated[
    WelcomeMessage | StateMessage | ResultMessage | EventMessage | ErrorMessage,
    Field(discriminator="type"),
]

client_adapter: Final = TypeAdapter(ClientMessage)
server_adapter: Final = TypeAdapter(ServerMessage)


# --------------------------------------------------------------------------
# frames.sock -- ARCHITECTURE 5.3
# --------------------------------------------------------------------------


class FrameHeader(StrictModel):
    """One NDJSON header line, then exactly ``bytes`` octets of JPEG.

    ``frame_mono_ns`` is ``CLOCK_MONOTONIC`` and is the only value robotd's
    freshness gate reads: the Pi 4 has no RTC, so a wall-clock age is wrong by
    hours before the first NTP sync.
    """

    v: Literal[1] = 1
    frame_id: FrameId
    kind: EnumValue[FrameKind]
    frame_mono_ns: MonoNs
    frame_wallclock_ns: int = Field(ge=0)
    w: int = Field(gt=0)
    h: int = Field(gt=0)
    fmt: Literal["jpeg"] = "jpeg"
    quality: int = Field(ge=1, le=100)
    bytes: int = Field(ge=0)
    exposure_us: int | None = Field(default=None, ge=0)
    gain: float | None = Field(default=None, ge=0.0)
    lux: float | None = Field(default=None, ge=0.0)


# --------------------------------------------------------------------------
# brain.sock -- ARCHITECTURE 5.9
# --------------------------------------------------------------------------


class UtteranceMessage(StrictModel):
    """Text, PTT and the wake word all emit this identical shape (A30).

    ``confidence`` is ``null`` for text, PTT and ``roverctl``, and a ``null``
    counts as authorized in ARCHITECTURE 7's ``authorized_motion`` rule.
    """

    v: Literal[1] = 1
    type: Literal["utterance"] = "utterance"
    source: EnumValue[UtteranceSource]
    text: str = Field(max_length=500)
    confidence: float | None = Field(default=None, ge=0.0, le=1.0)
    is_final: bool
    mono_ns: MonoNs


class PttStartMessage(StrictModel):
    v: Literal[1] = 1
    type: Literal["ptt_start"] = "ptt_start"
    source: EnumValue[UtteranceSource]


class PttEndMessage(StrictModel):
    v: Literal[1] = 1
    type: Literal["ptt_end"] = "ptt_end"
    source: EnumValue[UtteranceSource]


class BrainCancelMessage(StrictModel):
    v: Literal[1] = 1
    type: Literal["cancel"] = "cancel"
    source: EnumValue[UtteranceSource]


class FaceMessage(StrictModel):
    """What ``set_face`` actually executes; rover-web's face page renders it."""

    v: Literal[1] = 1
    type: Literal["face"] = "face"
    expr: EnumValue[Face]


class FsmMessage(StrictModel):
    """The agent state, for the face page and for logs."""

    v: Literal[1] = 1
    type: Literal["fsm"] = "fsm"
    state: EnumValue[FsmState]


BrainClientMessage = Annotated[
    UtteranceMessage | PttStartMessage | PttEndMessage | BrainCancelMessage,
    Field(discriminator="type"),
]

BrainServerMessage = Annotated[
    FaceMessage | FsmMessage,
    Field(discriminator="type"),
]

brain_client_adapter: Final = TypeAdapter(BrainClientMessage)
brain_server_adapter: Final = TypeAdapter(BrainServerMessage)
