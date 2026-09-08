"""The three filler tiers of A31, and the fixed completion table.

1. **Acknowledgement** -- a tone, not speech, played the moment an utterance is
   accepted.  A tone cannot be wrong.
2. **Waiting line** -- one fixed sentence during PLANNING, started on a delay so
   that a fast plan never talks over itself, and cancelled the instant the
   model's first sentence is ready.
3. **Intent** -- the model's own ``speech``, spoken in SPEAKING_INTENT by the
   FSM; the skill is not dispatched until it has finished playing.

Completion speech is a table, not a generated sentence, and it is driven by the
executor's result: nothing here can be spoken before robotd has reported one,
because :func:`completion_sentence` takes the status and reason as arguments
and there is no other way to reach these strings.  "Never say a movement
succeeded until the executor reports it" is the whole point of the table.

The one number a sentence may carry is the heading error robotd reports on a
``turn_to`` that ran out of time, rounded to whole degrees -- it is the
executor's measurement, not the model's claim.
"""

from __future__ import annotations

import asyncio
import math
import struct
from typing import Final

from rover_contracts.messages import ResultDetail, ResultReason, ResultStatus, SkillName
from rover_contracts.units import round_half_away

from rover_brain.audio.tts import Tts

__all__ = [
    "APOLOGY",
    "BOX_LOST",
    "EARCON_HZ",
    "EARCON_MS",
    "STOPPING",
    "WAITING",
    "FillerPlayer",
    "completion_sentence",
    "reason_sentence",
    "tone_pcm",
]

# -- the fixed sentences

WAITING: Final = "One moment."
"""Tier two: the waiting line, spoken during PLANNING only if the plan is slow.
Tier one is a tone (:func:`tone_pcm`) and tier three is the model's own
``speech``, spoken in SPEAKING_INTENT before dispatch."""

APOLOGY: Final = "I didn't get that."
"""ARCHITECTURE 7's failure path: box timeout or a second schema failure."""

BOX_LOST: Final = "I've lost the box."
"""A20/ARCHITECTURE 4.6: a dead box degrades the rover to the local router."""

STOPPING: Final = "Stopping."

_BY_STATUS: Final[dict[ResultStatus, str]] = {
    ResultStatus.DONE: "Done.",
    ResultStatus.REJECTED: "I can't do that.",
    ResultStatus.PREEMPTED: "Cancelled that one.",
    ResultStatus.ABORTED: "I had to stop.",
    ResultStatus.TIMEOUT: "That took too long, so I stopped.",
    ResultStatus.ACCEPTED: "On it.",
}

_DONE_BY_SKILL: Final[dict[str, str]] = {
    SkillName.DRIVE_FOR: "Done driving.",
    SkillName.TURN_TO: "Facing that way now.",
}

_BY_REASON: Final[dict[ResultReason, str]] = {
    ResultReason.UNKNOWN_SKILL: "I don't know how to do that.",
    ResultReason.BAD_ARGS: "I didn't understand the numbers in that.",
    ResultReason.OUT_OF_BOUNDS: "That's outside what I'm allowed to do.",
    ResultReason.POWER_CLAMPED: "Done, at my top power.",
    ResultReason.STALE_SEQ: "I got that out of order.",
    ResultReason.DUPLICATE_CMD: "I've already done that one.",
    ResultReason.STALE_TURN: "That was for an older instruction.",
    ResultReason.GOAL_TTL_TOO_LONG: "That would take longer than I'm allowed.",
    ResultReason.GOAL_TTL_TOO_SHORT: "That deadline is too short for the move.",
    ResultReason.TTL_EXPIRED: "I lost the link to the controller.",
    ResultReason.NOT_READY: "I'm not ready to move.",
    ResultReason.FAULTED: "I've got a fault, so I stopped.",
    ResultReason.OBSTACLE: "Something is in the way.",
    ResultReason.BUDGET_EXCEEDED: "That's more moving than I can do in one go.",
    ResultReason.RATE_LIMITED: "Give me a moment before the next move.",
    ResultReason.OBS_STALE: "My camera view is too old to move on.",
    ResultReason.SOURCE_NOT_ALLOWED: "I'm not allowed to take that from there.",
    ResultReason.UNAUTHORIZED_UTTERANCE: "I didn't hear that clearly enough to move.",
    ResultReason.UNPATCHED_FIRMWARE: (
        "The controller isn't running my safety firmware, so I won't move."
    ),
    ResultReason.HEADING_UNAVAILABLE: "I can't tell which way I'm facing, so I stopped.",
    ResultReason.FEEDBACK_STALE: "I've lost the controller's feedback, so I stopped.",
    ResultReason.ESTOP_ACTIVE: "The emergency stop is on.",
    ResultReason.BOX_LOST: BOX_LOST,
}


def completion_sentence(
    status: ResultStatus,
    reason: ResultReason,
    *,
    skill: str | None = None,
    detail: ResultDetail | None = None,
) -> str:
    """What to say once the executor has reported ``status``/``reason``.

    A reason wins over a status.  Without one, a ``turn_to`` that timed out
    says how far off it is when robotd measured that, and a finished
    ``drive_for`` or ``turn_to`` gets its own sentence.
    """
    if reason is not ResultReason.NONE and reason in _BY_REASON:
        return _BY_REASON[reason]
    if (
        status is ResultStatus.TIMEOUT
        and skill == SkillName.TURN_TO
        and detail is not None
        and detail.heading_error_deg is not None
    ):
        off = round_half_away(abs(detail.heading_error_deg))
        return f"I ran out of time turning; I'm {off} degrees off."
    if status is ResultStatus.DONE and skill in _DONE_BY_SKILL:
        return _DONE_BY_SKILL[skill]
    return _BY_STATUS[status]


def reason_sentence(reason: ResultReason) -> str:
    """What to say about a refusal this host made itself (A12 stage three)."""
    return _BY_REASON.get(reason, _BY_STATUS[ResultStatus.REJECTED])


# -- tiers one and two

EARCON_HZ: Final = (784.0, 1046.5)
"""Two short notes, G5 then C6: recognisable at 40 mm of speaker and short
enough that the acknowledgement never delays the box call."""

EARCON_MS: Final = 70
"""Per note.  140 ms of tone in total."""

_FADE_MS: Final = 5
"""A linear fade at each end; a square-edged tone clicks on a small driver."""


def tone_pcm(
    frequencies: tuple[float, ...], *, ms: int, rate: int, amplitude: float = 0.25
) -> bytes:
    """Signed 16-bit little-endian mono PCM for one earcon.

    Pure: the same arguments give the same bytes, so the tone is testable
    without an audio device.
    """
    if not 0.0 < amplitude <= 1.0:
        raise ValueError(f"amplitude must be in (0, 1], got {amplitude!r}")
    samples_per_note = int(rate * ms / 1000)
    fade = max(1, int(rate * _FADE_MS / 1000))
    peak = int(amplitude * 32767)
    out = bytearray()
    for frequency in frequencies:
        step = 2.0 * math.pi * frequency / rate
        for n in range(samples_per_note):
            envelope = min(1.0, n / fade, (samples_per_note - n) / fade)
            out += struct.pack("<h", int(peak * envelope * math.sin(step * n)))
    return bytes(out)


class FillerPlayer:
    """Plays tier one immediately and schedules tier two."""

    def __init__(self, tts: Tts, *, delay_s: float = 0.6, line: str = WAITING) -> None:
        self._tts = tts
        self._delay_s = delay_s
        self._line = line
        self._task: asyncio.Task[None] | None = None

    async def ack(self) -> None:
        """Tier one: the acknowledgement tone.  Not speech (A31)."""
        await self._tts.play_pcm(
            tone_pcm(EARCON_HZ, ms=EARCON_MS, rate=self._tts.sample_rate),
            self._tts.sample_rate,
        )

    def start_waiting(self) -> None:
        """Arm tier two.  It speaks only if it survives ``delay_s``."""
        self.stop()
        self._task = asyncio.create_task(self._wait_then_speak())

    def stop(self) -> None:
        """Cancel tier two, whether it is still waiting or already speaking."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
        self._task = None

    async def _wait_then_speak(self) -> None:
        try:
            await asyncio.sleep(self._delay_s)
            await self._tts.speak(self._line)
        except asyncio.CancelledError:
            pass
