"""Tiers one and two of A31: a tone, then a waiting line.

Tier one is an **earcon, not speech** -- "a tone cannot be wrong", and it plays
the moment an utterance is accepted, before anything is known about what the
robot will do.  Tier two is one fixed sentence, and it is started on a delay so
that a fast plan never talks over itself; the instant the model's first
sentence is ready the FSM cancels it.  Tier three (the model's own ``speech``)
and the completion table are spoken by the FSM, not from here.
"""

from __future__ import annotations

import asyncio
import math
import struct
from typing import Final

from rover_brain.audio.tts import Tts
from rover_brain.filler.table import WAITING

__all__ = ["EARCON_HZ", "EARCON_MS", "FillerPlayer", "tone_pcm"]

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
