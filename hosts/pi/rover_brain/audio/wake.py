"""Wake word, behind one adapter chosen by ``[wake] backend``.

A30: text, PTT and the wake word emit an identical ``Utterance``, and text is
the default in sim and until G3b, so the wake word is the one input this repo
can least afford to make mandatory.  ``pyopen-wakeword`` ships a
``manylinux_2_35 aarch64`` wheel only, so it is a linux-only extra and is
imported inside :meth:`PyOpenWake.wait`.

The dev backend is a hotkey: a line on standard input fires the wake, which is
what ``make dev`` uses on a Mac with no microphone.
"""

from __future__ import annotations

import asyncio
import sys
from collections.abc import AsyncIterator
from typing import Any, Protocol

from rover_contracts.config import ConfigError, WakeConfig

__all__ = ["HotkeyWake", "NoWake", "PyOpenWake", "Wake", "make_wake"]


class Wake(Protocol):
    """Waits for the robot's name."""

    async def wait(self, frames: AsyncIterator[bytes]) -> None:
        """Return once the wake word has fired."""


class PyOpenWake:
    """``pyopen-wakeword`` 1.1.0 against a self-trained model.

    The pre-trained openWakeWord models are CC BY-NC-SA 4.0 and are not shipped
    here (open item 10); ``[wake] model`` names the repo's own ``rover.tflite``.
    """

    def __init__(self, model: str, threshold: float) -> None:
        self._model = model
        self._threshold = threshold
        self._detector: Any = None

    async def wait(self, frames: AsyncIterator[bytes]) -> None:
        if self._detector is None:
            import pyopen_wakeword  # noqa: PLC0415 - linux-only extra

            self._detector = pyopen_wakeword.Model(wakeword_models=[self._model])
        import numpy  # noqa: PLC0415 - speech extra

        async for frame in frames:
            samples = numpy.frombuffer(frame, dtype="<i2")
            scores = await asyncio.to_thread(self._detector.predict, samples)
            if any(score >= self._threshold for score in scores.values()):
                self._detector.reset()
                return


class HotkeyWake:
    """The dev backend: any line on standard input is the wake word."""

    async def wait(self, frames: AsyncIterator[bytes]) -> None:
        line = await asyncio.to_thread(sys.stdin.readline)
        if not line:
            raise EOFError("standard input closed; [wake] backend='hotkey' needs a tty")


class NoWake:
    """``[wake] backend="none"``: never fires.  PTT or text is the way in."""

    async def wait(self, frames: AsyncIterator[bytes]) -> None:
        await asyncio.Event().wait()


def make_wake(config: WakeConfig) -> Wake:
    """Select the wake word from configuration alone (principle 6)."""
    if config.backend == "pyopen":
        return PyOpenWake(config.model, config.threshold)
    if config.backend == "hotkey":
        return HotkeyWake()
    if config.backend == "none":
        return NoWake()
    raise ConfigError(
        '[wake] backend="pymicro" is open item 11\'s fallback and is a commented '
        "alternative in the speech extra, not installed by default; v1 ships pyopen, "
        "hotkey and none."
    )
