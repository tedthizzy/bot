"""Voice activity detection.  A27: **VAD owns end-of-speech** (Silero, 400 ms).

The real detector is Silero through ``sherpa_onnx.VoiceActivityDetector`` --
which is why there is no separate VAD package in ARCHITECTURE 12 -- and it is
imported lazily so this module loads on a Mac with nothing installed.

The dev detector is a null one: it never fires, because with ``[audio]
input`` set to ``text`` or ``ptt`` the boundary of an utterance is the typed
line or the released button, and a detector that invented an end-of-speech
would fight it.  That is the configuration deciding, not a code branch.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, Protocol

from rover_contracts.config import AudioConfig, VadConfig

__all__ = ["NullVad", "SherpaVad", "Vad", "VadEvent", "make_vad"]


class VadEvent(StrEnum):
    """What a period of audio just did to the utterance boundary."""

    SPEECH_START = "speech_start"
    SPEECH_END = "speech_end"


class Vad(Protocol):
    """Fed one 20 ms period at a time; answers only at a boundary."""

    def accept(self, frame: bytes) -> VadEvent | None: ...

    def reset(self) -> None: ...


class NullVad:
    """Never fires.  End-of-speech comes from the button or the keyboard."""

    def accept(self, frame: bytes) -> VadEvent | None:
        return None

    def reset(self) -> None:
        return None


class SherpaVad:
    """Silero through sherpa-onnx, with A27's 400 ms of trailing silence."""

    def __init__(
        self, model: str, *, sample_rate: int, min_silence_ms: int, chunk: int
    ) -> None:
        import numpy  # noqa: PLC0415 - speech extra
        import sherpa_onnx  # noqa: PLC0415 - speech extra

        self._numpy = numpy
        config = sherpa_onnx.VadModelConfig()
        config.silero_vad.model = model
        config.silero_vad.min_silence_duration = min_silence_ms / 1000.0
        config.sample_rate = sample_rate
        self._chunk = chunk
        self._vad: Any = sherpa_onnx.VoiceActivityDetector(
            config, buffer_size_in_seconds=30
        )
        self._speaking = False

    def accept(self, frame: bytes) -> VadEvent | None:
        samples = self._numpy.frombuffer(frame, dtype="<i2").astype("float32") / 32768.0
        self._vad.accept_waveform(samples)
        speaking = self._vad.is_speech_detected()
        if speaking and not self._speaking:
            self._speaking = True
            return VadEvent.SPEECH_START
        if not speaking and self._speaking:
            self._speaking = False
            return VadEvent.SPEECH_END
        return None

    def reset(self) -> None:
        self._vad.reset()
        self._speaking = False


def make_vad(config: VadConfig, audio: AudioConfig, *, sample_rate: int) -> Vad:
    """Select the detector from configuration alone (principle 6).

    ``[vad] backend`` has one legal value, so the input mode is what chooses:
    typed text and push-to-talk carry their own end-of-speech, and only the
    wake-word and recorded-file paths need Silero to find one.
    """
    if audio.input in ("text", "ptt"):
        return NullVad()
    return SherpaVad(
        config.model,
        sample_rate=sample_rate,
        min_silence_ms=config.min_silence_ms,
        chunk=config.chunk,
    )
