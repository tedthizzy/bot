"""Speech recognition, behind one adapter chosen by ``[stt] backend``.

A27: the default is a sherpa-onnx streaming Zipformer int8, which carries all
four bake-off candidates behind one API so that G3b is a config change.  It is
imported lazily; the dev recogniser takes its transcripts from
:meth:`QueueStt.submit` instead of from audio, so ``make dev`` and every test
run on a Mac with no model and no microphone.

A transcript is **data, never instruction**.  It reaches the model inside the
user turn and nowhere else (see :mod:`rover_brain.prompt`), and it is logged
(I-21).
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from dataclasses import dataclass
from typing import Any, Protocol

from rover_contracts.config import ConfigError, SttConfig

__all__ = ["QueueStt", "SherpaStt", "Stt", "Transcript", "make_stt"]


@dataclass(frozen=True, slots=True)
class Transcript:
    """One final transcript.

    ``confidence`` is ``None`` for every backend that supplies none, and
    ARCHITECTURE 7 counts a ``None`` as authorized -- that is what keeps the
    text path able to move.
    """

    text: str
    confidence: float | None = None


class Stt(Protocol):
    """Consumes 20 ms periods and answers once, at end of speech."""

    async def transcribe(self, frames: AsyncIterator[bytes]) -> Transcript: ...


class SherpaStt:
    """Streaming Zipformer int8 through sherpa-onnx (A27).

    ``model_dir`` holds the files the deploy script downloads; the recogniser
    is constructed from the directory's conventional layout so that swapping in
    Vosk, Moonshine or whisper at G3b is a config change and nothing else.
    """

    def __init__(self, model_dir: str, *, sample_rate: int) -> None:
        import sherpa_onnx  # noqa: PLC0415 - speech extra

        self._sample_rate = sample_rate
        self._recognizer: Any = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=f"{model_dir}/tokens.txt",
            encoder=f"{model_dir}/encoder.onnx",
            decoder=f"{model_dir}/decoder.onnx",
            joiner=f"{model_dir}/joiner.onnx",
            num_threads=1,
            sample_rate=sample_rate,
            feature_dim=80,
        )
        import numpy  # noqa: PLC0415 - speech extra

        self._numpy = numpy

    async def transcribe(self, frames: AsyncIterator[bytes]) -> Transcript:
        stream = self._recognizer.create_stream()
        async for frame in frames:
            samples = (
                self._numpy.frombuffer(frame, dtype="<i2").astype("float32") / 32768.0
            )
            stream.accept_waveform(self._sample_rate, samples)
            while self._recognizer.is_ready(stream):
                await asyncio.to_thread(self._recognizer.decode_stream, stream)
        stream.input_finished()
        while self._recognizer.is_ready(stream):
            await asyncio.to_thread(self._recognizer.decode_stream, stream)
        return Transcript(self._recognizer.get_result(stream).strip())


class QueueStt:
    """The dev recogniser: transcripts are submitted, not recognised.

    ``[stt] backend="text"`` is the Mac default and A30's "text is default in
    sim and until G3b"; ``"mock"`` is the same object driven by a gate script.
    Audio periods are consumed and discarded so that the capture, wake and VAD
    path is exercised exactly as it is with a real recogniser.
    """

    def __init__(self, *, drain: bool = True) -> None:
        self._pending: asyncio.Queue[Transcript] = asyncio.Queue()
        self._drain = drain

    def submit(self, text: str, confidence: float | None = None) -> None:
        self._pending.put_nowait(Transcript(text, confidence))

    async def transcribe(self, frames: AsyncIterator[bytes]) -> Transcript:
        if self._drain:
            async for _ in frames:
                if not self._pending.empty():
                    break
        return await self._pending.get()


def make_stt(config: SttConfig, *, sample_rate: int) -> Stt:
    """Select the recogniser from configuration alone (principle 6)."""
    if config.backend == "sherpa":
        return SherpaStt(config.model_dir, sample_rate=sample_rate)
    if config.backend in ("text", "mock"):
        return QueueStt()
    raise ConfigError(
        '[stt] backend="openai_http" is A32\'s box-speech flip, which is off on day '
        "one and enabled at G3b; v1 ships sherpa, text and mock."
    )
