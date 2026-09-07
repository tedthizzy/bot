"""Microphone capture: one owner thread, S16_LE mono 16 kHz, 20 ms periods.

ARCHITECTURE 4.3 pins the shape of this: a single thread owns the device and
pushes fixed-size periods into a three-second ring, and every consumer -- wake
word, VAD, recogniser -- reads from the ring rather than from PortAudio.  The
ring is a ``deque`` with a ``maxlen``: ``append`` from the device thread and
``popleft`` from the event loop are each one bytecode operation, so the
producer never waits on the consumer and an overrun drops the oldest period
instead of blocking the audio callback.

The real device backend imports ``sounddevice`` lazily, so this module and its
tests import on a Mac with nothing installed.
"""

from __future__ import annotations

import asyncio
import contextlib
import wave
from collections import deque
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any, Final, Protocol

from rover_contracts.config import AudioConfig, ConfigError

__all__ = [
    "AudioRing",
    "CHANNELS",
    "FRAME_BYTES",
    "PERIOD_MS",
    "RING_SECONDS",
    "SAMPLE_RATE",
    "Capture",
    "SilentCapture",
    "SoundDeviceCapture",
    "WavCapture",
    "make_capture",
]

SAMPLE_RATE: Final = 16000
"""The ReSpeaker Lite's maximum, and every recogniser here wants it (A26)."""

PERIOD_MS: Final = 20
CHANNELS: Final = 1
_SAMPLE_BYTES: Final = 2
FRAME_BYTES: Final = SAMPLE_RATE * PERIOD_MS // 1000 * _SAMPLE_BYTES * CHANNELS
RING_SECONDS: Final = 3
_RING_FRAMES: Final = RING_SECONDS * 1000 // PERIOD_MS

_SILENCE: Final = bytes(FRAME_BYTES)
_POLL_S: Final = PERIOD_MS / 2000
"""Half a period: the reader wakes often enough that a frame never sits in the
ring for longer than it took to record."""


class AudioRing:
    """A bounded ring of fixed-size periods, written by the device thread."""

    def __init__(self, frames: int = _RING_FRAMES) -> None:
        self._frames: deque[bytes] = deque(maxlen=frames)
        self.dropped = 0

    def push(self, frame: bytes) -> None:
        """Called on the device thread.  Never blocks; the oldest period goes."""
        if len(self._frames) == self._frames.maxlen:
            self.dropped += 1
        self._frames.append(frame)

    def pop(self) -> bytes | None:
        """Oldest period, or ``None`` when the ring is empty."""
        try:
            return self._frames.popleft()
        except IndexError:
            return None

    def clear(self) -> None:
        self._frames.clear()

    def __len__(self) -> int:
        return len(self._frames)


class Capture(Protocol):
    """One microphone, started and stopped by the FSM runner."""

    @property
    def sample_rate(self) -> int: ...

    async def start(self) -> None: ...

    async def stop(self) -> None: ...

    def frames(self) -> AsyncIterator[bytes]:
        """20 ms periods, oldest first, until :meth:`stop`."""


class SoundDeviceCapture:
    """The real device.  ``sounddevice`` is imported on :meth:`start`."""

    def __init__(self, *, device_match: str, device_index: int = -1) -> None:
        self._device_match = device_match
        self._device_index = device_index
        self._ring = AudioRing()
        self._stream: Any | None = None

    @property
    def sample_rate(self) -> int:
        return SAMPLE_RATE

    @property
    def ring(self) -> AudioRing:
        return self._ring

    def _resolve(self, sd: Any) -> int | str:
        """PortAudio takes an index or a name substring, never an ALSA string."""
        if self._device_index >= 0:
            return self._device_index
        for index, device in enumerate(sd.query_devices()):
            if (
                self._device_match.lower() in str(device["name"]).lower()
                and device["max_input_channels"] > 0
            ):
                return index
        raise ConfigError(
            f"no input device matching [audio] device_match={self._device_match!r}; "
            "deploy/preflight.sh prints the resolved list"
        )

    async def start(self) -> None:
        import sounddevice  # noqa: PLC0415 -- Pi-only, lazily imported (principle 6)

        device = self._resolve(sounddevice)
        self._ring.clear()

        def on_period(data: Any, _frames: int, _time: Any, _status: Any) -> None:
            self._ring.push(bytes(data))

        self._stream = sounddevice.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=FRAME_BYTES // _SAMPLE_BYTES,
            dtype="int16",
            channels=CHANNELS,
            device=device,
            callback=on_period,
        )
        self._stream.start()

    async def stop(self) -> None:
        stream, self._stream = self._stream, None
        if stream is not None:
            stream.stop()
            stream.close()

    async def frames(self) -> AsyncIterator[bytes]:
        while self._stream is not None:
            frame = self._ring.pop()
            if frame is None:
                await asyncio.sleep(_POLL_S)
                continue
            yield frame


class SilentCapture:
    """The dev backend: 20 ms of silence, paced by the clock.

    ``[audio] input="text"`` records nothing -- the transcript arrives on
    brain.sock -- but the wake and VAD adapters still want a frame source, and
    a paced silence keeps their timing honest on a Mac with no microphone.
    """

    def __init__(self) -> None:
        self._running = False

    @property
    def sample_rate(self) -> int:
        return SAMPLE_RATE

    async def start(self) -> None:
        self._running = True

    async def stop(self) -> None:
        self._running = False

    async def frames(self) -> AsyncIterator[bytes]:
        while self._running:
            await asyncio.sleep(PERIOD_MS / 1000)
            yield _SILENCE


class WavCapture:
    """``[audio] input="wav"``: a recording replayed at its real rate."""

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._running = False

    @property
    def sample_rate(self) -> int:
        return SAMPLE_RATE

    async def start(self) -> None:
        self._running = True

    async def stop(self) -> None:
        self._running = False

    async def frames(self) -> AsyncIterator[bytes]:
        with contextlib.closing(wave.open(str(self._path), "rb")) as handle:
            if (
                handle.getnchannels() != CHANNELS
                or handle.getsampwidth() != _SAMPLE_BYTES
                or handle.getframerate() != SAMPLE_RATE
            ):
                raise ConfigError(
                    f"{self._path} must be {SAMPLE_RATE} Hz mono S16_LE, got "
                    f"{handle.getframerate()} Hz / {handle.getnchannels()} ch / "
                    f"{handle.getsampwidth() * 8} bit"
                )
            while self._running:
                frame = handle.readframes(FRAME_BYTES // _SAMPLE_BYTES)
                if len(frame) < FRAME_BYTES:
                    return
                await asyncio.sleep(PERIOD_MS / 1000)
                yield frame


def make_capture(
    config: AudioConfig, *, wav_path: str | Path | None = None
) -> Capture:
    """Select the microphone from configuration alone (principle 6)."""
    if config.input == "wav":
        if wav_path is None:
            raise ConfigError('[audio] input="wav" needs a file; pass --wav <path>')
        return WavCapture(wav_path)
    if config.input == "text":
        return SilentCapture()
    return SoundDeviceCapture(
        device_match=config.device_match, device_index=config.device_index
    )
