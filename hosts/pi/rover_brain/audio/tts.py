"""Speech synthesis, behind one adapter chosen by ``[tts] backend``.

A28: Piper is a **subprocess** (``piper --output-raw``), never imported.  That
boundary is what keeps a GPL-3.0-or-later synthesiser out of an Apache-2.0
repo, so this module runs it, reads its raw output in chunks and feeds a
player; nothing here imports ``piper``.

A29/ARCHITECTURE 4.3: brain must never play TTS while the rover reports
motion.  That is enforced once, in :class:`HalfDuplexTts`, rather than
repeated in each backend.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import shutil
import tempfile
import wave
from collections.abc import Callable
from pathlib import Path
from typing import Final, Protocol

from rover_contracts.config import ConfigError, TtsConfig

log = logging.getLogger("rover.brain.tts")

__all__ = [
    "HalfDuplexTts",
    "NullTts",
    "PiperTts",
    "SayTts",
    "Tts",
    "make_tts",
    "voice_sample_rate",
]

_CHUNK_BYTES: Final = 4096
"""How much of the synthesiser's raw output is read at a time (A28: "reading
its raw output in chunks")."""

_LOW_VOICE_HZ: Final = 16000
_OTHER_VOICE_HZ: Final = 22050

_MOTION_POLL_S: Final = 0.05
"""How often HalfDuplexTts re-reads the rover's motion flag while a sentence plays."""


def voice_sample_rate(voice: str) -> int:
    """Piper's own naming convention: a ``-low`` voice is 16 kHz, the rest
    22.05 kHz.  The raw stream carries no header, so the player has to be told,
    and a wrong rate is audible as a pitch shift rather than an error.

    ``[tts] voice`` is an absolute path to the ``.onnx``, so the suffix is read
    off the stem: ``/data/models/tts/en_US-lessac-low.onnx`` is still ``-low``.
    """
    stem = Path(voice).name
    for suffix in (".onnx.json", ".onnx"):
        if stem.endswith(suffix):
            stem = stem[: -len(suffix)]
            break
    return _LOW_VOICE_HZ if stem.endswith("-low") else _OTHER_VOICE_HZ


class Tts(Protocol):
    """What the FSM, the filler player and ``say`` need from a synthesiser."""

    @property
    def sample_rate(self) -> int:
        """Rate of the PCM :meth:`play_pcm` expects, in hertz."""

    async def speak(self, text: str) -> None:
        """Say one sentence.  Returns when the audio has finished playing, and
        is cancellable: cancelling stops the sound."""

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        """Play signed 16-bit little-endian mono PCM (the earcon)."""


async def _run(argv: list[str], *, stdin: bytes | None = None) -> None:
    """Run a player to completion, killing it if we are cancelled."""
    proc = await asyncio.create_subprocess_exec(
        *argv,
        stdin=(
            asyncio.subprocess.PIPE if stdin is not None else asyncio.subprocess.DEVNULL
        ),
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
    )
    try:
        await proc.communicate(stdin)
    except asyncio.CancelledError:
        with contextlib.suppress(ProcessLookupError):
            proc.kill()
        raise


class PiperTts:
    """``piper --output-raw`` piped chunk by chunk into ``aplay`` (A28).

    ``[tts] threads`` reaches Piper as ``OMP_NUM_THREADS``: the binary exposes
    no thread flag this design can pin, and the ONNX runtime under it honours
    the environment variable.
    """

    def __init__(
        self,
        binary: str,
        voice: str,
        *,
        threads: int = 1,
        output_device: str = "",
    ) -> None:
        self._binary = binary
        self._voice = voice
        self._threads = threads
        self._device = output_device
        self._rate = voice_sample_rate(voice)

    @property
    def sample_rate(self) -> int:
        return self._rate

    async def speak(self, text: str) -> None:
        if not text.strip():
            return
        env = dict(os.environ, OMP_NUM_THREADS=str(self._threads))
        piper = await asyncio.create_subprocess_exec(
            self._binary,
            "-m",
            self._voice,
            "--output-raw",
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            # Captured, not discarded: a missing voice makes piper exit at once
            # with a message, and with stderr on DEVNULL and the return code
            # unread every spoken sentence was silent with nothing in the log.
            stderr=asyncio.subprocess.PIPE,
            env=env,
        )
        player = await asyncio.create_subprocess_exec(
            *self._aplay_argv(self._rate),
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        assert piper.stdin is not None and piper.stdout is not None
        assert player.stdin is not None
        try:
            piper.stdin.write(text.encode("utf-8"))
            await piper.stdin.drain()
            piper.stdin.close()
            while chunk := await piper.stdout.read(_CHUNK_BYTES):
                player.stdin.write(chunk)
                await player.stdin.drain()
            player.stdin.close()
            await player.wait()
            code = await piper.wait()
            if code != 0:
                assert piper.stderr is not None
                detail = (await piper.stderr.read()).decode("utf-8", "replace")
                log.error(
                    "piper exited %d for voice %s: %s",
                    code,
                    self._voice,
                    detail.strip()[:400] or "(no output)",
                )
        except (asyncio.CancelledError, BrokenPipeError, ConnectionResetError):
            for proc in (piper, player):
                with contextlib.suppress(ProcessLookupError):
                    proc.kill()
            raise

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        await _run(self._aplay_argv(rate), stdin=pcm)

    def _aplay_argv(self, rate: int) -> list[str]:
        # Without -D, playback goes to ALSA's default device -- card 0 on a
        # Pi 4, which is vc4-hdmi or the headphone jack, not the ReSpeaker
        # Lite's amp.  [audio] output_device names the card.
        device = ["-D", self._device] if self._device else []
        return [
            "aplay", "-q", *device,
            "-t", "raw", "-f", "S16_LE", "-c", "1", "-r", str(rate), "-",
        ]


class SayTts:
    """The dev backend: macOS ``say``, which needs nothing installed.

    Raw PCM has no command-line player on a stock macOS, so the earcon is
    written to a temporary WAV with the stdlib and handed to ``afplay``.
    """

    def __init__(self, *, rate: int = _LOW_VOICE_HZ, voice: str | None = None) -> None:
        self._rate = rate
        self._voice = voice

    @property
    def sample_rate(self) -> int:
        return self._rate

    async def speak(self, text: str) -> None:
        if not text.strip():
            return
        argv = ["say"]
        if self._voice:
            argv += ["-v", self._voice]
        await _run([*argv, text])

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        with tempfile.TemporaryDirectory(prefix="rover-tone-") as directory:
            path = Path(directory) / "earcon.wav"
            with wave.open(str(path), "wb") as handle:
                handle.setnchannels(1)
                handle.setsampwidth(2)
                handle.setframerate(rate)
                handle.writeframes(pcm)
            await _run(["afplay", str(path)])


class NullTts:
    """``[tts] backend="null"``: everything is exercised except the sound."""

    def __init__(self, *, rate: int = _LOW_VOICE_HZ) -> None:
        self._rate = rate
        self.spoken: list[str] = []

    @property
    def sample_rate(self) -> int:
        return self._rate

    async def speak(self, text: str) -> None:
        self.spoken.append(text)

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        return None


class HalfDuplexTts:
    """A29: no TTS plays while the rover reports ``motion``.

    The earcon is not speech and is left alone -- it is 140 ms and plays during
    PLANNING, when nothing is moving -- but every spoken sentence goes through
    this gate, so a ``say`` that arrives while a teleop stream is driving is
    dropped rather than blinding the recognizer.

    A29 states the rule as a *state* condition, not an entry check: 300 chars of
    ``say`` is ~20 s of audio, and a teleop twist can start a drive in the middle
    of it.  So the sentence is raced against a poller and cut short the moment
    the wheels turn.  brain's own path never needs this -- A31 dispatches only
    after the sentence finishes -- which is exactly why it was easy to miss.
    """

    def __init__(
        self,
        inner: Tts,
        is_moving: Callable[[], bool],
        *,
        poll_s: float = _MOTION_POLL_S,
    ) -> None:
        self._inner = inner
        self._is_moving = is_moving
        self._poll_s = poll_s
        self.suppressed = 0
        self.interrupted = 0

    @property
    def sample_rate(self) -> int:
        return self._inner.sample_rate

    async def speak(self, text: str) -> None:
        if self._is_moving():
            self.suppressed += 1
            return
        speaking = asyncio.ensure_future(self._inner.speak(text))
        watching = asyncio.ensure_future(self._until_moving())
        try:
            await asyncio.wait(
                (speaking, watching), return_when=asyncio.FIRST_COMPLETED
            )
        finally:
            watching.cancel()
            if not speaking.done():
                # PiperTts kills both subprocesses on cancellation.
                self.interrupted += 1
                speaking.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await asyncio.gather(speaking, watching, return_exceptions=False)

    async def _until_moving(self) -> None:
        while not self._is_moving():
            await asyncio.sleep(self._poll_s)

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        await self._inner.play_pcm(pcm, rate)


def make_tts(
    config: TtsConfig,
    *,
    is_moving: Callable[[], bool],
    output_device: str = "",
) -> Tts:
    """Select the synthesiser from configuration alone (principle 6)."""
    inner: Tts
    if config.backend == "piper":
        if shutil.which(config.bin) is None and not Path(config.bin).exists():
            raise ConfigError(
                f"[tts] bin {config.bin!r} does not exist; A28 installs piper into "
                "its own venv at /opt/rover/.venv-tts"
            )
        if not Path(config.voice).exists():
            raise ConfigError(
                f"[tts] voice {config.voice!r} does not exist; install.sh "
                "downloads it into /data/models/tts"
            )
        inner = PiperTts(
            config.bin,
            config.voice,
            threads=config.threads,
            output_device=output_device,
        )
    elif config.backend == "say":
        inner = SayTts()
    elif config.backend == "null":
        inner = NullTts()
    else:  # "openai_http"
        raise ConfigError(
            '[tts] backend="openai_http" is A32\'s box-speech flip, which is off on '
            "day one and enabled at G3b; v1 ships piper, say and null."
        )
    return HalfDuplexTts(inner, is_moving)
