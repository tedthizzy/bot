"""The audio adapters and the filler tiers.

Principle 6: which backend exists is decided by ``config/robot.toml`` and
nothing else, and every Pi-only import stays inside the adapter -- these tests
run on a Mac with no sounddevice, no sherpa-onnx and no piper.
"""

from __future__ import annotations

import asyncio
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.audio.capture import (  # noqa: E402
    FRAME_BYTES,
    RING_SECONDS,
    SAMPLE_RATE,
    AudioRing,
    SilentCapture,
    SoundDeviceCapture,
    WavCapture,
    make_capture,
)
from rover_brain.audio.stt import QueueStt, make_stt  # noqa: E402
from rover_brain.audio.tts import (  # noqa: E402
    HalfDuplexTts,
    NullTts,
    PiperTts,
    SayTts,
    make_tts,
    voice_sample_rate,
)
from rover_brain.audio.vad import NullVad, make_vad  # noqa: E402
from rover_brain.audio.wake import HotkeyWake, NoWake, make_wake  # noqa: E402
from rover_brain.filler import (  # noqa: E402
    EARCON_HZ,
    EARCON_MS,
    WAITING,
    FillerPlayer,
    completion_sentence,
    reason_sentence,
    tone_pcm,
)
from rover_contracts.config import (  # noqa: E402
    AudioConfig,
    ConfigError,
    SttConfig,
    TtsConfig,
    VadConfig,
    WakeConfig,
)
from rover_contracts.messages import ResultReason, ResultStatus  # noqa: E402

# -- capture -----------------------------------------------------------------


def test_the_frame_is_twenty_milliseconds_of_s16le_mono() -> None:
    assert SAMPLE_RATE == 16000
    assert FRAME_BYTES == 640


def test_the_ring_holds_three_seconds_and_drops_the_oldest() -> None:
    ring = AudioRing()
    frames = RING_SECONDS * 50
    for index in range(frames + 5):
        ring.push(bytes([index % 256]) * FRAME_BYTES)
    assert len(ring) == frames
    assert ring.dropped == 5
    assert ring.pop() is not None
    ring.clear()
    assert ring.pop() is None


@pytest.mark.parametrize(
    ("mode", "expected"),
    [("text", SilentCapture), ("wake", SoundDeviceCapture), ("ptt", SoundDeviceCapture)],
)
def test_the_microphone_is_chosen_by_configuration(mode: str, expected: type) -> None:
    assert isinstance(make_capture(AudioConfig(input=mode)), expected)


def test_a_recording_needs_a_file_and_says_so() -> None:
    with pytest.raises(ConfigError, match="--wav"):
        make_capture(AudioConfig(input="wav"))
    assert isinstance(
        make_capture(AudioConfig(input="wav"), wav_path="x.wav"), WavCapture
    )


@pytest.mark.asyncio
async def test_the_dev_microphone_produces_paced_silence() -> None:
    capture = SilentCapture()
    await capture.start()
    frames = []
    async for frame in capture.frames():
        frames.append(frame)
        if len(frames) == 3:
            await capture.stop()
    assert [len(frame) for frame in frames] == [FRAME_BYTES] * 3


# -- voice activity, recognition, wake ---------------------------------------


def test_the_detector_is_null_when_the_boundary_comes_from_elsewhere() -> None:
    for mode in ("text", "ptt"):
        vad = make_vad(VadConfig(), AudioConfig(input=mode), sample_rate=SAMPLE_RATE)
        assert isinstance(vad, NullVad)
        assert vad.accept(bytes(FRAME_BYTES)) is None


@pytest.mark.asyncio
async def test_the_dev_recogniser_answers_what_was_submitted() -> None:
    stt = make_stt(SttConfig(backend="text"), sample_rate=SAMPLE_RATE)
    assert isinstance(stt, QueueStt)
    stt.submit("turn left ninety degrees")

    async def frames():
        for _ in range(2):
            yield bytes(FRAME_BYTES)

    transcript = await stt.transcribe(frames())
    assert transcript.text == "turn left ninety degrees"
    assert transcript.confidence is None  # a null counts as authorized


def test_the_box_speech_backends_are_named_as_the_g3b_flip() -> None:
    with pytest.raises(ConfigError, match="G3b"):
        make_stt(SttConfig(backend="openai_http"), sample_rate=SAMPLE_RATE)
    with pytest.raises(ConfigError, match="G3b"):
        make_tts(TtsConfig(backend="openai_http"), is_moving=lambda: False)


def test_the_wake_word_is_chosen_by_configuration() -> None:
    assert isinstance(make_wake(WakeConfig(backend="hotkey")), HotkeyWake)
    assert isinstance(make_wake(WakeConfig(backend="none")), NoWake)
    with pytest.raises(ConfigError, match="open item 11"):
        make_wake(WakeConfig(backend="pymicro"))


# -- synthesis (A28, A29) ----------------------------------------------------


def test_piper_is_a_subprocess_and_is_never_imported() -> None:
    assert "piper" not in sys.modules
    assert PiperTts("/bin/true", "v-low.onnx")._aplay_argv(16000)[0] == "aplay"


def test_the_configured_output_device_reaches_aplay() -> None:
    """OP-15: without -D, playback goes to ALSA's default -- card 0 on a Pi 4,
    which is vc4-hdmi or the headphone jack, not the ReSpeaker Lite's amp."""
    default = PiperTts("/bin/true", "v-low.onnx")._aplay_argv(16000)
    assert "-D" not in default
    named = PiperTts(
        "/bin/true", "v-low.onnx", output_device="plughw:CARD=Lite,DEV=0"
    )._aplay_argv(16000)
    assert named[named.index("-D") + 1] == "plughw:CARD=Lite,DEV=0"


def test_a_low_voice_is_sixteen_kilohertz() -> None:
    assert voice_sample_rate("en_US-lessac-low") == 16000
    assert voice_sample_rate("en_US-lessac-medium") == 22050


def test_the_rate_is_read_off_an_absolute_voice_path() -> None:
    """OP-4: [tts] voice is the path piper -m is given, so the -low suffix that
    selects 16 kHz has to survive the directory and the .onnx extension."""
    assert voice_sample_rate("/data/models/tts/en_US-lessac-low.onnx") == 16000
    assert voice_sample_rate("/data/models/tts/en_US-lessac-medium.onnx") == 22050


def test_the_synthesiser_is_chosen_by_configuration(tmp_path: Path) -> None:
    say = make_tts(TtsConfig(backend="say"), is_moving=lambda: False)
    assert isinstance(say, HalfDuplexTts)
    with pytest.raises(ConfigError, match="A28"):
        make_tts(TtsConfig(backend="piper", bin="/nope/piper"), is_moving=lambda: False)
    binary = tmp_path / "piper"
    binary.write_text("#!/bin/sh\n")
    # A voice piper cannot open is a config error, not twenty silent turns:
    # install.sh downloads it to a directory nothing used to read.
    with pytest.raises(ConfigError, match=r"\[tts\] voice"):
        make_tts(
            TtsConfig(backend="piper", bin=str(binary), voice=str(tmp_path / "no.onnx")),
            is_moving=lambda: False,
        )
    voice = tmp_path / "en_US-lessac-low.onnx"
    voice.write_bytes(b"onnx")
    tts = make_tts(
        TtsConfig(backend="piper", bin=str(binary), voice=str(voice)),
        is_moving=lambda: False,
    )
    assert isinstance(tts, HalfDuplexTts)


@pytest.mark.asyncio
async def test_nothing_is_spoken_while_the_wheels_are_turning() -> None:
    inner = NullTts()
    moving = True
    gate = HalfDuplexTts(inner, lambda: moving)
    await gate.speak("I am driving")
    assert inner.spoken == [] and gate.suppressed == 1

    moving = False
    await gate.speak("I have stopped")
    assert inner.spoken == ["I have stopped"]


@pytest.mark.asyncio
async def test_a_sentence_is_cut_short_when_the_wheels_start() -> None:
    """A29 states the rule as a state condition, not an entry check: 300 chars
    of `say` is ~20 s of audio, and a teleop twist can start a drive inside it."""

    class SlowTts(NullTts):
        def __init__(self) -> None:
            super().__init__()
            self.finished = False

        async def speak(self, text: str) -> None:
            await asyncio.sleep(5.0)
            self.finished = True
            await super().speak(text)

    inner = SlowTts()
    moving = False
    gate = HalfDuplexTts(inner, lambda: moving, poll_s=0.01)
    task = asyncio.create_task(gate.speak("a very long sentence"))
    await asyncio.sleep(0.05)
    moving = True
    await asyncio.wait_for(task, 1.0)
    assert not inner.finished, "the sentence kept playing after the wheels started"
    assert gate.interrupted == 1


@pytest.mark.asyncio
async def test_the_tone_is_not_speech_and_plays_regardless() -> None:
    """A31: the acknowledgement is an earcon, so the half-duplex rule that
    silences sentences does not silence it."""
    inner = NullTts()
    gate = HalfDuplexTts(inner, lambda: True)
    await gate.play_pcm(tone_pcm(EARCON_HZ, ms=EARCON_MS, rate=16000), 16000)
    assert inner.spoken == []


def test_the_earcon_is_pure_and_the_right_length() -> None:
    pcm = tone_pcm(EARCON_HZ, ms=EARCON_MS, rate=16000)
    assert pcm == tone_pcm(EARCON_HZ, ms=EARCON_MS, rate=16000)
    assert len(pcm) == 2 * len(EARCON_HZ) * int(16000 * EARCON_MS / 1000)
    samples = struct.unpack(f"<{len(pcm) // 2}h", pcm)
    assert samples[0] == 0  # faded in, so it does not click
    assert max(abs(sample) for sample in samples) <= 32767
    with pytest.raises(ValueError, match="amplitude"):
        tone_pcm(EARCON_HZ, ms=10, rate=16000, amplitude=2.0)


# -- the filler tiers (A31) --------------------------------------------------


@pytest.mark.asyncio
async def test_tier_one_is_a_tone_and_tier_two_waits() -> None:
    class Recorder(NullTts):
        def __init__(self) -> None:
            super().__init__()
            self.tones = 0

        async def play_pcm(self, pcm: bytes, rate: int) -> None:
            self.tones += 1

    tts = Recorder()
    player = FillerPlayer(tts, delay_s=0.01)
    await player.ack()
    assert tts.tones == 1 and tts.spoken == []

    player.start_waiting()
    await asyncio.sleep(0.05)
    assert tts.spoken == [WAITING]


@pytest.mark.asyncio
async def test_the_waiting_line_is_cancelled_the_moment_the_plan_arrives() -> None:
    tts = NullTts()
    player = FillerPlayer(tts, delay_s=0.05)
    player.start_waiting()
    await asyncio.sleep(0.005)
    player.stop()
    await asyncio.sleep(0.08)
    assert tts.spoken == []


def test_completion_speech_is_a_table_driven_by_the_result() -> None:
    assert completion_sentence(ResultStatus.DONE, ResultReason.NONE) == "Done."
    assert completion_sentence(ResultStatus.ABORTED, ResultReason.OBSTACLE) == (
        "Something is in the way."
    )
    assert completion_sentence(ResultStatus.REJECTED, ResultReason.RATE_LIMITED) == (
        "Give me a moment before the next move."
    )
    assert reason_sentence(ResultReason.ESTOP_ACTIVE) == "The emergency stop is on."


def test_every_result_status_and_reason_has_a_sentence() -> None:
    for status in ResultStatus:
        assert completion_sentence(status, ResultReason.NONE)
    for reason in ResultReason:
        assert completion_sentence(ResultStatus.ABORTED, reason)
        assert reason_sentence(reason)


def test_the_dev_synthesiser_needs_nothing_installed() -> None:
    assert SayTts().sample_rate == 16000
