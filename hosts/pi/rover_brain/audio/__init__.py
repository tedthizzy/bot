"""The four audio adapters, each with a real backend and a dev backend.

Every Pi-only dependency -- ``sounddevice``, ``sherpa-onnx``,
``pyopen-wakeword``, the ``piper`` binary -- is imported inside the adapter
that needs it, and which adapter is built is decided by ``config/robot.toml``
alone.  Fakes are configuration, never a code branch (principle 6).
"""

from __future__ import annotations

from rover_brain.audio.capture import (
    FRAME_BYTES,
    PERIOD_MS,
    SAMPLE_RATE,
    Capture,
    make_capture,
)
from rover_brain.audio.stt import Stt, Transcript, make_stt
from rover_brain.audio.tts import Tts, make_tts
from rover_brain.audio.vad import Vad, VadEvent, make_vad
from rover_brain.audio.wake import Wake, make_wake

__all__ = [
    "FRAME_BYTES",
    "PERIOD_MS",
    "SAMPLE_RATE",
    "Capture",
    "Stt",
    "Transcript",
    "Tts",
    "Vad",
    "VadEvent",
    "Wake",
    "make_capture",
    "make_stt",
    "make_tts",
    "make_vad",
    "make_wake",
]
