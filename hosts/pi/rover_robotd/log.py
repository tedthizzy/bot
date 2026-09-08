"""robotd's JSONL logs (A34, principle 7).

Logs are datasets, not diagnostics: the feedback log carries the controller's
own fields -- the applied powers, the fused attitude, the bus voltage and the
fork's stop flags -- beside the host's monotonic arrival stamp, rather than
anything robotd derived from them, so a heading rate or a battery estimate is
recomputable offline and a derivation bug is discoverable after the fact.

Three streams, one file each per UTC day, appended a line at a time:
``feedback`` (decimated to ``[log] state_decimate_hz``), ``commands`` (every
accepted or refused command and its result) and ``events``.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Final

from pydantic import BaseModel
from rover_contracts.jsonl import JsonlWriter
from rover_contracts.wave_proto import Feedback

__all__ = ["RobotdLog"]

log = logging.getLogger("rover.robotd.log")

_STREAMS: Final = ("feedback", "commands", "events")


class RobotdLog:
    """Append-only JSONL, rotated by UTC date."""

    def __init__(self, directory: str | Path, *, state_decimate_hz: int = 5) -> None:
        self.directory = Path(directory)
        self.period_ns = int(1e9 / max(1, state_decimate_hz))
        self._writers: dict[str, JsonlWriter] = {}
        self._day: str | None = None
        self._last_feedback_ns = 0

    def _writer(self, stream: str) -> JsonlWriter:
        day = time.strftime("%Y%m%d", time.gmtime())
        if day != self._day:
            self.close()
            self._day = day
        writer = self._writers.get(stream)
        if writer is None:
            writer = JsonlWriter(self.directory / f"{stream}-{day}.jsonl")
            self._writers[stream] = writer
        return writer

    def feedback(self, feedback: Feedback, arrival_mono_ns: int) -> bool:
        """Record one feedback line, decimated.  Returns whether it was written."""
        if arrival_mono_ns - self._last_feedback_ns < self.period_ns:
            return False
        self._last_feedback_ns = arrival_mono_ns
        record: dict[str, Any] = {
            "stream": "feedback",
            "recv_mono_ns": arrival_mono_ns,
            "t_utc_ns": time.time_ns(),
            "left": feedback.left,
            "right": feedback.right,
            "roll_deg": feedback.roll_deg,
            "pitch_deg": feedback.pitch_deg,
            "yaw_deg": feedback.yaw_deg,
            "temp_c": feedback.temp_c,
            "bus_v": feedback.bus_v,
            "hb": feedback.hb,
            "st": None if feedback.st is None else int(feedback.st),
            "tof_mm": feedback.tof_mm,
            "bumper": feedback.bumper,
            "clamp_count": feedback.clamp_count,
        }
        self._writer("feedback").write(record)
        return True

    def command(self, record: dict[str, Any]) -> None:
        """Record one command decision: what arrived and what was answered."""
        self._writer("commands").write({"t_utc_ns": time.time_ns(), **record})

    def message(self, stream: str, message: BaseModel) -> None:
        """Record a published bus message verbatim."""
        if stream not in _STREAMS:  # pragma: no cover - programmer error
            raise ValueError(f"unknown log stream {stream!r}")
        self._writer(stream).write(message)

    def event(self, message: BaseModel) -> None:
        self.message("events", message)

    def close(self) -> None:
        for writer in self._writers.values():
            writer.close()
        self._writers.clear()
