"""robotd's JSONL logs (A34, principle 7).

Logs are datasets, not diagnostics: the telemetry log carries the ``T`` frame's
own fields -- **raw ticks and the controller's microseconds** -- rather than the
pose robotd happened to integrate from them, so a pose is recomputable offline
and an integration bug is discoverable after the fact.  Nothing here compares an
MCU microsecond against a host nanosecond (I-17); both are recorded side by side
and neither is subtracted from the other.

Three streams, one file each per UTC day, appended a line at a time:
``telemetry`` (decimated to ``[log] state_decimate_hz``), ``commands`` (every
accepted or refused command and its result) and ``events``.
"""

from __future__ import annotations

import logging
import time
from dataclasses import fields as dataclass_fields
from pathlib import Path
from typing import Any, Final

from pydantic import BaseModel
from rover_contracts.jsonl import JsonlWriter
from rover_contracts.serial_codec import TelemetryFrame

__all__ = ["RobotdLog"]

log = logging.getLogger("rover.robotd.log")

_STREAMS: Final = ("telemetry", "commands", "events")


class RobotdLog:
    """Append-only JSONL, rotated by UTC date."""

    def __init__(self, directory: str | Path, *, state_decimate_hz: int = 5) -> None:
        self.directory = Path(directory)
        self.period_ns = int(1e9 / max(1, state_decimate_hz))
        self._writers: dict[str, JsonlWriter] = {}
        self._day: str | None = None
        self._last_telemetry_ns = 0

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

    def telemetry(self, frame: TelemetryFrame, arrival_mono_ns: int) -> bool:
        """Record one ``T``, decimated.  Returns whether it was written."""
        if arrival_mono_ns - self._last_telemetry_ns < self.period_ns:
            return False
        self._last_telemetry_ns = arrival_mono_ns
        record: dict[str, Any] = {
            "stream": "telemetry",
            "recv_mono_ns": arrival_mono_ns,
            "t_utc_ns": time.time_ns(),
        }
        record.update(
            {field.name: getattr(frame, field.name) for field in dataclass_fields(frame)}
        )
        self._writer("telemetry").write(record)
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
