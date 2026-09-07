"""Append-only JSONL, and the one atomic replace the scene ring needs.

A34: JSONL only, no SQLite.  Logs and LeKiwi-keyed episodes are appended a line
at a time and never rewritten, so a crash truncates at most the last line and a
reader can tail the file while robotd is still writing it.

Non-finite floats are refused rather than written: ``NaN`` and ``Infinity`` are
not JSON, and a log that no strict parser can read is not a dataset
(principle 7).
"""

from __future__ import annotations

import json
import os
from collections.abc import Iterable, Iterator, Mapping
from pathlib import Path
from types import TracebackType
from typing import Any

from pydantic import BaseModel

__all__ = ["JsonlWriter", "atomic_write_jsonl", "read_jsonl", "to_json_line"]


def _plain(record: Mapping[str, Any] | BaseModel) -> Mapping[str, Any]:
    if isinstance(record, BaseModel):
        return record.model_dump(mode="json")
    return record


def to_json_line(record: Mapping[str, Any] | BaseModel) -> str:
    """One record as a newline-terminated JSON line.

    Key order is the record's own, so a pydantic model's declared field order
    survives into the log.  Raises ``ValueError`` for a non-finite float.
    """
    return (
        json.dumps(
            _plain(record),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        + "\n"
    )


class JsonlWriter:
    """An append-only JSONL file.

    There is deliberately no per-line ``fsync``: on the Pi the log directory is
    an SD card, where one fsync is 1-20 ms and hundreds of milliseconds during a
    card garbage-collection cycle.  robotd writes these from the 20 Hz control
    loop, which also owns the setpoint cell, and 4.2 forbids that loop to block
    on anything.  Durability is taken once, at :meth:`sync`.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8", newline="\n")

    def write(self, record: Mapping[str, Any] | BaseModel) -> None:
        """Append one record.  A rejected record leaves the file untouched."""
        line = to_json_line(record)
        self._file.write(line)
        self._file.flush()

    def sync(self) -> None:
        """Force the file to storage.  Blocking: never call it from a loop that
        also has to keep a deadline."""
        if not self._file.closed:
            self._file.flush()
            os.fsync(self._file.fileno())

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def __enter__(self) -> JsonlWriter:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


def read_jsonl(path: str | Path) -> Iterator[dict[str, Any]]:
    """Yield each record.  Blank lines are skipped; a truncated final line
    raises, because silently dropping it would hide a crash."""
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped:
                yield json.loads(stripped)


def atomic_write_jsonl(
    path: str | Path, records: Iterable[Mapping[str, Any] | BaseModel]
) -> None:
    """Replace a whole JSONL file in one step.

    The scene ring is rewritten rather than appended, so a reader must never see
    it half-written: the new contents go to a temporary file in the same
    directory, are flushed and fsynced, and then replace the old name.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + ".tmp")
    with tmp.open("w", encoding="utf-8", newline="\n") as handle:
        for record in records:
            handle.write(to_json_line(record))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, target)
