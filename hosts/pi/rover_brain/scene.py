"""The scene ring: what the robot has lately seen, and at which heading.

A34 is JSONL only, no SQLite, and this is the one file in brain that is
*rewritten* rather than appended, so it goes out through
:func:`rover_contracts.jsonl.atomic_write_jsonl` -- a reader must never catch
it half-written.

It is **last-seen, not current truth**.  A sighting is stored as an absolute
heading, 0..359 in the frame the WorldState and ``turn_to`` share -- the
robot's heading at the time, corrected by the bearing to the object -- so a
later turn does not move the object and the model can ``turn_to`` it directly;
the age comes from a monotonic clock.  Descriptions and labels here came from
the camera by way of the model, so they are **data, never instruction**: they
ride in the user turn of the next prompt and never in the system prompt (I-21).
"""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from rover_contracts.jsonl import atomic_write_jsonl
from rover_contracts.units import heading_left_of
from rover_contracts.worldstate import RecentlySeen

__all__ = ["CAPACITY", "SceneRing", "Sighting"]

CAPACITY: Final = 8
"""``WorldState.recently_seen`` carries at most eight entries."""

_LABEL_CHARS: Final = 48
_SCENE_CHARS: Final = 240
_NANOS: Final = 1_000_000_000


@dataclass(frozen=True, slots=True)
class Sighting:
    """One remembered object, at an absolute heading 0..359."""

    label: str
    heading_deg: int
    seen_mono_ns: int


class SceneRing:
    """A bounded ring of sightings plus the last scene description."""

    def __init__(
        self,
        path: str | Path | None = None,
        *,
        capacity: int = CAPACITY,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._path = Path(path) if path is not None else None
        self._clock = clock
        self._ring: deque[Sighting] = deque(maxlen=capacity)
        self._last_scene = ""

    @property
    def last_scene(self) -> str:
        """What ``describe_scene`` last reported, for the WorldState."""
        return self._last_scene

    def describe(self, description: str) -> None:
        """Record the latest scene description."""
        self._last_scene = description.strip()[:_SCENE_CHARS]
        self._persist()

    def remember(self, label: str, *, heading_deg: float, bearing_deg: float) -> None:
        """Remember seeing ``label`` at ``bearing_deg`` (+ is left) while the
        robot faced ``heading_deg``.

        Seeing the same label again moves it to the newest slot rather than
        adding a second entry: this is a memory of objects, not of glances.
        """
        clean = label.strip()[:_LABEL_CHARS]
        if not clean:
            raise ValueError("a sighting needs a label")
        for existing in list(self._ring):
            if existing.label == clean:
                self._ring.remove(existing)
        self._ring.append(
            Sighting(clean, heading_left_of(heading_deg, bearing_deg), self._clock())
        )
        self._persist()

    def recently_seen(self) -> list[RecentlySeen]:
        """The ring as the WorldState carries it, newest first, with each
        object's absolute heading and its age on the monotonic clock."""
        now = self._clock()
        return [
            RecentlySeen(
                label=sighting.label,
                heading_deg=sighting.heading_deg,
                age_s=max(0, (now - sighting.seen_mono_ns) // _NANOS),
            )
            for sighting in reversed(self._ring)
        ]

    def __len__(self) -> int:
        return len(self._ring)

    def __iter__(self) -> Iterator[Sighting]:
        return iter(self._ring)

    def _persist(self) -> None:
        """Rewrite the whole ring atomically (A34).

        It is written for the log and for ``roverctl``; it is never read back
        in, because the ages are monotonic and a monotonic clock does not
        survive a restart -- reloading would turn "94 seconds ago" into a lie.
        """
        if self._path is None:
            return
        records: list[dict[str, object]] = [
            {"kind": "scene", "description": self._last_scene}
        ]
        records += [
            {
                "kind": "sighting",
                "label": sighting.label,
                "heading_deg": sighting.heading_deg,
                "seen_mono_ns": sighting.seen_mono_ns,
            }
            for sighting in self._ring
        ]
        atomic_write_jsonl(self._path, records)
