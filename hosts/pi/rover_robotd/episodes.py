"""Teleop episodes, recorded as LeRobot-shaped JSONL (A34).

JSONL only, no SQLite.  Each step carries an ``action`` -- the wheel powers
robotd streamed that period, under exactly :data:`ACTION_KEYS` -- an
``observation`` of what the controller reported, and the host's monotonic
stamp, which is the shape ``tools/to_lerobot.py`` and G6's contract test read.
There is no pose and no encoder count to record (ADR-0013): the observation is
the fused heading and the powers the controller actually applied.

Steps are appended without an ``fsync``, which is taken once when the episode
closes: the recorder runs on robotd's control loop, and 4.2 forbids that loop
to block on anything.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Final

from rover_contracts.ids import new_ulid
from rover_contracts.jsonl import JsonlWriter
from rover_contracts.wave_proto import Feedback

__all__ = ["ACTION_KEYS", "EpisodeRecorder", "EpisodeStep"]

log = logging.getLogger("rover.robotd.episodes")

ACTION_KEYS: Final = ("left.power", "right.power")
"""The action features, in this order.  Not to be trimmed or reordered."""


@dataclass(frozen=True, slots=True)
class EpisodeStep:
    """One recorded control cycle."""

    episode_id: str
    index: int
    t_mono_ns: int
    action: dict[str, float]
    observation: dict[str, Any]
    source: str
    frame_id: str | None = None


@dataclass(slots=True)
class EpisodeRecorder:
    """Appends one JSONL file per teleop episode under ``<log dir>/episodes``."""

    directory: Path
    episode_id: str | None = None
    index: int = 0
    _writer: JsonlWriter | None = field(default=None, repr=False)

    @property
    def active(self) -> bool:
        return self._writer is not None

    def start(self, episode_id: str | None = None) -> str:
        """Open a new episode.  A running one is closed first."""
        self.stop()
        self.episode_id = episode_id or new_ulid()
        self.index = 0
        path = Path(self.directory) / "episodes" / f"{self.episode_id}.jsonl"
        self._writer = JsonlWriter(path)
        log.info("recording teleop episode %s to %s", self.episode_id, path)
        return self.episode_id

    def record(
        self,
        *,
        t_mono_ns: int,
        left: float,
        right: float,
        heading_deg: float,
        yaw_rate_dps: float | None,
        feedback: Feedback | None,
        source: str,
        frame_id: str | None = None,
    ) -> EpisodeStep | None:
        """Append one step.  A no-op when no episode is open."""
        if self._writer is None or self.episode_id is None:
            return None
        observation: dict[str, Any] = {
            "heading_deg": heading_deg,
            "yaw_rate_dps": yaw_rate_dps,
            "left.applied": feedback.left if feedback is not None else None,
            "right.applied": feedback.right if feedback is not None else None,
            "front_m": (
                feedback.tof_mm / 1000.0
                if feedback is not None and feedback.tof_valid
                else None
            ),
            "bumper": bool(feedback.bumper) if feedback is not None else None,
            "bus_v": feedback.bus_v if feedback is not None else None,
        }
        step = EpisodeStep(
            episode_id=self.episode_id,
            index=self.index,
            t_mono_ns=t_mono_ns,
            action={"left.power": float(left), "right.power": float(right)},
            observation=observation,
            source=source,
            frame_id=frame_id,
        )
        self.index += 1
        self._writer.write(
            {
                "episode_id": step.episode_id,
                "index": step.index,
                "t_mono_ns": step.t_mono_ns,
                "action": step.action,
                "observation": step.observation,
                "source": step.source,
                "frame_id": step.frame_id,
            }
        )
        return step

    def stop(self) -> str | None:
        """Close the open episode, if any, and return its id."""
        if self._writer is None:
            return None
        # The one fsync of an episode, taken here rather than 20 times a second
        # on the control loop: a crash then costs the OS page cache, while an
        # SD-card stall on the loop costs the firmware's 300 ms heartbeat and
        # zeroes the wheels mid-teleop.
        self._writer.sync()
        self._writer.close()
        self._writer = None
        finished, self.episode_id = self.episode_id, None
        log.info("teleop episode %s closed after %d steps", finished, self.index)
        return finished
