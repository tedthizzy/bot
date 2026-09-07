"""Teleop episodes, recorded as LeKiwi-keyed JSONL (A34).

JSONL only, no SQLite.  The ``action`` object carries **all three** LeKiwi keys
-- ``x.vel``, ``y.vel`` (a constant 0.0, because the rover is differential drive
and has no lateral velocity) and ``theta.vel`` -- and exactly those three, which
is what ``tools/to_lerobot.py`` and G6's contract test check.  Dropping
``y.vel`` is the day-one mistake that turns every recorded episode into scrap.

``theta.vel`` is written in **deg/s**, LeKiwi's unit (A34); ``x.vel`` is m/s.

Steps are appended without an ``fsync``, which is taken once when the episode
closes: the recorder runs on robotd's 20 Hz control loop, and 4.2 forbids that
loop to block on anything.
Every step also carries the raw encoder counts and the controller's own
microsecond stamp, so a pose can be recomputed offline from the log alone
(principle 7) rather than trusted from the integration that ran at the time.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Final

from rover_contracts.ids import new_ulid
from rover_contracts.jsonl import JsonlWriter
from rover_contracts.units import radps_to_dps

from rover_robotd.odom import Pose

__all__ = ["LEKIWI_ACTION_KEYS", "EpisodeRecorder", "EpisodeStep"]

log = logging.getLogger("rover.robotd.episodes")

LEKIWI_ACTION_KEYS: Final = ("x.vel", "y.vel", "theta.vel")
"""LeKiwi's ``action_features``, in LeKiwi's order.  Not to be trimmed."""


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
        linear_x_mps: float,
        angular_z_radps: float,
        pose: Pose,
        left_ticks: int,
        right_ticks: int,
        mcu_us: int,
        source: str,
        frame_id: str | None = None,
    ) -> EpisodeStep | None:
        """Append one step.  A no-op when no episode is open."""
        if self._writer is None or self.episode_id is None:
            return None
        step = EpisodeStep(
            episode_id=self.episode_id,
            index=self.index,
            t_mono_ns=t_mono_ns,
            action={
                "x.vel": float(linear_x_mps),
                "y.vel": 0.0,
                "theta.vel": radps_to_dps(angular_z_radps),
            },
            observation={
                "left_ticks": left_ticks,
                "right_ticks": right_ticks,
                "mcu_us": mcu_us,
                "x_m": pose.x_m,
                "y_m": pose.y_m,
                "yaw_rad": pose.yaw_rad,
            },
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
        # SD-card stall on the loop costs the MCU's 300 ms frame TTL and brakes
        # the wheels mid-teleop.
        self._writer.sync()
        self._writer.close()
        self._writer = None
        finished, self.episode_id = self.episode_id, None
        log.info("teleop episode %s closed after %d steps", finished, self.index)
        return finished
