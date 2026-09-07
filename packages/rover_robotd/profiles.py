"""Motion profiles and the one cell between the goal owner and the wire.

robotd executes the profile and the MCU executes velocity only (A7), so this is
where a ``drive`` becomes a trapezoid and a ``turn`` becomes its angular twin.
Both are recomputed every control cycle from *live* goal state -- measured
progress and the last command actually issued -- rather than played back from a
precomputed table, so a profile that is not being stepped produces nothing at
all instead of producing yesterday's velocity.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Final

__all__ = [
    "PROGRESS_MIN_CMD_MPS",
    "PROGRESS_MIN_FRACTION",
    "PROGRESS_WINDOW_MS",
    "SETPOINT_VALID_MS",
    "ProgressMonitor",
    "SetpointCell",
    "TrapezoidProfile",
]

SETPOINT_VALID_MS: Final = 100
"""How long one stamp is worth (ARCHITECTURE 4.2, "Setpoint ownership")."""

PROGRESS_WINDOW_MS: Final = 500
PROGRESS_MIN_FRACTION: Final = 0.20
PROGRESS_MIN_CMD_MPS: Final = 0.050
"""T2's progress half: under 20% of the expected delta-odom over any 500 ms
while the commanded speed is above 50 mm/s."""


class SetpointCell:
    """The only mutable state between the goal owner and the 20 Hz writer.

    I-14 holds here **structurally**, not by anyone remembering a rule:

    * The velocity is private.  The writer cannot reach it.  The only way out
      is :meth:`read`, which takes the reading clock as its argument and
      returns ``(0.0, 0.0)`` whenever the stamp has expired.  There is no code
      path in this class that yields a non-zero velocity without first
      comparing ``now`` against ``valid_until_mono_ns``, so the writer cannot
      repeat a stale non-zero setpoint even if it wanted to.
    * A stamp is worth :data:`SETPOINT_VALID_MS` and reading does not renew it.
      Renewal is only :meth:`stamp`, which the goal owner calls once per
      control cycle with a velocity it has *just recomputed* from measured
      progress.  A renewal is therefore a recomputation: there is no "send the
      previous value again" path to take.
    * A frozen, wedged or killed goal owner simply stops stamping.  Within
      :data:`SETPOINT_VALID_MS` every later read returns zeros, and the writer
      -- a separate task that keeps running -- streams ``v=w=0`` at 20 Hz,
      which brakes the wheels.  Nothing has to *detect* the freeze.
    """

    __slots__ = ("_v_mps", "_w_radps", "_valid_until_mono_ns")

    def __init__(self) -> None:
        self._v_mps = 0.0
        self._w_radps = 0.0
        self._valid_until_mono_ns = 0

    def stamp(
        self,
        v_mps: float,
        w_radps: float,
        now_mono_ns: int,
        *,
        valid_ms: int = SETPOINT_VALID_MS,
    ) -> None:
        """Publish a freshly computed velocity.  Only the goal owner calls this."""
        self._v_mps = float(v_mps)
        self._w_radps = float(w_radps)
        self._valid_until_mono_ns = now_mono_ns + valid_ms * 1_000_000

    def zero(self) -> None:
        """Drop the setpoint now: the next read returns zeros whatever the clock."""
        self._v_mps = 0.0
        self._w_radps = 0.0
        self._valid_until_mono_ns = 0

    def read(self, now_mono_ns: int) -> tuple[float, float]:
        """What the writer may send at ``now_mono_ns``.  Never raises."""
        if now_mono_ns > self._valid_until_mono_ns:
            return (0.0, 0.0)
        return (self._v_mps, self._w_radps)

    @property
    def valid_until_mono_ns(self) -> int:
        return self._valid_until_mono_ns


class TrapezoidProfile:
    """A trapezoid whose every setpoint is a function of measured progress.

    Each cycle the command is the smallest of three bounds: the cruise speed,
    one acceleration step above the command actually issued last cycle, and the
    speed from which the remaining distance is still stoppable at the same
    acceleration.  The braking bound is what shapes the trailing ramp, so an
    early or late arrival re-shapes it rather than overshooting a schedule.

    The acceleration must be no greater than the MCU's compiled slew cap
    (``[limits] accel_mps2`` / ``alpha_radps2``); the MCU bounds *increases*
    only and brakes far harder, so a deceleration computed here is always
    executable.
    """

    __slots__ = ("target", "cruise", "accel", "tolerance", "_command")

    def __init__(
        self, target: float, cruise: float, accel: float, tolerance: float
    ) -> None:
        if cruise <= 0.0 or accel <= 0.0 or tolerance <= 0.0:
            raise ValueError("cruise, accel and tolerance must all be positive")
        self.target = float(target)
        self.cruise = float(cruise)
        self.accel = float(accel)
        self.tolerance = float(tolerance)
        self._command = 0.0

    @property
    def command(self) -> float:
        """The signed velocity issued on the last :meth:`step`."""
        return self._command

    def remaining(self, progress: float) -> float:
        """Signed distance still to travel."""
        return self.target - progress

    def done(self, progress: float) -> bool:
        return abs(self.remaining(progress)) <= self.tolerance

    def fraction(self, progress: float) -> float:
        """Completion in ``0.0..1.0``, for ``state.active.progress``."""
        if self.target == 0.0:
            return 1.0
        return min(1.0, max(0.0, abs(progress) / abs(self.target)))

    def step(self, progress: float, dt_s: float) -> float:
        """The velocity to command now, given where the robot actually is."""
        remaining = self.remaining(progress)
        if abs(remaining) <= self.tolerance:
            self._command = 0.0
            return 0.0
        ramp_limit = abs(self._command) + self.accel * max(dt_s, 0.0)
        brake_limit = math.sqrt(2.0 * self.accel * abs(remaining))
        speed = min(self.cruise, ramp_limit, brake_limit)
        self._command = math.copysign(speed, remaining)
        return self._command


class ProgressMonitor:
    """T2's progress half, over a sliding 500 ms window.

    Expected travel is the integral of the commanded speed; actual travel is
    the integral of the measured step.  The window only judges once it is full
    and only while the command stayed above :data:`PROGRESS_MIN_CMD_MPS`
    throughout, so a legitimate ramp through zero never trips it.
    """

    __slots__ = ("_samples", "_actual", "_expected")

    def __init__(self) -> None:
        self._samples: deque[tuple[int, float, float, bool]] = deque()
        self._actual = 0.0
        self._expected = 0.0

    def update(
        self, now_mono_ns: int, moved: float, commanded: float, dt_s: float
    ) -> bool:
        """Feed one cycle; return ``True`` when the goal is judged stalled."""
        self._actual += abs(moved)
        self._expected += abs(commanded) * max(dt_s, 0.0)
        above_floor = abs(commanded) > PROGRESS_MIN_CMD_MPS
        self._samples.append((now_mono_ns, self._actual, self._expected, above_floor))

        window_ns = PROGRESS_WINDOW_MS * 1_000_000
        while len(self._samples) > 1 and now_mono_ns - self._samples[1][0] >= window_ns:
            self._samples.popleft()
        oldest_ns, actual0, expected0, _ = self._samples[0]
        if now_mono_ns - oldest_ns < window_ns:
            return False
        if not all(sample[3] for sample in self._samples):
            return False
        expected = self._expected - expected0
        if expected <= 0.0:
            return False
        return (self._actual - actual0) < PROGRESS_MIN_FRACTION * expected

    def reset(self) -> None:
        self._samples.clear()
        self._actual = 0.0
        self._expected = 0.0
