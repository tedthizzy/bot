"""The rover's heading, from the controller's fused yaw.

There is no odometry and no pose (ADR-0013): the one number robotd steers by is
the fused yaw the feedback carries.  ``[link] yaw_sign`` corrects the board's
sign so that a positive heading change is a left turn (counter-clockwise from
above), the same sign :func:`rover_contracts.units.heading_error_deg` and
``turn_to`` use.  Every wrap goes through ``rover_contracts.units``; nothing
here does bare arithmetic on an angle.
"""

from __future__ import annotations

from rover_contracts.units import wrap_deg_180

__all__ = ["HeadingTracker"]


class HeadingTracker:
    """The latest heading in (-180, 180], its rate, and how old it is.

    ``update`` is called once per feedback sample with the host's monotonic
    arrival stamp; the rate is the wrapped change between two samples over
    their arrival gap, so it is ``None`` until the second sample.  ``reset`` is
    for a link loss or a controller restart: the heading kept for display is
    the last one seen, but ``available`` and ``age_ms`` say there is no sample
    on this connection, and the next rate is computed only from samples that
    share one.
    """

    __slots__ = ("yaw_sign", "heading_deg", "yaw_rate_dps", "sample_ns")

    def __init__(self, yaw_sign: int) -> None:
        if yaw_sign not in (-1, 1):
            raise ValueError(f"yaw_sign must be 1 or -1, got {yaw_sign!r}")
        self.yaw_sign = yaw_sign
        self.heading_deg = 0.0
        self.yaw_rate_dps: float | None = None
        self.sample_ns: int | None = None

    @property
    def available(self) -> bool:
        return self.sample_ns is not None

    def update(self, yaw_deg: float, arrival_ns: int) -> None:
        """Take one feedback sample.  ``arrival_ns`` is the host's clock."""
        heading = wrap_deg_180(self.yaw_sign * yaw_deg)
        if self.sample_ns is not None and arrival_ns > self.sample_ns:
            dt_s = (arrival_ns - self.sample_ns) / 1e9
            self.yaw_rate_dps = wrap_deg_180(heading - self.heading_deg) / dt_s
        else:
            self.yaw_rate_dps = None
        self.heading_deg = heading
        self.sample_ns = arrival_ns

    def age_ms(self, now_ns: int) -> float | None:
        if self.sample_ns is None:
            return None
        return max(0.0, (now_ns - self.sample_ns) / 1e6)

    def reset(self) -> None:
        """Forget the sample continuity: the link or the controller restarted."""
        self.yaw_rate_dps = None
        self.sample_ns = None
