"""The heading frame brain reasons in, stated once.

``WorldState.heading_deg`` and ``turn_to.heading_deg`` share one frame: whole
degrees 0..359, increasing to the right (clockwise from above), like a compass.
``TurnToArgs`` states the consequence the model is told: turning left 90 means
asking for ``(heading - 90) mod 360``.  A bearing from
:func:`rover_contracts.units.bearing_deg_from_center_x` is positive to the left,
so an object seen at bearing ``+b`` sits at heading ``heading - b``.

Every place brain turns "left by so many degrees" into an absolute heading --
the router, the scene ring, ``find``'s sweep -- goes through this one function,
so if the unit at G5 shows the fused yaw running the other way the frame flips
here and nowhere else.
"""

from __future__ import annotations

from rover_contracts.units import round_half_away, wrap_deg_360

__all__ = ["heading_left_of"]


def heading_left_of(heading_deg: float, left_deg: float) -> int:
    """The heading ``left_deg`` to the left of ``heading_deg``, as the 0..359
    integer the WorldState and ``turn_to`` carry.  Negative ``left_deg`` turns
    right."""
    return round_half_away(wrap_deg_360(heading_deg - left_deg)) % 360
