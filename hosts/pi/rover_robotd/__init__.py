"""``rover-robotd`` -- the only process that can move the robot.

It owns the rover's serial line and is its only writer (I-18), validates every
command against ARCHITECTURE 4.2 before anything reaches the port, streams the
one command that moves the robot from the loop that owns the goal (ADR-0013),
arbitrates between sources, and publishes state on the NDJSON bus of 5.2.

It must not import an ML library, hold a socket open to the box, block on
anything, or resume motion after a restart.
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.2.0"
