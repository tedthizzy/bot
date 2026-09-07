"""``rover-robotd`` -- the only process that can move the robot.

It owns ``/dev/rover-mcu`` and is its only writer (I-18), validates every
command against ARCHITECTURE 4.2 before anything reaches the port, executes
motion profiles (A7), integrates odometry from raw ticks, arbitrates between
sources, and publishes state on the NDJSON bus of 5.2.

It must not import an ML library, hold a socket open to the box, block on
anything, or resume motion after a restart.
"""

from __future__ import annotations

__all__ = ["__version__"]

__version__ = "0.1.0"
