"""rover-web: the face, the teleop pad, the text box and the STOP button.

ARCHITECTURE 4.5.  The only WebSocket speaker in the design, and an additional
stop authority -- never a required one.  It never touches the serial port, and
the rover behaves identically with this process dead.
"""

from __future__ import annotations

from rover_web.app import LineClient, TeleopStream, create_app, main

__all__ = ["LineClient", "TeleopStream", "create_app", "main"]

__version__ = "0.1.0"
