"""rover-cam: the camera process of ARCHITECTURE 4.4.

Two paths, because one picamera2 API cannot serve both: a continuous MJPEG
preview stream, and an on-demand ``capture_request(flush=True)`` still whose
exposure begins after the call.  Both are published on ``frames.sock`` as an
NDJSON header line followed by exactly ``bytes`` octets of JPEG, newest only.

The backend is chosen by ``[camera] backend``.  This process never talks to
robotd and never holds more than two frames.
"""

from __future__ import annotations

from rover_cam.publisher import (
    DEFAULT_STILL_PERIOD_S,
    CameraBackend,
    CapturedFrame,
    FramePublisher,
    Plane,
    main,
    run,
)

__all__ = [
    "DEFAULT_STILL_PERIOD_S",
    "CameraBackend",
    "CapturedFrame",
    "FramePublisher",
    "Plane",
    "main",
    "run",
]

__version__ = "0.1.0"
