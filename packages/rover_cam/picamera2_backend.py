"""The real camera: picamera2 on ARCHITECTURE 4.4's two distinct paths.

1. The continuous preview stream is the hardware ``MJPEGEncoder`` on the
   ``lores`` plane and nothing else -- it cannot return a JPEG for a nominated
   ``CompletedRequest``, so it can never produce the frame that authorizes
   motion.
2. Every still is one ``capture_request(flush=True)``, whose exposure therefore
   begins after the call, encoded from the plane the caller names.

picamera2, libcamera and simplejpeg are imported inside the methods that need
them: this module must import cleanly on a Mac that has none of them.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from rover_contracts.config import CameraConfig
from rover_contracts.messages import FrameKind

from rover_cam.publisher import CapturedFrame, Plane

__all__ = ["Picamera2Backend"]

log = logging.getLogger("rover_cam.picamera2")

_STREAM_QUALITY = 80


class _StreamOutput:
    """A picamera2 ``Output`` that hands each whole MJPEG frame to a callback."""

    def __init__(self, sink: Callable[[CapturedFrame], None], w: int, h: int) -> None:
        self._sink = sink
        self._w = w
        self._h = h

    def outputframe(self, frame: bytes, *_args: Any, **_kwargs: Any) -> None:
        self._sink(
            CapturedFrame(
                kind=FrameKind.STREAM,
                jpeg=bytes(frame),
                w=self._w,
                h=self._h,
                quality=_STREAM_QUALITY,
                frame_mono_ns=time.monotonic_ns(),
                frame_wallclock_ns=time.time_ns(),
            )
        )

    def start(self) -> None:
        """picamera2 calls this when the encoder starts."""

    def stop(self) -> None:
        """picamera2 calls this when the encoder stops."""


def _wallclock_ns(metadata: dict[str, Any]) -> int:
    """``FrameWallClock`` in nanoseconds, or this host's realtime clock.

    libcamera reports the other metadata times in microseconds, and this one is
    documented alongside them, so a value small enough to be a microsecond epoch
    is scaled.  A value already large enough to be nanoseconds is taken as it is.
    """
    raw = metadata.get("FrameWallClock")
    if not isinstance(raw, int | float):
        return time.time_ns()
    value = int(raw)
    return value * 1000 if value < 10**17 else value


class Picamera2Backend:
    """``[camera] backend = "picamera2"``."""

    def __init__(self, camera: CameraConfig) -> None:
        self._camera = camera
        self._picam: Any = None
        self._encoder: Any = None

    def start(self, on_stream_frame: Callable[[CapturedFrame], None] | None) -> None:
        from picamera2 import Picamera2

        main_w, main_h = self._camera.main
        lores_w, lores_h = self._camera.lores
        self._picam = Picamera2()
        self._picam.configure(
            self._picam.create_still_configuration(
                main={"size": (main_w, main_h), "format": "RGB888"},
                lores={"size": (lores_w, lores_h), "format": "YUV420"},
                buffer_count=2,
            )
        )
        self._picam.start()
        if on_stream_frame is not None:
            from picamera2.encoders import MJPEGEncoder

            self._encoder = MJPEGEncoder()
            self._picam.start_encoder(
                self._encoder,
                _StreamOutput(on_stream_frame, lores_w, lores_h),
                name="lores",
            )
        log.info(
            "picamera2 started: main %s lores %s",
            self._camera.main,
            self._camera.lores,
        )

    def capture_still(self, plane: Plane) -> CapturedFrame:
        import simplejpeg

        request = self._picam.capture_request(flush=True)
        try:
            array = request.make_array(plane)
            metadata = request.get_metadata()
        finally:
            request.release()
        quality = self._camera.jpeg_quality
        if plane == "lores":
            width, height = self._camera.lores
            luma = array[:height, :width]
            chroma = height // 4
            base = height
            u = array[base : base + chroma].reshape(height // 2, width // 2)
            v = array[base + chroma : base + 2 * chroma].reshape(height // 2, width // 2)
            jpeg = simplejpeg.encode_jpeg_yuv_planes(luma, u, v, quality=quality)
        else:
            height, width = array.shape[0], array.shape[1]
            # picamera2's "RGB888" stores the components in B, G, R order.
            jpeg = simplejpeg.encode_jpeg(array, quality=quality, colorspace="BGR")
        gain = metadata.get("AnalogueGain")
        lux = metadata.get("Lux")
        return CapturedFrame(
            kind=FrameKind.STILL,
            jpeg=jpeg,
            w=width,
            h=height,
            quality=quality,
            frame_mono_ns=time.monotonic_ns(),
            frame_wallclock_ns=_wallclock_ns(metadata),
            exposure_us=metadata.get("ExposureTime"),
            gain=float(gain) if gain is not None else None,
            lux=float(lux) if lux is not None else None,
        )

    def close(self) -> None:
        if self._picam is None:
            return
        if self._encoder is not None:
            self._picam.stop_encoder()
            self._encoder = None
        self._picam.stop()
        self._picam.close()
        self._picam = None
