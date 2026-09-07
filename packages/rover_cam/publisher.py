"""The cam process: capture, encode, publish on ``frames.sock`` (ARCHITECTURE 4.4, 5.3).

One NDJSON header line then exactly ``bytes`` octets of JPEG, repeating.  Newest
only: a slow subscriber loses whole frames, never partial ones, and no more than
two frames are held per subscriber -- one being written, one pending.  A still
never yields its pending slot to a stream frame, because only a still may
authorize motion.

This process never talks to robotd.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol

from rover_contracts.config import RobotConfig, load_config
from rover_contracts.messages import FrameHeader, FrameKind

__all__ = [
    "DEFAULT_STILL_PERIOD_S",
    "CameraBackend",
    "CapturedFrame",
    "FramePublisher",
    "Plane",
    "main",
    "run",
]

log = logging.getLogger("rover_cam")

Plane = Literal["lores", "main"]
"""``lores`` is A18's 640x480 motion-turn image; ``main`` its 896x672 scene image."""

DEFAULT_STILL_PERIOD_S = 1.0
"""Cadence of the unrequested ``lores`` still (see docs/deviations.md)."""

_FRAME_ID_MODULUS = 1_000_000
_MAX_REQUEST_BYTES = 512
_SOCKET_MODE = 0o660


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    """One encoded JPEG and the two clocks it was stamped with.

    ``frame_mono_ns`` is ``CLOCK_MONOTONIC`` and is what robotd's freshness gate
    reads (I-23); ``frame_wallclock_ns`` is for logs, and is wrong by hours on a
    Pi 4 before its first NTP sync.
    """

    kind: FrameKind
    jpeg: bytes
    w: int
    h: int
    quality: int
    frame_mono_ns: int
    frame_wallclock_ns: int
    exposure_us: int | None = None
    gain: float | None = None
    lux: float | None = None

    def header(self, frame_id: str) -> FrameHeader:
        """The ARCHITECTURE 5.3 header line for this frame."""
        return FrameHeader(
            frame_id=frame_id,
            kind=self.kind,
            frame_mono_ns=self.frame_mono_ns,
            frame_wallclock_ns=self.frame_wallclock_ns,
            w=self.w,
            h=self.h,
            quality=self.quality,
            bytes=len(self.jpeg),
            exposure_us=self.exposure_us,
            gain=self.gain,
            lux=self.lux,
        )


class CameraBackend(Protocol):
    """A camera, selected by ``[camera] backend`` -- never by a platform test."""

    def start(self, on_stream_frame: Callable[[CapturedFrame], None] | None) -> None:
        """Open the device.  ``on_stream_frame`` receives preview frames, if any."""

    def capture_still(self, plane: Plane) -> CapturedFrame:
        """Capture and encode one still.  Blocking; callers use a thread."""

    def close(self) -> None:
        """Release the device."""


class _Subscriber:
    """One connected reader of ``frames.sock``."""

    def __init__(self, writer: asyncio.StreamWriter) -> None:
        self._writer = writer
        self._pending: bytes | None = None
        self._pending_kind: FrameKind | None = None
        self._wake = asyncio.Event()
        self.dropped = 0
        self.task = asyncio.create_task(self._drain_loop())

    def offer(self, kind: FrameKind, blob: bytes) -> None:
        """Replace the pending frame, or drop this one if a still is waiting."""
        if self._pending is not None:
            if self._pending_kind is FrameKind.STILL and kind is FrameKind.STREAM:
                self.dropped += 1
                return
            self.dropped += 1
        self._pending = blob
        self._pending_kind = kind
        self._wake.set()

    async def _drain_loop(self) -> None:
        try:
            while True:
                await self._wake.wait()
                self._wake.clear()
                blob, self._pending = self._pending, None
                self._pending_kind = None
                if blob is None:
                    continue
                self._writer.write(blob)
                await self._writer.drain()
        except (OSError, ConnectionError):
            pass

    async def close(self) -> None:
        self.task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await self.task
        self._writer.close()
        with contextlib.suppress(OSError, ConnectionError):
            await self._writer.wait_closed()


class FramePublisher:
    """The ``frames.sock`` server.  Fan-out only; it reads one message type."""

    def __init__(
        self,
        path: str | Path,
        *,
        on_request: Callable[[Plane], None] | None = None,
    ) -> None:
        self._path = Path(path)
        self._on_request = on_request
        self._subs: set[_Subscriber] = set()
        self._server: asyncio.AbstractServer | None = None
        self._seq = 0

    @property
    def subscriber_count(self) -> int:
        return len(self._subs)

    async def start(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        with contextlib.suppress(FileNotFoundError):
            self._path.unlink()
        self._server = await asyncio.start_unix_server(
            self._on_connect, path=str(self._path)
        )
        os.chmod(self._path, _SOCKET_MODE)
        log.info("frames.sock listening at %s", self._path)

    async def close(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        for sub in list(self._subs):
            await sub.close()
        self._subs.clear()
        with contextlib.suppress(FileNotFoundError):
            self._path.unlink()

    def publish(self, frame: CapturedFrame) -> str:
        """Offer one frame to every subscriber.  Returns its ``frame_id``."""
        self._seq = (self._seq + 1) % _FRAME_ID_MODULUS
        frame_id = f"cam-{self._seq:06d}"
        line = frame.header(frame_id).model_dump_json(exclude_none=True)
        blob = line.encode() + b"\n" + frame.jpeg
        for sub in self._subs:
            sub.offer(frame.kind, blob)
        return frame_id

    async def _on_connect(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        sub = _Subscriber(writer)
        self._subs.add(sub)
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                if len(line) > _MAX_REQUEST_BYTES:
                    continue
                self._handle_request(line)
        except (OSError, ConnectionError, ValueError):
            pass  # ValueError: readline() over the stream limit
        finally:
            self._subs.discard(sub)
            await sub.close()

    def _handle_request(self, line: bytes) -> None:
        if self._on_request is None:
            return
        try:
            message = json.loads(line)
        except ValueError:
            return
        if not isinstance(message, dict) or message.get("type") != "still":
            return
        plane = message.get("plane", "lores")
        if plane in ("lores", "main"):
            self._on_request(plane)


def _make_backend(config: RobotConfig) -> CameraBackend:
    """Fakes are configuration, never a code branch (principle 6)."""
    if config.camera.backend == "fake":
        from rover_cam.fake_backend import FakeBackend

        return FakeBackend(
            config.camera, still_path=config.camera.fake_still or None
        )
    from rover_cam.picamera2_backend import Picamera2Backend

    return Picamera2Backend(config.camera)


async def run(
    config: RobotConfig, *, still_period_s: float = DEFAULT_STILL_PERIOD_S
) -> None:
    """Serve ``frames.sock`` until cancelled."""
    backend = _make_backend(config)
    requested: set[Plane] = set()
    wake = asyncio.Event()

    def request(plane: Plane) -> None:
        requested.add(plane)
        wake.set()

    publisher = FramePublisher(config.bus.frames_sock, on_request=request)
    await publisher.start()
    loop = asyncio.get_running_loop()

    def on_stream_frame(frame: CapturedFrame) -> None:
        loop.call_soon_threadsafe(publisher.publish, frame)

    backend.start(on_stream_frame)
    try:
        while True:
            timeout = still_period_s if still_period_s > 0 else None
            try:
                await asyncio.wait_for(wake.wait(), timeout)
            except TimeoutError:
                requested.add("lores")
            wake.clear()
            planes = sorted(requested) or ["lores"]
            requested.clear()
            for plane in planes:
                try:
                    frame = await asyncio.to_thread(backend.capture_still, plane)
                except Exception:  # a camera stall must not kill the process
                    log.exception("still capture failed on plane %s", plane)
                    continue
                publisher.publish(frame)
    finally:
        backend.close()
        await publisher.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rover-cam")
    parser.add_argument("--config", default="config/robot.toml")
    parser.add_argument(
        "--still-period-s",
        type=float,
        default=DEFAULT_STILL_PERIOD_S,
        help="unrequested lores still cadence; 0 waits for requests only",
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    config = load_config(args.config)
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(run(config, still_period_s=args.still_period_s))
    return 0


if __name__ == "__main__":  # `python -m rover_cam.publisher`
    raise SystemExit(main())
