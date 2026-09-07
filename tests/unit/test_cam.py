"""rover-cam: the fake backend's clocks and sizes, and frames.sock's newest-only
fan-out (ARCHITECTURE 4.4, 5.3, A18)."""

from __future__ import annotations

import asyncio
import contextlib
import json
import sys
import tempfile
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_cam.fake_backend import FakeBackend, jpeg_size  # noqa: E402
from rover_cam.publisher import (  # noqa: E402
    CapturedFrame,
    FramePublisher,
    _Subscriber,
    run,
)
from rover_contracts.config import BusConfig, CameraConfig, RobotConfig  # noqa: E402
from rover_contracts.messages import FrameHeader, FrameKind  # noqa: E402

CAMERA = CameraConfig(backend="fake")


# --------------------------------------------------------------------------
# fake backend
# --------------------------------------------------------------------------


def test_planes_are_exactly_a18_sizes():
    backend = FakeBackend(CAMERA)
    lores = backend.capture_still("lores")
    main = backend.capture_still("main")
    assert (lores.w, lores.h) == (640, 480)
    assert (main.w, main.h) == (896, 672)
    assert jpeg_size(lores.jpeg) == (640, 480)
    assert jpeg_size(main.jpeg) == (896, 672)
    assert lores.quality == main.quality == 80


def test_every_still_is_a_still():
    frame = FakeBackend(CAMERA).capture_still("lores")
    assert frame.kind is FrameKind.STILL  # only a still may authorize motion


def test_both_clocks_are_stamped_after_the_request():
    backend = FakeBackend(CAMERA)
    before_mono, before_wall = time.monotonic_ns(), time.time_ns()
    frame = backend.capture_still("lores")
    after_mono, after_wall = time.monotonic_ns(), time.time_ns()
    assert before_mono <= frame.frame_mono_ns <= after_mono
    assert before_wall <= frame.frame_wallclock_ns <= after_wall


def test_observation_age_is_checkable():
    """robotd gates motion on the monotonic stamp (I-23), so it must be usable
    as an age against this host's own clock."""
    frame = FakeBackend(CAMERA).capture_still("lores")
    age_ms = (time.monotonic_ns() - frame.frame_mono_ns) / 1e6
    assert 0 <= age_ms < 1000


def test_consecutive_frames_differ():
    backend = FakeBackend(CAMERA)
    frames = [backend.capture_still("lores").jpeg for _ in range(3)]
    assert len(set(frames)) == 3


def test_monotonic_stamps_increase():
    backend = FakeBackend(CAMERA)
    stamps = [backend.capture_still("lores").frame_mono_ns for _ in range(4)]
    assert stamps == sorted(stamps)


def test_fixture_still_is_served_verbatim():
    backend = FakeBackend(CAMERA)
    fixture = backend.capture_still("main").jpeg
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "frame.jpg"
        path.write_bytes(fixture)
        served = FakeBackend(CAMERA, still_path=path)
        for plane in ("lores", "main"):
            frame = served.capture_still(plane)
            assert frame.jpeg == fixture
            assert (frame.w, frame.h) == (896, 672)  # the fixture's own size


def test_fixture_must_be_a_jpeg():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "not.jpg"
        path.write_bytes(b"GIF89a")
        with pytest.raises(ValueError, match="not a JPEG"):
            FakeBackend(CAMERA, still_path=path)


def test_header_round_trips_through_the_contract():
    frame = FakeBackend(CAMERA).capture_still("lores")
    header = frame.header("cam-000917")
    line = header.model_dump_json(exclude_none=True)
    assert FrameHeader.model_validate_json(line) == header
    assert header.bytes == len(frame.jpeg)
    assert header.fmt == "jpeg"


# --------------------------------------------------------------------------
# newest-only fan-out
# --------------------------------------------------------------------------


class _GatedWriter:
    """A writer whose ``drain`` blocks until the test opens the gate."""

    def __init__(self) -> None:
        self.written = bytearray()
        self.gate = asyncio.Event()
        self.drains = 0

    def write(self, data: bytes) -> None:
        self.written += data

    async def drain(self) -> None:
        self.drains += 1
        await self.gate.wait()

    def is_closing(self) -> bool:
        return False

    def close(self) -> None:
        self.gate.set()

    async def wait_closed(self) -> None:
        return None


def _blob(tag: bytes, n: int) -> bytes:
    return tag * n


@pytest.mark.asyncio
async def test_slow_subscriber_drops_whole_frames_and_keeps_the_newest():
    writer = _GatedWriter()
    sub = _Subscriber(writer)
    await asyncio.sleep(0)  # let the drain loop reach its first wait

    first, second, third, fourth = (_blob(t, 8) for t in (b"A", b"B", b"C", b"D"))
    sub.offer(FrameKind.STREAM, first)
    await asyncio.sleep(0)
    sub.offer(FrameKind.STREAM, second)
    sub.offer(FrameKind.STREAM, third)
    sub.offer(FrameKind.STREAM, fourth)
    assert sub.dropped == 2  # B and C, whole, never partially written

    writer.gate.set()
    for _ in range(8):
        await asyncio.sleep(0)
    await sub.close()

    assert bytes(writer.written) == first + fourth


@pytest.mark.asyncio
async def test_a_pending_still_never_yields_to_a_stream_frame():
    writer = _GatedWriter()
    sub = _Subscriber(writer)
    await asyncio.sleep(0)

    sub.offer(FrameKind.STREAM, _blob(b"A", 4))
    await asyncio.sleep(0)
    sub.offer(FrameKind.STILL, _blob(b"S", 4))
    sub.offer(FrameKind.STREAM, _blob(b"X", 4))

    writer.gate.set()
    for _ in range(8):
        await asyncio.sleep(0)
    await sub.close()

    assert bytes(writer.written) == _blob(b"A", 4) + _blob(b"S", 4)


def _socket_dir() -> tempfile.TemporaryDirectory:
    # Unix socket paths are capped near 104 bytes on macOS.
    return tempfile.TemporaryDirectory(prefix="rc")


async def _wait(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("timed out waiting for the publisher")


@pytest.mark.asyncio
async def test_subscriber_reads_header_then_exactly_bytes_octets():
    backend = FakeBackend(CAMERA)
    with _socket_dir() as tmp:
        path = Path(tmp) / "frames.sock"
        publisher = FramePublisher(path)
        await publisher.start()
        reader, writer = await asyncio.open_unix_connection(str(path))
        await _wait(lambda: publisher.subscriber_count == 1)

        ids = []
        for _ in range(2):
            # One at a time: a publisher running ahead of its reader is newest
            # only, and would legitimately drop the earlier frame.
            frame = backend.capture_still("lores")
            ids.append(publisher.publish(frame))
            header = FrameHeader.model_validate_json(await reader.readline())
            payload = await reader.readexactly(header.bytes)
            assert header.frame_id == ids[-1]
            assert payload == frame.jpeg
            assert header.bytes == len(frame.jpeg)
            assert (header.w, header.h) == (640, 480)
            assert header.kind is FrameKind.STILL

        assert ids == ["cam-000001", "cam-000002"]
        writer.close()
        await publisher.close()


@pytest.mark.asyncio
async def test_a_subscriber_may_request_a_plane():
    requested: list[str] = []
    with _socket_dir() as tmp:
        path = Path(tmp) / "frames.sock"
        publisher = FramePublisher(path, on_request=requested.append)
        await publisher.start()
        _, writer = await asyncio.open_unix_connection(str(path))
        await _wait(lambda: publisher.subscriber_count == 1)
        writer.write(json.dumps({"v": 1, "type": "still", "plane": "main"}).encode())
        writer.write(b"\n")
        writer.write(b'{"type":"nonsense"}\nnot json at all\n')
        await writer.drain()
        await _wait(lambda: bool(requested))
        assert requested == ["main"]
        writer.close()
        await publisher.close()


@pytest.mark.asyncio
async def test_publishing_with_no_subscribers_is_harmless():
    with _socket_dir() as tmp:
        path = Path(tmp) / "frames.sock"
        publisher = FramePublisher(path)
        await publisher.start()
        assert publisher.subscriber_count == 0
        frame = CapturedFrame(
            kind=FrameKind.STILL,
            jpeg=b"\xff\xd8\xff\xd9",
            w=640,
            h=480,
            quality=80,
            frame_mono_ns=time.monotonic_ns(),
            frame_wallclock_ns=time.time_ns(),
        )
        assert publisher.publish(frame) == "cam-000001"
        await publisher.close()
        assert not path.exists()


@pytest.mark.asyncio
async def test_run_publishes_on_the_cadence_and_on_request():
    """The process itself: a lores still arrives unasked, and a subscriber can
    ask for the 896x672 plane that describe_scene and find need."""
    with _socket_dir() as tmp:
        sock = Path(tmp) / "frames.sock"
        config = RobotConfig(
            camera=CameraConfig(backend="fake"),
            bus=BusConfig(frames_sock=str(sock)),
        )
        task = asyncio.create_task(run(config, still_period_s=0.05))
        try:
            await _wait(sock.exists)
            reader, writer = await asyncio.open_unix_connection(str(sock))

            async def next_header() -> FrameHeader:
                line = await asyncio.wait_for(reader.readline(), 3.0)
                header = FrameHeader.model_validate_json(line)
                await reader.readexactly(header.bytes)
                return header

            first = await next_header()
            assert (first.w, first.h) == (640, 480)
            assert first.kind is FrameKind.STILL

            writer.write(b'{"v":1,"type":"still","plane":"main"}\n')
            await writer.drain()
            for _ in range(40):
                header = await next_header()
                if (header.w, header.h) == (896, 672):
                    break
            else:
                raise AssertionError("the requested main plane never arrived")
            writer.close()
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
