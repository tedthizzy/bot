"""The fake camera: a fixture JPEG, or a synthetic one generated here.

Directive 3 says nothing ships that cannot run on the MacBook first, and
ARCHITECTURE 12 keeps every image library out of the base dependency set --
`numpy` sits in the speech extra so it cannot shadow apt's `python3-numpy` under
picamera2, and `simplejpeg` arrives only from apt on the Pi.  So this module
carries the smallest thing that can honestly answer "give me a JPEG of exactly
these dimensions": a baseline encoder that writes one DC coefficient per 8x8
block and an EOB for the rest.  The result is a real, decodable, flat-block
JPEG; it is not a photograph, and nothing here pretends otherwise.
"""

from __future__ import annotations

import time
from pathlib import Path

from rover_contracts.config import CameraConfig
from rover_contracts.messages import FrameKind

from rover_cam.publisher import CapturedFrame, Plane

__all__ = ["FakeBackend", "jpeg_size"]

# The standard luminance DC Huffman table (ITU T.81 Annex K), and an AC table
# holding one symbol -- end-of-block -- because no block here has AC content.
_DC_BITS = bytes([0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0])
_DC_VALS = bytes(range(12))
_AC_BITS = bytes([1] + [0] * 15)
_AC_VALS = bytes([0])
_BASE_QUANT = 16  # the DC entry of the standard luminance quantisation table


def _huffman_codes(bits: bytes, values: bytes) -> dict[int, tuple[int, int]]:
    code = 0
    index = 0
    table: dict[int, tuple[int, int]] = {}
    for length in range(1, 17):
        for _ in range(bits[length - 1]):
            table[values[index]] = (code, length)
            index += 1
            code += 1
        code <<= 1
    return table


_DC_CODES = _huffman_codes(_DC_BITS, _DC_VALS)
_AC_EOB = _huffman_codes(_AC_BITS, _AC_VALS)[0]


class _BitWriter:
    """MSB-first, with the 0xFF00 byte stuffing baseline entropy data needs."""

    def __init__(self) -> None:
        self._out = bytearray()
        self._acc = 0
        self._n = 0

    def write(self, value: int, length: int) -> None:
        for shift in range(length - 1, -1, -1):
            self._acc = (self._acc << 1) | ((value >> shift) & 1)
            self._n += 1
            if self._n == 8:
                self._out.append(self._acc)
                if self._acc == 0xFF:
                    self._out.append(0x00)
                self._acc = 0
                self._n = 0

    def flush(self) -> bytes:
        while self._n:
            self.write(1, 1)
        return bytes(self._out)


def _segment(marker: int, payload: bytes) -> bytes:
    return bytes([0xFF, marker]) + (len(payload) + 2).to_bytes(2, "big") + payload


def _quant_value(quality: int) -> int:
    scale = 5000 // quality if quality < 50 else 200 - 2 * quality
    return max(1, min(255, (_BASE_QUANT * scale + 50) // 100))


def _encode_flat_blocks(
    w: int, h: int, quality: int, block: list[list[int]]
) -> bytes:
    """Encode an 8x8-block-resolution greyscale image as a baseline JPEG."""
    quant = _quant_value(quality)
    out = bytearray(b"\xff\xd8")
    out += _segment(0xDB, bytes([0x00]) + bytes([quant]) * 64)
    out += _segment(
        0xC0,
        bytes([8])
        + h.to_bytes(2, "big")
        + w.to_bytes(2, "big")
        + bytes([1, 1, 0x11, 0]),
    )
    out += _segment(0xC4, bytes([0x00]) + _DC_BITS + _DC_VALS)
    out += _segment(0xC4, bytes([0x10]) + _AC_BITS + _AC_VALS)
    out += _segment(0xDA, bytes([1, 1, 0x00, 0, 63, 0]))

    writer = _BitWriter()
    previous = 0
    for row in block:
        for value in row:
            dc = max(-1023, min(1023, round((value - 128) * 8 / quant)))
            diff = dc - previous
            previous = dc
            size = abs(diff).bit_length()
            code, length = _DC_CODES[size]
            writer.write(code, length)
            if size:
                writer.write(diff if diff > 0 else diff + (1 << size) - 1, size)
            writer.write(*_AC_EOB)
    out += writer.flush()
    out += b"\xff\xd9"
    return bytes(out)


def _pattern(w: int, h: int, tick: int) -> list[list[int]]:
    """A left-to-right ramp with one bright block that walks across it."""
    columns = (w + 7) // 8
    rows = (h + 7) // 8
    marker_x = tick % columns
    marker_y = rows // 2
    return [
        [
            240
            if abs(x - marker_x) <= 2 and abs(y - marker_y) <= 2
            else 32 + (x * 176) // max(1, columns - 1)
            for x in range(columns)
        ]
        for y in range(rows)
    ]


_SOF_MARKERS = frozenset(
    {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
)


def jpeg_size(data: bytes) -> tuple[int, int]:
    """``(width, height)`` from a JPEG's frame header.  Raises on a non-JPEG."""
    if data[:2] != b"\xff\xd8":
        raise ValueError("not a JPEG: no SOI")
    i = 2
    while i + 3 < len(data):
        if data[i] != 0xFF:
            raise ValueError(f"not a JPEG: expected a marker at byte {i}")
        marker = data[i + 1]
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        length = int.from_bytes(data[i + 2 : i + 4], "big")
        if marker in _SOF_MARKERS:
            return (
                int.from_bytes(data[i + 7 : i + 9], "big"),
                int.from_bytes(data[i + 5 : i + 7], "big"),
            )
        i += 2 + length
    raise ValueError("not a JPEG: no frame header")


class FakeBackend:
    """``[camera] backend = "fake"``.

    With ``still_path`` set, every still is that file, served byte for byte and
    described by its own frame header -- the fixture is not resized, because
    nothing in the base dependency set can resize it.  Without it, each still is
    a freshly generated synthetic frame at exactly the configured plane size.
    """

    def __init__(
        self, camera: CameraConfig, *, still_path: str | Path | None = None
    ) -> None:
        self._camera = camera
        self._tick = 0
        self._fixture: tuple[bytes, int, int] | None = None
        if still_path:
            data = Path(still_path).read_bytes()
            width, height = jpeg_size(data)
            self._fixture = (data, width, height)

    def start(self, on_stream_frame: object | None = None) -> None:
        """No preview stream: the MJPEG path is the Pi's hardware encoder."""

    def close(self) -> None:
        """Nothing to release."""

    def capture_still(self, plane: Plane) -> CapturedFrame:
        width, height = self._camera.main if plane == "main" else self._camera.lores
        quality = self._camera.jpeg_quality
        if self._fixture is not None:
            jpeg, width, height = self._fixture
        else:
            jpeg = _encode_flat_blocks(
                width, height, quality, _pattern(width, height, self._tick)
            )
        self._tick += 1
        # Stamped after the frame exists, so an age computed from it can never
        # be younger than the capture itself (A18, I-23).
        return CapturedFrame(
            kind=FrameKind.STILL,
            jpeg=jpeg,
            w=width,
            h=height,
            quality=quality,
            frame_mono_ns=time.monotonic_ns(),
            frame_wallclock_ns=time.time_ns(),
        )
