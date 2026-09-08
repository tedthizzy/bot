"""A readable live decode of the rover link (``docs/protocol.md``).

The bench check on deploy day is this tool: with the board wired to the Pi's
UART and ``rover.target`` stopped, ``python -m rover_devtools.wirecat
/dev/serial0`` shows the ``T:1006`` banner and the ``T:1001`` feedback stream
exactly as the host's decoder sees them, and says whether the banner's
heartbeat and cap agree with the configuration robotd will refuse motion
against.  In simulation the same tool reads ``rover-stub``'s pty path or its
TCP port; the bytes are identical.

Decoding is entirely ``rover_contracts.wave_proto.decode_line``: this module
renders and counts, it never re-implements the grammar.  It is read-only by
default. Even a read-only serial reader consumes feedback intended for robotd,
so serial access is refused while its bus is live. ``--request`` additionally
transmits feedback-on and a banner request; capture-file replay stays available.
"""

from __future__ import annotations

import argparse
import os
import socket
import stat
import sys
import termios
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

from rover_contracts.wave_proto import (
    LINE_MAX_BYTES,
    POWER_CAP,
    Banner,
    Dropped,
    Feedback,
    Imu,
    StopFlag,
    Unknown,
    banner_request,
    decode_line,
    feedback_flow,
)

from rover_devtools import load_config_or_default

__all__ = [
    "KINDS",
    "Counters",
    "LineSplitter",
    "banner_verdict",
    "kind_of",
    "main",
    "open_serial",
    "open_tcp",
    "render",
]

KINDS: tuple[str, ...] = ("feedback", "imu", "banner", "unknown", "dropped")
"""What ``decode_line`` can return, in the names ``--only``/``--exclude`` take."""

Decoded = Feedback | Imu | Banner | Unknown | Dropped


def kind_of(decoded: Decoded) -> str:
    if isinstance(decoded, Feedback):
        return "feedback"
    if isinstance(decoded, Imu):
        return "imu"
    if isinstance(decoded, Banner):
        return "banner"
    if isinstance(decoded, Unknown):
        return "unknown"
    return "dropped"


@dataclass
class Counters:
    """What went past, so a corrupted link looks different from a silent one."""

    feedback: int = 0
    imu: int = 0
    banner: int = 0
    unknown: int = 0
    dropped: int = 0
    stock: int = 0
    """Feedback lines without the fork's fields: stock firmware."""

    def count(self, decoded: Decoded) -> None:
        kind = kind_of(decoded)
        setattr(self, kind, getattr(self, kind) + 1)
        if isinstance(decoded, Feedback) and not decoded.patched:
            self.stock += 1

    def summary(self) -> str:
        return (
            f"{self.feedback} feedback ({self.stock} stock), {self.imu} imu, "
            f"{self.banner} banner, {self.unknown} unknown, {self.dropped} dropped"
        )


class LineSplitter:
    """Bytes in, ``\\n``-terminated lines out.  A line that grows past the
    protocol's 512 bytes without a newline is handed on whole, so
    ``decode_line`` drops it as oversize rather than the buffer growing for
    ever; blank lines (a ``\\r\\n`` ending) are not lines."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        lines: list[bytes] = []
        while True:
            index = self._buf.find(b"\n")
            if index < 0:
                if len(self._buf) > LINE_MAX_BYTES:
                    lines.append(bytes(self._buf))
                    self._buf.clear()
                break
            line = bytes(self._buf[:index])
            del self._buf[: index + 1]
            if line.strip():
                lines.append(line)
        return lines


# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------


def open_serial(path: str | Path, baud: int = 115200, *, write: bool = False) -> int:
    """Open a tty (or a pty slave) raw at ``baud`` and return the fd.

    The default is ``O_RDONLY``: I-18 says only robotd writes the port, and a
    read-only fd is what makes that structural rather than a promise in a help
    string.  ``write=True`` is the ``--request`` opt-in.

    A pty has no line speed and ``termios`` on macOS lacks some ``B<baud>``
    constants, so the speed is set when the constant exists and skipped when it
    does not.  A regular file opens as itself, for replaying a capture.
    """
    mode = os.O_RDWR if write else os.O_RDONLY
    fd = os.open(str(path), mode | os.O_NOCTTY | os.O_NONBLOCK)
    if not stat.S_ISCHR(os.fstat(fd).st_mode):
        return fd
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag &= ~(
        termios.IGNBRK
        | termios.BRKINT
        | termios.PARMRK
        | termios.ISTRIP
        | termios.INLCR
        | termios.IGNCR
        | termios.ICRNL
        | termios.IXON
    )
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG)
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    speed = getattr(termios, f"B{baud}", None)
    if speed is not None:
        ispeed = ospeed = speed
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(
        fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    )
    return fd


def open_tcp(host: str, port: int) -> int:
    """Connect to ``rover-stub --tcp`` (or a robotd ``[link] backend="tcp"``
    peer) and return the socket's fd, non-blocking, so the read loop treats it
    like the serial fd."""
    sock = socket.create_connection((host, port), timeout=3.0)
    sock.setblocking(False)
    return sock.detach()


def _parse_tcp(spec: str) -> tuple[str, int]:
    host, sep, port = spec.rpartition(":")
    if not sep:
        host, port = "", spec
    return host or "127.0.0.1", int(port)


def _socket_is_live(path: str) -> bool:
    """True when something is accepting on ``path``.  A stale socket file with
    no listener answers ECONNREFUSED and is not a reason to refuse."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(0.3)
    try:
        sock.connect(path)
    except OSError:
        return False
    finally:
        sock.close()
    return True


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def _flags(flags: StopFlag | None) -> str:
    if flags is None:
        return "-"
    # Iteration yields named members; only an arbitrary combined mask can lack a name.
    names = "|".join(cast(str, flag.name).lower() for flag in StopFlag if flags & flag)
    return f"0x{int(flags):X}[{names or '-'}]"


def _tof(value: int | None) -> str:
    if value is None:
        return "-"
    return "none" if value < 0 else f"{value}mm"


def _body(decoded: Decoded, raw: bytes) -> str:
    if isinstance(decoded, Feedback):
        base = (
            f"L={decoded.left:+.3f} R={decoded.right:+.3f} "
            f"yaw={decoded.yaw_deg:+.1f} roll={decoded.roll_deg:+.1f} "
            f"pitch={decoded.pitch_deg:+.1f} temp={decoded.temp_c:.1f}C "
            f"v={decoded.bus_v:.2f}V"
        )
        if not decoded.patched:
            return f"{base}  stock"
        return (
            f"{base}  hb={int(bool(decoded.hb))} st={_flags(decoded.st)} "
            f"tf={_tof(decoded.tof_mm)} bp={int(bool(decoded.bumper))} "
            f"cc={decoded.clamp_count}"
        )
    if isinstance(decoded, Banner):
        return (
            f"fw={decoded.fw} hb_ms={decoded.hb_ms} cap={decoded.cap:g} "
            f"proto={decoded.proto}"
        )
    if isinstance(decoded, Imu):
        gx, gy, gz = decoded.gyro_dps
        ax, ay, az = decoded.accel
        return (
            f"yaw={decoded.yaw_deg:+.1f} roll={decoded.roll_deg:+.1f} "
            f"pitch={decoded.pitch_deg:+.1f} gyro=({gx:+.1f},{gy:+.1f},{gz:+.1f})dps "
            f"accel=({ax:+.2f},{ay:+.2f},{az:+.2f}) temp={decoded.temp_c:.1f}C"
        )
    if isinstance(decoded, Unknown):
        return f"T={decoded.t}"
    return f"{decoded.reason}: {raw[:80]!r}"


def render(
    decoded: Decoded, *, elapsed: float = 0.0, raw: bytes = b"", show_raw: bool = False
) -> str:
    """One display line for one wire line.

    A dropped line prints its reason and its bytes rather than vanishing: the
    protocol counts and drops, and the operator needs to see the difference
    between a link that is quiet and one that is corrupt.
    """
    kind = kind_of(decoded)
    marker = "!" if kind == "dropped" else " "
    line = f"{elapsed:8.3f} {marker} {kind:<8} {_body(decoded, raw)}"
    if show_raw and kind != "dropped":
        line += f"\n{'':10}raw {raw!r}"
    return line


def banner_verdict(banner: Banner, heartbeat_ms: int) -> tuple[bool, str]:
    """robotd's own test of a banner: the compiled heartbeat must equal
    ``[safety] heartbeat_ms`` and the cap the 0.30 ceiling."""
    matched = banner.hb_ms == heartbeat_ms and abs(banner.cap - POWER_CAP) < 1e-6
    return matched, (
        f"wirecat: banner {'MATCH' if matched else 'MISMATCH'}: "
        f"hb_ms {banner.hb_ms} (config {heartbeat_ms}), "
        f"cap {banner.cap:g} (ceiling {POWER_CAP:g})"
    )


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------


def _kinds(text: str, parser: argparse.ArgumentParser) -> frozenset[str]:
    names = frozenset(part.strip() for part in text.split(",") if part.strip())
    unknown = names - set(KINDS)
    if unknown:
        parser.error(f"unknown kind {', '.join(sorted(unknown))}; choose from {KINDS}")
    return names


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rover_devtools.wirecat",
        description="Decode the rover link live (docs/protocol.md).",
    )
    parser.add_argument(
        "device",
        nargs="?",
        help="/dev/serial0, the pty path rover-stub printed, or a capture file",
    )
    parser.add_argument("--tcp", metavar="HOST:PORT", help="read a rover-stub --tcp port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--only",
        default="",
        metavar="KINDS",
        help="show only these kinds, comma-separated: " + ",".join(KINDS),
    )
    parser.add_argument(
        "--exclude",
        default="",
        metavar="KINDS",
        help="hide these kinds, e.g. --exclude feedback for the 20 Hz stream",
    )
    parser.add_argument("--count", type=int, default=0, help="stop after N shown lines")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after N s")
    parser.add_argument("--raw", action="store_true", help="also print the line bytes")
    parser.add_argument(
        "--config",
        metavar="PATH",
        help="the [safety] heartbeat_ms a banner is compared with",
    )
    parser.add_argument(
        "--request",
        action="store_true",
        help="send feedback-on and a banner request once. This WRITES to the "
        "link: never use it while robotd is running (I-18)",
    )
    args = parser.parse_args(argv)
    if (args.device is None) == (args.tcp is None):
        parser.error("give a device path or --tcp HOST:PORT, not both")
    only, exclude = _kinds(args.only, parser), _kinds(args.exclude, parser)

    try:
        config, origin = load_config_or_default(args.config)
    except (OSError, ValueError) as exc:
        print(f"wirecat: {exc}", file=sys.stderr)
        return 2
    heartbeat_ms = config.safety.heartbeat_ms
    where = args.tcp or args.device
    print(f"wirecat: [safety] heartbeat_ms={heartbeat_ms} from {origin}; reading {where}")

    # Serial reads consume robotd's feedback and raw mode changes its tty.
    serial_device = args.device is not None and Path(args.device).is_char_device()
    if (args.request or serial_device) and _socket_is_live(config.bus.sock):
        print(
            f"wirecat: {config.bus.sock} has a listener, so robotd owns {where}. "
            "Only robotd writes the port (I-18), and another serial reader "
            "would consume its feedback. Stop rover-robotd first, or use roverctl watch.",
            file=sys.stderr,
        )
        return 2

    try:
        if args.tcp:
            fd = open_tcp(*_parse_tcp(args.tcp))
        else:
            fd = open_serial(args.device, args.baud, write=args.request)
    except (OSError, ValueError) as exc:
        print(f"wirecat: cannot open {where}: {exc}", file=sys.stderr)
        return 2

    mode = os.fstat(fd).st_mode
    eof_ends = stat.S_ISREG(mode) or stat.S_ISSOCK(mode)
    if args.request:
        os.write(fd, feedback_flow(True))
        os.write(fd, banner_request())

    splitter = LineSplitter()
    counters = Counters()
    started = time.monotonic()
    shown = 0
    status = 0
    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - started >= args.seconds:
                break
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            except OSError as exc:
                print(f"wirecat: read failed: {exc}", file=sys.stderr)
                return 1
            if not data:
                # A file or a socket has nothing more to say; a tty is quiet.
                if eof_ends:
                    break
                time.sleep(0.005)
                continue
            for line in splitter.feed(data):
                decoded = decode_line(line)
                counters.count(decoded)
                if isinstance(decoded, Banner):
                    matched, verdict = banner_verdict(decoded, heartbeat_ms)
                    print(verdict)
                    if not matched:
                        status = 1
                kind = kind_of(decoded)
                if (only and kind not in only) or kind in exclude:
                    continue
                print(render(decoded, elapsed=now - started, raw=line, show_raw=args.raw))
                shown += 1
                if args.count and shown >= args.count:
                    return status
            sys.stdout.flush()
    except KeyboardInterrupt:
        return 130
    finally:
        print(f"wirecat: {counters.summary()}", file=sys.stderr)
        os.close(fd)
    return status


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
