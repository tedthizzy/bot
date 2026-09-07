"""A readable live decode of the Pi<->MCU line protocol (ARCHITECTURE 5.1).

Deploy step 10 is this tool: with UART5 wired and the udev rule in place but
``rover.target`` not yet started, ``python -m rover_devtools.wirecat
/dev/rover-mcu`` reads the ``B`` banner on the operational link and confirms the
release ``caps`` word, the ``safety_hash`` and ``ctrl_flags`` b7 clear (I-18).

Decoding is entirely ``rover_contracts.serial_codec``: this module renders, it
never re-implements the grammar.  It is read-only by default -- the fd itself is
``O_RDONLY``, because I-18 says only robotd writes the port -- and ``--ping`` is
the one opt-in that transmits, for the diagnostic ``P``/``O`` round trip 5.1
names wirecat as a sender of.  ``--ping`` refuses to run while something is
listening on the robotd bus, the same guard ``roverctl arm`` carries.
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
from pathlib import Path

from rover_contracts.serial_codec import (
    TOF_ERROR_MM,
    TOF_NO_TARGET_MM,
    AckFrame,
    AckReason,
    AckResult,
    ArmFrame,
    BootFrame,
    CapBit,
    ClearFaultFrame,
    CtrlFlag,
    DecodeErr,
    DecodeOk,
    DecodeResult,
    EventCode,
    EventFrame,
    FrameReader,
    HelloFrame,
    McuState,
    PingFrame,
    PongFrame,
    StopFrame,
    TelemetryFrame,
    VelocityFrame,
    VFlag,
    ack_type_letter,
    encode_frame,
    fault_names,
)

from rover_devtools import load_config_or_default

__all__ = ["main", "open_serial", "render"]


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


def open_serial(path: str | Path, baud: int = 921600, *, write: bool = False) -> int:
    """Open a tty (or a pty slave) raw at ``baud`` and return the fd.

    The default is ``O_RDONLY``: I-18 says only robotd writes the port, and a
    read-only fd is what makes that structural rather than a promise in a help
    string.  ``write=True`` is the ``--ping`` opt-in.

    ``termios`` carries no ``B921600`` on macOS, where the only serial device
    that matters is ``mcu_sim``'s pty and the speed is meaningless.  The speed
    is set when the constant exists and skipped when it does not, rather than
    refusing to run on the platform every sim gate is developed on.
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


def _bits(value: int, enum: type[CtrlFlag] | type[VFlag] | type[CapBit]) -> str:
    names = "|".join(bit.name or "?" for bit in enum if value & bit) or "-"
    return f"0x{value:X}[{names}]"


def _fault(value: int) -> str:
    return f"0x{value:X}[{'|'.join(fault_names(value)) or '-'}]"


def _range_mm(value: int) -> str:
    if value == TOF_NO_TARGET_MM:
        return "no_target"
    if value == TOF_ERROR_MM:
        return "ERROR"
    return f"{value}mm"


def _member(enum: type[McuState] | type[EventCode] | type[AckReason] | type[AckResult],
            value: int) -> str:
    try:
        return enum(value).name
    except ValueError:
        return str(value)


def _body(frame: object) -> str:
    """The type-specific half of a display line."""
    if isinstance(frame, BootFrame):
        version = (
            f"{frame.fw_ver >> 16 & 0xFF}."
            f"{frame.fw_ver >> 8 & 0xFF}.{frame.fw_ver & 0xFF}"
        )
        return (
            f"fw={version} proto={frame.proto_ver} caps={_bits(frame.caps, CapBit)} "
            f"reset={frame.reset_reason} safety_hash={frame.safety_hash}"
        )
    if isinstance(frame, TelemetryFrame):
        return (
            f"{_member(McuState, frame.state)} ack={frame.ack_seq} "
            f"flags={_bits(frame.ctrl_flags, CtrlFlag)} fault={_fault(frame.fault)} "
            f"v={frame.v_meas_mm_s}/{frame.v_cmd_mm_s}mm/s "
            f"w={frame.w_meas_mrad_s}/{frame.w_cmd_mrad_s}mrad/s "
            f"ticks={frame.left_ticks},{frame.right_ticks} "
            f"front={_range_mm(frame.tof_front_mm)} "
            f"cliff={_range_mm(frame.tof_cliff_mm)} "
            f"bat={frame.vbat_mv}mV {frame.imotor_ma}mA age={frame.sensor_age_ms}ms "
            f"late={frame.loop_late_pct}% rx_drop={frame.rx_drop} "
            f"motion={frame.motion}"
        )
    if isinstance(frame, AckFrame):
        return (
            f"acks {ack_type_letter(frame.ack_type)} seq={frame.ack_seq} "
            f"{_member(AckResult, frame.result)} "
            f"reason={_member(AckReason, frame.reason)} echo={frame.echo}"
        )
    if isinstance(frame, EventFrame):
        return f"{_member(EventCode, frame.event)} arg={frame.arg} mcu_us={frame.mcu_us}"
    if isinstance(frame, PongFrame):
        return f"echo={frame.echo_pi_mono_us} mcu_us={frame.mcu_us}"
    if isinstance(frame, VelocityFrame):
        return (
            f"v={frame.v_mm_s}mm/s w={frame.w_mrad_s}mrad/s "
            f"ttl={frame.frame_ttl_ms}ms flags={_bits(frame.flags, VFlag)}"
        )
    if isinstance(frame, HelloFrame):
        return f"host_boot_id={frame.host_boot_id}"
    if isinstance(frame, ArmFrame):
        return f"nonce={frame.nonce}"
    if isinstance(frame, StopFrame):
        return "brake" if frame.mode == 0 else "coast"
    if isinstance(frame, ClearFaultFrame):
        return f"clear {_fault(frame.mask)}"
    if isinstance(frame, PingFrame):
        return f"pi_mono_us={frame.pi_mono_us}"
    return ""


def render(result: DecodeResult, *, elapsed: float = 0.0, raw: bool = False) -> str:
    """One display line for one decoded line.

    A rejected line prints its reason rather than vanishing: I-2's whole point
    is that drops are counted and visible, so a corrupted link looks different
    from a silent one.
    """
    if isinstance(result, DecodeErr):
        return (
            f"{elapsed:8.3f}  !  {result.reason.name}({int(result.reason)}) "
            f"{result.detail}: {result.raw!r}"
        )
    frame = result.frame
    line = (
        f"{elapsed:8.3f}  {type(frame).TYPE}  seq={frame.seq} sess={frame.session}  "
        f"{_body(frame)}"
    )
    if raw:
        line += f"\n{'':10}raw {encode_frame(frame)!r}"
    return line


def _selected(letter: str, only: str, exclude: str) -> bool:
    return (not only or letter in only) and letter not in exclude


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rover_devtools.wirecat",
        description="Decode the Pi<->MCU line protocol live (ARCHITECTURE 5.1).",
    )
    parser.add_argument("device", help="/dev/rover-mcu, or ./run/mcu.pty in sim")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument(
        "--only", default="", metavar="TYPES", help="show only these type letters"
    )
    parser.add_argument(
        "--exclude",
        default="",
        metavar="TYPES",
        help="hide these type letters, e.g. --exclude T for the 50 Hz stream",
    )
    parser.add_argument("--count", type=int, default=0, help="stop after N frames")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after N s")
    parser.add_argument("--raw", action="store_true", help="also print the frame bytes")
    parser.add_argument(
        "--config",
        metavar="PATH",
        help="assert the B banner's safety_hash against this config's [safety]",
    )
    parser.add_argument(
        "--ping",
        action="store_true",
        help="send a diagnostic P once a second once the session is known. "
        "This WRITES to the port: never use it while robotd is running (I-18)",
    )
    args = parser.parse_args(argv)

    config, origin = load_config_or_default(args.config)
    bus_sock = config.bus.sock
    expected_hash: int | None = None
    if args.config is not None:
        expected_hash = config.safety_hash()
        print(f"wirecat: [safety] from {origin}, safety_hash={expected_hash}")

    # --ping is the one path that transmits, so it gets the guard roverctl's
    # arm/disarm already has: a live listener on the bus means robotd owns the
    # port, and a second writer advancing the MCU's last_down desynchronises
    # robotd's stream until it resyncs -- a diagnostic that stops the robot.
    if args.ping and _socket_is_live(bus_sock):
        print(
            f"wirecat: {bus_sock} has a listener, so robotd owns {args.device}. "
            "Only robotd writes the port (I-18); --ping would be a second "
            "writer. Stop rover-robotd, or drop --ping to watch read-only.",
            file=sys.stderr,
        )
        return 2

    try:
        fd = open_serial(args.device, args.baud, write=args.ping)
    except OSError as exc:
        print(f"wirecat: cannot open {args.device}: {exc}", file=sys.stderr)
        return 2

    reader = FrameReader()
    started = time.monotonic()
    shown = 0
    session = 0
    down_seq = 0
    next_ping = started + 1.0
    banner_checked = False
    status = 0
    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - started >= args.seconds:
                break
            if args.ping and session and down_seq and now >= next_ping:
                os.write(
                    fd,
                    encode_frame(
                        PingFrame(down_seq, session, time.monotonic_ns() // 1000)
                    ),
                )
                down_seq = (down_seq + 1) & 0xFFFF
                next_ping = now + 1.0
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                data = b""
            except OSError as exc:
                print(f"wirecat: read failed: {exc}", file=sys.stderr)
                return 1
            if not data:
                time.sleep(0.005)
                continue
            for result in reader.feed(data):
                if isinstance(result, DecodeOk):
                    frame = result.frame
                    # The seq re-seed of A8: the MCU's last_down never moves
                    # backwards, so a diagnostic P has to start above its ack.
                    # Re-seeded once, and again on a session change -- which 5.1
                    # says to treat exactly like a port open.
                    if isinstance(frame, TelemetryFrame) and (
                        not down_seq or frame.session != session
                    ):
                        session, down_seq = frame.session, (frame.ack_seq + 1) & 0xFFFF
                    if not _selected(type(frame).TYPE, args.only, args.exclude):
                        continue
                print(render(result, elapsed=now - started, raw=args.raw))
                shown += 1
                if (
                    expected_hash is not None
                    and not banner_checked
                    and isinstance(result, DecodeOk)
                    and isinstance(result.frame, BootFrame)
                ):
                    banner_checked = True
                    got = result.frame.safety_hash
                    matched = got == expected_hash
                    print(
                        f"wirecat: safety_hash "
                        f"{'MATCH' if matched else 'MISMATCH'}: banner {got}, "
                        f"config {expected_hash}"
                    )
                    status = 0 if matched else 1
                if args.count and shown >= args.count:
                    return status
            sys.stdout.flush()
    except KeyboardInterrupt:
        return 130
    finally:
        counters = reader.counters
        print(
            f"wirecat: {counters.ok} ok, {counters.dropped} dropped "
            f"(crc {counters.bad_crc}, length {counters.bad_length}, "
            f"type {counters.unknown_type}, version {counters.unsupported_version}, "
            f"session {counters.bad_session})",
            file=sys.stderr,
        )
        os.close(fd)
    return status


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
