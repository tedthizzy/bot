"""Run the firmware host simulator behind a pseudo terminal.

ARCHITECTURE 10: ``mcu-sim`` links the same ``firmware/core/`` C code that runs
on the ESP32-S3, mints a random session, streams 50 Hz telemetry and runs the
TTL.  This module is the thin wrapper that gives it a *device path*: it
allocates a pty, pumps bytes between the pty master and the simulator's stdin
and stdout, and symlinks ``./run/mcu.pty`` at the slave the OS handed out
(``/dev/ttysNNN`` on macOS is unpredictable).  robotd then opens
``[serial] port`` exactly as it opens ``/dev/rover-mcu`` on the Pi -- principle
6, fakes are configuration and not a code branch.

Pure stdlib, so it starts before anything is installed.  The fault-injection
flags are passed through to the binary verbatim, in the spelling ARCHITECTURE
10 uses (``obstacle=231``, ``tof_error=fl``, ``no_t_before_h``).
"""

from __future__ import annotations

import argparse
import contextlib
import os
import pty
import selectors
import signal
import subprocess
import sys
import tty
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import FrameType, TracebackType

__all__ = [
    "BUILD_HINT",
    "FAULT_FLAGS",
    "McuSim",
    "SEARCH_PATHS",
    "SimulatorMissing",
    "find_binary",
    "main",
    "parse_flags",
]

BINARY_ENV = "ROVER_MCU_SIM"

SEARCH_PATHS: tuple[str, ...] = (
    "firmware/build/host/mcu-sim",
    "firmware/host/build/mcu-sim",
    "firmware/build/mcu-sim",
    "build/mcu-sim",
)
"""Where the host build drops the binary, in order.  ``$ROVER_MCU_SIM`` wins."""

BUILD_HINT = (
    "Build it with `make sim` (the host build, cmake -DROVER_HOST_TEST=ON), "
    f"or point {BINARY_ENV} at the binary."
)

DEFAULT_LINK = "./run/mcu.pty"
"""``config/robot.mac.toml`` sets ``[serial] port`` to exactly this."""

FAULT_FLAGS: dict[str, bool] = {
    # name -> takes a value.  ARCHITECTURE 10, in its own spelling.
    "ttl_drop": False,
    "garbage": False,
    "crc_flip": False,
    "reset_mid_drive": False,
    "hang": True,
    "obstacle": True,
    "no_target": False,
    "tof_error": True,
    "cliff": False,
    "bumper": False,
    "estop": False,
    "seq_replay": False,
    "seq_desync": False,
    "no_t_before_h": False,
    "lag": True,
    "vbat": True,
    "stall": False,
    "stall_one_channel": False,
    "slip_one_channel": False,
}

_TOF_ERROR_VALUES = frozenset({"fl", "fr", "both"})


class SimulatorMissing(FileNotFoundError):
    """The host simulator binary is not built.  The message names the target."""


def parse_flags(tokens: Iterable[str]) -> list[str]:
    """Validate fault-injection tokens and return them unchanged.

    Names and value-ness are checked here so a typo fails at the wrapper rather
    than being swallowed by an argv the binary ignores.  Values themselves stay
    opaque -- ``lag=200ms`` carries its own unit and the binary owns its range.
    """
    checked: list[str] = []
    for token in tokens:
        name, sep, value = token.partition("=")
        if name not in FAULT_FLAGS:
            known = ", ".join(sorted(FAULT_FLAGS))
            raise ValueError(f"unknown fault flag {name!r}; known flags: {known}")
        if FAULT_FLAGS[name] and not sep:
            raise ValueError(f"fault flag {name!r} needs a value, as {name}=<value>")
        if not FAULT_FLAGS[name] and sep:
            raise ValueError(f"fault flag {name!r} takes no value, got {token!r}")
        if name == "tof_error" and value not in _TOF_ERROR_VALUES:
            raise ValueError(f"tof_error must be fl, fr or both, got {value!r}")
        checked.append(token)
    return checked


def find_binary(explicit: str | Path | None = None) -> Path:
    """Locate the host simulator, or raise with the target that builds it."""
    if explicit is not None:
        path = Path(explicit)
        if path.is_file() and os.access(path, os.X_OK):
            return path
        raise SimulatorMissing(f"{path} is not an executable file. {BUILD_HINT}")
    tried: list[str] = []
    from_env = os.environ.get(BINARY_ENV)
    candidates = [from_env] if from_env else []
    candidates += list(SEARCH_PATHS)
    for candidate in candidates:
        path = Path(candidate)
        tried.append(str(path))
        if path.is_file() and os.access(path, os.X_OK):
            return path
    listing = "\n".join(f"  {entry}" for entry in tried)
    raise SimulatorMissing(
        f"mcu-sim binary not found. Looked in:\n{listing}\n{BUILD_HINT}"
    )


def _symlink(target: str, link: Path) -> None:
    """Replace ``link`` with a symlink to ``target`` without a gap."""
    link.parent.mkdir(parents=True, exist_ok=True)
    tmp = link.with_name(link.name + f".{os.getpid()}.tmp")
    if tmp.is_symlink() or tmp.exists():
        tmp.unlink()
    os.symlink(target, tmp)
    os.replace(tmp, link)


@dataclass
class McuSim:
    """One running simulator and the pty it speaks through.

    The parent keeps the pty *slave* open for its whole life, so robotd may
    close and reopen the port -- an unplug/replug, I-13 -- without the master
    ever seeing EOF.
    """

    binary: Path
    flags: Sequence[str] = ()
    link: Path | None = None

    device: str = field(default="", init=False)
    process: subprocess.Popen[bytes] | None = field(default=None, init=False)
    _master: int = field(default=-1, init=False)
    _slave: int = field(default=-1, init=False)

    def start(self) -> str:
        """Allocate the pty, spawn the binary, and return the device path."""
        self._master, self._slave = pty.openpty()
        tty.setraw(self._slave)
        self.device = os.ttyname(self._slave)
        if self.link is not None:
            _symlink(self.device, self.link)
        try:
            self.process = subprocess.Popen(  # noqa: S603 - argv is validated above
                [str(self.binary), *self.flags],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                bufsize=0,
            )
        except OSError:
            self.close()
            raise
        return self.device

    def pump(self) -> int:
        """Copy bytes both ways until the simulator exits; return its code."""
        process = self.process
        if process is None or process.stdin is None or process.stdout is None:
            raise RuntimeError("start() first")
        child_out = process.stdout.fileno()
        child_in = process.stdin.fileno()
        selector = selectors.DefaultSelector()
        selector.register(self._master, selectors.EVENT_READ, "host")
        selector.register(child_out, selectors.EVENT_READ, "sim")
        try:
            while True:
                for key, _ in selector.select(timeout=0.2):
                    if key.data == "host":
                        data = _read(self._master)
                        target = child_in
                    else:
                        data = _read(child_out)
                        target = self._master
                    if data is None:
                        return self._reap()
                    if data:
                        _write_all(target, data)
                if process.poll() is not None:
                    return self._reap()
        finally:
            selector.close()

    def _reap(self) -> int:
        process = self.process
        assert process is not None
        try:
            return process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()

    def close(self) -> None:
        """Stop the simulator and remove the symlink."""
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for name in ("stdin", "stdout"):
            stream = getattr(process, name, None)
            if stream is not None and not stream.closed:
                stream.close()
        for fd in (self._master, self._slave):
            if fd >= 0:
                with contextlib.suppress(OSError):
                    os.close(fd)
        self._master = self._slave = -1
        if self.link is not None and self.link.is_symlink():
            with contextlib.suppress(OSError):
                self.link.unlink()

    def __enter__(self) -> McuSim:
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


def _read(fd: int) -> bytes | None:
    """Read what is ready.  ``None`` means the far end is gone."""
    try:
        data = os.read(fd, 4096)
    except OSError:
        return None
    return data if data else None


def _write_all(fd: int, data: bytes) -> None:
    while data:
        try:
            written = os.write(fd, data)
        except BlockingIOError:
            continue
        except OSError:
            return
        data = data[written:]


def _terminate_on_sigterm(signum: int, frame: FrameType | None) -> None:
    """``make dev`` stops the simulator with SIGTERM; turn it into the same
    unwind Ctrl-C takes, so the symlink is removed either way."""
    raise KeyboardInterrupt


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rover_devtools.mcu_sim",
        description=(
            "Run the firmware host simulator behind a pty and print its device "
            "path, so robotd opens it exactly as it opens the real port."
        ),
        epilog="fault flags: " + ", ".join(sorted(FAULT_FLAGS)),
    )
    parser.add_argument(
        "flags",
        nargs="*",
        metavar="FLAG",
        help="fault injection, e.g. obstacle=231 tof_error=fl no_t_before_h",
    )
    parser.add_argument("--binary", help=f"simulator path (default: ${BINARY_ENV})")
    parser.add_argument(
        "--link",
        default=DEFAULT_LINK,
        help=f"symlink to point at the pty slave (default: {DEFAULT_LINK})",
    )
    parser.add_argument(
        "--no-link", action="store_true", help="do not create the symlink"
    )
    args = parser.parse_args(argv)

    try:
        flags = parse_flags(args.flags)
        binary = find_binary(args.binary)
    except (ValueError, SimulatorMissing) as exc:
        print(f"mcu-sim: {exc}", file=sys.stderr)
        return 2

    sim = McuSim(
        binary=binary,
        flags=flags,
        link=None if args.no_link else Path(args.link),
    )
    device = sim.start()
    print(f"mcu-sim: device {device}")
    if sim.link is not None:
        print(f"mcu-sim: symlink {sim.link} -> {device}")
    print(f"mcu-sim: {binary}" + (f" {' '.join(flags)}" if flags else ""))
    sys.stdout.flush()

    signal.signal(signal.SIGTERM, _terminate_on_sigterm)
    try:
        return sim.pump()
    except KeyboardInterrupt:
        return 130
    finally:
        sim.close()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
