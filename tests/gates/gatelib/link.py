"""The gate's own end of the Pi-MCU link, and a handle on ``mcu-sim``.

G2 benches the controller, so the device under test is the MCU -- simulated over
a pty on the Mac, real over ``/dev/rover-mcu`` on the rover.  The gate therefore
drives the link itself rather than going through robotd: routing G2 through
robotd would test robotd, and the invariants here are the MCU's.

The client half is the architecture's own: re-seed from one ``T`` (5.1), then
``H``, then ``A``, then ``V`` at 20 Hz with ``frame_ttl_ms=300``.  Receive runs
in a thread that timestamps every frame on the Pi's monotonic clock, because
every criterion in I-1, I-5 and I-20 is a time measured on this side --
principle 5 forbids comparing it against ``mcu_us``.
"""

from __future__ import annotations

import contextlib
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from rover_contracts import (
    SESSION_WILDCARD,
    AckFrame,
    ArmFrame,
    BootFrame,
    ClearFaultFrame,
    CtrlFlag,
    DisarmFrame,
    EventFrame,
    FrameReader,
    HelloFrame,
    PingFrame,
    StopFrame,
    TelemetryFrame,
    VelocityFrame,
    encode_frame,
)

__all__ = ["McuLink", "SimProcess", "Sample", "child_env"]


def child_env(repo: Path) -> dict[str, str]:
    """The environment a spawned rover component needs.

    ``packages/`` on ``PYTHONPATH``, exactly as the Makefile exports it, so a
    gate can start ``mcu-sim`` or ``fakebox`` from an uninstalled clone.
    """
    env = dict(os.environ)
    packages = str(repo / "packages")
    existing = env.get("PYTHONPATH", "")
    if packages not in existing.split(os.pathsep):
        env["PYTHONPATH"] = f"{packages}{os.pathsep}{existing}" if existing else packages
    return env


def _open_raw(path: str, baud: int) -> int:
    """A raw, non-blocking fd on a tty, at ``baud`` where the platform can.

    Not pyserial: a macOS pty refuses ``IOSSIOSPEED``, which is how pyserial
    sets a non-standard rate, so opening ``run/mcu.pty`` at 921600 fails with
    ENOTTY.  A pty has no line rate to set in the first place -- the number only
    means anything on the Pi's PL011, where ``termios.B921600`` exists.
    """
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    speed = getattr(termios, f"B{baud}", None)
    if speed is not None:
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = speed  # ispeed, ospeed
        with contextlib.suppress(termios.error, OSError):
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


@dataclass(frozen=True, slots=True)
class Sample:
    """One received frame, stamped on the Pi's monotonic clock."""

    mono: float
    frame: object


class McuLink:
    """A minimal, blocking robotd-shaped writer for the link.

    Not a substitute for robotd: it holds no goal, integrates no odometry and
    arbitrates nothing.  It exists so a gate can put an exact frame on the wire
    at an exact time and measure what came back.
    """

    def __init__(self, port: str | Path, baud: int = 921600) -> None:
        self.port = str(port)
        self.baud = baud
        self.session = SESSION_WILDCARD
        self.down_seq = 1
        self.reader = FrameReader()
        self.samples: deque[Sample] = deque(maxlen=20000)
        self._lock = threading.Lock()
        self._fd: int | None = None
        self._rx: threading.Thread | None = None
        self._stream: threading.Thread | None = None
        self._stop_rx = threading.Event()
        self._stop_stream = threading.Event()
        self._setpoint = (0, 0, 300, 0)
        self.last_tx = 0.0
        """Monotonic time of the last frame written.

        The TTL runs from the last ``V`` the MCU accepted, so every I-1 and I-20
        deadline is measured from here rather than from when the gate decided to
        stop -- the difference is up to one 50 ms stream period, which is a sixth
        of the 300 ms TTL."""

    # -- lifecycle ------------------------------------------------------

    def open(self) -> None:
        self._fd = _open_raw(self.port, self.baud)
        self._stop_rx.clear()
        self._rx = threading.Thread(target=self._pump, name="gate-mcu-rx", daemon=True)
        self._rx.start()

    def close(self) -> None:
        self.stop_stream()
        self._stop_rx.set()
        if self._rx is not None:
            self._rx.join(timeout=1.0)
        if self._fd is not None:
            with contextlib.suppress(OSError):
                os.close(self._fd)
            self._fd = None

    def __enter__(self) -> McuLink:
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- receive --------------------------------------------------------

    def _pump(self) -> None:
        fd = self._fd
        assert fd is not None
        while not self._stop_rx.is_set():
            try:
                ready, _w, _x = select.select([fd], [], [], 0.02)
                if not ready:
                    continue
                data = os.read(fd, 4096)
            except (OSError, ValueError):
                return
            if not data:
                continue
            now = time.monotonic()
            for result in self.reader.feed(data):
                if result.ok:
                    with self._lock:
                        frame = result.frame  # type: ignore[attr-defined]
                        self.samples.append(Sample(now, frame))

    def snapshot(self) -> list[Sample]:
        with self._lock:
            return list(self.samples)

    def latest(self, kind: type) -> Sample | None:
        with self._lock:
            for sample in reversed(self.samples):
                if isinstance(sample.frame, kind):
                    return sample
        return None

    def telemetry(self) -> TelemetryFrame | None:
        sample = self.latest(TelemetryFrame)
        return sample.frame if sample is not None else None  # type: ignore[return-value]

    def wait(
        self,
        kind: type,
        predicate: Callable[[object], bool],
        timeout: float,
        *,
        since: float | None = None,
    ) -> Sample | None:
        """Block until a frame of ``kind`` satisfies ``predicate``."""
        start = time.monotonic() if since is None else since
        end = time.monotonic() + timeout
        seen = 0
        while time.monotonic() < end:
            with self._lock:
                samples = [
                    s
                    for s in self.samples
                    if s.mono >= start and isinstance(s.frame, kind)
                ]
            for sample in samples[seen:]:
                if predicate(sample.frame):
                    return sample
            seen = len(samples)
            time.sleep(0.002)
        return None

    def events(self, since: float) -> list[EventFrame]:
        return [
            s.frame  # type: ignore[misc]
            for s in self.snapshot()
            if s.mono >= since and isinstance(s.frame, EventFrame)
        ]

    def acks(self, since: float) -> list[AckFrame]:
        return [
            s.frame  # type: ignore[misc]
            for s in self.snapshot()
            if s.mono >= since and isinstance(s.frame, AckFrame)
        ]

    def boots(self, since: float) -> list[BootFrame]:
        return [
            s.frame  # type: ignore[misc]
            for s in self.snapshot()
            if s.mono >= since and isinstance(s.frame, BootFrame)
        ]

    # -- transmit -------------------------------------------------------

    def raw(self, data: bytes) -> None:
        """Put bytes on the wire untouched -- corruption injection for I-2."""
        fd = self._fd
        assert fd is not None
        while data:
            data = data[os.write(fd, data) :]
        self.last_tx = time.monotonic()

    def send(self, frame: object) -> int:
        """Encode and write one frame; returns the seq it went out with."""
        self.raw(encode_frame(frame))  # type: ignore[arg-type]
        return frame.seq  # type: ignore[attr-defined]

    def next_seq(self) -> int:
        seq = self.down_seq
        self.down_seq = (self.down_seq + 1) & 0xFFFF or 1
        return seq

    def reseed(self, wait_s: float = 0.5) -> bool:
        """ARCHITECTURE 5.1: listen for one ``T``, adopt ``ack_seq + 1``.

        Returns True when a ``T`` supplied the seed.  On timeout the fallback is
        ``down_seq = 1`` with the wildcard session, which is correct for a fresh
        MCU whose ``last_down`` is 0 -- and is the case ``no_t_before_h`` covers.
        """
        sample = self.wait(TelemetryFrame, lambda _f: True, wait_s)
        if sample is None:
            self.session = SESSION_WILDCARD
            self.down_seq = 1
            return False
        frame: TelemetryFrame = sample.frame  # type: ignore[assignment]
        self.session = frame.session
        self.down_seq = (frame.ack_seq + 1) & 0xFFFF or 1
        return True

    def hello(self, host_boot_id: int = 0xDEADBEEF) -> int:
        session = self.session
        return self.send(HelloFrame(self.next_seq(), session, host_boot_id))

    def arm(self, nonce: int = 90210, timeout: float = 1.0) -> AckFrame | None:
        if self.session == SESSION_WILDCARD:
            telemetry = self.telemetry()
            if telemetry is not None:
                self.session = telemetry.session
        since = time.monotonic()
        seq = self.send(ArmFrame(self.next_seq(), self.session, nonce))
        sample = self.wait(
            AckFrame,
            lambda f: f.ack_type == ord("A") and f.ack_seq == seq,  # type: ignore[attr-defined]
            timeout,
            since=since,
        )
        return sample.frame if sample is not None else None  # type: ignore[return-value]

    def disarm(self) -> int:
        return self.send(DisarmFrame(self.next_seq(), self.session))

    def stop(self, mode: int = 0) -> int:
        return self.send(StopFrame(self.next_seq(), self.session, mode))

    def clear(self, mask: int) -> int:
        return self.send(ClearFaultFrame(self.next_seq(), self.session, mask))

    def ping(self, token: int) -> int:
        return self.send(PingFrame(self.next_seq(), self.session, token))

    def velocity(
        self, v_mm_s: int, w_mrad_s: int, ttl_ms: int = 300, flags: int = 0
    ) -> int:
        return self.send(
            VelocityFrame(self.next_seq(), self.session, v_mm_s, w_mrad_s, ttl_ms, flags)
        )

    # -- the 20 Hz setpoint stream --------------------------------------

    def stream(
        self, v_mm_s: int, w_mrad_s: int, ttl_ms: int = 300, flags: int = 0
    ) -> None:
        """Start or re-aim the 20 Hz ``V`` stream robotd would be emitting."""
        self._setpoint = (v_mm_s, w_mrad_s, ttl_ms, flags)
        if self._stream is not None and self._stream.is_alive():
            return
        self._stop_stream.clear()
        self._stream = threading.Thread(
            target=self._stream_loop, name="gate-mcu-v", daemon=True
        )
        self._stream.start()

    def _stream_loop(self) -> None:
        period = 0.05
        next_at = time.monotonic()
        while not self._stop_stream.is_set():
            v, w, ttl, flags = self._setpoint
            try:
                self.velocity(v, w, ttl, flags)
            except OSError:
                return
            next_at += period
            time.sleep(max(0.0, next_at - time.monotonic()))

    def stop_stream(self) -> None:
        """Stop renewing.  This is how I-1's cable pull is simulated in sim."""
        self._stop_stream.set()
        if self._stream is not None:
            self._stream.join(timeout=1.0)
            self._stream = None

    def bring_up(
        self, reseed_wait_s: float = 0.5, cal_wait_s: float = 5.0
    ) -> tuple[bool, AckFrame | None]:
        """Re-seed, ``H``, ``A`` -- the sequence robotd runs on port open.

        Plus one wait the sequence itself does not contain: ``ctrl_flags`` b6
        ``cal_valid``.  The MCU takes 50 cliff samples on entry to DISARMED and
        refuses *all* forward motion until their median is stored (4.1), so a
        gate that arms at 100 ms gets an accepted ``A``, an accepted ``V`` and a
        stationary robot -- which looks like a broken plant and is not one.
        """
        self.wait(
            TelemetryFrame, lambda f: bool(f.ctrl_flags & CtrlFlag.CAL_VALID), cal_wait_s
        )
        seeded = self.reseed(reseed_wait_s)
        self.hello()
        time.sleep(0.05)
        return seeded, self.arm()


class SimProcess:
    """``mcu-sim`` as a subprocess, with the flags of ARCHITECTURE 10.

    The flags are passed as the bare ``key=value`` tokens the document writes
    (``mcu-sim obstacle=231``).  ``--sim-cmd`` on every gate overrides the
    command, so the gates keep running unchanged if the simulator's entry point
    differs from ``python -m rover_devtools.mcu_sim``.

    The default link is ``run/gate-mcu.pty``, not the ``run/mcu.pty`` that
    ``[serial] port`` names: a gate that took over that symlink would pull the
    port out from under a running ``make sim``, and the cases that spawn their
    own simulator drive the link directly and never go through robotd.
    """

    MODULE = "rover_devtools.mcu_sim"

    def __init__(
        self,
        flags: Sequence[str] = (),
        *,
        command: Sequence[str] | None = None,
        pty_path: str | Path = "run/gate-mcu.pty",
        cwd: str | Path | None = None,
    ) -> None:
        self.flags = list(flags)
        default = command is None
        self.command = list(command) if command else [sys.executable, "-m", self.MODULE]
        # The wrapper's own --link default is ./run/mcu.pty, so the path has to
        # be handed over explicitly.  Only for the default command: --sim-cmd
        # exists because the entry point may not take this flag.
        if default:
            self.command += ["--link", str(pty_path)]
        self.pty_path = Path(pty_path)
        self.cwd = Path(cwd) if cwd else Path.cwd()
        self.process: subprocess.Popen[bytes] | None = None
        self.error = ""

    def start(self, timeout: float = 5.0) -> bool:
        """Spawn, and wait for the pty symlink to appear.  False with
        :attr:`error` set when it did not."""
        target = (
            self.pty_path if self.pty_path.is_absolute() else self.cwd / self.pty_path
        )
        with contextlib.suppress(OSError):
            target.unlink()
        try:
            self.process = subprocess.Popen(
                [*self.command, *self.flags],
                cwd=str(self.cwd),
                env=child_env(self.cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            self.error = f"could not start {' '.join(self.command)}: {exc}"
            return False
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if target.exists() or os.path.islink(target):
                return True
            if self.process.poll() is not None:
                self.error = self._tail()
                return False
            time.sleep(0.05)
        self.error = f"{target} did not appear within {timeout:.1f}s; {self._tail()}"
        return False

    def _tail(self) -> str:
        """The first line of the simulator's complaint, not its whole usage text."""
        if self.process is None:
            return ""
        try:
            _out, err = self.process.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            return "still running"
        lines = [
            line.strip()
            for line in err.decode("utf-8", "replace").splitlines()
            if line.strip()
        ]
        head = lines[0] if lines else "exited with no message"
        if self.flags:
            head += f" (flags: {' '.join(self.flags)})"
        return head[:200]

    def stop(self) -> None:
        if self.process is None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.process = None

    def __enter__(self) -> SimProcess:
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()
