"""The serial link to the MCU.  robotd owns ``/dev/rover-mcu`` and is its only
writer (I-18); this module is the only place in robotd that writes bytes to it.

What lives here is everything the wire forces to be in one place: the hello
handshake and the bounded sequence re-seed of A8, the learned session, the one
down-direction sequence counter shared by all frame types, the 20 Hz ``V``
stream, telemetry ingest stamped with a **host-monotonic arrival time**, and a
reconnect that mints nothing of its own, starts disarmed and never replays
motion (I-13).

Two rules shape the file:

* **No host clock is ever compared against the MCU clock** (I-17, principle 5).
  ``T.mcu_us`` and ``E.mcu_us`` are recorded and logged and never differenced
  with anything of ours; freshness is the difference between two host arrival
  times.  ``P.pi_mono_us`` is our own token, echoed back in ``O`` unmodified,
  and the round-trip is computed entirely on our clock -- the one allow-listed
  echo of I-17's grep gate.
* **An invalid frame renews nothing** (I-2, principle 3).  A line failing
  length, CRC, type, version, session or seq is dropped and counted by the
  codec's :class:`~rover_contracts.serial_codec.FrameReader` and never reaches
  the ingest path, so it cannot refresh telemetry age, the learned session or
  the arm state.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import secrets
import termios
import time
import tty
from collections.abc import Callable
from typing import Final

from rover_contracts.config import RobotConfig
from rover_contracts.serial_codec import (
    DOWN_FRAME_TYPES,
    SESSION_WILDCARD,
    AckFrame,
    AckReason,
    AckResult,
    ArmFrame,
    BootFrame,
    ClearFaultFrame,
    DecodeOk,
    DisarmFrame,
    Frame,
    FrameReader,
    HelloFrame,
    McuState,
    PingFrame,
    PongFrame,
    SessionGuard,
    StopFrame,
    TelemetryFrame,
    VelocityFrame,
    ack_type_code,
    encode_frame,
)
from rover_contracts.skills import SKILLS, TWIST, mcu_cap_magnitude
from rover_contracts.units import mps_to_mm_s, radps_to_mrad_s

from rover_robotd.profiles import SetpointCell

__all__ = [
    "MAX_V_MM_S",
    "MAX_W_MRAD_S",
    "Link",
    "ResyncReason",
    "open_serial",
]

log = logging.getLogger("rover.robotd.link")

_TWIST_BOUNDS: Final = {bound.field: bound for bound in SKILLS[TWIST].bounds}
MAX_V_MM_S: Final = mcu_cap_magnitude(_TWIST_BOUNDS["linear_x_mps"])
MAX_W_MRAD_S: Final = mcu_cap_magnitude(_TWIST_BOUNDS["angular_z_radps"])
"""The last narrowing before the port, taken from ``skills.CATALOG`` rather
than written out again: 300 mm/s and 1047 mrad/s.  The MCU's own compiled caps
(300 / 1200) sit at or above these, so this clamp can only narrow -- I-8's
"zero out-of-bounds values reach the port" with nothing to keep in step."""

_PING_PERIOD_S: Final = 1.0
_ARM_ACK_CODE: Final = ack_type_code(ArmFrame.TYPE)

ResyncReason = str


def open_serial(backend: str, port: str, baud: int) -> int:
    """Open the device raw and non-blocking, and return its fd.

    The two backends are the platform adapter of principle 6 and are chosen by
    ``[serial] backend``, never by inspecting the platform: ``pty`` is the
    ``mcu-sim`` slave on the Mac, which has no line speed, and ``uart`` is
    ``/dev/rover-mcu`` on the Pi, configured 8N1 at ``baud`` through stdlib
    ``termios``.
    """
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        tty.setraw(fd)
        if backend == "uart":
            speed = getattr(termios, f"B{baud}", None)
            if speed is None:
                raise OSError(f"this platform has no termios constant for {baud} baud")
            attrs = termios.tcgetattr(fd)
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = speed
            attrs[5] = speed
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except BaseException:
        os.close(fd)
        raise
    return fd


def _clamp(value: int, limit: int) -> int:
    return max(-limit, min(limit, value))


class Link:
    """One long-lived connection to the MCU, reopened for as long as it runs."""

    def __init__(
        self,
        config: RobotConfig,
        cell: SetpointCell,
        *,
        on_frame: Callable[[Frame], None] | None = None,
        on_resync: Callable[[int, int, int | None, ResyncReason], None] | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = config
        self.cell = cell
        self.host_boot_id = secrets.randbits(32)
        self.session = SESSION_WILDCARD
        self.telemetry: TelemetryFrame | None = None
        self.telemetry_arrival_ns: int | None = None
        self.boot: BootFrame | None = None
        self.rtt_ms: float | None = None
        self.reader = FrameReader()

        self._clock = clock
        self._on_frame = on_frame
        self._on_resync = on_resync
        self._fd: int | None = None
        self._out = bytearray()
        self._down_seq = 1
        self._guard = SessionGuard()
        self._armed = False
        self._ready = False
        self._stale_logged = False
        self._arm_nonce: int | None = None
        self._ping_sent_us: int | None = None
        self._reseed: asyncio.Future[tuple[int, int, int]] | None = None
        self._resync = asyncio.Event()
        self._resync_reason: ResyncReason = "port open"
        self._closed = asyncio.Event()
        self._closing = False
        self._loop: asyncio.AbstractEventLoop | None = None

    # -- observable state ---------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._fd is not None

    @property
    def ready(self) -> bool:
        """The re-seed has completed and ``H`` has been sent (A8)."""
        return self._ready

    @property
    def armed(self) -> bool:
        """``state.armed``, so I-3 is observable from the bus."""
        return self._armed

    @property
    def link_down(self) -> bool:
        """T0: telemetry older than ``cmd_gate_max_age_ms``, or none at all.

        Derived from the arrival times rather than latched by the writer, so it
        reads the same whether or not the link happens to be armed.  This is the
        *command* gate of section 7 -- what stops ``V`` going out and fails the
        active goal.
        """
        age = self.telemetry_age_ms()
        return age is None or age > self.config.serial.cmd_gate_max_age_ms

    @property
    def link_alive(self) -> bool:
        """``[safety] link_alive_max_age_ms``: what readiness reads (5.8).

        Deliberately looser than the command gate above, and the config
        validator enforces ``cmd_gate_max_age_ms < link_alive_max_age_ms``: a
        gap wide enough to stop commanding is not yet wide enough to call the
        controller absent.  Section 14's LeKiwi ``is_connected`` is this same
        predicate, stated as ``state.mcu.age_ms < link_alive_max_age_ms``.
        """
        age = self.telemetry_age_ms()
        return age is not None and age < self.config.safety.link_alive_max_age_ms

    def telemetry_age_ms(self, now_mono_ns: int | None = None) -> float | None:
        """Age of the newest accepted ``T``, on the host's monotonic clock."""
        if self.telemetry_arrival_ns is None:
            return None
        now = self._clock() if now_mono_ns is None else now_mono_ns
        return (now - self.telemetry_arrival_ns) / 1e6

    # -- lifecycle ----------------------------------------------------------

    async def run(self) -> None:
        """Open, serve and reopen until :meth:`close`.

        ``open()`` is retried every ``[serial] open_retry_ms`` until the path
        exists, which is what removes ``make dev``'s start-order dependency and
        is the same behaviour ``Restart=always`` needs on the Pi while the MCU
        is unplugged.
        """
        retry_s = self.config.serial.open_retry_ms / 1000.0
        while not self._closing:
            try:
                await self._serve()
            except asyncio.CancelledError:
                raise
            except OSError as exc:
                log.info("serial link unavailable (%s); retrying", exc)
            except Exception:  # pragma: no cover - defensive
                log.exception("serial link failed; retrying")
            finally:
                self._teardown()
            if not self._closing:
                await asyncio.sleep(retry_s)

    async def close(self) -> None:
        """Stop and disarm, then stop reopening the port."""
        if self.connected and self._ready:
            self.stop(mode=0)
            self.disarm()
            await asyncio.sleep(0)
        self._closing = True
        self._closed.set()

    async def _serve(self) -> None:
        serial = self.config.serial
        self._fd = open_serial(serial.backend, serial.port, serial.baud)
        self._closed.clear()
        self._resync.clear()
        loop = self._loop = asyncio.get_running_loop()
        loop.add_reader(self._fd, self._on_readable)
        tasks = [
            asyncio.create_task(self._writer_loop()),
            asyncio.create_task(self._ping_loop()),
        ]
        log.info("serial link open on %s (%s)", serial.port, serial.backend)
        try:
            reason: ResyncReason = "port open"
            while not self._closed.is_set() and not self._closing:
                await self._handshake(reason)
                waiters: list[asyncio.Task[bool]] = [
                    asyncio.create_task(self._resync.wait()),
                    asyncio.create_task(self._closed.wait()),
                ]
                _, pending = await asyncio.wait(
                    waiters, return_when=asyncio.FIRST_COMPLETED
                )
                for waiter in pending:
                    waiter.cancel()
                if self._closed.is_set() or self._closing:
                    break
                self._resync.clear()
                reason = self._resync_reason
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    def _teardown(self) -> None:
        if self._fd is None:
            return
        if self._loop is not None:
            for remove in (self._loop.remove_reader, self._loop.remove_writer):
                with contextlib.suppress(RuntimeError, ValueError):  # pragma: no cover
                    remove(self._fd)
        with contextlib.suppress(OSError):  # pragma: no cover
            os.close(self._fd)
        self._fd = None
        # Whatever _flush could not write dies with the port.  Left in place,
        # the next _serve appends _handshake's deliberate leading newline behind
        # it and flushes: the MCU's framer then sees a truncated frame from the
        # dead session, fails its CRC, and consumes the resync newline as that
        # fragment's terminator -- so the one byte whose whole job is to put the
        # framer on a line boundary does the opposite.
        self._out.clear()
        self._ready = False
        self._armed = False
        self.telemetry_arrival_ns = None
        self.cell.zero()

    # -- handshake and re-seed ---------------------------------------------

    async def _handshake(self, reason: ResyncReason) -> None:
        """A8's bounded re-seed, then ``H``.

        Listen for one ``T`` for up to ``reseed_wait_ms``, adopt
        ``down_seq = T.ack_seq + 1`` and seed the up-direction guard from that
        frame.  On timeout, or on a ``B`` -- a banner means the MCU has not
        accepted an ``H``, so its ``last_down`` is 0 -- fall back to
        ``down_seq = 1``.  Nothing is sent until this finishes, and the setpoint
        is dropped first so a reconnect can never replay motion (I-13).
        """
        old_session = self.session
        self._ready = False
        self._armed = False
        self.cell.zero()
        self._guard = SessionGuard()
        loop = asyncio.get_running_loop()
        self._reseed = loop.create_future()
        # A leading newline is legal on this link and costs one byte: it puts
        # the MCU's framer on a line boundary after a robotd restart that may
        # have died mid-frame.
        self._write(b"\n")
        try:
            down_seq, session, up_seq = await asyncio.wait_for(
                self._reseed, self.config.serial.reseed_wait_ms / 1000.0
            )
        except TimeoutError:
            down_seq, session, up_seq = 1, SESSION_WILDCARD, -1
            log.info("no T within reseed_wait_ms; falling back to down_seq = 1")
        finally:
            self._reseed = None
        self._down_seq = down_seq
        self.session = session
        self._guard = SessionGuard(
            session=session, last_seq=None if up_seq < 0 else up_seq
        )
        self._send(
            HelloFrame(
                seq=self._next_seq(),
                session=SESSION_WILDCARD,
                host_boot_id=self.host_boot_id,
            )
        )
        self._ready = True
        log.info(
            "link ready: session %s, down_seq %s (%s)", session, down_seq, reason
        )
        if self._on_resync is not None:
            reset_reason = self.boot.reset_reason if self.boot is not None else None
            self._on_resync(old_session, session, reset_reason, reason)

    def _request_resync(self, reason: ResyncReason) -> None:
        if self._ready and not self._resync.is_set():
            self._resync_reason = reason
            self._ready = False
            self._armed = False
            self.cell.zero()
            self._resync.set()

    # -- reading ------------------------------------------------------------

    def _on_readable(self) -> None:
        assert self._fd is not None
        try:
            data = os.read(self._fd, 4096)
        except BlockingIOError:  # pragma: no cover - spurious readiness
            return
        except OSError as exc:
            log.info("serial read failed (%s); reopening", exc)
            self._closed.set()
            return
        if not data:
            log.info("serial port reached EOF; reopening")
            self._closed.set()
            return
        for result in self.reader.feed(data):
            if isinstance(result, DecodeOk):
                self._ingest(result.frame)
            # A DecodeErr is already counted by the reader and is dropped here
            # without touching any state: an invalid frame renews nothing.

    def _ingest(self, frame: Frame) -> None:
        if type(frame).TYPE in DOWN_FRAME_TYPES:
            return  # our own echo on a loopback; never a controller message

        if not self._ready:
            self._resolve_reseed(frame)
            return

        if isinstance(frame, BootFrame):
            self.boot = frame
            self._request_resync("boot banner")
            return
        if self.session == SESSION_WILDCARD:
            # The bounded re-seed timed out and left the wildcard behind.  Both
            # `_emit_setpoint` and `arm` bail on it, so without this the link
            # never sends a V or an A again for the life of the process: a real
            # MCU stops sending B the moment it accepts our H, so no banner is
            # coming.  A T proves the stream is live, which is exactly what the
            # re-seed needed and did not have.
            self._request_resync("session learned")
            return
        if frame.session != self.session:
            self._request_resync("session change")
            return
        reason = self._guard.accept(frame)
        if reason is not AckReason.NONE:
            # Stale seq or a foreign session: dropped, renews nothing, and
            # counted on the same terms the controller counts it (A8).
            self.reader.note_drop(reason)
            return
        self._handle(frame)

    def _resolve_reseed(self, frame: Frame) -> None:
        future = self._reseed
        if future is None or future.done():
            return
        if isinstance(frame, BootFrame):
            self.boot = frame
            future.set_result((1, frame.session, frame.seq))
        elif isinstance(frame, TelemetryFrame):
            future.set_result(
                ((frame.ack_seq + 1) & 0xFFFF, frame.session, frame.seq)
            )

    def _handle(self, frame: Frame) -> None:
        if isinstance(frame, TelemetryFrame):
            self.telemetry = frame
            self.telemetry_arrival_ns = self._clock()
            if frame.state in (
                McuState.BOOT,
                McuState.DISARMED,
                McuState.FAULT,
                McuState.ESTOP,
            ):
                self._armed = False
        elif isinstance(frame, AckFrame):
            if frame.ack_type == _ARM_ACK_CODE and frame.echo == self._arm_nonce:
                self._armed = frame.result == AckResult.OK
        elif isinstance(frame, PongFrame):
            self._note_pong(frame)
        if self._on_frame is not None:
            self._on_frame(frame)

    def _note_pong(self, frame: PongFrame) -> None:
        """Round-trip time, computed entirely on our own clock.

        ``echo_pi_mono_us`` is the token we sent in ``P``; the MCU never read
        it as a time and neither does anything here read an MCU timestamp.
        """
        if self._ping_sent_us is None or frame.echo_pi_mono_us != self._ping_sent_us:
            return
        elapsed_ns = self._clock() - frame.echo_pi_mono_us * 1000
        if elapsed_ns >= 0:
            self.rtt_ms = elapsed_ns / 1e6

    # -- writing ------------------------------------------------------------

    async def _writer_loop(self) -> None:
        """The 20 Hz ``V`` stream.  Its only input is the setpoint cell."""
        period = 1.0 / max(1, self.config.serial.setpoint_hz)
        loop = asyncio.get_running_loop()
        next_at = loop.time()
        while True:
            next_at += period
            delay = next_at - loop.time()
            if delay < -period:
                next_at = loop.time()
                delay = 0.0
            await asyncio.sleep(max(0.0, delay))
            self._emit_setpoint()

    def _emit_setpoint(self) -> None:
        now = self._clock()
        v_mps, w_radps = self.cell.read(now)
        if not self._ready or not self._armed or self.session == SESSION_WILDCARD:
            return
        age_ms = self.telemetry_age_ms(now)
        if age_ms is None or age_ms > self.config.serial.cmd_gate_max_age_ms:
            # T0: refuse to command a controller we cannot observe.
            if not self._stale_logged:
                self._stale_logged = True
                log.warning("telemetry stale (%s ms); no V emitted", age_ms)
            return
        self._stale_logged = False
        self._send(
            VelocityFrame(
                seq=self._next_seq(),
                session=self.session,
                v_mm_s=_clamp(mps_to_mm_s(v_mps), MAX_V_MM_S),
                w_mrad_s=_clamp(radps_to_mrad_s(w_radps), MAX_W_MRAD_S),
                frame_ttl_ms=self.config.limits.frame_ttl_ms,
                flags=0,
            )
        )

    async def _ping_loop(self) -> None:
        """The 1 Hz diagnostic RTT probe.  ``P``/``O`` are never a liveness
        input -- the heartbeat the MCU watches is the ``V`` stream."""
        while True:
            await asyncio.sleep(_PING_PERIOD_S)
            if not self._ready or self.session == SESSION_WILDCARD:
                continue
            self._ping_sent_us = self._clock() // 1000
            self._send(
                PingFrame(
                    seq=self._next_seq(),
                    session=self.session,
                    pi_mono_us=self._ping_sent_us,
                )
            )

    def _next_seq(self) -> int:
        seq = self._down_seq
        self._down_seq = (self._down_seq + 1) & 0xFFFF
        return seq

    def _send(self, frame: Frame) -> None:
        line = encode_frame(frame)
        # I-18 makes robotd the only writer, so `wirecat` cannot be attached to
        # a live port to see what went out: this is the wire, at DEBUG, off in
        # every default configuration.
        if log.isEnabledFor(logging.DEBUG):
            log.debug("tx %s", line.rstrip(b"\r\n").decode("ascii", "replace"))
        self._write(line)

    def _write(self, data: bytes) -> None:
        if self._fd is None:
            return
        self._out += data
        self._flush()

    def _flush(self) -> None:
        if self._fd is None or not self._out:
            return
        try:
            written = os.write(self._fd, self._out)
        except BlockingIOError:
            written = 0
        except OSError as exc:  # pragma: no cover - the reader sees this too
            log.info("serial write failed (%s); reopening", exc)
            self._closed.set()
            return
        del self._out[:written]
        if self._loop is None:
            return
        if self._out:
            self._loop.add_writer(self._fd, self._flush)
        else:
            with contextlib.suppress(RuntimeError, ValueError):  # pragma: no cover
                self._loop.remove_writer(self._fd)

    # -- commands -----------------------------------------------------------

    def arm(self) -> int | None:
        """Send ``A`` with a fresh nonce; ``K.echo`` makes a duplicate visible."""
        if not self._ready or self.session == SESSION_WILDCARD:
            return None
        self._arm_nonce = secrets.randbits(32)
        self._send(
            ArmFrame(
                seq=self._next_seq(), session=self.session, nonce=self._arm_nonce
            )
        )
        return self._arm_nonce

    def disarm(self) -> None:
        """Send ``D``.  ``armed`` drops immediately, before the ack: the
        fail-safe direction never waits for the controller to agree."""
        self._armed = False
        self._arm_nonce = None
        if self._ready and self.session != SESSION_WILDCARD:
            self._send(DisarmFrame(seq=self._next_seq(), session=self.session))

    def stop(self, mode: int = 0) -> None:
        """Send ``S``: 0 brake, 1 coast.  The setpoint is dropped first so the
        writer cannot follow the stop with a stale velocity."""
        self.cell.zero()
        if self._ready and self.session != SESSION_WILDCARD:
            self._send(
                StopFrame(seq=self._next_seq(), session=self.session, mode=mode)
            )

    def clear(self, mask: int) -> None:
        """Send ``C`` naming latched bits.  robotd never clears obstacle-class
        bits: the MCU owns both the samples and the rule."""
        if self._ready and self.session != SESSION_WILDCARD and mask:
            self._send(
                ClearFaultFrame(
                    seq=self._next_seq(), session=self.session, mask=mask
                )
            )
