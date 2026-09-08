"""The rover link: Waveshare JSON lines over one serial port or one TCP socket.

robotd is the only writer of the rover's line (I-18) and this module is the
only place in robotd that writes bytes to it.  What lives here is what the wire
forces into one place: opening the transport and reopening it after a loss, the
bring-up sequence of ``docs/protocol.md``, the banner gate that refuses stock
firmware, feedback ingest stamped with the host's monotonic arrival time, and
the encoder for the one command that moves the robot.

Two rules shape the file:

* **The link never sends a speed command on its own.**  :meth:`Link.command`
  writes one ``{"T":1,...}`` line and returns; its caller is robotd's control
  loop, once per period, with zeros when idle.  There is no writer task here,
  so a frozen control loop stops the stream and the firmware's heartbeat
  zeroes the motors within ``[safety] heartbeat_ms`` (I-1, I-14).  The only
  lines written unprompted are the configuration lines of bring-up.
* **A line that does not decode renews nothing.**  ``decode_line`` never
  raises; a ``Dropped`` or ``Unknown`` value is counted and discarded before it
  can touch the feedback stamp or the banner.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import Callable

from rover_contracts.config import RobotConfig
from rover_contracts.wave_proto import (
    LINE_MAX_BYTES,
    POWER_CAP,
    Banner,
    Dropped,
    Feedback,
    Unknown,
    banner_request,
    decode_line,
    echo,
    feedback_flow,
    feedback_interval,
    heartbeat,
    quiet,
    speed,
)

__all__ = ["Link", "LinkProtocol"]

log = logging.getLogger("rover.robotd.link")

FeedbackCallback = Callable[[Feedback, int], None]
LinkCallback = Callable[["Link"], None]
RestartCallback = Callable[[Banner], None]


class LinkProtocol(asyncio.Protocol):
    """Bytes in, lines out.  One instance per connection, owned by the link.

    A run of more than ``LINE_MAX_BYTES`` without a newline is not a line of
    this protocol whatever it turns out to be: it is dropped where it stands
    and the bytes up to the next newline go with it, so the buffer is bounded.
    """

    def __init__(self, link: Link) -> None:
        self._link = link
        self._rx = bytearray()
        self._skipping = False

    def data_received(self, data: bytes) -> None:
        self._rx += data
        while True:
            end = self._rx.find(b"\n")
            if end < 0:
                if len(self._rx) > LINE_MAX_BYTES:
                    self._rx.clear()
                    if not self._skipping:
                        self._link.dropped += 1
                    self._skipping = True
                return
            line = bytes(self._rx[:end])
            del self._rx[: end + 1]
            if self._skipping:
                self._skipping = False
                continue
            self._link.ingest(line)

    def connection_lost(self, exc: Exception | None) -> None:
        self._link._connection_lost(self, exc)


class Link:
    """One long-lived connection to the rover, reopened for as long as it runs."""

    def __init__(
        self,
        config: RobotConfig,
        *,
        on_feedback: FeedbackCallback | None = None,
        on_up: LinkCallback | None = None,
        on_lost: LinkCallback | None = None,
        on_restart: RestartCallback | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = config
        self.banner: Banner | None = None
        self.feedback: Feedback | None = None
        self.feedback_arrival_ns: int | None = None
        self.cmd_left = 0.0
        self.cmd_right = 0.0
        self.dropped = 0
        self.unknown = 0

        self._clock = clock
        self._on_feedback = on_feedback
        self._on_up = on_up
        self._on_lost = on_lost
        self._on_restart = on_restart
        self._transport: asyncio.BaseTransport | None = None
        self._protocol: LinkProtocol | None = None
        self._up = False
        self._closing = False
        self._banner_waiter: asyncio.Future[Banner] | None = None
        self._lost: asyncio.Event | None = None
        self._restart: asyncio.Event | None = None

    # -- observable state ---------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._transport is not None

    @property
    def up(self) -> bool:
        """Bring-up has finished on the current connection."""
        return self._up

    @property
    def fw(self) -> str | None:
        """``welcome.rover_fw`` / ``state.rover.fw``: the banner's tag, or
        ``None`` while the link is down or the firmware sent no banner."""
        return self.banner.fw if self._up and self.banner is not None else None

    def feedback_age_ms(self, now_mono_ns: int | None = None) -> float | None:
        """Age of the newest feedback line, on the host's monotonic clock."""
        if self.feedback_arrival_ns is None:
            return None
        now = self._clock() if now_mono_ns is None else now_mono_ns
        return max(0.0, (now - self.feedback_arrival_ns) / 1e6)

    def feedback_fresh(self, now_mono_ns: int | None = None) -> bool:
        """T0: feedback no older than ``[safety] feedback_max_age_ms``."""
        age = self.feedback_age_ms(now_mono_ns)
        return age is not None and age <= self.config.safety.feedback_max_age_ms

    def firmware_refusal(self) -> str:
        """Why the firmware gate refuses motion, or ``""`` when the fork is
        confirmed.  The banner is judged first, then the feedback's fork
        fields; ``[safety] require_patched_firmware = false`` waives both."""
        safety = self.config.safety
        if not safety.require_patched_firmware:
            return ""
        banner = self.banner
        if banner is None:
            return "no banner: stock firmware, or no controller answering"
        if banner.hb_ms != safety.heartbeat_ms:
            return (
                f"banner hb_ms={banner.hb_ms} differs from "
                f"[safety] heartbeat_ms={safety.heartbeat_ms}"
            )
        if not math.isclose(banner.cap, POWER_CAP, abs_tol=1e-6):
            return f"banner cap={banner.cap} differs from the compiled cap {POWER_CAP}"
        if self.feedback is not None and not self.feedback.patched:
            return "feedback lacks the fork fields hb, st, tf, bp, cc"
        return ""

    @property
    def firmware_ok(self) -> bool:
        return self.firmware_refusal() == ""

    def motion_allowed(self, now_mono_ns: int | None = None) -> bool:
        """Whether a non-zero speed may go out right now: bring-up done, the
        firmware confirmed as the fork, and feedback inside T0."""
        return self._up and self.firmware_ok and self.feedback_fresh(now_mono_ns)

    # -- the command stream -------------------------------------------------

    def command(
        self, left: float, right: float, now_mono_ns: int | None = None
    ) -> tuple[float, float]:
        """Write one speed line and return what went out.

        The caller is the control loop, once per period.  Whatever it asks
        for, nothing above ``[limits] power_max`` reaches the port, and nothing
        but zeros does while :meth:`motion_allowed` is false.  Nothing is
        written while the transport is closed.
        """
        if self._transport is None:
            self.cmd_left = self.cmd_right = 0.0
            return (0.0, 0.0)
        if not self.motion_allowed(now_mono_ns):
            left = right = 0.0
        limit = self.config.limits.power_max
        left = _clamp(left, limit)
        right = _clamp(right, limit)
        self.cmd_left, self.cmd_right = left, right
        self._write(speed(left, right))
        return (left, right)

    # -- lifecycle ----------------------------------------------------------

    async def run(self) -> None:
        """Open, bring up, serve and reopen until :meth:`close`.

        ``open`` is retried every ``[link] open_retry_ms`` while the device or
        the peer is absent, which removes ``make sim``'s start-order dependency
        and is what ``Restart=always`` needs on the Pi while the controller is
        unplugged.  A goal never survives a reopen: the loss aborts it and the
        new connection starts with nothing active (I-13).
        """
        retry_s = self.config.link.open_retry_ms / 1000.0
        while not self._closing:
            try:
                await self._serve()
            except asyncio.CancelledError:
                raise
            except OSError as exc:
                log.info("rover link unavailable (%s); retrying", exc)
            except Exception:  # pragma: no cover - defensive
                log.exception("rover link failed; retrying")
            finally:
                had_connection = self._teardown()
                if had_connection and not self._closing and self._on_lost is not None:
                    self._on_lost(self)
            if not self._closing:
                await asyncio.sleep(retry_s)

    async def close(self) -> None:
        """Send zeros once more, then stop reopening."""
        self._closing = True
        if self._transport is not None:
            self._write(speed(0.0, 0.0))
            self.cmd_left = self.cmd_right = 0.0
            await asyncio.sleep(0)
        if self._lost is not None:
            self._lost.set()

    async def _serve(self) -> None:
        loop = asyncio.get_running_loop()
        self._lost = asyncio.Event()
        self._restart = asyncio.Event()
        await self._open(loop)
        log.info("rover link open (%s)", self._describe())
        while not self._lost.is_set():
            await self._bring_up()
            waiters = [
                asyncio.ensure_future(self._lost.wait()),
                asyncio.ensure_future(self._restart.wait()),
            ]
            try:
                await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            finally:
                for waiter in waiters:
                    waiter.cancel()
            self._restart.clear()

    async def _open(self, loop: asyncio.AbstractEventLoop) -> None:
        link = self.config.link
        protocol = LinkProtocol(self)
        if link.backend == "tcp":
            transport, _ = await loop.create_connection(
                lambda: protocol, link.tcp_host, link.tcp_port
            )
        else:
            # A pseudo terminal opens exactly like the Pi's UART: pyserial
            # sets the line discipline and the speed on whatever path it is
            # given, so `rover-stub --pty` and /dev/serial0 share this branch.
            import serial_asyncio_fast

            transport, _ = await serial_asyncio_fast.create_serial_connection(
                loop, lambda: protocol, link.port, baudrate=link.baud
            )
        self._transport = transport
        self._protocol = protocol

    async def _bring_up(self) -> None:
        """The bring-up sequence of ``docs/protocol.md``: configure the line,
        ask for the banner, wait up to ``[link] banner_wait_ms`` for it.

        The verdict is not taken here.  ``firmware_refusal`` reads the banner
        and the feedback whenever asked, so a firmware that sends a banner and
        then stock feedback is refused the moment that feedback arrives.
        """
        config = self.config
        self._up = False
        self.banner = None
        loop = asyncio.get_running_loop()
        self._banner_waiter = loop.create_future()
        for line in (
            quiet(),
            echo(False),
            heartbeat(config.safety.heartbeat_ms),
            feedback_interval(config.link.feedback_interval_ms),
            feedback_flow(True),
            banner_request(),
        ):
            self._write(line)
        try:
            self.banner = await asyncio.wait_for(
                self._banner_waiter, config.link.banner_wait_ms / 1000.0
            )
        except TimeoutError:
            self.banner = None
        finally:
            self._banner_waiter = None
        self._up = True
        refusal = self.firmware_refusal()
        if self.banner is None:
            log.warning(
                "no banner within %d ms; motion %s",
                config.link.banner_wait_ms,
                "refused" if refusal else "allowed by config",
            )
        else:
            log.info(
                "rover firmware %s (hb_ms=%d cap=%.2f proto=%d)%s",
                self.banner.fw,
                self.banner.hb_ms,
                self.banner.cap,
                self.banner.proto,
                f"; motion refused: {refusal}" if refusal else "",
            )
        if self._on_up is not None:
            self._on_up(self)

    def _teardown(self) -> bool:
        """Drop the connection state.  Returns whether a transport was open."""
        transport, self._transport = self._transport, None
        self._protocol = None
        self._up = False
        self.banner = None
        self.feedback = None
        self.feedback_arrival_ns = None
        self.cmd_left = self.cmd_right = 0.0
        if transport is None:
            return False
        if not transport.is_closing():
            transport.close()
        return True

    # -- ingest -------------------------------------------------------------

    def ingest(self, line: bytes) -> None:
        """Decode one line and apply it.  Never raises."""
        decoded = decode_line(line)
        if isinstance(decoded, Feedback):
            self.feedback = decoded
            self.feedback_arrival_ns = self._clock()
            if self._on_feedback is not None:
                self._on_feedback(decoded, self.feedback_arrival_ns)
        elif isinstance(decoded, Banner):
            self._banner_seen(decoded)
        elif isinstance(decoded, Dropped):
            self.dropped += 1
        elif isinstance(decoded, Unknown):
            self.unknown += 1
        # An Imu line is never requested and carries nothing the loop reads.

    def _banner_seen(self, banner: Banner) -> None:
        waiter = self._banner_waiter
        if waiter is not None:
            # During bring-up the boot banner and the answer to our request
            # may both arrive; the first settles the wait, the rest are it.
            if not waiter.done():
                waiter.set_result(banner)
            return
        if not self._up:
            return
        # A banner after bring-up is the controller announcing a boot.  It has
        # lost the heartbeat, interval and flow settings, and whatever goal
        # was running was computed against a controller that no longer exists.
        log.warning("rover restarted (banner %s); re-running bring-up", banner.fw)
        self._up = False
        self.feedback = None
        self.feedback_arrival_ns = None
        if self._on_restart is not None:
            self._on_restart(banner)
        if self._restart is not None:
            self._restart.set()

    def _connection_lost(self, protocol: LinkProtocol, exc: Exception | None) -> None:
        if protocol is not self._protocol:
            return  # a transport already torn down, reporting late
        if exc is not None:
            log.info("rover link lost: %s", exc)
        elif not self._closing:
            log.info("rover link closed by the peer")
        waiter = self._banner_waiter
        if waiter is not None and not waiter.done():
            waiter.set_exception(ConnectionResetError("link lost during bring-up"))
        if self._lost is not None:
            self._lost.set()

    # -- writing ------------------------------------------------------------

    def _write(self, line: bytes) -> None:
        transport = self._transport
        if transport is None or transport.is_closing():
            return
        # I-18 makes robotd the only writer, so nothing can be attached to a
        # live port to see what went out: this is the wire, at DEBUG.
        if log.isEnabledFor(logging.DEBUG):
            log.debug("tx %s", line.rstrip(b"\n").decode("ascii", "replace"))
        transport.write(line)  # type: ignore[attr-defined]

    def _describe(self) -> str:
        link = self.config.link
        if link.backend == "tcp":
            return f"tcp {link.tcp_host}:{link.tcp_port}"
        return f"serial {link.port} at {link.baud}"


def _clamp(value: float, limit: float) -> float:
    """The last narrowing before the port.  A non-finite value is zero, not
    the limit: ``min(limit, nan)`` would return the limit."""
    if not math.isfinite(value):
        return 0.0
    return round(max(-limit, min(limit, float(value))), 3)
