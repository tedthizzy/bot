"""A simulated WAVE ROVER: ``docs/protocol.md`` on a pseudo terminal, a TCP port
or stdio, with a first-order wheel model and an integrating heading.

Two firmwares.  By default the stub is the patched fork the protocol document
describes: the ``T:1006`` banner at boot and on ``T:1007``, the 0.30 cap with
its clamp counter ``cc``, the 300 ms heartbeat that ``T:136`` may only lower,
the ``hb st tf bp cc`` fields in every ``T:1001`` line, and the stop flags that
refuse forward motion while the time-of-flight sensor or the bumper say so.
``--stock`` is unpatched Waveshare firmware: no banner and ``T:1007`` ignored,
stock feedback fields only, a 3000 ms heartbeat that accepts any ``T:136``
value, no clamp, no blocking, every accepted command echoed back until
``T:143 cmd 0``, and a few lines of human-readable text at boot.  The gates use
both, because the host has to refuse motion against stock.

Physics, deliberately crude.  ``L`` and ``R`` in the feedback are the values the
firmware applies to the motor driver -- the host's command after the clamp, or
zero after a block, a coast or a heartbeat expiry -- exactly as the real
firmware reports them.  Behind that, each wheel's speed follows its applied
value with a first-order lag of ``--tau-s`` (0.15 s).  Yaw rate in deg/s is
``--yaw-per-diff`` (150) times (R - L) of the lagged wheel values, with the
sign of ``--yaw-sign``; the heading integrates it and wraps to (-180, 180].
Forward speed in m/s is ``--speed-per-power`` (1.2) times (L + R) / 2, and
``x``, ``y`` are tracked for the tests only.  ``--yaw-noise-deg`` adds gaussian
measurement noise to the reported ``y``, sample by sample.  Roll, pitch and the
temperature are constants.  None of these numbers is a claim about the real
chassis: ground speed at a given power is measured on the floor.

The heartbeat watchdog runs from boot, as on the board: 300 ms after power-up
with no host the motors are (already) zero, ``hb`` reads 0 and ``st`` bit 0 is
set, and the first speed command clears both.

The class is usable without any transport.  :class:`RoverStub` takes explicit
timestamps -- ``start(now)``, ``feed(line, now)``, ``tick(now)`` -- and returns
the lines the rover would have sent, so a test drives it with a fake clock and
asserts on the 300 ms heartbeat without waiting 300 ms.  :class:`StubServer`
wires one stub to the transports in real time; :func:`main` is the CLI::

    rover-stub --pty                 # prints /dev/ttysNNN, serves the fork on it
    rover-stub --tcp 7777 --stock    # unpatched firmware on 127.0.0.1:7777
    rover-stub --pty --tof-mm 200 --latency-ms 40 --garbage

Pure standard library, so it runs from a bare clone before anything is
installed, and ``python packages/rover_devtools/rover_stub.py`` works too.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import math
import os
import random
import signal
import sys
import time
import tty
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import IntFlag
from typing import Any, Final, TextIO

__all__ = [
    "BANNER",
    "CAP",
    "FEEDBACK_MS",
    "FORK_FIELDS",
    "FW_NAME",
    "HB_MS",
    "LINE_MAX_BYTES",
    "LOWBAT_V",
    "PROTO",
    "STOCK_BOOT_TEXT",
    "STOCK_FIELDS",
    "STOCK_HB_MS",
    "TOF_STOP_MM",
    "Counters",
    "LineSplitter",
    "Params",
    "Plant",
    "RoverStub",
    "StopFlag",
    "StubServer",
    "main",
    "wrap_deg",
]

log = logging.getLogger("rover.stub")

# --------------------------------------------------------------------------
# The contract's constants
# --------------------------------------------------------------------------

FW_NAME: Final = "bot-wr-1"
PROTO: Final = 1
CAP: Final = 0.30
"""The fork's compiled cap on each speed value.  Sixty percent duty."""
HB_MS: Final = 300
"""The fork's compiled heartbeat.  ``T:136`` may lower it, never raise it."""
STOCK_HB_MS: Final = 3000
FEEDBACK_MS: Final = 50
"""Default feedback interval: 20 Hz.  ``T:142`` sets it."""
FEEDBACK_MIN_MS: Final = 5
LINE_MAX_BYTES: Final = 512
"""Longer lines are dropped and counted, as the protocol says of both ends."""
LOWBAT_V: Final = 9.9
TOF_STOP_MM: Final = 250
TEMP_C: Final = 36.5
CC_MODULUS: Final = 65536
"""``cc`` wraps at 65535."""
TICK_S: Final = 0.01
"""How often :class:`StubServer` integrates the plant and checks the timers."""
GARBAGE_PERIOD_S: Final = 1.0
SCHEDULE_EPS_S: Final = 1e-6
"""Slack when comparing a tick against the feedback schedule."""

T_SPEED: Final = 1
T_OLED: Final = 3
T_COAST: Final = 115
T_IMU_REQUEST: Final = 126
T_BASE_REQUEST: Final = 130
T_FEEDBACK_FLOW: Final = 131
T_HEARTBEAT: Final = 136
T_FEEDBACK_INTERVAL: Final = 142
T_ECHO: Final = 143
T_INFO_PRINT: Final = 605
T_FEEDBACK: Final = 1001
T_IMU: Final = 1002
T_BANNER: Final = 1006
T_BANNER_REQUEST: Final = 1007

STOCK_FIELDS: Final = ("L", "R", "r", "p", "y", "temp", "v")
"""``T:1001`` fields every firmware sends."""
FORK_FIELDS: Final = ("hb", "st", "tf", "bp", "cc")
"""``T:1001`` fields only the fork sends.  Their absence means stock."""

STOCK_BOOT_TEXT: Final = (
    "UGV started.",
    "ugv_base_general: IMU ok, OLED ok.",
    "ESP-NOW init ok.",
)
"""What unpatched firmware prints on the control line at power-up."""


class StopFlag(IntFlag):
    """``st`` in the fork's feedback."""

    HEARTBEAT = 1
    TOF = 2
    BUMPER = 4
    LOWBAT = 8
    COAST = 16


def wrap_deg(angle: float) -> float:
    """Wrap to (-180, 180], the interval the protocol gives for ``y``."""
    return angle - 360.0 * math.ceil((angle - 180.0) / 360.0)


def _line(obj: dict[str, Any]) -> bytes:
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("ascii")


BANNER: Final = _line(
    {"T": T_BANNER, "fw": FW_NAME, "hb_ms": HB_MS, "cap": CAP, "proto": PROTO}
)
"""``{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}``, verbatim."""


def _number(value: object) -> float:
    """A JSON number as a float; anything else, and any non-finite value, is
    0.0 -- the firmware reads a missing key as zero."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return 0.0
    result = float(value)
    return result if math.isfinite(result) else 0.0


def _integer(value: object) -> int | None:
    """A JSON integer (an integral float will do), or ``None``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if isinstance(value, float) and not value.is_integer():
        return None
    return int(value)


# --------------------------------------------------------------------------
# Configuration and counters
# --------------------------------------------------------------------------


@dataclass
class Params:
    """Everything the command line sets.  A test mutates the instance between
    calls (``stub.params.tof_mm = 200``) to move the obstacle mid-run."""

    stock: bool = False
    tau_s: float = 0.15
    yaw_per_diff: float = 150.0
    yaw_sign: int = 1
    speed_per_power: float = 1.2
    yaw_noise_deg: float = 0.0
    vbat: float = 11.4
    tof_mm: int = -1
    tof_stop_mm: int = TOF_STOP_MM
    bumper: bool = False
    latency_ms: float = 0.0
    drop_every: int = 0
    garbage: bool = False
    freeze_after_s: float | None = None
    banner_delay_ms: float = 0.0
    seed: int | None = None


@dataclass
class Counters:
    """What came in and what happened to it.  ``dropped`` is the sum of the
    reasons below it; ``injected_drops`` are ``--drop-every``'s and are not
    counted as dropped by the firmware, because the firmware never saw them."""

    received: int = 0
    accepted: int = 0
    dropped: int = 0
    oversize: int = 0
    not_json: int = 0
    not_object: int = 0
    no_type: int = 0
    unknown: int = 0
    injected_drops: int = 0


# --------------------------------------------------------------------------
# The plant
# --------------------------------------------------------------------------


def _lag(value: float, target: float, dt: float, tau: float) -> tuple[float, float]:
    """One zero-order-hold step of a first-order lag: the new value and the
    exact integral of the value over the step, so the integrated heading does
    not depend on the tick rate."""
    if tau <= 0.0:
        return target, target * dt
    k = 1.0 - math.exp(-dt / tau)
    return value + (target - value) * k, target * dt - (target - value) * tau * k


@dataclass
class Plant:
    """Two lagged wheels, a heading and a position.  Speed units are the
    host's (0.30 is sixty percent duty); the two gains turn them into deg/s
    and m/s."""

    tau_s: float = 0.15
    yaw_per_diff: float = 150.0
    yaw_sign: int = 1
    speed_per_power: float = 1.2
    wheel_l: float = 0.0
    wheel_r: float = 0.0
    heading_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0

    def advance(self, dt: float, target_l: float, target_r: float) -> None:
        """Integrate ``dt`` seconds with the applied values held constant."""
        if dt <= 0.0:
            return
        self.wheel_l, int_l = _lag(self.wheel_l, target_l, dt, self.tau_s)
        self.wheel_r, int_r = _lag(self.wheel_r, target_r, dt, self.tau_s)
        gain = self.yaw_sign * self.yaw_per_diff
        self.yaw_rate_dps = gain * (self.wheel_r - self.wheel_l)
        self.heading_deg = wrap_deg(self.heading_deg + gain * (int_r - int_l))
        distance = self.speed_per_power * (int_l + int_r) / 2.0
        heading = math.radians(self.heading_deg)
        self.x_m += distance * math.cos(heading)
        self.y_m += distance * math.sin(heading)


# --------------------------------------------------------------------------
# The rover
# --------------------------------------------------------------------------

Handler = Callable[[dict[str, Any], float], list[bytes]]


class RoverStub:
    """The rover's half of ``docs/protocol.md`` with an explicit clock.

    Every method takes ``now`` in seconds on any monotonic scale (``None``
    means :func:`time.monotonic`) and returns the lines the rover sends, so
    one object serves a transport in real time and a test with a fake clock.
    Call :meth:`start` once, :meth:`feed` with each received line (its
    newline optional), and :meth:`tick` as often as the transport likes: the
    plant and the heartbeat watchdog are integrated exactly between calls, so
    the tick rate changes only how promptly feedback goes out.

    Public state for tests: ``applied_l`` / ``applied_r`` (what the feedback
    reports as ``L`` / ``R``), ``plant``, ``hb_alive``, ``stop_flags``,
    ``clamp_count``, ``hb_ms``, ``feedback_ms``, ``flow_on``, ``echo``,
    ``info_print`` and ``counters``.
    """

    def __init__(self, params: Params | None = None) -> None:
        self.params = params or Params()
        p = self.params
        self.stock = p.stock
        self.plant = Plant(p.tau_s, p.yaw_per_diff, p.yaw_sign, p.speed_per_power)
        self.counters = Counters()
        self.hb_ms = STOCK_HB_MS if p.stock else HB_MS
        self.feedback_ms = FEEDBACK_MS
        self.flow_on = False
        self.echo = p.stock
        self.info_print = True
        self.applied_l = 0.0
        self.applied_r = 0.0
        self.hb_alive = True
        self.stop_flags = StopFlag(0)
        self.clamp_count = 0
        self.started = False
        self._random = random.Random(p.seed)
        self._boot_at = 0.0
        self._last_tick = 0.0
        self._hb_deadline = 0.0
        self._next_feedback = 0.0
        self._next_garbage = 0.0
        self._banner_due: float | None = None
        self._garbage_count = 0
        self._pending: list[tuple[float, bytes]] = []
        self._handlers: dict[int, Handler] = {
            T_SPEED: self._on_speed,
            T_OLED: self._on_oled,
            T_COAST: self._on_coast,
            T_IMU_REQUEST: self._on_imu_request,
            T_BASE_REQUEST: self._on_base_request,
            T_FEEDBACK_FLOW: self._on_feedback_flow,
            T_HEARTBEAT: self._on_heartbeat,
            T_FEEDBACK_INTERVAL: self._on_feedback_interval,
            T_ECHO: self._on_echo,
            T_INFO_PRINT: self._on_info_print,
        }
        if not p.stock:
            self._handlers[T_BANNER_REQUEST] = self._on_banner_request

    # -- the three entry points ---------------------------------------------

    def start(self, now: float | None = None) -> list[bytes]:
        """Power up.  Returns the boot output: stock text, or the banner
        (after ``banner_delay_ms``, in which case :meth:`tick` delivers it)."""
        now = self._clock(now)
        self.started = True
        self._boot_at = self._last_tick = now
        self._hb_deadline = now + self.hb_ms / 1000.0
        self._next_feedback = now
        self._next_garbage = now + GARBAGE_PERIOD_S
        self._banner_due = None
        if not self.stock:
            self._banner_due = now + self.params.banner_delay_ms / 1000.0
        boot = [f"{text}\n".encode() for text in STOCK_BOOT_TEXT] if self.stock else []
        return boot + self.tick(now)

    def feed(self, raw: bytes, now: float | None = None) -> list[bytes]:
        """One received line.  Never raises: a malformed line is counted."""
        now = self._clock(now)
        out = [] if self.started else self.start(now)
        self._advance_to(now)
        self.counters.received += 1
        if self._frozen(now):
            return out
        p = self.params
        if p.drop_every > 0 and self.counters.received % p.drop_every == 0:
            self.counters.injected_drops += 1
            log.info("injected drop: %r", raw[:80])
            return out
        if p.latency_ms > 0:
            self._pending.append((now + p.latency_ms / 1000.0, raw))
            return out
        return out + self._process(raw, now)

    def tick(self, now: float | None = None) -> list[bytes]:
        """Advance to ``now`` and return whatever fell due: delayed commands'
        replies, the banner, garbage, feedback."""
        now = self._clock(now)
        if not self.started:
            return self.start(now)
        self._advance_to(now)
        if self._frozen(now):
            return []
        out: list[bytes] = []
        while self._pending and self._pending[0][0] <= now:
            _due, raw = self._pending.pop(0)
            out += self._process(raw, now)
        if self._banner_due is not None and now >= self._banner_due:
            self._banner_due = None
            out.append(BANNER)
        self._refresh_environment()
        if self.params.garbage and now >= self._next_garbage:
            self._next_garbage += GARBAGE_PERIOD_S
            self._garbage_count += 1
            out.append(f"** not json {self._garbage_count} **\n".encode())
        if self.flow_on and now + SCHEDULE_EPS_S >= self._next_feedback:
            # Rounded to the microsecond so 0.05 added twenty times is 1.0 and
            # a sample lands on the tick a test expects, not one tick late.
            period = self.feedback_ms / 1000.0
            self._next_feedback = round(self._next_feedback + period, 6)
            if self._next_feedback <= now:
                self._next_feedback = round(now + period, 6)
            out.append(self._feedback_line())
        return out

    # -- time ----------------------------------------------------------------

    @staticmethod
    def _clock(now: float | None) -> float:
        return time.monotonic() if now is None else float(now)

    def _frozen(self, now: float) -> bool:
        """``--freeze-after``: a hung controller.  Nothing in, nothing out, no
        watchdog; the motors keep their last value."""
        after = self.params.freeze_after_s
        return after is not None and now - self._boot_at >= after

    def _advance_to(self, now: float) -> None:
        """Integrate the plant to ``now``, expiring the heartbeat at the exact
        moment it fell due on the way."""
        if now <= self._last_tick:
            return
        deadline = self._hb_deadline
        if self.hb_alive and deadline <= now and not self._frozen(deadline):
            self.plant.advance(deadline - self._last_tick, self.applied_l, self.applied_r)
            self._last_tick = max(self._last_tick, deadline)
            self._expire_heartbeat()
        self.plant.advance(now - self._last_tick, self.applied_l, self.applied_r)
        self._last_tick = now

    def _expire_heartbeat(self) -> None:
        self.hb_alive = False
        self.applied_l = self.applied_r = 0.0
        if not self.stock:
            self.stop_flags |= StopFlag.HEARTBEAT
        log.info("heartbeat expired after %d ms: motors zeroed", self.hb_ms)

    def _refresh_environment(self) -> None:
        """Bits 1-3 follow the sensors; a fork refuses motion the moment they
        set, not only at the next command."""
        if self.stock:
            return
        p = self.params
        env = StopFlag(0)
        if 0 <= p.tof_mm < p.tof_stop_mm:
            env |= StopFlag.TOF
        if p.bumper:
            env |= StopFlag.BUMPER
        if p.vbat < LOWBAT_V:
            env |= StopFlag.LOWBAT
        latched = self.stop_flags & (StopFlag.HEARTBEAT | StopFlag.COAST)
        self.stop_flags = latched | env
        self.applied_l, self.applied_r = self._gate(self.applied_l, self.applied_r)

    def _gate(self, left: float, right: float) -> tuple[float, float]:
        """The fork's refusals: nothing under low battery; no forward motion
        (both sides positive) while the ToF or bumper bit is set.  Reverse and
        rotation pass.  Stock refuses nothing."""
        if self.stock:
            return left, right
        if self.stop_flags & StopFlag.LOWBAT:
            return 0.0, 0.0
        if self.stop_flags & (StopFlag.TOF | StopFlag.BUMPER) and left > 0 and right > 0:
            return 0.0, 0.0
        return left, right

    # -- receiving -------------------------------------------------------------

    def _process(self, raw: bytes, now: float) -> list[bytes]:
        if len(raw.rstrip(b"\r\n")) > LINE_MAX_BYTES:
            return self._drop("oversize", raw)
        text = raw.strip()
        if not text:
            return []
        try:
            obj = json.loads(text.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return self._drop("not_json", raw)
        if not isinstance(obj, dict):
            return self._drop("not_object", raw)
        t = obj.get("T")
        if isinstance(t, bool) or not isinstance(t, int):
            return self._drop("no_type", raw)
        handler = self._handlers.get(t)
        if handler is None:
            return self._drop("unknown", raw)
        self.counters.accepted += 1
        log.info("rx %s", text.decode("utf-8"))
        out = [_line(obj)] if self.echo else []
        return out + handler(obj, now)

    def _drop(self, reason: str, raw: bytes) -> list[bytes]:
        setattr(self.counters, reason, getattr(self.counters, reason) + 1)
        self.counters.dropped += 1
        log.info("drop %s: %r", reason, raw[:80])
        return []

    def _on_speed(self, obj: dict[str, Any], now: float) -> list[bytes]:
        left, right = _number(obj.get("L")), _number(obj.get("R"))
        if not self.stock:
            left, right = self._clamp(left), self._clamp(right)
            self.stop_flags &= ~(StopFlag.HEARTBEAT | StopFlag.COAST)
        self.hb_alive = True
        self._hb_deadline = now + self.hb_ms / 1000.0
        self._refresh_environment()
        self.applied_l, self.applied_r = self._gate(left, right)
        return []

    def _clamp(self, value: float) -> float:
        if abs(value) > CAP:
            self.clamp_count = (self.clamp_count + 1) % CC_MODULUS
            return math.copysign(CAP, value)
        return value

    def _on_coast(self, obj: dict[str, Any], now: float) -> list[bytes]:
        self.applied_l = self.applied_r = 0.0
        if not self.stock:
            self.stop_flags |= StopFlag.COAST
        return []

    def _on_oled(self, obj: dict[str, Any], now: float) -> list[bytes]:
        return []

    def _on_imu_request(self, obj: dict[str, Any], now: float) -> list[bytes]:
        return [self._imu_line()]

    def _on_base_request(self, obj: dict[str, Any], now: float) -> list[bytes]:
        return [self._feedback_line()]

    def _on_feedback_flow(self, obj: dict[str, Any], now: float) -> list[bytes]:
        cmd = _integer(obj.get("cmd"))
        if cmd is None:
            return []
        if cmd and not self.flow_on:
            self._next_feedback = now
        self.flow_on = bool(cmd)
        return []

    def _on_heartbeat(self, obj: dict[str, Any], now: float) -> list[bytes]:
        ms = _integer(obj.get("cmd"))
        if ms is None or ms <= 0:
            return []
        if self.stock or ms <= HB_MS:
            self.hb_ms = ms
        else:
            log.info("T:136 cmd %d ignored: above the compiled %d ms", ms, HB_MS)
        return []

    def _on_feedback_interval(self, obj: dict[str, Any], now: float) -> list[bytes]:
        ms = _integer(obj.get("cmd"))
        if ms is None or ms < 0:
            return []
        self.feedback_ms = max(ms, FEEDBACK_MIN_MS)
        return []

    def _on_echo(self, obj: dict[str, Any], now: float) -> list[bytes]:
        cmd = _integer(obj.get("cmd"))
        if cmd is not None:
            self.echo = bool(cmd)
        return []

    def _on_info_print(self, obj: dict[str, Any], now: float) -> list[bytes]:
        cmd = _integer(obj.get("cmd"))
        if cmd is not None:
            self.info_print = bool(cmd)
        return []

    def _on_banner_request(self, obj: dict[str, Any], now: float) -> list[bytes]:
        return [BANNER]

    # -- sending ---------------------------------------------------------------

    def _yaw(self) -> float:
        yaw = self.plant.heading_deg
        sigma = self.params.yaw_noise_deg
        if sigma > 0.0:
            yaw = wrap_deg(yaw + self._random.gauss(0.0, sigma))
        return round(yaw, 2)

    def _feedback_line(self) -> bytes:
        p = self.params
        obj: dict[str, Any] = {
            "T": T_FEEDBACK,
            "L": round(self.applied_l, 3),
            "R": round(self.applied_r, 3),
            "r": 0.0,
            "p": 0.0,
            "y": self._yaw(),
            "temp": TEMP_C,
            "v": p.vbat,
        }
        if not self.stock:
            obj["hb"] = 1 if self.hb_alive else 0
            obj["st"] = int(self.stop_flags)
            obj["tf"] = int(p.tof_mm) if p.tof_mm >= 0 else -1
            obj["bp"] = 1 if p.bumper else 0
            obj["cc"] = self.clamp_count
        return _line(obj)

    def _imu_line(self) -> bytes:
        return _line(
            {
                "T": T_IMU,
                "r": 0.0,
                "p": 0.0,
                "y": self._yaw(),
                "ax": 0.0,
                "ay": 0.0,
                "az": 9.81,
                "gx": 0.0,
                "gy": 0.0,
                "gz": round(self.plant.yaw_rate_dps, 2),
                "mx": 0.0,
                "my": 0.0,
                "mz": 0.0,
                "temp": TEMP_C,
            }
        )


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------


class LineSplitter:
    """Bytes in, lines out, terminator removed.  A line that runs past the
    limit without a newline is cut at ``limit + 1`` bytes and the rest
    discarded up to the next newline, so the stub still sees one oversize
    line to count rather than a flood of fragments."""

    def __init__(self, limit: int = LINE_MAX_BYTES) -> None:
        self._limit = limit
        self._buffer = bytearray()
        self._discarding = False

    def feed(self, data: bytes) -> list[bytes]:
        lines: list[bytes] = []
        start = 0
        while True:
            index = data.find(b"\n", start)
            if index < 0:
                self._append(data[start:])
                return lines
            self._append(data[start:index])
            lines.append(bytes(self._buffer))
            self._buffer.clear()
            self._discarding = False
            start = index + 1

    def _append(self, piece: bytes) -> None:
        if self._discarding:
            return
        room = self._limit + 1 - len(self._buffer)
        if len(piece) > room:
            self._buffer += piece[: max(room, 0)]
            self._discarding = True
        else:
            self._buffer += piece


def _write_dropping(fd: int, data: bytes) -> None:
    """Write what fits and drop the rest.  A pty master whose slave nobody is
    reading fills after about a kilobyte; a UART with no listener transmits
    into the void just the same."""
    try:
        os.write(fd, data)
    except BlockingIOError:
        pass
    except OSError as exc:
        log.warning("write to fd %d failed: %s", fd, exc)


class StubServer:
    """One :class:`RoverStub` on any combination of a pty, a TCP port and
    stdio, in real time.  Every transport sees the same rover: a command from
    any of them acts on the one plant, and every line the rover sends goes to
    all of them.  A TCP client that stops reading is dropped lines, not waited
    for.  ``tcp_port=0`` binds an ephemeral port, read back from
    :attr:`tcp_port` after :meth:`start`."""

    def __init__(
        self,
        stub: RoverStub,
        *,
        pty: bool = False,
        tcp_port: int | None = None,
        stdio: bool = False,
        announce: TextIO | None = None,
    ) -> None:
        self.stub = stub
        self._want_pty = pty
        self._want_tcp = tcp_port
        self._want_stdio = stdio
        self._announce = announce
        self.pty_path: str | None = None
        self.tcp_port: int | None = None
        self.stopped = asyncio.Event()
        self._master = -1
        self._slave = -1
        self._server: asyncio.Server | None = None
        self._writers: set[asyncio.StreamWriter] = set()
        self._stdin_fd = -1
        self._tick_task: asyncio.Task[None] | None = None
        self._pty_lines = LineSplitter()
        self._stdin_lines = LineSplitter()

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        announce = self._announce if self._announce is not None else sys.stdout
        if self._want_pty:
            self._master, self._slave = os.openpty()
            tty.setraw(self._slave)
            os.set_blocking(self._master, False)
            self.pty_path = os.ttyname(self._slave)
            loop.add_reader(self._master, self._on_pty_readable)
            print(self.pty_path, file=announce, flush=True)
        if self._want_tcp is not None:
            self._server = await asyncio.start_server(
                self._serve_tcp, "127.0.0.1", self._want_tcp
            )
            self.tcp_port = int(self._server.sockets[0].getsockname()[1])
            print(
                f"rover-stub: tcp 127.0.0.1:{self.tcp_port}", file=sys.stderr, flush=True
            )
        if self._want_stdio:
            self._stdin_fd = sys.stdin.fileno()
            os.set_blocking(self._stdin_fd, False)
            loop.add_reader(self._stdin_fd, self._on_stdin_readable)
        self._broadcast(self.stub.start(time.monotonic()))
        self._tick_task = loop.create_task(self._tick_forever())

    async def stop(self) -> None:
        """Close everything: the pty pair, the listening socket, every client."""
        loop = asyncio.get_running_loop()
        if self._tick_task is not None:
            self._tick_task.cancel()
            await asyncio.gather(self._tick_task, return_exceptions=True)
            self._tick_task = None
        if self._stdin_fd >= 0:
            loop.remove_reader(self._stdin_fd)
            self._stdin_fd = -1
        if self._master >= 0:
            loop.remove_reader(self._master)
            for fd in (self._master, self._slave):
                with contextlib.suppress(OSError):
                    os.close(fd)
            self._master = self._slave = -1
        for writer in list(self._writers):
            writer.close()
        self._writers.clear()
        if self._server is not None:
            self._server.close()
            with contextlib.suppress(TimeoutError, asyncio.TimeoutError):
                await asyncio.wait_for(self._server.wait_closed(), 2.0)
            self._server = None
        self.stopped.set()

    async def _tick_forever(self) -> None:
        while True:
            await asyncio.sleep(TICK_S)
            self._broadcast(self.stub.tick(time.monotonic()))

    def _ingest(self, splitter: LineSplitter, data: bytes) -> None:
        now = time.monotonic()
        for line in splitter.feed(data):
            self._broadcast(self.stub.feed(line, now))

    def _broadcast(self, lines: list[bytes]) -> None:
        if not lines:
            return
        data = b"".join(lines)
        if self._master >= 0:
            _write_dropping(self._master, data)
        for writer in list(self._writers):
            if writer.is_closing():
                self._writers.discard(writer)
            elif writer.transport.get_write_buffer_size() < 65536:
                writer.write(data)
        if self._want_stdio:
            with contextlib.suppress(OSError, ValueError):
                sys.stdout.buffer.write(data)
                sys.stdout.flush()

    def _on_pty_readable(self) -> None:
        try:
            data = os.read(self._master, 4096)
        except BlockingIOError:
            return
        except OSError as exc:
            log.warning("pty read failed: %s", exc)
            return
        if data:
            self._ingest(self._pty_lines, data)

    def _on_stdin_readable(self) -> None:
        try:
            data = os.read(self._stdin_fd, 4096)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if data:
            self._ingest(self._stdin_lines, data)
            return
        # EOF on stdin.  With no other transport there is nobody left to
        # serve, and a pipeline that closed our input expects us to exit.
        asyncio.get_running_loop().remove_reader(self._stdin_fd)
        self._stdin_fd = -1
        if not self._want_pty and self._want_tcp is None:
            self.stopped.set()

    async def _serve_tcp(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._writers.add(writer)
        splitter = LineSplitter()
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                self._ingest(splitter, data)
        except (OSError, asyncio.IncompleteReadError):
            pass
        finally:
            self._writers.discard(writer)
            writer.close()
            with contextlib.suppress(OSError, asyncio.CancelledError):
                await writer.wait_closed()


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rover-stub",
        description=(
            "Simulate the WAVE ROVER of docs/protocol.md on a pty, a TCP port "
            "and/or stdio.  Transports combine; all of them see one rover."
        ),
    )
    transport = parser.add_argument_group("transports (choose at least one)")
    transport.add_argument(
        "--pty",
        action="store_true",
        help="open a pseudo terminal, print its device path on stdout, serve on it",
    )
    transport.add_argument(
        "--tcp", type=int, metavar="PORT", help="listen on 127.0.0.1:PORT (0: ephemeral)"
    )
    transport.add_argument("--stdio", action="store_true", help="serve on stdin/stdout")

    firmware = parser.add_argument_group("firmware")
    firmware.add_argument(
        "--stock",
        action="store_true",
        help="unpatched Waveshare firmware: no banner, stock fields, 3000 ms "
        "heartbeat, no clamp, no blocking, command echo",
    )

    physics = parser.add_argument_group("physics")
    physics.add_argument("--tau-s", type=float, default=0.15, help="wheel lag (0.15)")
    physics.add_argument(
        "--yaw-per-diff", type=float, default=150.0, help="deg/s per unit of R-L (150)"
    )
    physics.add_argument(
        "--yaw-sign", type=int, choices=(1, -1), default=1, help="sign of yaw (+1)"
    )
    physics.add_argument(
        "--speed-per-power", type=float, default=1.2, help="m/s per unit of (L+R)/2 (1.2)"
    )
    physics.add_argument(
        "--yaw-noise-deg", type=float, default=0.0, help="gaussian noise on y (0)"
    )

    sensors = parser.add_argument_group("sensors")
    sensors.add_argument("--vbat", type=float, default=11.4, help="pack volts (11.4)")
    sensors.add_argument(
        "--tof-mm", type=int, default=-1, help="front range in mm; -1 for no sensor"
    )
    sensors.add_argument(
        "--tof-stop-mm",
        type=int,
        default=TOF_STOP_MM,
        help=f"range below which forward is blocked ({TOF_STOP_MM})",
    )
    sensors.add_argument("--bumper", action="store_true", help="bumper pressed")

    faults = parser.add_argument_group("fault injection")
    faults.add_argument(
        "--latency-ms", type=float, default=0.0, help="delay before applying a command"
    )
    faults.add_argument(
        "--drop-every", type=int, default=0, metavar="N", help="drop every Nth line"
    )
    faults.add_argument(
        "--garbage", action="store_true", help="emit a non-JSON line every second"
    )
    faults.add_argument(
        "--freeze-after",
        type=float,
        default=None,
        metavar="S",
        help="hang after S seconds: no feedback, motors keep the last command",
    )
    faults.add_argument(
        "--banner-delay-ms", type=float, default=0.0, help="delay the boot banner"
    )

    parser.add_argument(
        "-v", "--verbose", action="store_true", help="log received commands to stderr"
    )
    return parser


def _params(args: argparse.Namespace) -> Params:
    return Params(
        stock=args.stock,
        tau_s=args.tau_s,
        yaw_per_diff=args.yaw_per_diff,
        yaw_sign=args.yaw_sign,
        speed_per_power=args.speed_per_power,
        yaw_noise_deg=args.yaw_noise_deg,
        vbat=args.vbat,
        tof_mm=args.tof_mm,
        tof_stop_mm=args.tof_stop_mm,
        bumper=args.bumper,
        latency_ms=args.latency_ms,
        drop_every=args.drop_every,
        garbage=args.garbage,
        freeze_after_s=args.freeze_after,
        banner_delay_ms=args.banner_delay_ms,
    )


async def _serve(params: Params, args: argparse.Namespace) -> int:
    loop = asyncio.get_running_loop()
    server = StubServer(
        RoverStub(params), pty=args.pty, tcp_port=args.tcp, stdio=args.stdio
    )
    for signum in (signal.SIGINT, signal.SIGTERM):
        with contextlib.suppress(NotImplementedError, RuntimeError, ValueError):
            loop.add_signal_handler(signum, server.stopped.set)
    try:
        await server.start()
    except OSError as exc:
        print(f"rover-stub: cannot start: {exc}", file=sys.stderr)
        await server.stop()
        return 2
    try:
        await server.stopped.wait()
    finally:
        await server.stop()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if not (args.pty or args.tcp is not None or args.stdio):
        parser.error("choose a transport: --pty, --tcp PORT and/or --stdio")
    for name in ("tau_s", "latency_ms", "banner_delay_ms", "yaw_noise_deg"):
        if getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must not be negative")
    if args.drop_every < 0:
        parser.error("--drop-every must not be negative")
    if args.freeze_after is not None and args.freeze_after < 0:
        parser.error("--freeze-after must not be negative")
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        stream=sys.stderr,
        format="rover-stub: %(message)s",
    )
    try:
        return asyncio.run(_serve(_params(args), args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
