"""The serial link, end to end over a pty.

Two levels.  Most of the file drives a minimal in-test controller on the master
side of a pty, which exercises the handshake, the bounded re-seed of A8, the
20 Hz stream, the T0 gate, the clamp before the port and the reconnect that
never replays motion (I-13) with nothing built.  The last test drives the real
``mcu-sim`` over ``./run/mcu.pty`` and **skips with a clear reason** when the
simulator is not built.
"""

from __future__ import annotations

import asyncio
import contextlib
import os
import pty
import subprocess
import sys
import time
from collections.abc import AsyncIterator, Callable
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "packages"))

from rover_contracts.config import RobotConfig, SerialConfig  # noqa: E402
from rover_contracts.serial_codec import (  # noqa: E402
    SESSION_WILDCARD,
    AckFrame,
    AckResult,
    ArmFrame,
    ClearFaultFrame,
    CtrlFlag,
    DecodeOk,
    DisarmFrame,
    Frame,
    FrameReader,
    HelloFrame,
    McuState,
    PingFrame,
    PongFrame,
    StopFrame,
    TelemetryFrame,
    VelocityFrame,
    ack_type_code,
    encode_frame,
)
from rover_robotd.link import MAX_V_MM_S, MAX_W_MRAD_S, Link  # noqa: E402
from rover_robotd.profiles import SetpointCell  # noqa: E402

TIMEOUT_S = 5.0
CLEAR_FLAGS = int(
    CtrlFlag.TTL_OK
    | CtrlFlag.ESTOP_RELEASED
    | CtrlFlag.BUMPER_CLEAR
    | CtrlFlag.TOF_CLEAR
    | CtrlFlag.CAL_VALID
    | CtrlFlag.TOF_FL_OK
    | CtrlFlag.TOF_FR_OK
)


class FakeMcu:
    """Just enough controller to answer the handshake and stream telemetry.

    With ``plant=True`` it also integrates the last commanded velocity into the
    encoder counts, which is what lets a whole drive run to completion against
    it.  It is not the real ``mcu-sim``: it enforces no cap, no TTL and no
    obstacle rule, because those belong to the firmware core and its own
    golden vectors.
    """

    def __init__(
        self,
        fd: int,
        *,
        session: int = 40010,
        ack_seq: int = 0,
        plant: bool = False,
        track_m: float = 0.150,
        metres_per_tick: float = 2 * 3.141592653589793 * 0.045 / 2200,
    ) -> None:
        self.fd = fd
        self.session = session
        self.ack_seq = ack_seq
        self.reader = FrameReader()
        self.received: list[Frame] = []
        self.state = McuState.DISARMED
        self.fault = 0
        self.ctrl_flags = CLEAR_FLAGS
        self.telemetry_on = True
        self.plant = plant
        self.track_m = track_m
        self.metres_per_tick = metres_per_tick
        self.left_ticks = 0
        self.right_ticks = 0
        self.v_mm_s = 0
        self.w_mrad_s = 0
        self.vbat_mv = 11620
        self.imotor_ma = 410
        self.rx_drop = 0
        self._left_partial = 0.0
        self._right_partial = 0.0
        self._up_seq = 1
        self._tasks: list[asyncio.Task[None]] = []

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        loop = asyncio.get_running_loop()
        loop.add_reader(self.fd, self._readable)
        self._tasks.append(asyncio.create_task(self._telemetry_loop()))

    async def stop(self) -> None:
        with contextlib.suppress(RuntimeError, ValueError):
            asyncio.get_running_loop().remove_reader(self.fd)
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)

    # -- views over what arrived -------------------------------------------

    def frames(self, kind: type) -> list[Frame]:
        return [frame for frame in self.received if isinstance(frame, kind)]

    @property
    def velocities(self) -> list[VelocityFrame]:
        return self.frames(VelocityFrame)  # type: ignore[return-value]

    # -- the wire -----------------------------------------------------------

    def _readable(self) -> None:
        try:
            data = os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return
        for result in self.reader.feed(data):
            if isinstance(result, DecodeOk):
                self._on_frame(result.frame)

    def _on_frame(self, frame: Frame) -> None:
        self.received.append(frame)
        self.ack_seq = frame.seq
        if isinstance(frame, VelocityFrame):
            self.v_mm_s, self.w_mrad_s = frame.v_mm_s, frame.w_mrad_s
            self.state = (
                McuState.ARMED_MOVING
                if frame.v_mm_s or frame.w_mrad_s
                else McuState.ARMED_IDLE
            )
        elif isinstance(frame, ArmFrame):
            self.state = McuState.ARMED_IDLE
            self._ack(ArmFrame.TYPE, frame.seq, echo=frame.nonce)
        elif isinstance(frame, DisarmFrame):
            self.state = McuState.DISARMED
            self._ack(DisarmFrame.TYPE, frame.seq)
        elif isinstance(frame, StopFrame):
            self.v_mm_s = self.w_mrad_s = 0
            self._ack(StopFrame.TYPE, frame.seq)
        elif isinstance(frame, ClearFaultFrame):
            self.fault = 0
            self._ack(ClearFaultFrame.TYPE, frame.seq)
        elif isinstance(frame, PingFrame):
            self._send(
                PongFrame(
                    seq=self._next_seq(),
                    session=self.session,
                    echo_pi_mono_us=frame.pi_mono_us,
                    mcu_us=1234,
                )
            )

    def _ack(self, letter: str, seq: int, *, echo: int = 0) -> None:
        self._send(
            AckFrame(
                seq=self._next_seq(),
                session=self.session,
                ack_type=ack_type_code(letter),
                ack_seq=seq,
                result=int(AckResult.OK),
                reason=0,
                echo=echo,
            )
        )

    async def _telemetry_loop(self) -> None:
        while True:
            await asyncio.sleep(0.02)
            if self.plant:
                self._advance(0.02)
            if self.telemetry_on:
                self._send(self.telemetry())

    def _advance(self, dt_s: float) -> None:
        """A first-order wheel plant: the commanded body velocity, integrated."""
        half = self.w_mrad_s / 1000.0 * self.track_m / 2.0
        left_m = (self.v_mm_s / 1000.0 - half) * dt_s
        right_m = (self.v_mm_s / 1000.0 + half) * dt_s
        self._left_partial += left_m / self.metres_per_tick
        self._right_partial += right_m / self.metres_per_tick
        left_ticks = int(self._left_partial)
        right_ticks = int(self._right_partial)
        self._left_partial -= left_ticks
        self._right_partial -= right_ticks
        self.left_ticks += left_ticks
        self.right_ticks += right_ticks

    def telemetry(self) -> TelemetryFrame:
        return TelemetryFrame(
            seq=self._next_seq(),
            session=self.session,
            mcu_us=int(time.monotonic() * 1e6),
            ack_seq=self.ack_seq,
            state=int(self.state),
            ctrl_flags=self.ctrl_flags,
            fault=self.fault,
            left_ticks=self.left_ticks,
            right_ticks=self.right_ticks,
            v_meas_mm_s=self.v_mm_s,
            w_meas_mrad_s=self.w_mrad_s,
            v_cmd_mm_s=self.v_mm_s,
            w_cmd_mrad_s=self.w_mrad_s,
            vbat_mv=self.vbat_mv,
            imotor_ma=self.imotor_ma,
            tof_front_mm=1204,
            tof_cliff_mm=98,
            sensor_age_ms=18,
            loop_late_pct=0,
            rx_drop=self.rx_drop,
            gyro_z_mrad_s=0,
            rails=0,
            motion=0,
        )

    def _next_seq(self) -> int:
        seq = self._up_seq
        self._up_seq = (self._up_seq + 1) & 0xFFFF
        return seq

    def _send(self, frame: Frame) -> None:
        with contextlib.suppress(OSError):
            os.write(self.fd, encode_frame(frame))


async def until(predicate: Callable[[], bool], timeout: float = TIMEOUT_S) -> None:
    """Poll ``predicate`` until it holds, or fail the test."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.005)
    raise AssertionError("condition not reached within the timeout")


@contextlib.asynccontextmanager
async def linked(
    *, ack_seq: int = 0, **serial: object
) -> AsyncIterator[tuple[Link, SetpointCell, FakeMcu]]:
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    settings: dict[str, object] = {
        "backend": "pty",
        "port": os.ttyname(slave),
        "setpoint_hz": 50,
        "reseed_wait_ms": 300,
        "open_retry_ms": 50,
    }
    settings.update(serial)
    config = RobotConfig(serial=SerialConfig(**settings))  # type: ignore[arg-type]
    cell = SetpointCell()
    link = Link(config, cell)
    mcu = FakeMcu(master, ack_seq=ack_seq)
    mcu.start()
    task = asyncio.create_task(link.run())
    try:
        yield link, cell, mcu
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await mcu.stop()
        for fd in (master, slave):
            with contextlib.suppress(OSError):
                os.close(fd)


async def arm(link: Link, mcu: FakeMcu) -> None:
    await until(lambda: link.ready and link.telemetry is not None)
    link.arm()
    await until(lambda: link.armed)


async def stream(
    cell: SetpointCell, link: Link, v: float, w: float, seconds: float
) -> None:
    """Stamp the cell the way a live goal owner would, once per 20 ms."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        cell.stamp(v, w, time.monotonic_ns())
        await asyncio.sleep(0.02)


# ---------------------------------------------------------------------------
# Handshake and re-seed (A8)
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_the_reseed_adopts_ack_seq_plus_one() -> None:
    async def scenario() -> None:
        async with linked(ack_seq=7) as (link, _cell, mcu):
            await until(lambda: bool(mcu.frames(HelloFrame)))
            hello = mcu.frames(HelloFrame)[0]
            assert hello.seq == 8, "down_seq must be T.ack_seq + 1"
            assert hello.session == 0, "H carries the wildcard session"
            await until(lambda: link.session == 40010)

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_the_reseed_falls_back_to_one_when_no_telemetry_arrives() -> None:
    """mcu-sim's ``no_t_before_h``: an unbounded wait would deadlock exactly
    the case (unflashed or unplugged MCU) deploy step 10 leaves standing.

    Telemetry resuming is not the property: the fallback leaves the session at
    the wildcard, and both ``_emit_setpoint`` and ``arm`` bail on it, so a link
    that never learns the session emits no ``V`` and sends no ``A`` for the life
    of the process.  A real MCU stops sending ``B`` the moment it accepts the
    ``H``, so no banner is coming to trigger a resync -- the ``T`` stream has to.
    """

    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            mcu.telemetry_on = False
            await until(lambda: bool(mcu.frames(HelloFrame)), timeout=6.0)
            assert mcu.frames(HelloFrame)[0].seq == 1
            assert link.session == SESSION_WILDCARD
            mcu.telemetry_on = True
            await until(lambda: link.telemetry is not None)

            # The session is learned from the live T stream, without a banner.
            await until(lambda: link.session == 40010, timeout=6.0)
            await until(lambda: link.ready)
            assert link.arm() is not None, "arm() must work once the session is known"
            await until(lambda: link.armed)
            cell.stamp(0.20, 0.0, time.monotonic_ns())
            await until(lambda: bool(mcu.frames(VelocityFrame)), timeout=6.0)

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_no_velocity_is_sent_before_the_handshake_completes() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            cell.stamp(0.30, 0.0, time.monotonic_ns())
            await until(lambda: bool(mcu.frames(HelloFrame)))
            hello_index = mcu.received.index(mcu.frames(HelloFrame)[0])
            assert not [
                f for f in mcu.received[:hello_index] if isinstance(f, VelocityFrame)
            ]
            assert not link.armed

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# The 20 Hz stream
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_an_armed_link_streams_the_setpoint() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.210, 0.4)
            moving = [f for f in mcu.velocities if f.v_mm_s != 0]
            assert moving, "an armed link with a live setpoint must emit V"
            assert moving[-1].v_mm_s == 250
            assert moving[-1].w_mrad_s == 210
            assert moving[-1].frame_ttl_ms == 300
            assert moving[-1].flags == 0
            assert moving[-1].session == 40010

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_zero_setpoint_still_renews_the_ttl() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            before = len(mcu.velocities)
            await stream(cell, link, 0.0, 0.0, 0.3)
            assert len(mcu.velocities) > before
            assert all(f.v_mm_s == 0 for f in mcu.velocities[before:])

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_frozen_goal_owner_stops_the_wheels() -> None:
    """I-14: the writer keeps running, nobody stamps, V goes to zero within
    the setpoint window without anything detecting the freeze."""

    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.3)
            assert any(f.v_mm_s == 250 for f in mcu.velocities)
            frozen_at = len(mcu.velocities)
            await asyncio.sleep(0.15)  # the goal owner has stopped stamping
            steady = len(mcu.velocities)
            await asyncio.sleep(0.25)
            assert len(mcu.velocities) > steady, "the writer must keep streaming"
            assert all(
                f.v_mm_s == 0 and f.w_mrad_s == 0 for f in mcu.velocities[steady:]
            ), "every frame after the stamp expired must be zero"
            assert frozen_at <= steady

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_setpoint_beyond_the_controller_cap_is_clamped_before_the_port() -> None:
    """I-8: zero out-of-bounds values reach the port."""

    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 5.0, 9.0, 0.3)
            moving = [f for f in mcu.velocities if f.v_mm_s != 0]
            assert moving
            assert moving[-1].v_mm_s == MAX_V_MM_S == 300
            assert moving[-1].w_mrad_s == MAX_W_MRAD_S == 1047

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_stale_telemetry_stops_the_velocity_stream() -> None:
    """T0: the Pi refuses to command a controller it cannot observe."""

    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.2)
            mcu.telemetry_on = False
            await until(lambda: link.link_down, timeout=2.0)
            await asyncio.sleep(0.1)  # let anything already on the wire arrive
            quiet = len(mcu.velocities)
            await stream(cell, link, 0.250, 0.0, 0.3)
            assert len(mcu.velocities) == quiet

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_disarm_stops_the_stream_immediately() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.2)
            link.disarm()
            assert not link.armed
            await asyncio.sleep(0.1)  # let anything already on the wire arrive
            quiet = len(mcu.velocities)
            await stream(cell, link, 0.250, 0.0, 0.2)
            assert len(mcu.velocities) == quiet

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_stop_drops_the_setpoint_before_sending_the_frame() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            cell.stamp(0.30, 0.0, time.monotonic_ns())
            link.stop(mode=0)
            assert cell.read(time.monotonic_ns()) == (0.0, 0.0)
            await until(lambda: bool(mcu.frames(StopFrame)))
            assert mcu.frames(StopFrame)[0].mode == 0  # type: ignore[attr-defined]

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_shutdown_brakes_and_disarms_before_the_link_task_is_cancelled() -> None:
    """4.2: robotd sends ``D`` on shutdown.

    Cancelling ``Link.run`` first runs its per-iteration ``finally`` --
    ``_teardown()`` -- which clears the fd, so ``Link.close``'s ``connected and
    _ready`` guard is false and neither frame is ever written.  The order is
    the whole of the fix, so the test asserts the order: close, then cancel.
    """

    async def scenario() -> None:
        master, slave = pty.openpty()
        os.set_blocking(master, False)
        config = RobotConfig(
            serial=SerialConfig(  # type: ignore[arg-type]
                backend="pty",
                port=os.ttyname(slave),
                setpoint_hz=50,
                reseed_wait_ms=300,
                open_retry_ms=50,
            )
        )
        cell = SetpointCell()
        link = Link(config, cell)
        mcu = FakeMcu(master)
        mcu.start()
        task = asyncio.create_task(link.run())
        try:
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.2)

            await link.close()  # Robotd.run's order, after the fix
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

            await until(lambda: bool(mcu.frames(StopFrame)))
            assert mcu.frames(StopFrame)[0].mode == 0  # type: ignore[attr-defined]
            assert mcu.frames(DisarmFrame)
        finally:
            await mcu.stop()
            for fd in (master, slave):
                with contextlib.suppress(OSError):
                    os.close(fd)

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_cancelling_the_link_task_first_sends_neither_frame() -> None:
    """The failure this replaced, pinned so the order cannot drift back."""

    async def scenario() -> None:
        master, slave = pty.openpty()
        os.set_blocking(master, False)
        config = RobotConfig(
            serial=SerialConfig(  # type: ignore[arg-type]
                backend="pty",
                port=os.ttyname(slave),
                setpoint_hz=50,
                reseed_wait_ms=300,
                open_retry_ms=50,
            )
        )
        cell = SetpointCell()
        link = Link(config, cell)
        mcu = FakeMcu(master)
        mcu.start()
        task = asyncio.create_task(link.run())
        try:
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.2)

            task.cancel()  # the old order
            await asyncio.gather(task, return_exceptions=True)
            await link.close()
            await asyncio.sleep(0.2)

            assert not mcu.frames(StopFrame)
            assert not mcu.frames(DisarmFrame)
        finally:
            await mcu.stop()
            for fd in (master, slave):
                with contextlib.suppress(OSError):
                    os.close(fd)

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_readiness_reads_link_alive_and_the_command_gate_reads_its_own_key() -> None:
    """5.8: ``link_alive_max_age_ms`` drives readiness; ``cmd_gate_max_age_ms``
    is the separate T0 gate that blocks emitting ``V``.

    Both predicates derived from ``cmd_gate_max_age_ms``, so an operator who
    tightened ``ROVER__SAFETY__LINK_ALIVE_MAX_AGE_MS`` changed nothing at all --
    the key was validated, logged at WARN and republished in ``welcome.safety``
    with no reader anywhere.
    """

    async def scenario() -> None:
        async with linked() as (link, _cell, mcu):
            await until(lambda: link.telemetry is not None)
            assert not link.link_down
            assert link.link_alive

            mcu.telemetry_on = False
            # Past the 150 ms command gate but inside the 200 ms liveness
            # window: robotd stops commanding and is still connected.
            await until(lambda: link.link_down)
            age = link.telemetry_age_ms()
            assert age is not None
            if age < link.config.safety.link_alive_max_age_ms:
                assert link.link_alive
            await until(lambda: not link.link_alive)

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# Session change and reconnect (I-3, I-13)
# ---------------------------------------------------------------------------


@pytest.mark.timeout(30)
def test_a_session_change_is_handled_like_a_port_open_and_replays_nothing() -> None:
    async def scenario() -> None:
        async with linked() as (link, cell, mcu):
            await arm(link, mcu)
            await stream(cell, link, 0.250, 0.0, 0.3)
            assert any(f.v_mm_s == 250 for f in mcu.velocities)

            mcu.session = 51882
            mcu.state = McuState.DISARMED
            await until(lambda: link.session == 51882, timeout=3.0)
            boundary = len(mcu.velocities)
            assert not link.armed, "a restart must start disarmed (I-3)"
            assert cell.read(time.monotonic_ns()) == (0.0, 0.0)
            await until(
                lambda: len(mcu.frames(HelloFrame)) >= 2, timeout=2.0
            )  # a session change re-sends H
            await asyncio.sleep(0.2)
            assert all(
                f.v_mm_s == 0 and f.w_mrad_s == 0
                for f in mcu.velocities[boundary:]
            ), "no motion may be replayed across a session change (I-13)"

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_the_round_trip_probe_uses_only_the_host_clock() -> None:
    """P/O is diagnostic; the echo is an opaque token and the RTT is computed
    entirely on our side (I-17)."""

    async def scenario() -> None:
        async with linked() as (link, _cell, mcu):
            await until(lambda: link.ready)
            await until(lambda: bool(mcu.frames(PingFrame)), timeout=3.0)
            await until(lambda: link.rtt_ms is not None, timeout=2.0)
            assert link.rtt_ms is not None and 0.0 <= link.rtt_ms < 1000.0

    asyncio.run(scenario())


@pytest.mark.timeout(30)
def test_a_corrupted_line_is_counted_and_changes_nothing() -> None:
    """I-2: a bad frame is dropped, counted, and renews no TTL."""

    async def scenario() -> None:
        async with linked() as (link, _cell, mcu):
            await until(lambda: link.telemetry is not None)
            before = link.telemetry
            os.write(mcu.fd, b"$T,2,9,40010,not-an-integer*0000\n")
            await until(lambda: link.reader.counters.dropped > 0, timeout=2.0)
            assert link.telemetry is not None
            assert link.telemetry.session == before.session  # type: ignore[union-attr]

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# The real simulator
# ---------------------------------------------------------------------------


_SIM_SEARCH_PATHS = (
    "firmware/build/host/mcu-sim",
    "firmware/host/build/mcu-sim",
    "firmware/build/mcu-sim",
    "build/mcu-sim",
)
"""Where the host build drops the binary; the same list rover_devtools uses."""


def _simulator_reason() -> str | None:
    """Why the mcu-sim test cannot run, or ``None`` when it can."""
    module = REPO / "packages" / "rover_devtools" / "mcu_sim.py"
    if not module.exists():
        return f"mcu-sim is not available: {module} does not exist"
    override = os.environ.get("ROVER_MCU_SIM")
    if override and Path(override).exists():
        return None
    if any((REPO / candidate).exists() for candidate in _SIM_SEARCH_PATHS):
        return None
    return (
        "the mcu-sim binary is not built: none of "
        f"{', '.join(_SIM_SEARCH_PATHS)} exists under {REPO}. "
        "Build it with `make sim` (cmake -DROVER_HOST_TEST=ON), or point "
        "ROVER_MCU_SIM at the binary."
    )


@pytest.mark.timeout(60)
@pytest.mark.skipif(_simulator_reason() is not None, reason=_simulator_reason() or "")
def test_the_link_drives_the_real_simulator_over_a_pty() -> None:
    """``mcu-sim`` symlinks ``./run/mcu.pty`` to whatever slave the OS gave it
    and removes it on exit (ARCHITECTURE 10); robotd retries ``open()`` until
    the path exists, which is what removes make dev's start-order dependency."""

    async def scenario(port: Path) -> None:
        config = RobotConfig(
            serial=SerialConfig(
                backend="pty", port=str(port), setpoint_hz=20, open_retry_ms=100
            )
        )
        cell = SetpointCell()
        link = Link(config, cell)
        task = asyncio.create_task(link.run())
        try:
            await until(lambda: link.ready and link.telemetry is not None, timeout=10.0)
            assert link.session != 0
            # ARCHITECTURE 4.1: the MCU takes 50 cliff samples on every entry to
            # DISARMED and refuses *all* forward motion while ctrl_flags b6
            # cal_valid is 0.  Arming inside that window gives an accepted A,
            # accepted V frames and a stationary robot, so the calibration is a
            # precondition of the drive rather than a race to lose.
            await until(
                lambda: link.telemetry is not None
                and bool(link.telemetry.ctrl_flags & int(CtrlFlag.CAL_VALID)),
                timeout=10.0,
            )
            link.arm()
            await until(lambda: link.armed, timeout=5.0)
            await stream(cell, link, 0.150, 0.0, 1.0)
            assert link.telemetry is not None
            assert link.telemetry.v_cmd_mm_s != 0
            link.stop(mode=0)
            await asyncio.sleep(0.3)
            await link.close()
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    # Not run/mcu.pty: that is the symlink `[serial] port` names and a running
    # `make sim` owns.  This test drives its own simulator and must not touch it.
    port = REPO / "run" / "test-mcu.pty"
    process = subprocess.Popen(  # noqa: S603 - a repo-local developer tool
        [sys.executable, "-m", "rover_devtools.mcu_sim", "--link", str(port)],
        cwd=REPO,
        env={**os.environ, "PYTHONPATH": str(REPO / "packages")},
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not port.exists():
            if process.poll() is not None:
                pytest.skip(
                    "mcu-sim exited immediately: "
                    f"{(process.stderr.read() if process.stderr else b'')!r}"
                )
            time.sleep(0.1)
        if not port.exists():
            pytest.skip(f"mcu-sim did not create {port} within 15 s")
        asyncio.run(scenario(port))
    finally:
        process.terminate()
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=5)
        # Only the symlink this test caused.  Removing run/ wholesale takes the
        # sockets out from under a `make sim` running in another shell, which
        # leaves six live processes and no way to reach any of them.
        with contextlib.suppress(OSError):
            port.unlink()
