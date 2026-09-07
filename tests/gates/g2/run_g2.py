#!/usr/bin/env python3
"""G2 -- the MCU bench gate (ARCHITECTURE 13), sim first, then hardware.

Sub-gates a to i are I-1, I-2, I-3, I-4, I-6, I-7, I-10, I-20 and I-24, each with
the pass criterion stated in ARCHITECTURE 8.  Order matters and this script
enforces it: **braking deceleration from the encoders is measured first**,
because I-1's own time criteria derive from it and a fixed "stationary within
500 ms" would demand 1.5 m/s2 from a design whose own fallback threshold is
0.6 m/s2.  The obstacle-to-halt *distance* at 0.30 m/s is measured next, since
that -- not the deceleration -- is what triggers the A21 fallback.

The gate drives the link itself rather than going through robotd: the subject of
every invariant here is the controller.  On the Mac that link is ``mcu-sim`` over
a pty; on the rover it is ``/dev/rover-mcu``.  Cases that cannot be simulated
(e-stop electrics, the battery ladder on a bench supply, pin state through a
reset edge) report "requires hardware" and never a pass.

    python tests/gates/g2/run_g2.py                 # mcu-sim on this Mac
    python tests/gates/g2/run_g2.py --hardware      # on the rover, wheels up
"""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve()
sys.path[:0] = [str(_HERE.parents[3] / "packages"), str(_HERE.parents[1])]

from gatelib.env import REPO, have_module, load_robot_config  # noqa: E402
from gatelib.link import McuLink, SimProcess  # noqa: E402
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for  # noqa: E402
from rover_contracts import (  # noqa: E402
    AckReason,
    AckResult,
    CtrlFlag,
    EventCode,
    Fault,
    FrameReader,
    HelloFrame,
    McuState,
    SessionGuard,
    TelemetryFrame,
    VelocityFrame,
    crc16_ccitt_false,
    encode_frame,
)

GATE = "G2"

CRUISE_MM_S = 300
"""The MCU cap, and the only speed the normal command path can produce."""

BRAKE_FLOOR_MPS2 = 0.6
"""A21's own floor.  Below it the A21 fallback applies -- 350 mm stop zone,
800 mm slow zone, or a 250 mm/s hard cap -- and G4 must not run until it does."""

TELEMETRY_PERIOD_MS = 20
"""``T`` streams at 50 Hz, so nothing on this link is observable sooner."""

ABORT_RAMP_MM_S2 = 2000.0
"""ARCHITECTURE 4.1: every decrease bypasses ``accel_mps2`` and uses this."""

SLOW_ZONE_V_CAP_MM_S = 150
"""A21's TTC law: ``v <= (d - tof_stop_mm) / 1.0 s``, capped here."""

TTC_OBSTACLE_MM = 400
"""Inside the 600 mm slow zone and outside the 250 mm stop zone, so both halves
of the law are live: the cap binds and b0 has something to refuse."""

HALT_DISTANCE_LIMIT_MM = 150
"""A21: above this the fallback triggers.  A distance, not a deceleration, since
half the error budget is detect latency that a deceleration cannot see."""


# ---------------------------------------------------------------------------
# Link helpers
# ---------------------------------------------------------------------------


def wheel_travel_m(config: Any, first: TelemetryFrame, last: TelemetryFrame) -> float:
    """Metres travelled between two telemetry frames, from raw ticks."""
    import math

    left = last.left_ticks - first.left_ticks
    right = last.right_ticks - first.right_ticks
    revolutions = (left + right) / 2.0 / config.robot.ticks_per_rev
    return revolutions * 2.0 * math.pi * config.robot.wheel_radius_m


def open_link(args: Any, config: Any) -> tuple[McuLink | None, SimProcess | None, str]:
    """The link under test, and why it is missing when it is."""
    if args.hardware:
        port = args.port or config.serial.port
        if not Path(port).exists():
            return None, None, f"the MCU at {port}"
        link = McuLink(port, config.serial.baud)
        link.open()
        return link, None, ""

    if not have_module("rover_devtools.mcu_sim"):
        return None, None, "rover_devtools.mcu_sim"
    command = args.sim_cmd.split() if args.sim_cmd else None
    # No pty_path: SimProcess defaults to run/gate-mcu.pty, which keeps a gate
    # from stealing the symlink a running `make sim` has robotd open on.
    # --port still names a real device for the hardware run.
    sim = SimProcess(args.sim_flags.split(), command=command, cwd=REPO)
    if not sim.start():
        return None, None, f"rover_devtools.mcu_sim to start ({sim.error})"
    link = McuLink(sim.pty_path if sim.pty_path.is_absolute() else REPO / sim.pty_path)
    link.open()
    return link, sim, ""


def with_sim(
    args: Any, config: Any, flags: str
) -> tuple[McuLink | None, SimProcess | None, str]:
    """A fresh simulator carrying one injection flag set."""
    scoped = type(args)(**{**vars(args), "sim_flags": flags})
    return open_link(scoped, config)


# ---------------------------------------------------------------------------
# The measurements that come first
# ---------------------------------------------------------------------------


def measure_braking(
    gate: GateRun,
    link: McuLink,
    config: Any,
    speed_mm_s: int,
    surface: str,
    *,
    gated: bool,
) -> float | None:
    """Command a cruise, brake, and read the deceleration off the encoders.

    ``gated`` only at 0.30 m/s: A21 states the 0.6 m/s2 floor as the equivalent
    of the 150 mm obstacle-to-halt distance *at 0.30 m/s*, so applying it to the
    0.15 m/s run would fail a plant the argument never covered.  The 0.15 m/s
    number is recorded because ARCHITECTURE 13 asks for both.
    """
    since = time.monotonic()
    link.stream(speed_mm_s, 0)
    reached = link.wait(
        TelemetryFrame,
        lambda f: abs(f.v_meas_mm_s) >= speed_mm_s * 0.85,
        6.0,
        since=since,
    )
    if reached is None:
        gate.skip(
            "brake",
            "OI-1",
            f"braking deceleration at {speed_mm_s} mm/s on {surface}",
            f"the plant to reach {speed_mm_s} mm/s (it did not)",
        )
        link.stream(0, 0)
        return None
    start_frame: TelemetryFrame = reached.frame  # type: ignore[assignment]
    v0 = abs(start_frame.v_meas_mm_s)
    braked_at = time.monotonic()
    link.stop_stream()
    link.stop(0)
    stopped = link.wait(
        TelemetryFrame, lambda f: abs(f.v_meas_mm_s) < 10, 3.0, since=braked_at
    )
    if stopped is None:
        gate.fail(
            "brake",
            "OI-1",
            f"braking deceleration at {speed_mm_s} mm/s on {surface}",
            "never reached |v| < 10 mm/s within 3 s",
        )
        return None
    elapsed = stopped.mono - braked_at
    end_frame: TelemetryFrame = stopped.frame  # type: ignore[assignment]
    travel_mm = abs(wheel_travel_m(config, start_frame, end_frame)) * 1000.0
    decel = (v0 / 1000.0) / elapsed if elapsed > 1e-6 else 0.0
    gate.record(
        "brake",
        "OI-1",
        f"braking deceleration at {speed_mm_s} mm/s on {surface}",
        Status.PASS if not gated or decel >= BRAKE_FLOOR_MPS2 else Status.FAIL,
        f"{decel:.2f} m/s2 over {elapsed * 1000:.0f} ms and {travel_mm:.0f} mm"
        + (
            f" (A21 floor {BRAKE_FLOOR_MPS2} m/s2)"
            if gated
            else " (recorded, not gated: A21 states the floor at 0.30 m/s)"
        ),
        surface=surface,
        v0_mm_s=v0,
        decel_mps2=round(decel, 3),
        brake_ms=round(elapsed * 1000, 1),
        travel_mm=round(travel_mm, 1),
    )
    return decel


# ---------------------------------------------------------------------------
# Sub-gates
# ---------------------------------------------------------------------------


def sub_a_ttl(gate: GateRun, link: McuLink, config: Any, t_brake_s: float) -> None:
    """I-1: motion requires a ``V`` accepted within ``frame_ttl_ms``."""
    link.stream(CRUISE_MM_S, 0)
    # Cut at cruise, not at the first frame that moved: the abort ramp term below
    # is v0/2000, so measuring from half speed would halve the budget.
    moving = link.wait(
        TelemetryFrame, lambda f: abs(f.v_meas_mm_s) >= CRUISE_MM_S * 0.85, 6.0
    )
    if moving is None:
        link.stop_stream()
        gate.skip(
            "a", "I-1", "TTL expiry zeroes the command and brakes", "a moving plant"
        )
        return
    start: TelemetryFrame = moving.frame  # type: ignore[assignment]
    link.stop_stream()
    cut = link.last_tx  # the TTL runs from the last V, not from the decision

    ttl_ms = config.limits.frame_ttl_ms
    # The TTL must *fire* within its own window.  Observing that it did costs one
    # 10 ms control period to act on it, one 20 ms telemetry period to publish it,
    # and one more for the gate to be scheduled and read it -- 350 ms, which still
    # fails a 500 ms TTL and is what the criterion is for.
    fire_deadline_ms = ttl_ms + 10 + 2 * TELEMETRY_PERIOD_MS
    raised = link.wait(
        TelemetryFrame, lambda f: bool(f.fault & Fault.TTL), 2.0, since=cut
    )
    ok_fire = raised is not None and (raised.mono - cut) * 1000.0 <= fire_deadline_ms

    zeroed = link.wait(
        TelemetryFrame,
        lambda f: f.v_cmd_mm_s == 0 and f.w_cmd_mrad_s == 0,
        2.0,
        since=cut,
    )
    # v_cmd_mm_s is post-clamp and post-slew (5.1), and every decrease uses the
    # 2000 mm/s2 abort ramp (4.1), so reaching zero costs v0/2000 on top of the
    # TTL.  I-1's literal "within 300 ms + one control period" is only reachable
    # if v_cmd were the pre-slew setpoint; see docs/deviations.md.
    ramp_ms = 1000.0 * abs(start.v_cmd_mm_s) / ABORT_RAMP_MM_S2
    zero_deadline_ms = fire_deadline_ms + ramp_ms
    ok_cmd = zeroed is not None and (zeroed.mono - cut) * 1000.0 <= zero_deadline_ms

    stopped = link.wait(TelemetryFrame, lambda f: abs(f.v_meas_mm_s) < 10, 3.0, since=cut)
    # TTL, then the abort ramp, then the brake tail the G2 braking run measured
    # with an immediate S.  The ramp and the tail overlap in reality, so this is
    # conservative -- travel <= 200 mm is the criterion that is not.
    stop_budget = ttl_ms / 1000.0 + ramp_ms / 1000.0 + t_brake_s
    ok_stop = stopped is not None and (stopped.mono - cut) <= stop_budget
    travel_mm = (
        abs(wheel_travel_m(config, start, stopped.frame)) * 1000.0  # type: ignore[arg-type]
        if stopped is not None
        else float("inf")
    )
    if zeroed is None or stopped is None or raised is None:
        gate.fail(
            "a",
            "I-1",
            "TTL expiry zeroes the command and brakes",
            f"missing: TTL bit={raised is not None}, v_cmd zero={zeroed is not None}, "
            f"halt={stopped is not None}",
        )
        return
    gate.check(
        ok_fire and ok_cmd and ok_stop and travel_mm <= 200.0,
        "a",
        "I-1",
        "TTL expiry zeroes the command and brakes",
        f"TTL bit in {(raised.mono - cut) * 1000:.0f} ms (<= {fire_deadline_ms}), "
        f"v_cmd zero in {(zeroed.mono - cut) * 1000:.0f} ms "
        f"(<= {zero_deadline_ms:.0f} = that plus the {ramp_ms:.0f} ms abort ramp), "
        f"|v| < 10 mm/s in {(stopped.mono - cut) * 1000:.0f} ms "
        f"(<= {stop_budget * 1000:.0f} = TTL + ramp + t_brake), "
        f"travel {travel_mm:.0f} mm (<= 200)",
        ttl_fire_ms=round((raised.mono - cut) * 1000, 1),
        zero_ms=round((zeroed.mono - cut) * 1000, 1),
        halt_ms=round((stopped.mono - cut) * 1000, 1),
        travel_mm=round(travel_mm, 1) if travel_mm != float("inf") else None,
        abort_ramp_ms=round(ramp_ms, 1),
    )
    link.stop(0)


def sub_b_codec(gate: GateRun) -> None:
    """I-2's Pi-side half, which needs no controller at all.

    Six ways of being invalid, one frame each, each asserted to be dropped and
    counted.  This is the half that runs on a bare checkout; the link half below
    proves the MCU does the same and does not renew its TTL.
    """
    good = encode_frame(VelocityFrame(9, 40010, 250, 0, 300, 0))
    body = good[1 : good.index(b"*")]
    cases = {
        "bad_crc": b"$" + body + b"*0000\n",
        "bad_length": b"$V,2,9,40010,250,0*"
        + f"{crc16_ccitt_false(b'V,2,9,40010,250,0'):04X}".encode()
        + b"\n",
        "unknown_type": _reframe(b"Z,2,9,40010,250,0,300,0"),
        "unsupported_version": _reframe(b"V,3,9,40010,250,0,300,0"),
        "bad_session": _reframe(b"V,2,9,0,250,0,300,0"),
        "overlong": b"$V,2,9,40010," + b"9" * 200 + b"*FFFF\n",
    }
    reader = FrameReader()
    missed = []
    for name, line in cases.items():
        before = reader.counters.dropped
        results = reader.feed(line)
        if not results or any(r.ok for r in results):
            missed.append(f"{name} was accepted")
        elif reader.counters.dropped == before:
            missed.append(f"{name} was not counted")
    reader.feed(good)
    guard = SessionGuard(session=40010, last_seq=9)
    replay = guard.check(VelocityFrame(9, 40010, 250, 0, 300, 0))
    if replay is not AckReason.STALE_SEQ:
        missed.append(f"a replayed seq gave {replay.name}, not STALE_SEQ")
    if guard.check(VelocityFrame(10, 51882, 0, 0, 300, 0)) is not AckReason.BAD_SESSION:
        missed.append("a foreign session was accepted")
    if guard.check(HelloFrame(11, 0, 1)) is not AckReason.NONE:
        missed.append("the H wildcard session was refused")
    gate.check(
        not missed,
        "b-codec",
        "I-2",
        "every invalid frame is dropped and counted (Pi side)",
        "; ".join(missed) if missed else f"{len(cases)} forms + replay + foreign session",
        forms=len(cases),
        counted=reader.counters.dropped,
    )


def _reframe(body: bytes) -> bytes:
    return b"$" + body + b"*" + f"{crc16_ccitt_false(body):04X}".encode() + b"\n"


def sub_b_link(gate: GateRun, link: McuLink) -> None:
    """I-2 on the wire: corruption raises ``rx_drop`` and does not renew the TTL."""
    link.stop_stream()
    time.sleep(0.1)
    telemetry = link.telemetry()
    if telemetry is None:
        gate.skip(
            "b",
            "I-2",
            "corruption raises rx_drop and does not renew the TTL",
            "telemetry",
        )
        return
    before_drop, before_ack = telemetry.rx_drop, telemetry.ack_seq
    seq, session = link.down_seq, link.session
    body = b"V,2,%d,%d,250,0,300,0" % (seq, session)
    for line in (
        b"$" + body + b"*0000\n",
        _reframe(b"Z,2,%d,%d,0" % (seq, session)),
        _reframe(b"V,3,%d,%d,250,0,300,0" % (seq, session)),
    ):
        link.raw(line)
        time.sleep(0.05)
    after = link.wait(TelemetryFrame, lambda f: f.rx_drop > before_drop, 1.0)
    latest = link.telemetry()
    gate.check(
        after is not None and latest is not None and latest.ack_seq == before_ack,
        "b",
        "I-2",
        "corruption raises rx_drop and does not renew the TTL",
        f"rx_drop {before_drop} -> {latest.rx_drop if latest else '?'}, "
        f"ack_seq {before_ack} -> {latest.ack_seq if latest else '?'} (must not move)",
        rx_drop_before=before_drop,
        rx_drop_after=latest.rx_drop if latest else None,
    )


def sub_c_boot(gate: GateRun, link: McuLink) -> None:
    """I-3: boots DISARMED, no motion until ``H`` + ``A`` in the current session."""
    first = link.wait(TelemetryFrame, lambda _f: True, 3.0)
    if first is None:
        gate.skip("c", "I-3", "boots DISARMED and refuses V before H+A", "telemetry")
        return
    telemetry: TelemetryFrame = first.frame  # type: ignore[assignment]
    link.session = telemetry.session
    before = time.monotonic()
    link.velocity(CRUISE_MM_S, 0)
    time.sleep(0.2)
    moved = link.wait(TelemetryFrame, lambda f: abs(f.v_cmd_mm_s) > 0, 0.5, since=before)
    disarmed = telemetry.state in (McuState.BOOT, McuState.DISARMED)
    gate.check(
        disarmed and moved is None,
        "c",
        "I-3",
        "boots DISARMED and refuses V before H+A",
        f"state {McuState(telemetry.state).name}, "
        f"v_cmd after an unarmed V: {moved.frame.v_cmd_mm_s if moved else 0}",  # type: ignore[attr-defined]
        state=McuState(telemetry.state).name,
    )


def sub_c_restart(gate: GateRun, args: Any, config: Any) -> None:
    """I-3's other half: a session change under a still-open fd."""
    if args.hardware:
        gate.skip(
            "c2",
            "I-3",
            "a T.SESS change is handled like a port open",
            "a power-cycle of the MCU mid-drive (do it by hand and watch the event)",
        )
        return
    link, sim, missing = with_sim(args, config, "reset_mid_drive")
    if link is None:
        gate.skip("c2", "I-3", "a T.SESS change is handled like a port open", missing)
        return
    try:
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)
        first = link.session
        link.stream(CRUISE_MM_S, 0)
        changed = link.wait(TelemetryFrame, lambda f: f.session != first, 8.0)
        link.stop_stream()
        if changed is None:
            gate.fail(
                "c2",
                "I-3",
                "a T.SESS change is handled like a port open",
                "the simulator never changed session under reset_mid_drive",
            )
            return
        recovered_at = time.monotonic()
        link.reseed(config.serial.reseed_wait_ms / 1000.0)
        link.hello()
        # 4.1: entry to DISARMED retakes the 50-sample cliff baseline, and the
        # arm policy is only satisfiable once it has.  Waiting for b6 is the
        # documented precondition, not a way to buy time.
        link.wait(
            TelemetryFrame,
            lambda f: bool(f.ctrl_flags & CtrlFlag.CAL_VALID),
            2.0,
        )
        ack = link.arm(timeout=2.0)
        elapsed = time.monotonic() - recovered_at
        gate.check(
            ack is not None and ack.result == AckResult.OK and elapsed <= 2.0,
            "c2",
            "I-3",
            "a T.SESS change is handled like a port open",
            f"session {first} -> {link.session}, re-seed + H + A accepted in "
            f"{elapsed * 1000:.0f} ms (<= 2000)",
            old_session=first,
            new_session=link.session,
            recover_ms=round(elapsed * 1000, 1),
        )
    finally:
        link.close()
        if sim is not None:
            sim.stop()


def sub_d_caps(gate: GateRun, link: McuLink | None) -> None:
    """I-4: no frame raises a compiled cap, for any value of ``V.flags``."""
    if link is None:
        gate.skip("d", "I-4", "900 mm/s clamps to 300 and raises CAP_CLAMPED", "a link")
        return
    since = time.monotonic()
    link.stream(900, 0)
    clamped = link.wait(
        TelemetryFrame,
        lambda f: bool(f.fault & Fault.CAP_CLAMPED),
        2.0,
        since=since,
    )
    # Let the slew limiter finish climbing: 900 clamped to 300 at accel_mps2 is
    # 600 ms of ramp, and a peak read on the first non-zero frame is 9 mm/s and
    # proves nothing.
    time.sleep(1.5)
    peak = max(
        (
            abs(sample.frame.v_cmd_mm_s)  # type: ignore[attr-defined]
            for sample in link.snapshot()
            if sample.mono >= since and isinstance(sample.frame, TelemetryFrame)
        ),
        default=0,
    )
    advisory = clamped is not None
    gate.check(
        0 < peak <= CRUISE_MM_S and advisory,
        "d",
        "I-4",
        "900 mm/s clamps to 300 and raises CAP_CLAMPED",
        f"peak v_cmd {peak} mm/s over 1.5 s of a 900 mm/s command "
        f"(<= {CRUISE_MM_S}), CAP_CLAMPED "
        f"{'raised' if advisory else 'NOT raised'}",
        peak_v_cmd_mm_s=peak,
        cap_clamped=advisory,
    )
    link.stream(0, 0)


def hold_flags(link: McuLink, flags: int, settle_s: float = 1.0) -> int:
    """Stream a forward cruise at one ``V.flags`` value until the asymmetric
    slew has settled, then return the peak ``v_cmd_mm_s`` over a short window.

    Settling is the point.  A sweep that changes the flag value every 20 ms
    reads the *previous* value's ramp-down -- 40 mm/s per step at the 2000
    mm/s2 abort rate -- so it cannot say what any single value produces.
    """
    end = time.monotonic() + settle_s
    while time.monotonic() < end:
        link.velocity(CRUISE_MM_S, 0, 300, flags)
        time.sleep(0.02)
    peak = 0
    end = time.monotonic() + 0.4
    while time.monotonic() < end:
        link.velocity(CRUISE_MM_S, 0, 300, flags)
        time.sleep(0.02)
        telemetry = link.telemetry()
        if telemetry is not None:
            peak = max(peak, telemetry.v_cmd_mm_s)
    return peak


def sub_d_ttc(gate: GateRun, args: Any, config: Any, full_sweep: bool) -> None:
    """I-4's TTC criterion, on a simulator that actually has an obstacle.

    ARCHITECTURE 8 states the criterion as "sweep all 256 flag values against
    the ToF fixture and assert forward speed never exceeds (d-250)/1.0 s capped
    at 150 mm/s".  Run with a clear path -- which is what ``--sim-flags``
    defaults to -- ``rover_control_step`` enters neither the obstacle branch nor
    the slow-zone branch, b0 ``require_slow_zone_stop`` is a no-op for all 256
    values, and the sweep proves only the plain 300 mm/s clamp that G2-d already
    proved with one frame.  A regression making b0 *widen* rather than narrow
    would pass such a sweep unchanged, and this is the invariant that exists to
    catch it.  So the fixture is a fresh simulator with an obstacle inside the
    slow zone, and the bound asserted is the TTC law, not the cruise cap.
    """
    obstacle_mm = TTC_OBSTACLE_MM
    link, sim, missing = with_sim(args, config, f"obstacle={obstacle_mm}")
    if link is None:
        gate.skip(
            "d2",
            "I-4",
            "the V.flags sweep against an obstacle, never above the TTC law",
            missing,
        )
        return
    try:
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)
        law_cap = min(obstacle_mm - config.safety.tof_stop_mm, SLOW_ZONE_V_CAP_MM_S)

        # The three-value sweep always runs: b0 clear, b0 set, and b1 (the
        # reserved SERVO_EN request, which must not touch forward speed).
        settled = {flags: hold_flags(link, flags) for flags in (0, 1, 2)}
        b0_clear_ok = settled[0] <= law_cap and settled[2] <= law_cap
        b0_set_ok = settled[1] == 0
        gate.check(
            b0_clear_ok and b0_set_ok and settled[0] > 0,
            "d2",
            "I-4",
            "the V.flags sweep against an obstacle, never above the TTC law",
            f"obstacle {obstacle_mm} mm: flags 0 -> {settled[0]} mm/s, "
            f"flags 2 -> {settled[2]} mm/s (both <= {law_cap} = "
            f"min({obstacle_mm} - {config.safety.tof_stop_mm}, "
            f"{SLOW_ZONE_V_CAP_MM_S})), flags 1 (b0 require_slow_zone_stop) -> "
            f"{settled[1]} mm/s (must be 0: b0 narrows, never widens)",
            obstacle_mm=obstacle_mm,
            law_cap_mm_s=law_cap,
            settled_v_cmd_mm_s=settled,
        )

        if not full_sweep:
            gate.skip(
                "d4",
                "I-4",
                "all 256 V.flags values against the same obstacle",
                "--full (about 26 s of control cycles)",
            )
            return
        # The full sweep is interleaved at 20 ms, so a b0-set value is read
        # part-way down its abort ramp from the previous value.  That makes
        # `== 0` unassertable per value -- but it makes the *upper* bound
        # stricter, not looser, so the law cap is exactly the right assertion
        # for all 256, and d2 above is where b0's floor is scored.
        worst = 0
        worst_flags = 0
        for flags in range(256):
            link.velocity(CRUISE_MM_S, 0, 300, flags)
            time.sleep(0.02)
            telemetry = link.telemetry()
            if telemetry is not None and telemetry.v_cmd_mm_s > worst:
                worst = telemetry.v_cmd_mm_s
                worst_flags = flags
        gate.check(
            worst <= law_cap,
            "d4",
            "I-4",
            "all 256 V.flags values against the same obstacle",
            f"worst v_cmd over 256 flag values: {worst} mm/s at flags "
            f"0x{worst_flags:02X} (<= {law_cap}; every bit is narrowing-only)",
            worst_v_cmd_mm_s=worst,
            worst_flags=worst_flags,
            law_cap_mm_s=law_cap,
        )
        link.stop(0)
    finally:
        link.close()
        if sim is not None:
            sim.stop()


_C_COMMENT = re.compile(r"/\*.*?\*/|//[^\n]*", re.S)


def strip_c_comments(text: str) -> str:
    """Blank out C comments, keeping line numbers.

    A grep gate that fires on the sentence "there is no config_set on the link"
    is a grep gate nobody keeps.
    """
    return _C_COMMENT.sub(lambda m: re.sub(r"[^\n]", " ", m.group(0)), text)


def sub_d_grep(gate: GateRun) -> None:
    """I-4's static half: no path on the link writes a firmware constant."""
    firmware = REPO / "firmware"
    if not firmware.exists():
        gate.skip("d3", "I-4", "no config_set path exists in the firmware", "firmware/")
        return
    pattern = re.compile(r"config_set|set_config|write_cfg|config_get", re.I)
    hits = [
        f"{path.relative_to(REPO)}:{number}"
        for path in sorted(firmware.rglob("*.[ch]"))
        for number, line in enumerate(
            strip_c_comments(
                path.read_text(encoding="utf-8", errors="replace")
            ).splitlines(),
            1,
        )
        if pattern.search(line)
    ]
    gate.check(
        not hits,
        "d3",
        "I-4",
        "no config_set path exists in the firmware",
        "; ".join(hits[:3]) if hits else "5.1 deletes config_set and cuts config_get",
        hits=hits,
    )


def sub_h_wdt(gate: GateRun, args: Any, config: Any) -> None:
    """I-20: a control-loop hang reboots into DISARMED within 1 s + reset time."""
    if args.hardware:
        gate.skip(
            "h",
            "I-20",
            "a hung control task reboots into DISARMED with WDT_REBOOT",
            "a test build with an infinite loop in the control task",
        )
        return
    link, sim, missing = with_sim(args, config, f"hang={args.hang_ms}")
    if link is None:
        gate.skip("h", "I-20", "a hung control task reboots into DISARMED", missing)
        return
    try:
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)
        link.stream(CRUISE_MM_S, 0)
        moving = link.wait(TelemetryFrame, lambda f: abs(f.v_meas_mm_s) > 100, 4.0)
        if moving is None:
            gate.skip(
                "h",
                "I-20",
                "a hung control task reboots into DISARMED",
                "a moving plant",
            )
            return
        start: TelemetryFrame = moving.frame  # type: ignore[assignment]
        hang_at = time.monotonic()
        # ARCHITECTURE 4.1's restart paragraph says the MCU boots DISARMED;
        # 5.1's fault-class table puts WDT_REBOOT in the latched class, which
        # enters FAULT.  The firmware implements the class table, so all three
        # states are accepted here -- FAULT is strictly the stricter of the two
        # (it refuses V *and* needs a C) and I-20's substance is motors off,
        # not a particular enum value.  Recorded in docs/deviations.md.
        rebooted = link.wait(
            TelemetryFrame,
            lambda f: f.state in (McuState.BOOT, McuState.DISARMED, McuState.FAULT),
            6.0,
            since=hang_at + 0.2,
        )
        link.stop_stream()
        banners = link.boots(hang_at)
        travel_mm = (
            abs(wheel_travel_m(config, start, rebooted.frame)) * 1000.0  # type: ignore[arg-type]
            if rebooted is not None
            else float("inf")
        )
        wdt = any(banner.reset_reason for banner in banners)
        stationary = (
            rebooted is not None and rebooted.frame.v_cmd_mm_s == 0  # type: ignore[attr-defined]
        )
        gate.check(
            rebooted is not None and travel_mm <= 400.0 and wdt and stationary,
            "h",
            "I-20",
            "a hung control task reboots into DISARMED with WDT_REBOOT",
            f"travel after the hang {travel_mm:.0f} mm (<= 400), "
            f"{len(banners)} B banners with reset_reason set: {wdt}, "
            f"v_cmd after the reboot "
            f"{rebooted.frame.v_cmd_mm_s if rebooted else '?'} mm/s, state "
            f"{McuState(rebooted.frame.state).name if rebooted else '?'}",  # type: ignore[attr-defined]
            travel_after_hang_mm=round(travel_mm, 1)
            if travel_mm != float("inf")
            else None,
            banners=len(banners),
            wdt_event=any(
                event.event == EventCode.WDT_REBOOT for event in link.events(hang_at)
            ),
        )
    finally:
        link.close()
        if sim is not None:
            sim.stop()


def sub_i_reset(gate: GateRun) -> None:
    """I-24's software half: ``app_main`` drives every driver pin low first."""
    main = REPO / "firmware" / "main"
    if not main.exists():
        gate.skip(
            "i",
            "I-24",
            "app_main drives the driver pins low before peripheral init",
            "firmware/main/",
        )
        return
    text = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in sorted(main.rglob("*.[ch]"))
    )
    required = ("4", "5", "6", "7", "12", "21", "39")
    missing = [pin for pin in required if not re.search(rf"\b(GPIO_?)?{pin}\b", text)]
    drives_low = bool(re.search(r"gpio_set_level\s*\([^,]+,\s*0\s*\)", text))
    gate.check(
        not missing and drives_low,
        "i",
        "I-24",
        "app_main drives the driver pins low before peripheral init",
        f"pins named: {'all' if not missing else 'missing ' + ','.join(missing)}; "
        f"a gpio_set_level(..., 0) call is {'present' if drives_low else 'ABSENT'}",
        missing_pins=missing,
    )
    gate.skip(
        "i2",
        "I-24",
        "zero motor current through reset, power-cycle and the download bootloader",
        "hardware: a meter or an LED across the driver output",
    )


HARDWARE_ONLY = (
    (
        "e",
        "I-6",
        "e-stop breaks the coil and firmware cannot close it",
        "hardware: the e-stop wiring, and a scope on the MDD3A input",
    ),
    (
        "f",
        "I-7",
        "the battery ladder warns, refuses and disables on V_oc",
        "hardware: a bench supply swept 11.5 -> 9.4 V, with and without a stall",
    ),
    (
        "g",
        "I-10",
        "the per-wheel slip/stall detector trips within 250 ms",
        "hardware: one wheel held while the other runs free",
    ),
    (
        "halt",
        "A21",
        "obstacle-to-halt distance at 0.30 m/s <= 150 mm",
        "hardware: a real obstacle and a tape measure -- the fallback triggers on "
        "this distance, and half its error budget is detect latency",
    ),
    (
        "rpack",
        "A25",
        "R_pack, k_e and R_motor measured by stepping a known current",
        "hardware: the INA226 and a known load step",
    ),
    (
        "bumper",
        "I-5",
        "either bumper alone, and a cut harness, all raise BUMPER",
        "hardware: the series-NC pair and a wire to cut",
    ),
    (
        "rail",
        "I-6",
        "the Pi rail stays up with the MCU held in reset",
        "hardware: the D24V50F5 and a reset button",
    ),
    (
        "trapezoid",
        "T2",
        "v_cmd tracks robotd's trapezoid within one control period",
        "a running robotd, which owns the profile (A7)",
    ),
)


def main() -> int:
    parser = base_parser(GATE, __doc__ or "")
    parser.add_argument("--port", default=None, help="pty or /dev/rover-mcu override")
    parser.add_argument(
        "--sim-cmd", default=None, help="override 'python -m rover_devtools.mcu_sim'"
    )
    parser.add_argument(
        "--sim-flags",
        default="",
        help="one string of mcu-sim flags, split on spaces (for example "
        "'obstacle=231' or '--obstacle-mm 231')",
    )
    parser.add_argument("--hang-ms", type=int, default=1500, help="mcu-sim hang=<ms>")
    parser.add_argument(
        "--surface",
        default="carpet",
        help="what the wheels are on, recorded with the braking measurement",
    )
    parser.add_argument(
        "--full", action="store_true", help="include I-4's 256-value flag sweep"
    )
    args = parser.parse_args()

    config, source = load_robot_config()
    gate = GateRun(
        GATE,
        metrics=metrics_path_for(GATE, args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="hardware" if args.hardware else "mcu-sim",
    )
    print(f"config: {source}")

    # The two cases that need no controller run first and always.
    if gate.selected("b-codec"):
        sub_b_codec(gate)
    if gate.selected("d3"):
        sub_d_grep(gate)
    if gate.selected("i"):
        sub_i_reset(gate)

    link, sim, missing = open_link(args, config)
    if link is None:
        for sub, inv, name in (
            ("brake", "OI-1", "braking deceleration from the encoders"),
            ("a", "I-1", "TTL expiry zeroes the command and brakes"),
            ("b", "I-2", "corruption raises rx_drop and does not renew the TTL"),
            ("c", "I-3", "boots DISARMED and refuses V before H+A"),
            ("d", "I-4", "900 mm/s clamps to 300 and raises CAP_CLAMPED"),
        ):
            if gate.selected(sub):
                gate.skip(sub, inv, name, missing)
    else:
        try:
            if gate.selected("c"):
                with gate.guard("c", "I-3", "boots DISARMED and refuses V"):
                    sub_c_boot(gate, link)
            link.bring_up(config.serial.reseed_wait_ms / 1000.0)

            # ARCHITECTURE 13: braking first, because I-1's criteria derive from it.
            decel = None
            if gate.selected("brake"):
                surface = args.surface if args.hardware else "sim plant"
                with gate.guard("brake", "OI-1", "braking deceleration"):
                    decel = measure_braking(
                        gate, link, config, CRUISE_MM_S, surface, gated=True
                    )
                    measure_braking(gate, link, config, 150, surface, gated=False)
            t_brake = (CRUISE_MM_S / 1000.0) / decel if decel else 0.5
            if gate.selected("a"):
                with gate.guard("a", "I-1", "TTL expiry zeroes the command"):
                    sub_a_ttl(gate, link, config, t_brake)
            if gate.selected("b"):
                with gate.guard("b", "I-2", "corruption raises rx_drop"):
                    sub_b_link(gate, link)
            if gate.selected("d"):
                with gate.guard("d", "I-4", "900 mm/s clamps to 300"):
                    sub_d_caps(gate, link)
        finally:
            link.close()
            if sim is not None:
                sim.stop()

    # d2 runs on its own simulator, carrying the obstacle the TTC law needs.
    if gate.selected("d2") or gate.selected("d4"):
        with gate.guard("d2", "I-4", "the V.flags sweep against an obstacle"):
            sub_d_ttc(gate, args, config, args.full)
    if gate.selected("c2"):
        sub_c_restart(gate, args, config)
    if gate.selected("h"):
        sub_h_wdt(gate, args, config)

    for sub, inv, name, reason in HARDWARE_ONLY:
        if not gate.selected(sub):
            continue
        if args.hardware and sub in {"e", "f", "g", "halt", "rpack", "bumper", "rail"}:
            gate.record(
                sub,
                inv,
                name,
                Status.SKIP,
                "requires a human at the bench: this case is a physical procedure, "
                f"not an automatable one -- {reason}",
            )
        else:
            gate.skip(sub, inv, name, reason)

    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
