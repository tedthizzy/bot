"""Motion profiles, the setpoint cell of I-14, and arbitration."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.messages import Source  # noqa: E402
from rover_contracts.skills import goal_deadline_s  # noqa: E402
from rover_robotd.arbiter import (  # noqa: E402
    SOURCE_PRIORITY,
    ActiveCommand,
    Arbiter,
    Arbitration,
    CommandKind,
    arbitrate,
)
from rover_robotd.profiles import (  # noqa: E402
    PROGRESS_WINDOW_MS,
    SETPOINT_VALID_MS,
    ProgressMonitor,
    SetpointCell,
    TrapezoidProfile,
)

MS = 1_000_000


# ---------------------------------------------------------------------------
# The setpoint cell (I-14)
# ---------------------------------------------------------------------------


def test_a_fresh_cell_reads_zero() -> None:
    assert SetpointCell().read(0) == (0.0, 0.0)


def test_a_stamped_cell_reads_back_inside_its_window() -> None:
    cell = SetpointCell()
    cell.stamp(0.25, 0.21, 1_000 * MS)
    assert cell.read(1_000 * MS) == (0.25, 0.21)
    assert cell.read(1_000 * MS + (SETPOINT_VALID_MS - 1) * MS) == (0.25, 0.21)


def test_a_frozen_goal_owner_stops_the_wheels() -> None:
    """The writer keeps reading; nobody stamps.  Every read past the window is
    zero, so a wedged control loop brakes the robot without being detected."""
    cell = SetpointCell()
    frozen_at = 1_000 * MS
    cell.stamp(0.30, 0.0, frozen_at)
    assert cell.read(frozen_at + SETPOINT_VALID_MS * MS) == (0.30, 0.0)
    for elapsed_ms in (SETPOINT_VALID_MS + 1, 150, 300, 5_000):
        assert cell.read(frozen_at + elapsed_ms * MS) == (0.0, 0.0)


def test_zero_expires_the_stamp_immediately() -> None:
    cell = SetpointCell()
    cell.stamp(0.30, 0.0, 1_000 * MS)
    cell.zero()
    assert cell.read(1_000 * MS) == (0.0, 0.0)


def test_the_cell_exposes_no_way_to_read_a_stale_velocity() -> None:
    """Structural, not a rule: there is no public attribute holding v or w, so
    a writer cannot repeat a stale setpoint even by mistake."""
    cell = SetpointCell()
    cell.stamp(0.30, 1.0, 0)
    public = [name for name in dir(cell) if not name.startswith("_")]
    assert public == ["read", "stamp", "valid_until_mono_ns", "zero"]
    assert not hasattr(cell, "__dict__")  # __slots__: nothing can be attached


# ---------------------------------------------------------------------------
# The trapezoid
# ---------------------------------------------------------------------------


def _drive_profile(target: float = 0.40, cruise: float = 0.15) -> TrapezoidProfile:
    return TrapezoidProfile(target=target, cruise=cruise, accel=0.5, tolerance=0.01)


def _run(profile: TrapezoidProfile, dt: float = 0.05, limit: int = 2000):
    """Integrate the profile against a perfect plant; return the trace."""
    progress = 0.0
    trace = []
    for _ in range(limit):
        command = profile.step(progress, dt)
        trace.append((progress, command))
        if profile.done(progress):
            break
        progress += command * dt
    return trace


def test_a_drive_reaches_its_target_and_stops() -> None:
    profile = _drive_profile()
    trace = _run(profile)
    assert trace[-1][1] == 0.0
    assert profile.done(trace[-1][0])
    assert trace[-1][0] == pytest.approx(0.40, abs=0.01)


def test_the_leading_edge_never_exceeds_the_acceleration() -> None:
    profile = _drive_profile()
    previous = 0.0
    for _, command in _run(profile):
        assert abs(command) - abs(previous) <= 0.5 * 0.05 + 1e-12
        previous = command


def test_the_cruise_speed_is_never_exceeded() -> None:
    profile = _drive_profile(target=1.0, cruise=0.15)
    assert max(abs(command) for _, command in _run(profile)) <= 0.15 + 1e-12


def test_a_reverse_drive_commands_negative_velocity() -> None:
    profile = _drive_profile(target=-0.30)
    trace = _run(profile)
    assert all(command <= 0.0 for _, command in trace)
    assert trace[-1][0] == pytest.approx(-0.30, abs=0.01)


def test_the_braking_bound_shapes_the_trailing_ramp() -> None:
    """Every command must be stoppable inside what is left at the same accel."""
    profile = _drive_profile(target=1.0)
    for progress, command in _run(profile):
        remaining = abs(1.0 - progress)
        assert abs(command) <= math.sqrt(2 * 0.5 * remaining) + 1e-9


def test_a_turn_profile_is_the_same_maths_in_radians() -> None:
    profile = TrapezoidProfile(
        target=math.radians(90.0),
        cruise=math.radians(60.0),
        accel=1.0,
        tolerance=math.radians(1.0),
    )
    trace = _run(profile, dt=0.05)
    assert trace[-1][0] == pytest.approx(math.radians(90.0), abs=math.radians(1.0))


def test_the_profile_follows_live_progress_not_a_schedule() -> None:
    """Held wheels do not advance the profile: it keeps commanding, because a
    setpoint is a function of where the robot actually is."""
    profile = _drive_profile()
    for _ in range(20):
        command = profile.step(0.0, 0.05)
    assert command == pytest.approx(0.15)
    assert not profile.done(0.0)


def test_fraction_reports_completion_for_the_state_message() -> None:
    profile = _drive_profile(target=0.40)
    assert profile.fraction(0.0) == 0.0
    assert profile.fraction(0.20) == pytest.approx(0.5)
    assert profile.fraction(0.80) == 1.0


def test_a_profile_refuses_a_non_positive_cruise_or_accel() -> None:
    with pytest.raises(ValueError, match="positive"):
        TrapezoidProfile(target=1.0, cruise=0.0, accel=0.5, tolerance=0.01)


@pytest.mark.parametrize(
    ("distance", "speed", "expected_s"),
    [(0.40, 0.15, 4.5), (1.0, 0.05, 30.5), (0.0, 0.30, 0.5)],
)
def test_goal_deadline_is_the_t2_estimate(
    distance: float, speed: float, expected_s: float
) -> None:
    assert goal_deadline_s(distance, speed) == pytest.approx(expected_s)


# ---------------------------------------------------------------------------
# T2's progress half
# ---------------------------------------------------------------------------


def test_a_healthy_drive_never_trips_the_progress_monitor() -> None:
    monitor = ProgressMonitor()
    now = 0
    for _ in range(60):
        now += 50 * MS
        assert not monitor.update(now, 0.15 * 0.05, 0.15, 0.05)


def test_a_wheel_that_stops_moving_trips_the_progress_monitor() -> None:
    monitor = ProgressMonitor()
    now = 0
    tripped = False
    for _ in range(40):
        now += 50 * MS
        tripped = monitor.update(now, 0.0, 0.15, 0.05)
        if tripped:
            break
    assert tripped
    assert now <= (PROGRESS_WINDOW_MS + 100) * MS


def test_a_slow_command_is_not_judged() -> None:
    """Below 50 mm/s the rule does not apply, so a ramp through zero is safe."""
    monitor = ProgressMonitor()
    now = 0
    for _ in range(40):
        now += 50 * MS
        assert not monitor.update(now, 0.0, 0.02, 0.05)


# ---------------------------------------------------------------------------
# Arbitration
# ---------------------------------------------------------------------------


def _active(kind: CommandKind, source: Source, cmd_id: str = "A") -> ActiveCommand:
    return ActiveCommand(
        cmd_id=cmd_id,
        kind=kind,
        source=source,
        session_id="deadbeef",
        started_mono_ns=0,
        deadline_mono_ns=10**12,
        skill="drive" if kind is CommandKind.SKILL else None,
    )


def test_priority_is_web_then_teleop_then_brain() -> None:
    assert (
        SOURCE_PRIORITY[Source.WEB]
        > SOURCE_PRIORITY[Source.TELEOP]
        > SOURCE_PRIORITY[Source.BRAIN]
    )


def test_nothing_active_means_start() -> None:
    assert arbitrate(Source.BRAIN, CommandKind.SKILL, "B", None) is Arbitration.START


def test_a_twist_preempts_an_active_skill() -> None:
    active = _active(CommandKind.SKILL, Source.BRAIN)
    assert (
        arbitrate(Source.TELEOP, CommandKind.TWIST, "B", active) is Arbitration.PREEMPT
    )


def test_a_skill_preempts_an_active_twist_from_a_lower_source() -> None:
    active = _active(CommandKind.TWIST, Source.TELEOP)
    assert arbitrate(Source.WEB, CommandKind.SKILL, "B", active) is Arbitration.PREEMPT


def test_a_lower_priority_source_is_outranked() -> None:
    active = _active(CommandKind.TWIST, Source.WEB)
    assert (
        arbitrate(Source.BRAIN, CommandKind.SKILL, "B", active) is Arbitration.OUTRANKED
    )


def test_one_motion_in_flight_refuses_a_second_skill() -> None:
    active = _active(CommandKind.SKILL, Source.BRAIN)
    assert arbitrate(Source.BRAIN, CommandKind.SKILL, "B", active) is Arbitration.BUSY


def test_the_same_stream_renews_rather_than_restarting() -> None:
    active = _active(CommandKind.TWIST, Source.TELEOP)
    assert (
        arbitrate(Source.TELEOP, CommandKind.TWIST, "B", active) is Arbitration.RENEW
    )


def test_the_arbiter_hands_back_the_loser_to_report() -> None:
    arbiter = Arbiter()
    first = _active(CommandKind.SKILL, Source.BRAIN, cmd_id="one")
    assert arbiter.start(first) is None
    second = _active(CommandKind.TWIST, Source.WEB, cmd_id="two")
    assert arbiter.start(second) is first
    assert arbiter.active is second


def test_the_arbiter_remembers_the_last_motion_start_for_the_cooldown() -> None:
    arbiter = Arbiter()
    command = _active(CommandKind.SKILL, Source.BRAIN)
    command.profile = _drive_profile()
    command.started_mono_ns = 4242
    command.turn_id = "01J9ZC7K000000000000000000"
    arbiter.start(command)
    assert arbiter.last_motion_start_mono_ns == 4242
    assert arbiter.last_motion_turn_id == "01J9ZC7K000000000000000000"


def test_finish_only_retires_the_named_command() -> None:
    arbiter = Arbiter()
    command = _active(CommandKind.SKILL, Source.BRAIN, cmd_id="one")
    arbiter.start(command)
    assert arbiter.finish("other") is None
    assert arbiter.active is command
    assert arbiter.finish("one") is command
    assert arbiter.active is None
