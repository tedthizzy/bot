"""Motion profiles, the twist mixer, and arbitration."""

from __future__ import annotations

import pytest
from rover_contracts.config import RobotConfig
from rover_contracts.messages import Source
from rover_contracts.skills import goal_deadline_s
from rover_contracts.units import wrap_deg_180
from rover_robotd.arbiter import (
    SOURCE_PRIORITY,
    ActiveCommand,
    Arbiter,
    Arbitration,
    CommandKind,
    arbitrate,
)
from rover_robotd.profiles import RAMP_S, DriveForProfile, TurnToProfile, mix_twist

LIMITS = RobotConfig().limits
MS = 1_000_000


# ---------------------------------------------------------------------------
# drive_for: a power for a time, ramped
# ---------------------------------------------------------------------------


def test_a_drive_ramps_in_holds_and_ramps_out_inside_its_duration() -> None:
    profile = DriveForProfile(0.20, 1.0)
    assert RAMP_S == 0.100
    assert profile.command(0.0) == (0.0, 0.0)
    assert profile.command(0.05) == pytest.approx((0.10, 0.10))
    assert profile.command(0.10) == pytest.approx((0.20, 0.20))
    assert profile.command(0.50) == pytest.approx((0.20, 0.20))
    assert profile.command(0.90) == pytest.approx((0.20, 0.20))
    assert profile.command(0.95) == pytest.approx((0.10, 0.10))
    assert profile.command(1.0) == (0.0, 0.0)


def test_a_drive_completes_on_time_and_not_before() -> None:
    profile = DriveForProfile(0.20, 0.8)
    assert not profile.done(0.799)
    assert profile.done(0.8)
    assert profile.done(5.0)


def test_both_sides_carry_the_same_signed_power() -> None:
    forward = DriveForProfile(0.15, 1.0).command(0.5)
    reverse = DriveForProfile(-0.15, 1.0).command(0.5)
    assert forward == (0.15, 0.15)
    assert reverse == (-0.15, -0.15)


def test_the_ramp_never_exceeds_the_requested_power() -> None:
    profile = DriveForProfile(0.25, 2.0)
    samples = [profile.command(t / 100.0)[0] for t in range(0, 201)]
    assert max(samples) == pytest.approx(0.25)
    assert min(samples) >= 0.0


def test_a_drive_shorter_than_two_ramps_ramps_to_its_midpoint() -> None:
    """``duration_ms`` may be as short as 100 ms: the ramps then meet."""
    profile = DriveForProfile(0.20, 0.1)
    assert profile.ramp_s == pytest.approx(0.05)
    assert profile.command(0.05) == pytest.approx((0.20, 0.20))
    assert profile.command(0.025) == pytest.approx((0.10, 0.10))


def test_fraction_reports_elapsed_time_for_the_state_message() -> None:
    profile = DriveForProfile(0.20, 2.0)
    assert profile.fraction(0.0) == 0.0
    assert profile.fraction(1.0) == 0.5
    assert profile.fraction(9.0) == 1.0


def test_a_drive_refuses_zero_power_or_a_non_positive_duration() -> None:
    with pytest.raises(ValueError, match="power"):
        DriveForProfile(0.0, 1.0)
    with pytest.raises(ValueError, match="duration"):
        DriveForProfile(0.2, 0.0)


@pytest.mark.parametrize(("duration", "expected_s"), [(1.0, 2.0), (2.0, 3.5), (0.1, 0.65)])
def test_goal_deadline_is_the_t2_estimate(duration: float, expected_s: float) -> None:
    assert goal_deadline_s(duration) == pytest.approx(expected_s)


# ---------------------------------------------------------------------------
# turn_to: a proportional law on the fused yaw
# ---------------------------------------------------------------------------


def turn_profile(target: float, **overrides: float) -> TurnToProfile:
    settings = {
        "tolerance_deg": LIMITS.turn_tolerance_deg,
        "kp": LIMITS.turn_kp,
        "power_min": LIMITS.power_min,
        "power_max": LIMITS.power_max,
    }
    settings.update(overrides)
    return TurnToProfile(target, **settings)


def simulate(
    profile: TurnToProfile,
    heading: float,
    *,
    dps_per_power: float = 200.0,
    dt_s: float = 0.05,
    limit: int = 400,
) -> tuple[list[tuple[float, float]], float]:
    """Close the loop on a plant whose yaw rate follows the wheel difference.
    Returns the commands issued and the final heading."""
    commands: list[tuple[float, float]] = []
    for index in range(limit):
        profile.observe(heading, index * 50 * MS)
        if profile.done:
            break
        left, right = profile.command()
        commands.append((left, right))
        heading = wrap_deg_180(heading + (right - left) * dps_per_power * dt_s)
    return commands, heading


def test_a_turn_converges_on_a_simulated_heading() -> None:
    profile = turn_profile(90.0)
    commands, heading = simulate(profile, 0.0)
    assert profile.done
    assert abs(wrap_deg_180(90.0 - heading)) <= LIMITS.turn_tolerance_deg
    assert profile.error_deg is not None
    assert abs(profile.error_deg) <= LIMITS.turn_tolerance_deg
    assert profile.turned_deg == pytest.approx(heading, abs=1e-6)
    assert all(left < 0.0 < right for left, right in commands), "left turn: left back, right forward"


def test_a_positive_error_turns_left_and_a_negative_one_right() -> None:
    left_turn = turn_profile(45.0)
    left_turn.observe(0.0, 0)
    assert left_turn.command() == pytest.approx((-0.18, 0.18))
    right_turn = turn_profile(315.0)  # -45 in the (-180, 180] frame
    right_turn.observe(0.0, 0)
    assert right_turn.command() == pytest.approx((0.18, -0.18))


def test_the_command_is_kp_times_the_error_clamped_to_the_power_floor_and_cap() -> None:
    profile = turn_profile(0.0, power_min=0.08, power_max=0.30, kp=0.004)
    profile.observe(-10.0, 0)  # 10 degrees of error: kp*10 = 0.04, below the floor
    assert profile.command() == pytest.approx((-0.08, 0.08))
    profile.observe(-45.0, 1)  # 0.18, inside the band
    assert profile.command() == pytest.approx((-0.18, 0.18))
    profile.observe(-170.0, 2)  # 0.68, capped
    assert profile.command() == pytest.approx((-0.30, 0.30))


def test_inside_the_tolerance_the_command_is_zero_so_the_floor_cannot_overshoot() -> None:
    profile = turn_profile(90.0, tolerance_deg=5.0)
    profile.observe(87.0, 0)
    assert profile.command() == (0.0, 0.0)
    assert not profile.done, "one sample inside the tolerance is not yet done"


def test_done_needs_two_consecutive_samples_inside_the_tolerance() -> None:
    profile = turn_profile(90.0, tolerance_deg=5.0)
    profile.observe(88.0, 0)
    profile.observe(84.0, 1)  # back outside: the count restarts
    profile.observe(88.0, 2)
    assert not profile.done
    profile.observe(89.0, 3)
    assert profile.done


def test_a_repeated_sample_stamp_is_not_a_second_sample() -> None:
    """The control loop runs faster than a sample changes; the same stamp
    observed twice must not count as two in-tolerance samples."""
    profile = turn_profile(90.0)
    profile.observe(89.0, 7)
    profile.observe(89.0, 7)
    profile.observe(89.0, 7)
    assert not profile.done


def test_turned_deg_accumulates_through_the_wrap() -> None:
    profile = turn_profile(200.0)  # -160 in the (-180, 180] frame
    profile.observe(170.0, 0)
    assert profile.error_deg == pytest.approx(30.0)
    profile.observe(178.0, 1)
    profile.observe(-174.0, 2)
    profile.observe(-162.0, 3)
    assert profile.turned_deg == pytest.approx(28.0)


def test_before_the_first_sample_a_turn_commands_nothing() -> None:
    profile = turn_profile(90.0)
    assert profile.command() == (0.0, 0.0)
    assert profile.error_deg is None
    assert profile.fraction() == 0.0


def test_fraction_is_the_share_of_the_initial_error_removed() -> None:
    profile = turn_profile(100.0)
    profile.observe(0.0, 0)
    assert profile.fraction() == 0.0
    profile.observe(50.0, 1)
    assert profile.fraction() == pytest.approx(0.5)
    profile.observe(99.0, 2)
    assert profile.fraction() == pytest.approx(0.99)


def test_a_turn_refuses_a_bad_law() -> None:
    with pytest.raises(ValueError):
        turn_profile(0.0, kp=0.0)
    with pytest.raises(ValueError):
        turn_profile(0.0, power_min=0.5, power_max=0.3)


# ---------------------------------------------------------------------------
# twist: the mixer
# ---------------------------------------------------------------------------


def test_the_mixer_adds_the_turn_to_the_right_and_subtracts_it_from_the_left() -> None:
    assert mix_twist(0.10, 0.05, 0.30) == pytest.approx((0.05, 0.15))
    assert mix_twist(-0.10, 0.0, 0.30) == pytest.approx((-0.10, -0.10))
    assert mix_twist(0.0, 0.20, 0.30) == pytest.approx((-0.20, 0.20))


def test_each_side_is_clamped_to_twist_power() -> None:
    assert mix_twist(0.30, 0.30, 0.30) == (0.0, 0.30)
    assert mix_twist(-0.30, 0.30, 0.30) == (-0.30, 0.0)
    assert mix_twist(0.30, 0.30, 0.10) == (0.0, 0.10)


def test_a_non_finite_twist_mixes_to_zero() -> None:
    assert mix_twist(float("nan"), 0.0, 0.30) == (0.0, 0.0)
    assert mix_twist(0.1, float("inf"), 0.30) == (0.0, 0.0)


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
        skill="drive_for" if kind is CommandKind.SKILL else None,
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
    command.profile = DriveForProfile(0.2, 1.0)
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


def test_the_forward_component_is_what_the_obstacle_flags_block() -> None:
    drive = _active(CommandKind.SKILL, Source.BRAIN)
    drive.profile = DriveForProfile(-0.2, 1.0)
    assert drive.forward_power() == -0.2
    turn = _active(CommandKind.SKILL, Source.BRAIN)
    turn.profile = turn_profile(90.0)
    assert turn.forward_power() == 0.0
    stream = _active(CommandKind.TWIST, Source.TELEOP)
    stream.twist_lin = 0.1
    assert stream.forward_power() == 0.1
