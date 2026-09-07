"""I-15: at most 1.5 m of path and 12 s of motion per user instruction."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.config import RobotConfig  # noqa: E402
from rover_robotd.budget import BudgetLedger, BudgetLimits  # noqa: E402

TURN = "01J9ZC7K000000000000000000"
OTHER = "01J9ZC7K000000000000000001"


def _ledger() -> BudgetLedger:
    return BudgetLedger(BudgetLimits(path_m=1.5, motion_s=12.0))


def test_the_defaults_are_the_architecture_s_numbers() -> None:
    limits = RobotConfig().limits
    assert (limits.budget_path_m, limits.budget_motion_s) == (1.5, 12)


def test_a_fresh_instruction_has_the_whole_budget() -> None:
    remaining = _ledger().remaining(TURN)
    assert (remaining.path_m, remaining.motion_s) == (1.5, 12.0)


def test_ten_legal_drives_in_one_turn_exhaust_the_path_budget() -> None:
    """I-15's own test: ten consecutive legal drives inside one instruction."""
    ledger = _ledger()
    accepted = 0
    for _ in range(10):
        if not ledger.allows(TURN, 0.4, 2.5):
            break
        accepted += 1
        ledger.charge(TURN, 0.4, 2.5)
    assert accepted == 3  # 3 x 0.4 m = 1.2 m; a fourth would exceed 1.5 m
    assert ledger.remaining(TURN).path_m == pytest.approx(0.3)


def test_time_runs_out_before_path_on_a_slow_drive() -> None:
    ledger = _ledger()
    ledger.charge(TURN, 0.2, 11.0)
    assert ledger.allows(TURN, 0.2, 0.5)
    assert not ledger.allows(TURN, 0.2, 1.5)


def test_a_turn_charges_time_but_no_path() -> None:
    ledger = _ledger()
    ledger.charge(TURN, 0.0, 3.0)
    remaining = ledger.remaining(TURN)
    assert remaining.path_m == 1.5
    assert remaining.motion_s == pytest.approx(9.0)


def test_the_budget_is_per_instruction_not_global() -> None:
    ledger = _ledger()
    ledger.charge(TURN, 1.5, 12.0)
    assert not ledger.allows(TURN, 0.1, 0.1)
    assert ledger.allows(OTHER, 1.5, 12.0)


def test_consumption_is_measured_not_reserved() -> None:
    """A drive preempted after 10 cm costs 10 cm, not its estimate."""
    ledger = _ledger()
    assert ledger.allows(TURN, 1.0, 8.0)
    ledger.charge(TURN, 0.10, 0.8)
    assert ledger.remaining(TURN).path_m == pytest.approx(1.4)


def test_charging_never_reports_a_negative_remainder() -> None:
    ledger = _ledger()
    ledger.charge(TURN, 9.0, 99.0)
    remaining = ledger.remaining(TURN)
    assert remaining.path_m == 0.0 and remaining.motion_s == 0.0


def test_reverse_travel_charges_its_magnitude() -> None:
    ledger = _ledger()
    ledger.charge(TURN, -0.5, 1.0)
    assert ledger.remaining(TURN).path_m == pytest.approx(1.0)


def test_old_instructions_are_evicted_but_the_live_one_is_not() -> None:
    ledger = _ledger()
    ledger.charge(TURN, 0.5, 1.0)
    for index in range(12):
        ledger.charge(f"01J9ZC7KZZZZZZZZZZZZZZ{index:04d}", 0.1, 0.1)
        ledger.charge(TURN, 0.0, 0.0)  # still the live instruction
    assert ledger.remaining(TURN).path_m == pytest.approx(1.0)
