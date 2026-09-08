"""I-15: at most ``budget_motion_s`` seconds of motion per user instruction."""

from __future__ import annotations

import pytest
from rover_contracts.config import RobotConfig
from rover_robotd.budget import BudgetLedger

TURN = "01J9ZC7K000000000000000000"
OTHER = "01J9ZC7K000000000000000001"


def test_the_default_is_the_architecture_s_number() -> None:
    assert RobotConfig().limits.budget_motion_s == 12


def test_a_fresh_instruction_has_the_whole_budget() -> None:
    ledger = BudgetLedger(12.0)
    assert ledger.remaining(TURN) == 12.0
    assert ledger.spent(TURN) == 0.0
    assert not ledger.exhausted(TURN)


def test_charging_one_period_at_a_time_adds_up() -> None:
    """The control loop charges every 20 Hz period with a non-zero stream."""
    ledger = BudgetLedger(12.0)
    for _ in range(50):
        ledger.charge(TURN, 0.05)
    assert ledger.spent(TURN) == pytest.approx(2.5)
    assert ledger.remaining(TURN) == pytest.approx(9.5)


def test_six_two_second_drives_spend_the_budget_and_a_seventh_finds_none() -> None:
    ledger = BudgetLedger(12.0)
    for _ in range(6):
        assert not ledger.exhausted(TURN)
        ledger.charge(TURN, 2.0)
    assert ledger.exhausted(TURN)
    assert ledger.remaining(TURN) == 0.0


def test_the_budget_is_per_instruction_not_global() -> None:
    ledger = BudgetLedger(12.0)
    ledger.charge(TURN, 12.0)
    assert ledger.exhausted(TURN)
    assert ledger.remaining(OTHER) == 12.0


def test_consumption_is_measured_not_reserved() -> None:
    """A drive preempted after 200 ms costs 200 ms, not its duration."""
    ledger = BudgetLedger(12.0)
    ledger.charge(TURN, 0.2)
    assert ledger.remaining(TURN) == pytest.approx(11.8)


def test_charging_never_reports_a_negative_remainder() -> None:
    ledger = BudgetLedger(12.0)
    assert ledger.charge(TURN, 99.0) == 0.0
    assert ledger.remaining(TURN) == 0.0


def test_a_negative_charge_is_ignored() -> None:
    ledger = BudgetLedger(12.0)
    ledger.charge(TURN, -5.0)
    assert ledger.spent(TURN) == 0.0


def test_old_instructions_are_evicted_but_the_live_one_is_not() -> None:
    ledger = BudgetLedger(12.0)
    ledger.charge(TURN, 5.0)
    for index in range(12):
        ledger.charge(f"01J9ZC7KZZZZZZZZZZZZZZ{index:04d}", 0.1)
        ledger.charge(TURN, 0.0)  # still the live instruction
    assert ledger.remaining(TURN) == pytest.approx(7.0)


def test_a_non_positive_budget_is_refused() -> None:
    with pytest.raises(ValueError, match="positive"):
        BudgetLedger(0.0)
