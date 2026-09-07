"""G4 through pytest: the fuzz, the races and the fault injection."""

from __future__ import annotations

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(900)
def test_g4_fault_injection() -> None:
    """I-5, I-8, I-9, I-11 to I-17, I-22 and I-23, as far as this host reaches."""
    run_gate("g4/run_g4.py", timeout=900)
