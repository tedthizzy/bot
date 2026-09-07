"""G1 through pytest: ``make gate-g1``, and ``BOX=real`` on deploy day."""

from __future__ import annotations

import os

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(2400)
def test_g1_model_and_validator() -> None:
    """510 requests against the endpoint ``$BOX`` names (default fakebox)."""
    run_gate("g1/run_g1.py", "--box", os.environ.get("BOX", "fake"), timeout=2400)
