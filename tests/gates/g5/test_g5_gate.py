"""G5 through pytest: the bearing calibration and the observation schema."""

from __future__ import annotations

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(300)
def test_g5_behaviours() -> None:
    run_gate("g5/run_g5.py", timeout=300)
