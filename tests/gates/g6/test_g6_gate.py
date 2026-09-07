"""G6 through pytest: the merge track, after G4 is green."""

from __future__ import annotations

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(300)
def test_g6_merge_track() -> None:
    run_gate("g6/run_g6.py", timeout=300)
