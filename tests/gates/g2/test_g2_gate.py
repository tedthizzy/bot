"""G2 through pytest: the sim half, which is what ``make gates`` runs."""

from __future__ import annotations

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(600)
def test_g2_controller_bench() -> None:
    """I-1 to I-4, I-20 and I-24's software half against mcu-sim."""
    run_gate("g2/run_g2.py", timeout=600)


@pytest.mark.gate
@pytest.mark.timeout(900)
def test_g2_flag_sweep() -> None:
    """I-4's 256-value sweep against an obstacle inside the slow zone.

    ``make gates-full`` territory, ~26 s of cycles.  d2's three-value sweep
    runs in the default subset above; only d4's 256 values are behind --full.
    """
    run_gate("g2/run_g2.py", "--only", "d,d2,d4", "--full", timeout=900)
