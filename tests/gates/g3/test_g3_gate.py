"""G3 through pytest: ``make gates-pi``.

The soak is 30 minutes on the Pi and a much shorter smoke elsewhere, so the
duration comes from ``$ROVER_GATE_SOAK_S``.  The default is 60 s: long enough to
exercise the loop, short enough that nobody disables the target.
"""

from __future__ import annotations

import os

import pytest
from gatelib.pytest_gate import run_gate


@pytest.mark.gate
@pytest.mark.timeout(2400)
def test_g3_desk_soak() -> None:
    seconds = os.environ.get("ROVER_GATE_SOAK_S", "60")
    run_gate("g3/run_g3.py", "--duration", seconds, timeout=float(seconds) + 600)
