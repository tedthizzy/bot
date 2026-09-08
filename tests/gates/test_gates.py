"""One pytest entry point for the independently runnable acceptance gates."""

import os

import pytest
from gatelib.pytest_gate import run_gate

CASES = {
    "g1": (2400, ["--box", os.environ.get("BOX", "fake")]),
    "g2": (600, []),
    "g2_full": (900, ["--only", "d,d2,d4", "--full"]),
    "g3": (
        float(os.environ.get("ROVER_GATE_SOAK_S", "60")) + 600,
        ["--duration", os.environ.get("ROVER_GATE_SOAK_S", "60")],
    ),
    "g4": (900, []),
    "g5": (300, []),
    "g6": (300, []),
}


@pytest.mark.gate
@pytest.mark.parametrize(
    "case,timeout,args",
    [
        pytest.param(name, timeout, args, id=name, marks=pytest.mark.timeout(timeout))
        for name, (timeout, args) in CASES.items()
    ],
)
def test_gate(case: str, timeout: float, args: list[str]) -> None:
    gate = case.split("_", 1)[0]
    run_gate(f"{gate}/run_{gate}.py", *args, timeout=timeout)
