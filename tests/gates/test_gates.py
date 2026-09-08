"""One pytest entry point for the independently runnable acceptance gates.

``make gates`` is ``pytest tests/gates -m gate``, so every gate needs a pytest
door as well as a command line.  The mapping from the scripts' three exit codes
is the whole content of :func:`run_gate`, and it is deliberate:

    0  every case that ran passed          -> the test passes
    1  at least one case failed            -> the test fails
    2  nothing ran, every case skipped     -> the test **skips**

A gate that could not reach its subject must not report a green test.  Mapping
exit 2 to a skip is what keeps ``make gates`` honest on a laptop with no rover
attached while still failing the moment a real case goes red.
"""

from __future__ import annotations

import os
import subprocess
import sys

import pytest
from gatelib.env import REPO

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


def run_gate(script: str, *args: str, timeout: float = 600.0) -> None:
    """Run one gate script and turn its exit code into a pytest outcome."""
    path = REPO / "tests" / "gates" / script
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [
            str(REPO / "packages"),
            str(REPO / "hosts" / "pi"),
            *(p for p in env.get("PYTHONPATH", "").split(os.pathsep) if p),
        ]
    )
    try:
        result = subprocess.run(
            [sys.executable, str(path), *args],
            cwd=str(REPO),
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        pytest.fail(f"{script} did not finish within {timeout:.0f}s")
    sys.stdout.write(result.stdout)
    sys.stderr.write(result.stderr)
    if result.returncode == 2:
        pytest.skip(f"{script}: no case could run -- see the skip list above")
    assert result.returncode == 0, f"{script} exited {result.returncode}"


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
