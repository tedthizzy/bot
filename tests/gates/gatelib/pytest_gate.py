"""Running a gate script from pytest, which is how the Makefile invokes them.

``make gates`` is ``pytest tests/gates -m gate``, so every gate needs a pytest
door as well as a command line.  The mapping from the scripts' three exit codes
is the whole content of this module, and it is deliberate:

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
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]

__all__ = ["REPO", "run_gate"]


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
