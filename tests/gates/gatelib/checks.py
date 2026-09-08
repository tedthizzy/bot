"""Run existing boundary tests once per criterion and retain their evidence.

Required software missing, skipped, empty, timed out, or failed is a gate FAIL.
Hardware procedures are recorded separately by each gate, never inferred here.
"""

import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from gatelib.env import REPO


def child_env(repo=REPO):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(repo / "packages"), str(repo / "hosts/pi"), env.get("PYTHONPATH", "")]
    )
    return env


def command_case(gate, sub, invariant, name, command, *, timeout=120):
    if not gate.selected(sub):
        return
    with gate.guard(sub, invariant, name):
        result = subprocess.run(
            command,
            cwd=REPO,
            env=child_env(),
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = (result.stdout + result.stderr).strip()
        gate.check(
            result.returncode == 0,
            sub,
            invariant,
            name,
            output[-3000:],
            command=command,
            returncode=result.returncode,
        )


def pytest_case(gate, sub, invariant, name, *selectors, timeout=180):
    if not gate.selected(sub):
        return
    with (
        gate.guard(sub, invariant, name),
        tempfile.TemporaryDirectory(prefix="rover-gate-") as directory,
    ):
        report = Path(directory) / "result.xml"
        env = child_env()
        env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
        command = [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "--tb=short",
            "-p",
            "pytest_asyncio.plugin",
            "-p",
            "pytest_timeout",
            "-p",
            "no:cacheprovider",
            "-o",
            "faulthandler_timeout=30",
            f"--junitxml={report}",
            *selectors,
        ]
        result = subprocess.run(
            command, cwd=REPO, env=env, capture_output=True, text=True, timeout=timeout
        )
        cases = ET.parse(report).findall(".//testcase") if report.exists() else []
        skipped = sum(case.find("skipped") is not None for case in cases)
        passed = result.returncode == 0 and bool(cases) and skipped == 0
        detail = f"{len(cases)} executed tests; {skipped} skips; software evidence only"
        if not passed:
            detail += "\n" + (result.stdout + result.stderr)[-5000:]
        gate.check(
            passed,
            sub,
            invariant,
            name,
            detail,
            selectors=list(selectors),
            tests=len(cases),
            skipped=skipped,
            test_cases=[case.attrib.get("name", "") for case in cases],
            returncode=result.returncode,
        )


def validate_subsets(gate, known):
    for sub in sorted((gate.only or set()) - set(known)):
        gate.fail(sub, "CLI", "unknown sub-gate", "choose one of: " + ", ".join(known))
