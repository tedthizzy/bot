"""One result model for all six gates.

Every gate prints one line per case, writes one JSONL record per case plus a
summary record, and exits non-zero on failure.  The third status matters as
much as the first two: a case whose subject is not present -- no hardware, no
``mcu-sim``, no running robotd -- reports ``SKIP`` with the reason named, never
``PASS``.  A gate that cannot reach its subject must not be able to look green.

Exit codes
    0   every case that ran passed
    1   at least one case failed
    2   nothing ran (every case skipped), so the run carries no result

``--strict`` promotes every skip to a failure, which is what the Pi run wants:
there, a skip means a component that should have been present was not.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import Path
from typing import Any

__all__ = ["Case", "GateRun", "Status", "base_parser", "metrics_path_for"]


class Status(StrEnum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass(slots=True)
class Case:
    """One scored check inside a sub-gate."""

    gate: str
    sub: str
    invariant: str
    name: str
    status: Status
    detail: str = ""
    metrics: dict[str, Any] = field(default_factory=dict)

    @property
    def label(self) -> str:
        return f"{self.gate}-{self.sub}" if self.sub else self.gate


def metrics_path_for(gate: str, override: str | None, repo: Path) -> Path:
    """Where a run writes its JSONL: ``logs/gates/<gate>-<utc>.jsonl``.

    The Makefile's gate targets point the tracker at that directory, so evidence
    is linked rather than asserted.
    """
    if override:
        return Path(override)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return repo / "logs" / "gates" / f"{gate.lower()}-{stamp}.jsonl"


def base_parser(gate: str, description: str) -> argparse.ArgumentParser:
    """The flags every gate shares."""
    parser = argparse.ArgumentParser(prog=f"run_{gate.lower()}", description=description)
    parser.add_argument(
        "--metrics", default=None, help="JSONL metrics path (default logs/gates/...)"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="treat every skip as a failure (use on hardware, where a skip is a bug)",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="the rover hardware is attached; run the cases that need it",
    )
    parser.add_argument(
        "--only",
        default=None,
        help="run only these sub-gates, comma separated (for example a,c,g)",
    )
    return parser


class GateRun:
    """Collects cases, prints them as they happen, writes JSONL, exits."""

    def __init__(
        self,
        gate: str,
        *,
        metrics: Path,
        strict: bool = False,
        hardware: bool = False,
        only: str | None = None,
        mode: str = "",
    ) -> None:
        self.gate = gate
        self.strict = strict
        self.hardware = hardware
        self.mode = mode
        self.only = (
            frozenset(part.strip() for part in only.split(",") if part.strip())
            if only
            else None
        )
        self.cases: list[Case] = []
        self.started = time.monotonic()
        self.path = metrics
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8", newline="\n")
        self._header()

    # -- output ---------------------------------------------------------

    def _header(self) -> None:
        stamp = datetime.now(UTC).isoformat(timespec="seconds")
        banner = f"{self.gate}  {stamp}"
        if self.mode:
            banner += f"  mode={self.mode}"
        print(banner)
        print("-" * len(banner))

    def _emit(self, case: Case) -> None:
        print(
            f"{case.label:<8} {case.invariant:<5} {case.status:<4}  {case.name}"
            + (f" -- {case.detail}" if case.detail else "")
        )
        import json

        record = {
            "gate": case.gate,
            "sub": case.sub,
            "invariant": case.invariant,
            "name": case.name,
            "status": str(case.status),
            "detail": case.detail,
            "mode": self.mode,
            "t_utc": datetime.now(UTC).isoformat(timespec="milliseconds"),
            **case.metrics,
        }
        self._file.write(
            json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            + "\n"
        )
        self._file.flush()

    # -- recording ------------------------------------------------------

    def selected(self, sub: str) -> bool:
        """False when ``--only`` excludes this sub-gate."""
        return self.only is None or sub in self.only

    def record(
        self,
        sub: str,
        invariant: str,
        name: str,
        status: Status,
        detail: str = "",
        **metrics: Any,
    ) -> Case:
        case = Case(self.gate, sub, invariant, name, status, detail, dict(metrics))
        self.cases.append(case)
        self._emit(case)
        return case

    def ok(self, sub: str, inv: str, name: str, detail: str = "", **m: Any) -> Case:
        return self.record(sub, inv, name, Status.PASS, detail, **m)

    def fail(self, sub: str, inv: str, name: str, detail: str = "", **m: Any) -> Case:
        return self.record(sub, inv, name, Status.FAIL, detail, **m)

    def skip(self, sub: str, inv: str, name: str, reason: str, **m: Any) -> Case:
        """A case whose subject is absent.  ``reason`` names what is missing."""
        return self.record(sub, inv, name, Status.SKIP, f"requires {reason}", **m)

    def check(
        self, cond: bool, sub: str, inv: str, name: str, detail: str = "", **m: Any
    ) -> bool:
        self.record(sub, inv, name, Status.PASS if cond else Status.FAIL, detail, **m)
        return cond

    @contextmanager
    def guard(self, sub: str, inv: str, name: str) -> Iterator[None]:
        """Run a case body; an unexpected exception is a failure, not a crash."""
        try:
            yield
        except Exception as exc:  # noqa: BLE001 - a gate reports, it does not crash
            self.fail(sub, inv, name, f"{type(exc).__name__}: {exc}")

    # -- summary --------------------------------------------------------

    def summary(self) -> int:
        counts = {status: 0 for status in Status}
        for case in self.cases:
            counts[case.status] += 1
        elapsed = time.monotonic() - self.started
        skipped = [c for c in self.cases if c.status is Status.SKIP]
        failed = [c for c in self.cases if c.status is Status.FAIL]

        print()
        if failed:
            print("failed:")
            for case in failed:
                print(f"  {case.label} {case.invariant} {case.name}: {case.detail}")
        if skipped:
            print("skipped:")
            for case in skipped:
                print(f"  {case.label} {case.invariant} {case.name}: {case.detail}")

        verdict = "PASS"
        code = 0
        if failed or (self.strict and skipped):
            verdict, code = "FAIL", 1
        elif counts[Status.PASS] == 0:
            verdict, code = "NO RESULT", 2
        elif skipped:
            verdict = "PASS (INCOMPLETE)"
        if self.mode:
            verdict = f"{verdict} [{self.mode}]"

        line = (
            f"{self.gate}: {verdict}  "
            f"{counts[Status.PASS]} passed, {counts[Status.FAIL]} failed, "
            f"{counts[Status.SKIP]} skipped in {elapsed:.1f}s"
        )
        print(line)
        print(f"metrics: {self.path}")

        import json

        self._file.write(
            json.dumps(
                {
                    "gate": self.gate,
                    "record": "summary",
                    "mode": self.mode,
                    "verdict": verdict,
                    "passed": counts[Status.PASS],
                    "failed": counts[Status.FAIL],
                    "skipped": counts[Status.SKIP],
                    "elapsed_s": round(elapsed, 3),
                    "exit_code": code,
                    "t_utc": datetime.now(UTC).isoformat(timespec="milliseconds"),
                },
                separators=(",", ":"),
            )
            + "\n"
        )
        self._file.close()
        return code

    def exit(self) -> None:
        sys.exit(self.summary())
