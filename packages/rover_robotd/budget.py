"""The per-instruction motion budget (I-15).

At most ``[limits] budget_path_m`` of path and ``budget_motion_s`` of motion per
user instruction, however many valid skills the model emits for it.  The unit of
accounting is the ``turn_id``, which is exactly what "one user instruction"
means everywhere else in the design (ARCHITECTURE 7).

Consumption is charged from **measured** odometry and elapsed time as the wheels
turn, not reserved at dispatch, so a drive that is preempted after 10 cm costs
10 cm.  An estimate is only ever compared against what is left; it is never
subtracted.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Final

__all__ = ["BudgetLedger", "BudgetLimits", "Remaining"]

_KEEP_TURNS: Final = 8
"""How many recent instructions stay in the ledger.  A turn is finished long
before eight more arrive; this only bounds the dictionary."""


@dataclass(frozen=True, slots=True)
class BudgetLimits:
    """``[limits] budget_path_m`` and ``budget_motion_s``."""

    path_m: float
    motion_s: float


@dataclass(frozen=True, slots=True)
class Remaining:
    """What one instruction has left."""

    path_m: float
    motion_s: float

    def covers(self, path_m: float, motion_s: float) -> bool:
        return path_m <= self.path_m and motion_s <= self.motion_s


class BudgetLedger:
    """Cumulative path and motion time, per ``turn_id``."""

    def __init__(self, limits: BudgetLimits) -> None:
        self.limits = limits
        self._spent: OrderedDict[str, tuple[float, float]] = OrderedDict()

    def spent(self, turn_id: str) -> tuple[float, float]:
        """``(path_m, motion_s)`` already consumed by this instruction."""
        return self._spent.get(turn_id, (0.0, 0.0))

    def remaining(self, turn_id: str) -> Remaining:
        path, seconds = self.spent(turn_id)
        return Remaining(
            path_m=max(0.0, self.limits.path_m - path),
            motion_s=max(0.0, self.limits.motion_s - seconds),
        )

    def allows(self, turn_id: str, path_m: float, motion_s: float) -> bool:
        """Whether an estimate of this size still fits inside the instruction."""
        return self.remaining(turn_id).covers(abs(path_m), max(0.0, motion_s))

    def charge(self, turn_id: str, path_m: float, motion_s: float) -> Remaining:
        """Book actual consumption and return what is left afterwards."""
        path, seconds = self.spent(turn_id)
        self._spent[turn_id] = (path + abs(path_m), seconds + max(0.0, motion_s))
        self._spent.move_to_end(turn_id)
        while len(self._spent) > _KEEP_TURNS:
            self._spent.popitem(last=False)
        return self.remaining(turn_id)
