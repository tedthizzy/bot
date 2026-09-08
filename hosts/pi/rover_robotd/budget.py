"""The per-instruction motion budget (I-15).

At most ``[limits] budget_motion_s`` of motion per user instruction, however
many valid skills the model emits for it.  The unit of accounting is the
``turn_id``, which is exactly what "one user instruction" means everywhere else
in the design (ARCHITECTURE 7).

The rover is open loop, so the budget is time (ADR-0013): the control loop
charges every command period in which it streamed a non-zero value to either
side.  Nothing is reserved at dispatch; a drive preempted after 200 ms costs
200 ms.  An estimate is only ever compared against what is left.
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Final

__all__ = ["BudgetLedger"]

_KEEP_TURNS: Final = 8
"""How many recent instructions stay in the ledger.  A turn is finished long
before eight more arrive; this only bounds the dictionary."""


class BudgetLedger:
    """Cumulative motion seconds, per ``turn_id``."""

    def __init__(self, motion_s: float) -> None:
        if motion_s <= 0.0:
            raise ValueError(f"budget_motion_s must be positive, got {motion_s!r}")
        self.motion_s = float(motion_s)
        self._spent: OrderedDict[str, float] = OrderedDict()

    def spent(self, turn_id: str) -> float:
        """Motion seconds already consumed by this instruction."""
        return self._spent.get(turn_id, 0.0)

    def remaining(self, turn_id: str) -> float:
        return max(0.0, self.motion_s - self.spent(turn_id))

    def exhausted(self, turn_id: str) -> bool:
        return self.remaining(turn_id) <= 0.0

    def charge(self, turn_id: str, seconds: float) -> float:
        """Book one period of non-zero streaming; return what is left."""
        self._spent[turn_id] = self.spent(turn_id) + max(0.0, seconds)
        self._spent.move_to_end(turn_id)
        while len(self._spent) > _KEEP_TURNS:
            self._spent.popitem(last=False)
        return self.remaining(turn_id)
