"""Put ``tests/gates`` on the path so the gate wrappers can import ``gatelib``.

The gate scripts do this for themselves when run directly; pytest imports the
wrappers, not the scripts, so it has to happen here too.
"""

from __future__ import annotations

import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
for _path in (str(_HERE), str(_HERE.parents[1] / "packages")):
    if _path not in sys.path:
        sys.path.insert(0, _path)
