"""Shared machinery for the six gate scripts.

``runner``  the three-way result model, the per-case print and the JSONL metrics
``env``     what is present to gate against -- config, sockets, modules, a Pi
``bus``     a deliberately impolite NDJSON client for robotd.sock and brain.sock
``checks``  isolated current-runtime tests and native firmware checks

Importing this package puts ``packages/`` on ``sys.path``, so the gates run from
any working directory whether or not ``rover_contracts`` is installed.
"""

from __future__ import annotations

from gatelib.env import REPO, have_binary, have_module, is_pi, socket_alive
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for

__all__ = [
    "REPO",
    "GateRun",
    "Status",
    "base_parser",
    "have_binary",
    "have_module",
    "is_pi",
    "metrics_path_for",
    "socket_alive",
]
