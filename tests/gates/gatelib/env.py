"""What is present to gate against.

Gates run in three environments -- a bare MacBook with only the contracts built,
a full ``make dev`` simulation, and the rover on a bench -- and the same script
has to give an honest answer in all three.  Everything here answers "is the
subject of this case actually here?" so a gate can skip with a named reason
instead of asserting against nothing.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
import socket
import sys
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]

if str(REPO / "packages") not in sys.path:
    sys.path.insert(0, str(REPO / "packages"))

from rover_contracts import (  # noqa: E402
    LimitsConfig,
    RobotConfig,
    SafetyConfig,
    load_config,
)

__all__ = [
    "REPO",
    "Limits",
    "have_binary",
    "have_module",
    "is_pi",
    "limits_from",
    "load_robot_config",
    "socket_alive",
]


def have_module(name: str) -> bool:
    """True when ``import name`` would find something.

    Used to decide whether a component of the rover exists yet; the gates are
    written before the components they drive, and a missing one is a skip.
    """
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def have_binary(name: str) -> bool:
    return shutil.which(name) is not None


def is_pi() -> bool:
    """True on Raspberry Pi hardware, by the device tree model string."""
    model = Path("/proc/device-tree/model")
    try:
        return "raspberry pi" in model.read_bytes().decode("ascii", "replace").lower()
    except OSError:
        return False


def socket_alive(path: str | Path, timeout: float = 0.5) -> bool:
    """True when something is listening on that Unix socket."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(str(path))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def load_robot_config() -> tuple[RobotConfig, str]:
    """The running configuration and where it came from.

    ``config/robot.mac.toml`` first when not on a Pi, then ``config/robot.toml``,
    then the contract defaults -- which are the architecture's own 5.8 values, so
    a gate on a bare checkout still has numbers to assert against and says so.
    """
    candidates = (
        [REPO / "config" / "robot.toml"]
        if is_pi()
        else [REPO / "config" / "robot.mac.toml", REPO / "config" / "robot.toml"]
    )
    for path in candidates:
        if path.exists():
            return load_config(path, env=os.environ), str(path.relative_to(REPO))
    return RobotConfig(), "contract defaults (no config/robot*.toml)"


@dataclass(frozen=True, slots=True)
class Limits:
    """The limits a gate asserts against, and their provenance.

    A33 and I-15 both insist a gate reads ``welcome.limits`` rather than the
    file, because an environment override changes what runs without changing
    what is written.  ``source`` records which it got, and the cases that name
    ``welcome`` in their criterion skip unless it is ``"welcome"``.
    """

    limits: LimitsConfig
    safety: SafetyConfig
    source: str

    @property
    def from_welcome(self) -> bool:
        return self.source == "welcome"


def limits_from(welcome: object | None) -> Limits:
    """Limits from a live ``welcome`` message if there is one, else config."""
    if welcome is not None:
        limits = getattr(welcome, "limits", None)
        safety = getattr(welcome, "safety", None)
        if isinstance(limits, LimitsConfig) and isinstance(safety, SafetyConfig):
            return Limits(limits, safety, "welcome")
    config, origin = load_robot_config()
    return Limits(config.limits, config.safety, origin)
