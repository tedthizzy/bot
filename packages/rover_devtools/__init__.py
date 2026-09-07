"""Fakes and operator tools that make the whole rover testable on a Mac.

Five entry points, one job each:

``mcu_sim``    the firmware host simulator behind a pty, so robotd opens a
               device path exactly as it opens the real serial port
``fakebox``    a deterministic OpenAI-compatible endpoint with injectable
               misbehaviour -- what the gates run against
``wirecat``    a readable live decode of the Pi<->MCU line protocol (5.1)
``roverctl``   the one operator CLI (ARCHITECTURE 12: ``rover-cli`` is not a name)
``doctor``     the preflight report, the first thing to run when a step fails

Nothing here is imported by a robot process.  Principle 6 keeps the fakes in
configuration: ``[serial] backend="pty"`` points robotd at ``mcu_sim``'s device
path and ``[box] url`` points brain at ``fakebox``, so no component carries a
branch for being simulated.
"""

from __future__ import annotations

from pathlib import Path

from rover_contracts.config import RobotConfig, load_config

__all__ = [
    "CONFIG_SEARCH_PATHS",
    "load_config_or_default",
    "resolve_config_path",
]

__version__ = "0.1.0"

CONFIG_SEARCH_PATHS: tuple[str, ...] = ("config/robot.toml", "config/robot.mac.toml")
"""Where the tools look when no ``--config`` is given, in order.  The Mac file
is second so a Pi checkout with both present uses the real one (5.8)."""


def resolve_config_path(explicit: str | Path | None = None) -> Path | None:
    """The config file the tools will read, or ``None`` if there is none.

    An explicit path is returned whether or not it exists, so a typo is
    reported as a missing file rather than silently falling back to defaults.
    """
    if explicit is not None:
        return Path(explicit)
    for candidate in CONFIG_SEARCH_PATHS:
        path = Path(candidate)
        if path.is_file():
            return path
    return None


def load_config_or_default(
    explicit: str | Path | None = None,
) -> tuple[RobotConfig, str]:
    """Load the config, or fall back to the compiled defaults.

    Returns the config and where it came from, for the tools to print.  A file
    that exists but does not validate raises ``ConfigError``: an operator tool
    must not quietly run against different numbers from the ones on disk.
    """
    path = resolve_config_path(explicit)
    if path is None:
        return RobotConfig(), "built-in defaults (no config/robot.toml)"
    if not path.is_file() and explicit is not None:
        raise FileNotFoundError(f"no such config file: {path}")
    return load_config(path), str(path)
