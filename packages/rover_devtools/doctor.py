"""The preflight report: what is present, what is missing, and what to do next.

ARCHITECTURE 12: "prints resolved config, effective limits, device and socket
status, and the venv's import map -- the first thing to run when a deploy step
fails".  So every check carries a fix line, and the summary ends by naming the
first thing to do.

The checks look at the machine, never at ``sys.platform``: a missing
``picamera2`` and a missing group ``rover`` are reported as facts about this
host, so the same code says something true on a Mac and on a Pi.  Nothing here
imports an optional dependency -- ``importlib.util.find_spec`` answers the
question without running module-level code that may block on a device.

No secret is ever printed: the box check reports whether ``[box] api_key_env``
names a variable that is set, never its value.
"""

from __future__ import annotations

import argparse
import grp
import importlib.util
import json
import os
import shutil
import socket
import stat
import subprocess
import sys
import urllib.error
import urllib.request
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from rover_contracts.config import RobotConfig

from rover_devtools import load_config_or_default
from rover_devtools.mcu_sim import SimulatorMissing, find_binary

__all__ = ["Check", "Dependency", "DEFAULT_IMPORTS", "main", "run_checks"]

OK, WARN, FAIL = "ok", "warn", "fail"

MIN_PYTHON = (3, 11)
"""``requires-python = ">=3.11"`` (ARCHITECTURE 12)."""


@dataclass(frozen=True, slots=True)
class Check:
    """One line of the report.  ``fix`` is what to do about a warn or a fail."""

    name: str
    status: str
    detail: str
    fix: str = ""

    def as_dict(self) -> dict[str, str]:
        return {
            "name": self.name,
            "status": self.status,
            "detail": self.detail,
            "fix": self.fix,
        }


@dataclass(frozen=True, slots=True)
class Dependency:
    """One row of the import map: a module, why it is here, and whether the
    rover can run without it."""

    module: str
    why: str
    required: bool
    fix: str


DEFAULT_IMPORTS: tuple[Dependency, ...] = (
    Dependency("rover_contracts", "every wire format", True,
               "uv pip install -e packages/rover_contracts"),
    Dependency("pydantic", "A12 stage two, strict validation", True, "uv sync"),
    # ARCHITECTURE 12 pins it in the base set, but robotd's link opens the
    # device itself (os.open + termios) and nothing imports this in v1, so its
    # absence is informational, not a fault.
    Dependency("serial_asyncio_fast", "pinned by ARCHITECTURE 12; unused in v1",
               False, "uv sync"),
    Dependency("openai", "brain's box client", False, "uv sync"),
    Dependency("aiohttp", "rover-web", False, "uv sync"),
    Dependency("picamera2", "rover-cam, Pi only", False,
               "Pi only: sudo apt install -y --no-install-recommends python3-picamera2"),
    Dependency("sounddevice", "the audio owner thread, speech extra", False,
               "uv pip install -e '.[speech]' (needs libportaudio2 on the Pi)"),
    Dependency("sherpa_onnx", "STT and Silero VAD, speech extra", False,
               "uv pip install -e '.[speech]'"),
    Dependency("numpy", "the audio ring, speech extra", False,
               "uv pip install -e '.[speech]'"),
)


# --------------------------------------------------------------------------
# Individual checks
# --------------------------------------------------------------------------


def _python() -> Check:
    version = ".".join(str(part) for part in sys.version_info[:3])
    if sys.version_info[:2] >= MIN_PYTHON:
        return Check("python", OK, f"{version} at {sys.executable}")
    wanted = ".".join(str(part) for part in MIN_PYTHON)
    return Check(
        "python", FAIL, f"{version} at {sys.executable}",
        f"the rover needs Python >= {wanted}; run the tools from the venv",
    )


def _venv() -> Check:
    if sys.prefix != sys.base_prefix:
        return Check("venv", OK, sys.prefix)
    return Check(
        "venv", WARN, f"running against the base interpreter {sys.prefix}",
        "uv venv --python /usr/bin/python3 --system-site-packages .venv, then "
        "use .venv/bin/python",
    )


def _import_check(dep: Dependency) -> Check:
    name = f"import {dep.module}"
    try:
        spec = importlib.util.find_spec(dep.module)
    except (ImportError, ValueError):
        spec = None
    if spec is not None:
        return Check(name, OK, f"{dep.why} -- {spec.origin or 'namespace package'}")
    return Check(
        name, FAIL if dep.required else WARN, f"{dep.why} -- not importable", dep.fix
    )


def _simulator() -> Check:
    try:
        binary = find_binary()
    except SimulatorMissing as exc:
        return Check("mcu-sim", WARN, "not built", str(exc).splitlines()[-1])
    return Check("mcu-sim", OK, str(binary))


def _writers(path: Path) -> str:
    """Who has the port open, when the host has a tool that can say (I-18)."""
    fuser = shutil.which("fuser")
    if fuser is None:
        return ""
    try:
        done = subprocess.run(  # noqa: S603 - fixed argv, resolved binary
            [fuser, "-v", str(path)], capture_output=True, timeout=3.0, text=True
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    pids = done.stdout.split() or done.stderr.split()
    return f", {len(pids)} holder(s)" if pids else ", no holder"


def _serial_device(config: RobotConfig) -> Check:
    port = Path(config.serial.port)
    backend = config.serial.backend
    if not port.exists():
        fix = (
            f"start the simulator: python -m rover_devtools.mcu_sim --link {port}"
            if backend == "pty"
            else "check dtoverlay=uart5 in /boot/firmware/config.txt and the udev "
            "rule (deploy steps 8 and 9); udevadm info -q all -n /dev/rover-mcu"
        )
        return Check("serial device", WARN, f"{port} does not exist ({backend})", fix)
    target = os.readlink(port) if port.is_symlink() else ""
    info = os.stat(port)
    kind = "char device" if stat.S_ISCHR(info.st_mode) else "not a char device"
    access = "rw" if os.access(port, os.R_OK | os.W_OK) else "no rw access"
    detail = (
        f"{port}{f' -> {target}' if target else ''}: {kind}, {access}, "
        f"mode {stat.filemode(info.st_mode)}{_writers(port)}"
    )
    if access != "rw":
        return Check(
            "serial device", FAIL, detail,
            "add your user to group rover, or check the udev rule's GROUP/MODE",
        )
    return Check("serial device", OK, detail)


_SOCKET_OWNERS = {
    "sock": ("robotd.sock", "rover-robotd"),
    "frames_sock": ("frames.sock", "rover-cam"),
    "brain_sock": ("brain.sock", "rover-brain"),
}


def _socket_check(key: str, path_text: str) -> Check:
    label, unit = _SOCKET_OWNERS[key]
    path = Path(path_text)
    name = f"socket {label}"
    if not path.exists():
        return Check(name, WARN, f"{path} does not exist", f"start {unit}")
    if not stat.S_ISSOCK(path.stat().st_mode):
        return Check(
            name, FAIL, f"{path} is not a socket", f"remove {path}, then restart {unit}"
        )
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(0.3)
    try:
        sock.connect(str(path))
    except OSError as exc:
        return Check(
            name, WARN, f"{path} exists but does not accept: {exc}",
            f"{unit} is not running; the stale socket is harmless",
        )
    finally:
        sock.close()
    mode = stat.filemode(path.stat().st_mode)
    return Check(name, OK, f"{path} accepting, mode {mode}")


def _box_check(config: RobotConfig, probe: bool) -> list[Check]:
    url = config.box.url.rstrip("/") + "/models"
    key_set = bool(os.environ.get(config.box.api_key_env))
    checks = [
        Check(
            "box api key",
            OK if key_set else WARN,
            f"${config.box.api_key_env} is {'set' if key_set else 'unset'}",
            "" if key_set else
            f"export {config.box.api_key_env}=... if the box needs one; "
            "fakebox and an open vLLM do not",
        )
    ]
    if not probe:
        checks.append(Check("box", WARN, f"{config.box.url} not probed", "drop --no-box"))
        return checks
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:  # noqa: S310
            body = json.loads(response.read())
    except (urllib.error.URLError, OSError, ValueError) as exc:
        checks.append(
            Check(
                "box", WARN, f"{url} unreachable: {exc}",
                "python -m rover_devtools.fakebox --port "
                f"{_port_of(config.box.url)}, or docker compose -f "
                "docker/compose.box.yml up -d vllm",
            )
        )
        return checks
    served = [str(entry.get("id")) for entry in body.get("data", [])]
    wanted = config.box.model
    checks.append(
        Check(
            "box", OK if wanted in served else WARN,
            f"{url} serves {served or 'nothing'}",
            "" if wanted in served else
            f"[box] model is {wanted!r}; set --served-model-name to match",
        )
    )
    checks.append(_box_caps(config))
    return checks


def _port_of(url: str) -> int:
    return urlparse(url).port or 8000


def _box_caps(config: RobotConfig) -> Check:
    """``box_caps.json`` is what brain's probe wrote (4.6); doctor only reads it."""
    for candidate in (Path("box_caps.json"), Path(config.log.dir) / "box_caps.json"):
        if candidate.is_file():
            try:
                caps = json.loads(candidate.read_text())
            except ValueError as exc:
                return Check("box caps", FAIL, f"{candidate} is not JSON: {exc}",
                             "re-run python -m rover_brain.box_probe")
            summary = " ".join(f"{k}={v}" for k, v in sorted(caps.items()))
            return Check("box caps", OK, f"{candidate}: {summary}")
    return Check(
        "box caps", WARN, "no box_caps.json",
        "python -m rover_brain.box_probe --url $ROVER__BOX__URL (deploy step 2)",
    )


def _storage_check(label: str, path_text: str) -> Check:
    path = Path(path_text)
    existing = path
    while not existing.exists() and existing != existing.parent:
        existing = existing.parent
    free_gib = shutil.disk_usage(existing).free / 2**30
    if not path.is_dir():
        return Check(
            f"storage {label}", WARN, f"{path} does not exist ({free_gib:.1f} GiB free "
            f"on {existing})",
            f"install -d -o rover -g rover {path}, or point [{label}] at a Mac path "
            "-- `make dev` must never write /data",
        )
    if not os.access(path, os.W_OK):
        return Check(f"storage {label}", FAIL, f"{path} is not writable",
                     f"chown -R rover:rover {path}")
    status = OK if free_gib >= 1.0 else WARN
    return Check(f"storage {label}", status, f"{path} writable, {free_gib:.1f} GiB free",
                 "" if status == OK else "free some space before the next soak")


def _group_check() -> Check:
    """A10: the 0660 socket in group ``rover`` is how "only robotd writes the
    port" becomes a filesystem permission."""
    try:
        group = grp.getgrnam("rover")
    except KeyError:
        return Check(
            "group rover", WARN, "no group rover on this host",
            "Pi only: deploy/install.sh creates rover:rover and the per-unit users",
        )
    user = os.environ.get("USER", "")
    member = user in group.gr_mem or os.getgid() == group.gr_gid
    return Check(
        "group rover", OK if member else WARN,
        f"gid {group.gr_gid}, members {', '.join(group.gr_mem) or 'none'}",
        "" if member else f"sudo usermod -aG rover {user}; then log in again",
    )


# --------------------------------------------------------------------------
# The report
# --------------------------------------------------------------------------


def run_checks(
    config_path: str | Path | None = None,
    *,
    imports: Sequence[Dependency] | None = None,
    probe_box: bool = True,
) -> tuple[RobotConfig, str, list[Check]]:
    """Every check, in the order a stuck user should read them.

    ``imports`` defaults to :data:`DEFAULT_IMPORTS`; pass a shorter table to ask
    about a different set of modules.
    """
    try:
        config, origin = load_config_or_default(config_path)
        config_check = Check(
            "config", OK,
            f"{origin}; safety_hash={config.safety_hash()}, "
            f"serial.backend={config.serial.backend}",
        )
    except (OSError, ValueError) as exc:
        config, origin = RobotConfig(), "built-in defaults (config did not load)"
        config_check = Check(
            "config", FAIL, str(exc),
            "fix config/robot.toml, or pass --config; a value over a compiled "
            "ceiling refuses startup and is never clamped (A33)",
        )

    checks = [_python(), _venv(), config_check]
    checks += [
        _import_check(dep)
        for dep in (DEFAULT_IMPORTS if imports is None else imports)
    ]
    checks.append(_simulator())
    checks.append(_serial_device(config))
    checks += [
        _socket_check(key, getattr(config.bus, key)) for key in _SOCKET_OWNERS
    ]
    checks += _box_check(config, probe_box)
    checks.append(_storage_check("log", config.log.dir))
    checks.append(_storage_check("stt", config.stt.model_dir))
    checks.append(_group_check())
    return config, origin, checks


def _limits_line(config: RobotConfig) -> str:
    limits = config.limits
    return (
        f"speed_mps={limits.speed_mps} default={limits.speed_default_mps} "
        f"drive_m={limits.drive_m} turn_deg={limits.turn_deg} "
        f"rate_dps={limits.rate_dps} budget={limits.budget_path_m}m/"
        f"{limits.budget_motion_s}s twist={limits.twist_linear_mps}mps/"
        f"{limits.twist_angular_radps}radps stream={config.bus.allow_stream or 'off'}"
    )


def report(config: RobotConfig, origin: str, checks: Sequence[Check]) -> str:
    """The human report.  Every warn and fail is followed by its fix."""
    lines = [f"rover doctor -- {origin}", ""]
    width = max(len(check.name) for check in checks)
    for check in checks:
        lines.append(f"  {check.status:<4}  {check.name:<{width}}  {check.detail}")
        if check.fix and check.status != OK:
            lines.append(f"  {'':<4}  {'':<{width}}  -> {check.fix}")
    counts = {status: 0 for status in (OK, WARN, FAIL)}
    for check in checks:
        counts[check.status] += 1
    lines += ["", f"effective limits: {_limits_line(config)}", ""]
    lines.append(
        f"{counts[OK]} ok, {counts[WARN]} warn, {counts[FAIL]} fail"
    )
    first = next(
        (c for c in checks if c.status == FAIL and c.fix),
        next((c for c in checks if c.status == WARN and c.fix), None),
    )
    if first is not None:
        lines.append(f"next: {first.fix}")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rover_devtools.doctor",
        description="Report what is present, what is missing, and what to do next.",
    )
    parser.add_argument("--config", metavar="PATH")
    parser.add_argument(
        "--no-box", action="store_true", help="skip the box HTTP probe"
    )
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)

    config, origin, checks = run_checks(args.config, probe_box=not args.no_box)
    if args.json:
        payload: dict[str, Any] = {
            "config_origin": origin,
            "safety_hash": config.safety_hash(),
            "limits": config.limits.model_dump(),
            "checks": [check.as_dict() for check in checks],
        }
        print(json.dumps(payload, indent=2))
    else:
        print(report(config, origin, checks))
    return 1 if any(check.status == FAIL for check in checks) else 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
