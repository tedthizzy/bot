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
import re
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

from rover_contracts.config import LinkConfig, RobotConfig, SafetyConfig
from rover_contracts.wave_proto import POWER_CAP

from rover_devtools import load_config_or_default

__all__ = [
    "Check",
    "DEFAULT_IMPORTS",
    "Dependency",
    "FORK_HEADER",
    "fork_check",
    "fork_constants",
    "link_check",
    "link_fix",
    "link_timing",
    "main",
    "report",
    "run_checks",
    "stub_check",
]

OK, WARN, FAIL = "ok", "warn", "fail"

MIN_PYTHON = (3, 11)
"""``requires-python = ">=3.11"`` (ARCHITECTURE 12)."""

FORK_HEADER = Path("firmware/General_Driver/bot_config.h")
"""Where the firmware fork compiles its safety constants.  Relative to the
repo; resolved against the working directory, then the source tree."""

_REPO_ROOT = Path(__file__).resolve().parents[2]
_FORK_CONSTANTS = ("BOT_HEARTBEAT_MS", "BOT_POWER_CAP")


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
    Dependency("rover_contracts", "every wire format", True, "uv pip install -e ."),
    Dependency("pydantic", "A12 stage two, strict validation", True, "uv sync"),
    Dependency("serial_asyncio_fast", "pinned by ARCHITECTURE 12", False, "uv sync"),
    Dependency("openai", "brain's box client", False, "uv sync"),
    Dependency("aiohttp", "rover-web", False, "uv sync"),
    Dependency(
        "picamera2",
        "rover-cam, Pi only",
        False,
        "Pi only: sudo apt install -y --no-install-recommends python3-picamera2",
    ),
    Dependency(
        "sounddevice",
        "the audio owner thread, speech extra",
        False,
        "uv pip install -e '.[speech]' (needs libportaudio2 on the Pi)",
    ),
    Dependency(
        "sherpa_onnx",
        "STT and Silero VAD, speech extra",
        False,
        "uv pip install -e '.[speech]'",
    ),
    Dependency(
        "numpy", "the audio ring, speech extra", False, "uv pip install -e '.[speech]'"
    ),
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
        "python",
        FAIL,
        f"{version} at {sys.executable}",
        f"the rover needs Python >= {wanted}; run the tools from the venv",
    )


def _venv() -> Check:
    if sys.prefix != sys.base_prefix:
        return Check("venv", OK, sys.prefix)
    return Check(
        "venv",
        WARN,
        f"running against the base interpreter {sys.prefix}",
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


def stub_check() -> Check:
    """``rover-stub`` is the simulated controller; a checkout without it has no
    way to run robotd on a Mac."""
    script = shutil.which("rover-stub")
    if script is not None:
        return Check("rover-stub", OK, f"console script {script}")
    try:
        spec = importlib.util.find_spec("rover_devtools.rover_stub")
    except (ImportError, ValueError):
        spec = None
    if spec is not None:
        return Check(
            "rover-stub",
            OK,
            f"module {spec.origin} (python -m rover_devtools.rover_stub --pty)",
        )
    return Check(
        "rover-stub",
        WARN,
        "neither the console script nor the module is present",
        "uv sync, or uv pip install -e . -- then rover-stub --pty",
    )


def _holders(path: Path) -> str:
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


def link_fix(port: str) -> str:
    """What to do about a missing ``[link] port``: the Pi's UART needs
    enabling; anything else is a stub path that is not running."""
    if port == "/dev/serial0" or port.startswith(("/dev/ttyAMA", "/dev/ttyS")):
        return (
            "Pi: enable_uart=1 and no serial console in /boot/firmware/config.txt "
            "(raspi-config > Interface Options > Serial Port), then reboot; "
            "on a Mac run rover-stub --pty and set [link] port to the path it prints"
        )
    return (
        "start the simulator: rover-stub --pty, and set [link] port to the path it prints"
    )


def link_check(config: RobotConfig) -> Check:
    """The rover link named by ``[link]``: a readable serial device, or a TCP
    peer that accepts."""
    link = config.link
    if link.backend == "tcp":
        where = f"{link.tcp_host}:{link.tcp_port}"
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.3)
        try:
            sock.connect((link.tcp_host, link.tcp_port))
        except OSError as exc:
            return Check(
                "link",
                WARN,
                f"tcp {where} does not accept: {exc}",
                f'rover-stub --tcp {link.tcp_port}, or set [link] backend = "serial"',
            )
        finally:
            sock.close()
        return Check("link", OK, f"tcp {where} accepting")

    port = Path(link.port)
    if not port.exists():
        return Check("link", WARN, f"{port} does not exist (serial)", link_fix(link.port))
    target = os.readlink(port) if port.is_symlink() else ""
    info = os.stat(port)
    if not stat.S_ISCHR(info.st_mode):
        return Check(
            "link",
            WARN,
            f"{port} is not a character device",
            "point [link] port at the UART or at the pty rover-stub printed",
        )
    readable = os.access(port, os.R_OK | os.W_OK)
    detail = (
        f"{port}{f' -> {target}' if target else ''}: char device, "
        f"{'rw' if readable else 'no rw access'}, mode {stat.filemode(info.st_mode)}"
        f"{_holders(port)}"
    )
    if not readable:
        return Check(
            "link",
            FAIL,
            detail,
            "add your user to the device's group (dialout on a Pi), then log in again",
        )
    return Check("link", OK, detail)


def link_timing(link: LinkConfig, safety: SafetyConfig) -> Check:
    """The three periods that make the heartbeat a stop guarantee, in order:
    at least three speed commands per heartbeat, and feedback that goes stale
    before the controller's own watchdog does (T0)."""
    period_ms = 1000.0 / link.command_hz
    problems: list[str] = []
    if period_ms * 3 > safety.heartbeat_ms:
        problems.append(
            f"command_hz {link.command_hz} gives {period_ms:.0f} ms per command, "
            f"x3 = {period_ms * 3:.0f} > heartbeat_ms {safety.heartbeat_ms}"
        )
    if safety.feedback_max_age_ms >= safety.heartbeat_ms:
        problems.append(
            f"feedback_max_age_ms {safety.feedback_max_age_ms} >= "
            f"heartbeat_ms {safety.heartbeat_ms}"
        )
    if link.feedback_interval_ms >= safety.feedback_max_age_ms:
        problems.append(
            f"feedback_interval_ms {link.feedback_interval_ms} >= "
            f"feedback_max_age_ms {safety.feedback_max_age_ms}"
        )
    if problems:
        return Check(
            "link timing",
            FAIL,
            "; ".join(problems),
            "keep feedback_interval_ms < feedback_max_age_ms < heartbeat_ms and "
            "command_hz >= 3000 / heartbeat_ms",
        )
    return Check(
        "link timing",
        OK,
        f"{link.command_hz} Hz = {period_ms:.0f} ms/command, x3 = {period_ms * 3:.0f} "
        f"<= heartbeat {safety.heartbeat_ms} ms; feedback {link.feedback_interval_ms} "
        f"< max_age {safety.feedback_max_age_ms} < heartbeat {safety.heartbeat_ms}",
    )


def fork_constants(text: str) -> dict[str, float]:
    """``#define BOT_HEARTBEAT_MS 300`` and ``#define BOT_POWER_CAP 0.30f``,
    whatever the spelling of the number, as a name -> value map."""
    found: dict[str, float] = {}
    for name in _FORK_CONSTANTS:
        match = re.search(
            rf"^\s*#\s*define\s+{name}\s+\(?\s*([-+]?\d+(?:\.\d+)?)\s*[fFuUlL]*\s*\)?",
            text,
            re.MULTILINE,
        )
        if match:
            found[name] = float(match.group(1))
    return found


def fork_header_path() -> Path:
    """The header in the working directory if a checkout is there, else in the
    source tree this module was imported from."""
    for base in (Path.cwd(), _REPO_ROOT):
        candidate = base / FORK_HEADER
        if candidate.exists():
            return candidate
    return _REPO_ROOT / FORK_HEADER


def fork_check(config: RobotConfig, header: Path | None = None) -> Check:
    """The fork's compiled constants against the host's: robotd refuses motion
    unless the banner it sees matches ``[safety] heartbeat_ms`` and the 0.30
    ceiling, so a disagreement here is a rover that will not move."""
    header = fork_header_path() if header is None else header
    fix = "the fork under firmware/ is being written; docs/protocol.md is its contract"
    if not header.parent.is_dir():
        return Check(
            "firmware fork",
            WARN,
            f"firmware fork not present ({header.parent} does not exist)",
            fix,
        )
    if not header.is_file():
        return Check(
            "firmware fork", WARN, f"firmware fork not present ({header} missing)", fix
        )
    values = fork_constants(header.read_text(encoding="utf-8", errors="replace"))
    missing = [name for name in _FORK_CONSTANTS if name not in values]
    if missing:
        return Check(
            "firmware fork",
            FAIL,
            f"{header} does not define {', '.join(missing)}",
            "add #define BOT_HEARTBEAT_MS 300 and #define BOT_POWER_CAP 0.30f to "
            "the header",
        )
    hb, cap = values["BOT_HEARTBEAT_MS"], values["BOT_POWER_CAP"]
    wanted_hb = config.safety.heartbeat_ms
    detail = (
        f"{header}: BOT_HEARTBEAT_MS={hb:g} vs [safety] heartbeat_ms={wanted_hb}, "
        f"BOT_POWER_CAP={cap:g} vs ceiling {POWER_CAP:g}"
    )
    if hb != wanted_hb or abs(cap - POWER_CAP) > 1e-6:
        return Check(
            "firmware fork",
            FAIL,
            detail,
            "the host and the fork must agree: rebuild the fork or fix [safety] "
            "heartbeat_ms; robotd refuses motion until the banner matches",
        )
    return Check("firmware fork", OK, detail)


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
            name,
            WARN,
            f"{path} exists but does not accept: {exc}",
            f"{unit} is not running; the stale socket is harmless",
        )
    finally:
        sock.close()
    mode = stat.filemode(path.stat().st_mode)
    return Check(name, OK, f"{path} accepting, mode {mode}")


def _box_check(config: RobotConfig, probe: bool) -> list[Check]:
    url = config.box.url.rstrip("/") + "/models"
    key = os.environ.get(config.box.api_key_env)
    key_set = bool(key)
    checks = [
        Check(
            "box api key",
            OK if key_set else WARN,
            f"${config.box.api_key_env} is {'set' if key_set else 'unset'}",
            ""
            if key_set
            else f"export {config.box.api_key_env}=... if the box needs one; "
            "fakebox and an open vLLM do not",
        )
    ]
    if not probe:
        checks.append(Check("box", WARN, f"{config.box.url} not probed", "drop --no-box"))
        return checks
    try:
        request = urllib.request.Request(
            url, headers={"Authorization": f"Bearer {key}"} if key else {}
        )
        with urllib.request.urlopen(request, timeout=2.0) as response:  # noqa: S310
            body = json.loads(response.read())
    except (urllib.error.URLError, OSError, ValueError) as exc:
        checks.append(
            Check(
                "box",
                WARN,
                f"{url} unreachable: {exc}",
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
            "box",
            OK if wanted in served else WARN,
            f"{url} serves {served or 'nothing'}",
            ""
            if wanted in served
            else f"[box] model is {wanted!r}; set --served-model-name to match",
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
                return Check(
                    "box caps",
                    FAIL,
                    f"{candidate} is not JSON: {exc}",
                    "re-run python -m rover_brain.box_probe",
                )
            summary = " ".join(f"{k}={v}" for k, v in sorted(caps.items()))
            return Check("box caps", OK, f"{candidate}: {summary}")
    return Check(
        "box caps",
        WARN,
        "no box_caps.json",
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
            f"storage {label}",
            WARN,
            f"{path} does not exist ({free_gib:.1f} GiB free on {existing})",
            f"install -d -o rover -g rover {path}, or point [{label}] at a Mac path "
            "-- `make sim` must never write /data",
        )
    if not os.access(path, os.W_OK):
        return Check(
            f"storage {label}",
            FAIL,
            f"{path} is not writable",
            f"chown -R rover:rover {path}",
        )
    status = OK if free_gib >= 1.0 else WARN
    return Check(
        f"storage {label}",
        status,
        f"{path} writable, {free_gib:.1f} GiB free",
        "" if status == OK else "free some space before the next soak",
    )


def _group_check() -> Check:
    """A10: the 0660 socket in group ``rover`` is how "only robotd writes the
    port" becomes a filesystem permission."""
    try:
        group = grp.getgrnam("rover")
    except KeyError:
        return Check(
            "group rover",
            WARN,
            "no group rover on this host",
            "Pi only: deploy/install.sh creates rover:rover and the per-unit users",
        )
    user = os.environ.get("USER", "")
    member = user in group.gr_mem or os.getgid() == group.gr_gid
    return Check(
        "group rover",
        OK if member else WARN,
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
    fork_header: Path | None = None,
) -> tuple[RobotConfig, str, list[Check]]:
    """Every check, in the order a stuck user should read them.

    ``imports`` defaults to :data:`DEFAULT_IMPORTS`; pass a shorter table to ask
    about a different set of modules.  ``fork_header`` overrides where the
    firmware fork's ``bot_config.h`` is looked for.
    """
    try:
        config, origin = load_config_or_default(config_path)
        link = config.link
        where = f"{link.tcp_host}:{link.tcp_port}" if link.backend == "tcp" else link.port
        config_check = Check(
            "config", OK, f"{origin}; link.backend={link.backend} port={where}"
        )
    except (OSError, ValueError) as exc:
        config, origin = RobotConfig(), "built-in defaults (config did not load)"
        config_check = Check(
            "config",
            FAIL,
            str(exc),
            "fix config/robot.toml, or pass --config; a value over a compiled "
            "ceiling refuses startup and is never clamped (A33)",
        )

    checks = [_python(), _venv(), config_check]
    checks += [
        _import_check(dep) for dep in (DEFAULT_IMPORTS if imports is None else imports)
    ]
    checks.append(stub_check())
    checks.append(link_check(config))
    checks.append(link_timing(config.link, config.safety))
    checks.append(fork_check(config, fork_header))
    checks += [_socket_check(key, getattr(config.bus, key)) for key in _SOCKET_OWNERS]
    checks += _box_check(config, probe_box)
    checks.append(_storage_check("log", config.log.dir))
    checks.append(_storage_check("stt", config.stt.model_dir))
    checks.append(_group_check())
    return config, origin, checks


def _limits_line(config: RobotConfig) -> str:
    limits, safety, link = config.limits, config.safety, config.link
    return (
        f"power_max={limits.power_max} default={limits.power_default} "
        f"min={limits.power_min} drive_for_max_s={limits.drive_for_max_s} "
        f"turn_timeout_max_s={limits.turn_timeout_max_s} "
        f"budget_motion_s={limits.budget_motion_s} twist_power={limits.twist_power} "
        f"heartbeat_ms={safety.heartbeat_ms} "
        f"feedback_max_age_ms={safety.feedback_max_age_ms} "
        f"command_hz={link.command_hz} stream={config.bus.allow_stream or 'off'}"
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
    lines.append(f"{counts[OK]} ok, {counts[WARN]} warn, {counts[FAIL]} fail")
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
    parser.add_argument("--no-box", action="store_true", help="skip the box HTTP probe")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)

    config, origin, checks = run_checks(args.config, probe_box=not args.no_box)
    if args.json:
        payload: dict[str, Any] = {
            "config_origin": origin,
            "limits": config.limits.model_dump(),
            "safety": config.safety.model_dump(),
            "link": config.link.model_dump(),
            "checks": [check.as_dict() for check in checks],
        }
        print(json.dumps(payload, indent=2))
    else:
        print(report(config, origin, checks))
    return 1 if any(check.status == FAIL for check in checks) else 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
