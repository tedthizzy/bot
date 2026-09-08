"""Exercise deployment scripts against an isolated checkout and command doubles."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
DEPLOY = REPO / "hosts/pi/deploy"


@pytest.fixture
def checkout(tmp_path: Path) -> Path:
    root = tmp_path / "checkout"
    scripts = root / "hosts/pi/deploy"
    scripts.mkdir(parents=True)
    for name in ("sync.sh", "check-secrets.sh"):
        shutil.copy2(DEPLOY / name, scripts / name)
    (root / "pyproject.toml").write_text("[project]\nname = 'rover'\n")
    (root / "packages/rover_contracts").mkdir(parents=True)
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    return root


@pytest.mark.parametrize("all_units", [False, True])
def test_sync_resolves_checkout_and_dry_run_never_restarts(
    checkout: Path, tmp_path: Path, all_units: bool
) -> None:
    commands = tmp_path / "bin"
    commands.mkdir()
    log = tmp_path / "calls.jsonl"
    for name in ("rsync", "ssh"):
        executable = commands / name
        executable.write_text(
            f"#!{sys.executable}\n"
            "import json, os, sys\n"
            "with open(os.environ['COMMAND_LOG'], 'a') as out:\n"
            "    out.write(json.dumps(sys.argv) + '\\n')\n"
        )
        executable.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{commands}:{os.environ['PATH']}",
        "COMMAND_LOG": str(log),
        "ROVER_HOST": "rover.local",
        "ROVER_ROOT": "/opt/rover",
    }
    argv = ["bash", str(checkout / "hosts/pi/deploy/sync.sh"), "--dry-run"]
    if all_units:
        argv.append("--all")
    subprocess.run(argv, cwd=tmp_path, env=env, check=True, capture_output=True)
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(calls) == 1
    assert Path(calls[0][0]).name == "rsync"
    assert "--dry-run" in calls[0]
    assert calls[0][-2:] == [f"{checkout}/", "rover.local:/opt/rover/"]
    for private in (".env", ".env.*", ".cache/", "config/robot.toml", "data/", "run/"):
        assert private in calls[0]


@pytest.mark.parametrize(
    "destination",
    ["/", "//", "/./", "/.", "/opt/./", "/opt/../", "/opt/..", "/opt/rover;false"],
)
def test_sync_rejects_unsafe_destination(checkout: Path, destination: str) -> None:
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/sync.sh"), "rover.local", "--dry-run"],
        env={**os.environ, "ROVER_ROOT": destination},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "ROVER_ROOT" in result.stderr


def test_scanner_checks_outside_hosts_and_redacts_credentials(checkout: Path) -> None:
    token = "ghp_" + "x" * 36
    (checkout / "packages/accidental.txt").write_text(token + "\n")
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        cwd=checkout / "hosts/pi",
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "packages/accidental.txt:1" in result.stdout
    assert "[match redacted]" in result.stdout
    assert token not in result.stdout


def test_scanner_accepts_clean_export_and_ignores_own_patterns(checkout: Path) -> None:
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        cwd=checkout.parent,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout
    assert "scanning 3 files" in result.stdout


@pytest.mark.parametrize("host", ["localhost", "example.com", "rover.local", "box.lan"])
def test_scanner_placeholder_host_does_not_hide_same_line_secret(
    checkout: Path, host: str
) -> None:
    token = "ghp_" + "x" * 36
    (checkout / "packages/accidental.txt").write_text(f"url={host} token={token}\n")
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "packages/accidental.txt:1" in result.stdout
    assert token not in result.stdout


def test_scanner_placeholder_host_does_not_hide_private_address(checkout: Path) -> None:
    address = ".".join(("192", "168", "42", "23"))
    (checkout / "packages/accidental.txt").write_text(f"url=box.lan peer={address}\n")
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert "RFC1918 address" in result.stdout
    assert address not in result.stdout


def test_scanner_accepts_exact_public_upstream_lines(checkout: Path) -> None:
    for name in (
        "wifi_ctrl.h",
        "web_page.h",
        "esp_now_ctrl.h",
        "json_cmd.h",
        "ugv_config.h",
    ):
        relative = Path("firmware/General_Driver") / name
        (checkout / relative.parent).mkdir(parents=True, exist_ok=True)
        shutil.copy2(REPO / relative, checkout / relative)
    relative = Path("firmware/patches/0003-radios-off.patch")
    (checkout / relative.parent).mkdir(parents=True)
    shutil.copy2(REPO / relative, checkout / relative)
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout


@pytest.mark.parametrize("mutation", ["edit", "append", "same_line", "different_path"])
def test_scanner_vendor_allowances_require_exact_line_and_path(
    checkout: Path, mutation: str
) -> None:
    relative = Path("firmware/General_Driver/wifi_ctrl.h")
    lines = (REPO / relative).read_text().splitlines()
    assignment = "ap_" + "password" + " ="
    public_line = next(line for line in lines if assignment in line)
    token = "ghp_" + "x" * 36
    if mutation == "edit":
        replacement = "test-" + "z" * 20
        content = public_line.replace(
            re.search(r'"[^\"]*"', public_line)[0], json.dumps(replacement)
        )
    elif mutation == "append":
        content = public_line + "\n" + token
    elif mutation == "same_line":
        content = public_line + f" // localhost {token}"
    else:
        relative = Path("packages/copied.txt")
        content = public_line
    (checkout / relative.parent).mkdir(parents=True, exist_ok=True)
    (checkout / relative).write_text(content + "\n")
    result = subprocess.run(
        ["bash", str(checkout / "hosts/pi/deploy/check-secrets.sh")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1
    assert str(relative) in result.stdout
    assert "[match redacted]" in result.stdout
    assert token not in result.stdout
    assert public_line not in result.stdout


def test_preflight_configuration_uses_current_fields() -> None:
    script = (DEPLOY / "preflight.sh").read_text()
    block = re.search(r'if "\$PY" - "\$CONFIG" <<\'PY\'\n(.*?)\nPY\n', script, re.S)
    assert block is not None
    result = subprocess.run(
        [sys.executable, "-", str(REPO / "config/robot.example.toml")],
        input=block[1],
        capture_output=True,
        text=True,
        env={**os.environ, "PYTHONPATH": str(REPO / "packages")},
    )
    assert result.returncode == 0, result.stderr
    assert "power_max=" in result.stdout
    assert 'if "$BRAIN_PY" - "$match"' in script
