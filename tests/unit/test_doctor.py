"""doctor names what is missing and what to do about it.

Every check is a fact about *this* host, so the assertions are about the shape
of the report -- a required dependency that is absent must be a ``fail`` with a
fix line, an optional one a ``warn`` -- and never about what happens to be
installed on the machine running the tests.
"""

from __future__ import annotations

import json
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_devtools.doctor import (  # noqa: E402
    DEFAULT_IMPORTS,
    FAIL,
    OK,
    WARN,
    Check,
    Dependency,
    main,
    report,
    run_checks,
)
from rover_devtools.mcu_sim import BINARY_ENV  # noqa: E402

ABSENT = Dependency(
    "rover_definitely_not_installed", "a module that cannot exist", True,
    "uv pip install -e packages/rover_contracts",
)
ABSENT_OPTIONAL = Dependency(
    "rover_optional_not_installed", "an optional module", False, "uv sync"
)
PRESENT = Dependency("json", "the standard library", True, "impossible")


@pytest.fixture()
def quiet_host(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """No simulator, no config, nothing on the network."""
    monkeypatch.delenv(BINARY_ENV, raising=False)
    monkeypatch.chdir(tmp_path)
    return tmp_path


def named(checks: list[Check], name: str) -> Check:
    matches = [check for check in checks if check.name == name]
    assert matches, f"no check named {name!r} in {[c.name for c in checks]}"
    return matches[0]


# --------------------------------------------------------------------------
# The import map
# --------------------------------------------------------------------------


def test_a_missing_required_dependency_fails_with_a_fix(quiet_host: Path) -> None:
    _, _, checks = run_checks(imports=(ABSENT,), probe_box=False)
    check = named(checks, f"import {ABSENT.module}")
    assert check.status == FAIL
    assert "not importable" in check.detail
    assert check.fix == ABSENT.fix


def test_a_missing_optional_dependency_only_warns(quiet_host: Path) -> None:
    _, _, checks = run_checks(imports=(ABSENT_OPTIONAL,), probe_box=False)
    assert named(checks, f"import {ABSENT_OPTIONAL.module}").status == WARN


def test_a_present_dependency_reports_where_it_came_from(quiet_host: Path) -> None:
    _, _, checks = run_checks(imports=(PRESENT,), probe_box=False)
    check = named(checks, "import json")
    assert check.status == OK
    assert "json" in check.detail


def test_a_missing_required_dependency_makes_the_exit_code_nonzero(
    quiet_host: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("rover_devtools.doctor.DEFAULT_IMPORTS", (ABSENT,))
    assert main(["--no-box"]) == 1


def test_the_default_import_map_covers_the_dependency_split(quiet_host: Path) -> None:
    modules = {dep.module for dep in DEFAULT_IMPORTS}
    assert {"rover_contracts", "pydantic"} <= modules
    assert {"picamera2", "sherpa_onnx", "sounddevice", "numpy"} <= modules
    # Only the two the tools themselves cannot run without are required.
    assert {dep.module for dep in DEFAULT_IMPORTS if dep.required} == {
        "rover_contracts",
        "pydantic",
    }


# --------------------------------------------------------------------------
# The rest of the report
# --------------------------------------------------------------------------


def test_a_missing_simulator_is_a_warn_naming_the_target(quiet_host: Path) -> None:
    _, _, checks = run_checks(imports=(), probe_box=False)
    check = named(checks, "mcu-sim")
    assert check.status == WARN
    assert "make sim" in check.fix


def test_an_unloadable_config_fails_and_says_so(quiet_host: Path) -> None:
    bad = quiet_host / "broken.toml"
    bad.write_text("[limits]\nspeed_mps = 3.0\n")
    _, origin, checks = run_checks(bad, imports=(), probe_box=False)
    check = named(checks, "config")
    assert check.status == FAIL
    assert "ceiling" in check.fix
    assert "did not load" in origin


def test_a_good_config_reports_its_safety_hash(quiet_host: Path) -> None:
    good = quiet_host / "robot.toml"
    good.write_text('[robot]\nname = "rover"\n')
    config, origin, checks = run_checks(good, imports=(), probe_box=False)
    check = named(checks, "config")
    assert check.status == OK
    assert str(config.safety_hash()) in check.detail
    assert origin.endswith("robot.toml")


def test_a_missing_socket_names_the_unit_that_owns_it(quiet_host: Path) -> None:
    _, _, checks = run_checks(imports=(), probe_box=False)
    assert named(checks, "socket robotd.sock").fix == "start rover-robotd"
    assert named(checks, "socket frames.sock").fix == "start rover-cam"
    assert named(checks, "socket brain.sock").fix == "start rover-brain"


def test_a_live_socket_is_ok(quiet_host: Path) -> None:
    # Bound relative to the chdir'ed tmp dir: an absolute pytest tmp path
    # overruns macOS's 104-byte sun_path.
    config_file = quiet_host / "robot.toml"
    config_file.write_text('[bus]\nsock = "robotd.sock"\n')
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind("robotd.sock")
    server.listen(1)
    try:
        _, _, checks = run_checks(config_file, imports=(), probe_box=False)
        assert named(checks, "socket robotd.sock").status == OK
    finally:
        server.close()


def test_a_missing_serial_device_points_at_the_simulator_in_pty_mode(
    quiet_host: Path,
) -> None:
    config_file = quiet_host / "robot.toml"
    config_file.write_text('[serial]\nbackend = "pty"\nport = "./run/mcu.pty"\n')
    _, _, checks = run_checks(config_file, imports=(), probe_box=False)
    check = named(checks, "serial device")
    assert check.status == WARN
    assert "rover_devtools.mcu_sim" in check.fix


def test_a_missing_serial_device_points_at_the_overlay_in_uart_mode(
    quiet_host: Path,
) -> None:
    _, _, checks = run_checks(imports=(), probe_box=False)
    assert "dtoverlay=uart5" in named(checks, "serial device").fix


def test_the_box_probe_is_skippable_and_never_prints_a_key(
    quiet_host: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ROVER_BOX_API_KEY", "sk-do-not-print-me")
    config, origin, checks = run_checks(imports=(), probe_box=False)
    assert named(checks, "box api key").status == OK
    assert "sk-do-not-print-me" not in report(config, origin, checks)


def test_the_report_ends_by_naming_the_first_thing_to_do(quiet_host: Path) -> None:
    config, origin, checks = run_checks(imports=(ABSENT,), probe_box=False)
    rendered = report(config, origin, checks)
    assert rendered.splitlines()[-1] == f"next: {ABSENT.fix}"
    assert "effective limits:" in rendered
    assert f"speed_mps={config.limits.speed_mps}" in rendered


def test_json_output_is_machine_readable(
    quiet_host: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["--no-box", "--json"]) in (0, 1)
    payload = json.loads(capsys.readouterr().out)
    assert payload["limits"]["speed_mps"] == 0.30
    assert payload["safety_hash"] == 3381018647
    assert {check["status"] for check in payload["checks"]} <= {OK, WARN, FAIL}
