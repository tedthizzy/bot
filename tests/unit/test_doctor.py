"""doctor names what is missing and what to do about it.

Every check is a fact about *this* host, so the assertions are about the shape
of the report -- a required dependency that is absent must be a ``fail`` with a
fix line, an optional one a ``warn`` -- and never about what happens to be
installed on the machine running the tests.
"""

from __future__ import annotations

import io
import json
import os
import pty
import socket
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.config import LinkConfig, RobotConfig, SafetyConfig  # noqa: E402
from rover_devtools import doctor  # noqa: E402
from rover_devtools.doctor import (  # noqa: E402
    DEFAULT_IMPORTS,
    FAIL,
    OK,
    WARN,
    Check,
    Dependency,
    fork_check,
    fork_constants,
    link_check,
    link_fix,
    link_timing,
    main,
    report,
    run_checks,
    stub_check,
)

ABSENT = Dependency(
    "rover_definitely_not_installed",
    "a module that cannot exist",
    True,
    "uv pip install -e .",
)
ABSENT_OPTIONAL = Dependency(
    "rover_optional_not_installed", "an optional module", False, "uv sync"
)
PRESENT = Dependency("json", "the standard library", True, "impossible")

FORK_OK = (
    "// the fork's constants\n#define BOT_HEARTBEAT_MS 300\n#define BOT_POWER_CAP 0.30f\n"
)


@pytest.fixture()
def quiet_host(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """No config, no fork, nothing on the network."""
    monkeypatch.chdir(tmp_path)
    return tmp_path


def named(checks: list[Check], name: str) -> Check:
    matches = [check for check in checks if check.name == name]
    assert matches, f"no check named {name!r} in {[c.name for c in checks]}"
    return matches[0]


@pytest.mark.parametrize("key", [None, "test-doctor-credential"])
def test_box_probe_uses_configured_credentials_without_reporting_them(
    key: str | None,
    monkeypatch: pytest.MonkeyPatch,
    quiet_host: Path,
) -> None:
    config = RobotConfig()
    monkeypatch.delenv(config.box.api_key_env, raising=False)
    if key:
        monkeypatch.setenv(config.box.api_key_env, key)

    def respond(request, *, timeout):
        assert timeout == 2.0
        assert request.full_url == config.box.url.rstrip("/") + "/models"
        assert request.get_header("Authorization") == (f"Bearer {key}" if key else None)
        return io.BytesIO(json.dumps({"data": [{"id": config.box.model}]}).encode())

    monkeypatch.setattr(doctor.urllib.request, "urlopen", respond)
    checks = doctor._box_check(config, probe=True)
    assert named(checks, "box").status == OK
    assert not key or key not in repr(checks)


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
# The simulator and the link
# --------------------------------------------------------------------------


def test_the_stub_is_found_as_a_script_or_a_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(doctor.shutil, "which", lambda name: "/opt/rover/bin/rover-stub")
    check = stub_check()
    assert check.status == OK and "console script" in check.detail

    monkeypatch.setattr(doctor.shutil, "which", lambda name: None)
    monkeypatch.setattr(doctor.importlib.util, "find_spec", lambda name: None)
    check = stub_check()
    assert check.status == WARN
    assert "rover-stub --pty" in check.fix


def test_a_missing_serial_port_points_at_the_uart_or_the_stub(quiet_host: Path) -> None:
    assert "enable_uart=1" in link_fix("/dev/serial0")
    assert "enable_uart=1" in link_fix("/dev/ttyAMA0")
    assert link_fix("./run/rover.pty").startswith("start the simulator: rover-stub --pty")

    config_file = quiet_host / "robot.toml"
    config_file.write_text('[link]\nport = "./run/rover.pty"\n')
    _, _, checks = run_checks(config_file, imports=(), probe_box=False)
    check = named(checks, "link")
    assert check.status == WARN
    assert "does not exist" in check.detail
    assert "rover-stub --pty" in check.fix


def test_a_readable_pty_is_an_ok_link(quiet_host: Path) -> None:
    master, slave = pty.openpty()
    try:
        config = RobotConfig(link=LinkConfig(port=os.ttyname(slave)))
        check = link_check(config)
        assert check.status == OK
        assert "char device" in check.detail
    finally:
        os.close(slave)
        os.close(master)


def test_a_regular_file_is_not_a_link(quiet_host: Path) -> None:
    not_a_tty = quiet_host / "capture.txt"
    not_a_tty.write_text("")
    check = link_check(RobotConfig(link=LinkConfig(port=str(not_a_tty))))
    assert check.status == WARN
    assert "not a character device" in check.detail


def test_a_tcp_link_is_probed(quiet_host: Path) -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    try:
        live = RobotConfig(link=LinkConfig(backend="tcp", tcp_port=port))
        assert link_check(live).status == OK
    finally:
        listener.close()
    dead = RobotConfig(link=LinkConfig(backend="tcp", tcp_port=port))
    check = link_check(dead)
    assert check.status == WARN
    assert f"rover-stub --tcp {port}" in check.fix


def test_link_timing_orders_the_three_periods() -> None:
    check = link_timing(LinkConfig(), SafetyConfig())
    assert check.status == OK
    assert "20 Hz = 50 ms/command, x3 = 150 <= heartbeat 300 ms" in check.detail
    assert "feedback 50 < max_age 150 < heartbeat 300" in check.detail

    too_slow = link_timing(LinkConfig(command_hz=5), SafetyConfig())
    assert too_slow.status == FAIL
    assert "x3 = 600 > heartbeat_ms 300" in too_slow.detail

    stale_late = link_timing(LinkConfig(), SafetyConfig(feedback_max_age_ms=400))
    assert stale_late.status == FAIL
    assert "feedback_max_age_ms 400 >= heartbeat_ms 300" in stale_late.detail

    interval = link_timing(LinkConfig(feedback_interval_ms=200), SafetyConfig())
    assert interval.status == FAIL
    assert "feedback_interval_ms 200 >= feedback_max_age_ms 150" in interval.detail


# --------------------------------------------------------------------------
# The firmware fork
# --------------------------------------------------------------------------


def test_fork_constants_reads_the_usual_spellings() -> None:
    assert fork_constants(FORK_OK) == {"BOT_HEARTBEAT_MS": 300.0, "BOT_POWER_CAP": 0.3}
    spelled = "#define BOT_POWER_CAP (0.3)\n# define BOT_HEARTBEAT_MS 300u\n"
    assert fork_constants(spelled) == {"BOT_HEARTBEAT_MS": 300.0, "BOT_POWER_CAP": 0.3}
    assert fork_constants("// #define BOT_POWER_CAP 0.5\nint x;\n") == {}


def test_a_missing_fork_directory_is_reported_not_failed(tmp_path: Path) -> None:
    header = tmp_path / "firmware" / "General_Driver" / "bot_config.h"
    check = fork_check(RobotConfig(), header)
    assert check.status == WARN
    assert "firmware fork not present" in check.detail
    assert "does not exist" in check.detail

    header.parent.mkdir(parents=True)
    check = fork_check(RobotConfig(), header)
    assert check.status == WARN
    assert "firmware fork not present" in check.detail
    assert "missing" in check.detail


def test_a_matching_fork_is_ok(tmp_path: Path) -> None:
    header = tmp_path / "bot_config.h"
    header.write_text(FORK_OK)
    check = fork_check(RobotConfig(), header)
    assert check.status == OK
    assert "BOT_HEARTBEAT_MS=300 vs [safety] heartbeat_ms=300" in check.detail
    assert "BOT_POWER_CAP=0.3 vs ceiling 0.3" in check.detail


def test_a_mismatched_heartbeat_fails_naming_both_numbers(tmp_path: Path) -> None:
    header = tmp_path / "bot_config.h"
    header.write_text("#define BOT_HEARTBEAT_MS 250\n#define BOT_POWER_CAP 0.30f\n")
    check = fork_check(RobotConfig(), header)
    assert check.status == FAIL
    assert "BOT_HEARTBEAT_MS=250 vs [safety] heartbeat_ms=300" in check.detail
    assert "refuses motion" in check.fix

    header.write_text("#define BOT_HEARTBEAT_MS 300\n#define BOT_POWER_CAP 0.5f\n")
    assert fork_check(RobotConfig(), header).status == FAIL


def test_a_fork_without_the_constants_fails(tmp_path: Path) -> None:
    header = tmp_path / "bot_config.h"
    header.write_text("#define SOMETHING_ELSE 1\n")
    check = fork_check(RobotConfig(), header)
    assert check.status == FAIL
    assert "BOT_HEARTBEAT_MS, BOT_POWER_CAP" in check.detail


def test_run_checks_takes_the_fork_header(quiet_host: Path) -> None:
    header = quiet_host / "bot_config.h"
    header.write_text(FORK_OK)
    _, _, checks = run_checks(imports=(), probe_box=False, fork_header=header)
    assert named(checks, "firmware fork").status == OK


# --------------------------------------------------------------------------
# The rest of the report
# --------------------------------------------------------------------------


def test_an_unloadable_config_fails_and_says_so(quiet_host: Path) -> None:
    bad = quiet_host / "broken.toml"
    bad.write_text("[limits]\npower_max = 3.0\n")
    _, origin, checks = run_checks(bad, imports=(), probe_box=False)
    check = named(checks, "config")
    assert check.status == FAIL
    assert "ceiling" in check.fix
    assert "did not load" in origin


def test_a_good_config_reports_its_link(quiet_host: Path) -> None:
    good = quiet_host / "robot.toml"
    good.write_text('[robot]\nname = "rover"\n')
    config, origin, checks = run_checks(good, imports=(), probe_box=False)
    check = named(checks, "config")
    assert check.status == OK
    assert "link.backend=serial port=/dev/serial0" in check.detail
    assert origin.endswith("robot.toml")
    assert config.safety.heartbeat_ms == 300


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
    assert f"power_max={config.limits.power_max}" in rendered
    assert f"heartbeat_ms={config.safety.heartbeat_ms}" in rendered


def test_json_output_is_machine_readable(
    quiet_host: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["--no-box", "--json"]) in (0, 1)
    payload = json.loads(capsys.readouterr().out)
    assert payload["limits"]["power_max"] == 0.30
    assert payload["safety"]["heartbeat_ms"] == 300
    assert payload["link"]["command_hz"] == 20
    assert "safety_hash" not in payload
    assert {check["status"] for check in payload["checks"]} <= {OK, WARN, FAIL}
    names = [check["name"] for check in payload["checks"]]
    assert {"rover-stub", "link", "link timing", "firmware fork"} <= set(names)
