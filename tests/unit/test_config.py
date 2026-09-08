"""``config/robot.toml`` (ARCHITECTURE 5.8 as amended by ADR-0013), A33.

The three shipped profiles load; a value over a compiled ceiling refuses
startup and is never clamped; and the cross-section rules that keep the host
able to observe what it commands hold.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from rover_contracts import (
    CEILINGS,
    POWER_CAP,
    ConfigError,
    LimitsConfig,
    LinkConfig,
    RobotConfig,
    RobotIdentity,
    SafetyConfig,
    Source,
    WelcomeMessage,
    load_config,
    server_adapter,
)
from rover_contracts.config import _SourceName

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
EXAMPLE = CONFIG_DIR / "robot.example.toml"
MAC = CONFIG_DIR / "robot.mac.toml"
LOCAL = CONFIG_DIR / "robot.toml"      # gitignored: THIS machine's; absent in CI

MINIMAL_TOML = """
[robot]
name = "rover"
[limits]
power_max = 0.30
budget_motion_s = 12
[link]
backend = "tcp"
tcp_port = 7777
[safety]
heartbeat_ms = 300
feedback_max_age_ms = 150
"""


def write_config(tmp_path: Path, text: str = MINIMAL_TOML) -> Path:
    path = tmp_path / "robot.toml"
    path.write_text(text)
    return path


def edited(source: Path, tmp_path: Path, **changes: str) -> Path:
    """A copy of ``source`` with each ``key = ...`` line changed."""
    text = source.read_text()
    for key, value in changes.items():
        new = re.sub(rf"(?m)^{key}\s*=\s*[^#\n]*", f"{key} = {value} ", text, count=1)
        assert new != text, f"{key} not found in {source.name}"
        text = new
    return write_config(tmp_path, text)


# --------------------------------------------------------------------------
# The shipped profiles
# --------------------------------------------------------------------------


@pytest.mark.parametrize("path", [EXAMPLE, MAC], ids=["example", "mac"])
def test_committed_profiles_load(path):
    assert isinstance(load_config(path, env={}), RobotConfig)


def test_local_robot_toml_loads_when_present():
    if not LOCAL.exists():
        pytest.skip("config/robot.toml is gitignored and absent here")
    assert isinstance(load_config(LOCAL, env={}), RobotConfig)


def test_pi_profile_is_the_serial_line_with_the_documented_values():
    config = load_config(EXAMPLE, env={})
    assert config.link == LinkConfig(backend="serial", port="/dev/serial0", baud=115200)
    assert config.robot == RobotIdentity(name="rover")
    assert config.limits == LimitsConfig()
    assert config.safety == SafetyConfig()
    assert config.safety.require_patched_firmware is True
    assert config.bus.allow_stream == []
    assert config.camera.backend == "picamera2"
    assert config.log.dir == "/data/logs"


def test_mac_profile_is_tcp_to_the_stub():
    config = load_config(MAC, env={})
    assert config.link == LinkConfig(backend="tcp", tcp_host="127.0.0.1", tcp_port=7777)
    assert config.camera.backend == "fake"
    assert config.tts.backend == "say"
    assert config.bus.allow_stream == ["teleop"]
    assert config.log.dir.startswith("./")


def test_simulation_runs_the_same_limits_and_safety_as_the_pi():
    # The gates assert against welcome.limits; a simulation that quietly ran
    # wider bounds would prove nothing.
    pi, mac = load_config(EXAMPLE, env={}), load_config(MAC, env={})
    assert mac.limits == pi.limits
    assert mac.safety == pi.safety
    assert mac.robot == pi.robot
    assert mac.battery == pi.battery


def test_no_profile_names_a_lan_address_or_a_secret():
    for path in (EXAMPLE, MAC):
        text = path.read_text()
        for host in re.findall(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b", text):
            assert host.startswith("127."), f"{path.name} names {host}"
        assert not re.search(r"(?m)^\s*\w*(_key|_token|_secret)\s*=", text), path.name


def test_welcome_carries_the_loaded_limits_and_safety():
    config = load_config(EXAMPLE, env={})
    welcome = WelcomeMessage(
        session="5f3c1a2b", robotd_version="0.2.0", rover_fw=None,
        limits=config.limits, safety=config.safety,
    )
    again = server_adapter.validate_json(server_adapter.dump_json(welcome))
    assert again == welcome
    assert again.limits.power_max == config.limits.power_max == 0.30


# --------------------------------------------------------------------------
# Ceilings refuse, never clamp (A33)
# --------------------------------------------------------------------------


@pytest.mark.parametrize("path", [EXAMPLE, MAC], ids=["example", "mac"])
def test_power_max_above_the_ceiling_refuses_startup(path, tmp_path):
    with pytest.raises(ConfigError) as excinfo:
        load_config(edited(path, tmp_path, power_max="0.31"), env={})
    assert "ceiling" in str(excinfo.value)
    assert "limits.power_max" in str(excinfo.value)


def test_power_max_override_above_the_ceiling_refuses_startup():
    # ROVER__LIMITS__POWER_MAX=0.35 must neither take effect nor be clamped.
    with pytest.raises(ConfigError) as excinfo:
        load_config(EXAMPLE, env={"ROVER__LIMITS__POWER_MAX": "0.35"})
    assert "ceiling" in str(excinfo.value)
    assert "limits.power_max" in str(excinfo.value)
    assert "environment" in str(excinfo.value)


@pytest.mark.parametrize(
    ("key", "value"),
    [("power_default", "0.31"), ("twist_power", "0.31"), ("drive_for_max_s", "2.5"),
     ("turn_timeout_max_s", "4.5"), ("budget_motion_s", "60"),
     ("goal_ttl_ms_max", "6000"), ("obs_max_age_ms", "5001"),
     ("heartbeat_ms", "301"), ("tof_stop_mm", "251"), ("low_battery_v", "10.0")],
)
def test_every_other_ceiling_refuses_too(key, value, tmp_path):
    with pytest.raises(ConfigError):
        load_config(edited(EXAMPLE, tmp_path, **{key: value}), env={})


def test_power_ceiling_is_the_firmware_cap():
    assert CEILINGS["limits.power_max"] == POWER_CAP == 0.30
    assert CEILINGS["limits.power_default"] == CEILINGS["limits.twist_power"] == POWER_CAP
    assert CEILINGS["safety.heartbeat_ms"] == 300
    assert CEILINGS["safety.feedback_max_age_ms"] == 150


def test_every_ceiling_names_a_real_key():
    config = RobotConfig()
    for dotted in CEILINGS:
        section, key = dotted.split(".")
        assert hasattr(getattr(config, section), key), dotted


def test_the_keys_without_a_ceiling_are_bounded_elsewhere():
    # power_min sits below power_default, turn_kp only reaches power_max sooner,
    # and require_patched_firmware is a switch.  Anything else added to
    # [limits] or [safety] needs a ceiling or a line here.
    without = {
        f"{section}.{key}"
        for section, model in (("limits", LimitsConfig), ("safety", SafetyConfig))
        for key in model.model_fields
    } - set(CEILINGS)
    assert without == {"limits.power_min", "limits.turn_kp",
                       "safety.require_patched_firmware"}


# --------------------------------------------------------------------------
# Cross-section rules: the host must be able to observe what it commands
# --------------------------------------------------------------------------


@pytest.mark.parametrize("age_ms", [300, 301, 5000])
def test_feedback_age_at_or_above_the_heartbeat_refuses_startup(age_ms, tmp_path):
    # T0 must trip before the firmware's own heartbeat does.
    path = edited(EXAMPLE, tmp_path, feedback_max_age_ms=str(age_ms))
    with pytest.raises(ConfigError) as excinfo:
        load_config(path, env={})
    assert "feedback_max_age_ms" in str(excinfo.value)
    assert "heartbeat_ms" in str(excinfo.value)


def test_lowering_the_heartbeat_onto_the_feedback_age_refuses_startup(tmp_path):
    # Both inside their ceilings (300 and 150): only the ordering rule is in play.
    path = edited(EXAMPLE, tmp_path, heartbeat_ms="150")
    with pytest.raises(ConfigError) as excinfo:
        load_config(path, env={})
    assert "feedback_max_age_ms" in str(excinfo.value)
    assert "heartbeat_ms" in str(excinfo.value)


def test_feedback_age_just_below_the_heartbeat_loads(tmp_path):
    path = edited(EXAMPLE, tmp_path, heartbeat_ms="151", feedback_max_age_ms="150")
    safety = load_config(path, env={}).safety
    assert (safety.heartbeat_ms, safety.feedback_max_age_ms) == (151, 150)


def test_command_rate_must_give_three_commands_per_heartbeat(tmp_path):
    with pytest.raises(ConfigError):
        load_config(edited(EXAMPLE, tmp_path, command_hz="5"), env={})
    config = load_config(edited(EXAMPLE, tmp_path, command_hz="10"), env={})
    assert config.link.command_hz == 10


def test_feedback_interval_must_be_below_the_feedback_age(tmp_path):
    with pytest.raises(ConfigError):
        load_config(edited(EXAMPLE, tmp_path, feedback_interval_ms="150"), env={})


@pytest.mark.parametrize(
    "extra",
    ["\n[limits]\npower_max = 0.25\npower_default = 0.26\n",
     "\n[limits]\npower_min = 0.20\npower_default = 0.20\n",
     "\n[limits]\npower_max = 0.25\ntwist_power = 0.30\n"],
    ids=["default_over_max", "min_not_below_default", "twist_over_max"],
)
def test_limits_must_be_ordered(extra, tmp_path):
    text = MINIMAL_TOML.replace("[limits]\npower_max = 0.30\nbudget_motion_s = 12\n", "")
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, text + extra), env={})


@pytest.mark.parametrize("yaw_sign", ["0", "2", "-2", '"1"'])
def test_yaw_sign_is_plus_or_minus_one(yaw_sign, tmp_path):
    with pytest.raises(ConfigError):
        load_config(edited(EXAMPLE, tmp_path, yaw_sign=yaw_sign), env={})


def test_yaw_sign_minus_one_is_a_valid_calibration(tmp_path):
    config = load_config(edited(EXAMPLE, tmp_path, yaw_sign="-1"), env={})
    assert config.link.yaw_sign == -1


# --------------------------------------------------------------------------
# The retired [serial] section is gone, not aliased
# --------------------------------------------------------------------------


def test_retired_serial_section_is_refused(tmp_path):
    text = MINIMAL_TOML + '\n[serial]\nbackend = "uart"\nport = "/dev/rover-mcu"\n'
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, text), env={})


@pytest.mark.parametrize("backend", ['"uart"', '"pty"', '"serial0"', "1"])
def test_retired_link_backends_are_refused(backend, tmp_path):
    with pytest.raises(ConfigError):
        load_config(edited(EXAMPLE, tmp_path, backend=backend), env={})


@pytest.mark.parametrize(
    "key",
    ["setpoint_hz", "cmd_gate_max_age_ms", "reseed_wait_ms", "link_alive_max_age_ms"],
)
def test_retired_link_and_safety_keys_are_refused(key, tmp_path):
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, MINIMAL_TOML + f"\n{key} = 1\n"), env={})


# --------------------------------------------------------------------------
# Overrides and the rest of A33
# --------------------------------------------------------------------------


def test_config_loads_with_documented_defaults(tmp_path):
    config = load_config(write_config(tmp_path), env={})
    assert config.limits == LimitsConfig()
    assert config.safety == SafetyConfig()
    assert config.link == LinkConfig(backend="tcp")
    assert config.camera.hfov_deg == 83.0
    assert config.bus.allow_stream == []


def test_env_override_applies_and_is_typed(tmp_path):
    config = load_config(
        write_config(tmp_path),
        env={"ROVER__LIMITS__POWER_DEFAULT": "0.25",
             "ROVER__LINK__BACKEND": '"serial"',
             "ROVER__LINK__YAW_SIGN": "-1",
             "ROVER__SAFETY__REQUIRE_PATCHED_FIRMWARE": "false",
             "ROVER__BUS__ALLOW_STREAM": '["teleop"]',
             "ROVER__CAMERA__BACKEND": '"fake"'},
    )
    assert config.limits.power_default == 0.25
    assert config.link.backend == "serial" and config.link.yaw_sign == -1
    assert config.safety.require_patched_firmware is False
    assert config.bus.allow_stream == ["teleop"]
    assert config.camera.backend == "fake"


def test_a_literal_secret_in_the_file_is_refused(tmp_path):
    path = write_config(tmp_path, MINIMAL_TOML + '\n[box]\napi_key = "sk-live"\n')
    with pytest.raises(ConfigError) as excinfo:
        load_config(path, env={})
    assert "secret" in str(excinfo.value)


def test_api_key_env_names_a_variable_and_is_allowed(tmp_path):
    path = write_config(
        tmp_path, MINIMAL_TOML + '\n[box]\napi_key_env = "ROVER_BOX_API_KEY"\n'
    )
    assert load_config(path, env={}).box.api_key_env == "ROVER_BOX_API_KEY"


def test_unknown_section_or_key_is_refused(tmp_path):
    with pytest.raises(ConfigError):
        path = write_config(tmp_path, MINIMAL_TOML + "\n[nonsense]\nx = 1\n")
        load_config(path, env={})
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path, MINIMAL_TOML + "\nspeed_mps = 0.3\n"), env={})


def test_malformed_override_name_is_refused(tmp_path):
    with pytest.raises(ConfigError):
        load_config(write_config(tmp_path), env={"ROVER__POWER": "1"})


def test_battery_table_must_be_a_table(tmp_path):
    path = write_config(
        tmp_path,
        MINIMAL_TOML + "\n[battery]\nocv_per_cell = [4.2, 3.9]\nsoc_pct = [100]\n",
    )
    with pytest.raises(ConfigError):
        load_config(path, env={})


def test_renaming_the_robot_without_a_wake_model_is_refused(tmp_path):
    # The wake model is trained for exactly one name (open item 11).
    pyopen = '\n[wake]\nbackend = "pyopen"\nmodel = "/data/models/wake/rover.tflite"\n'
    path = write_config(tmp_path, MINIMAL_TOML + pyopen)
    assert load_config(path, env={}).robot.name == "rover"
    with pytest.raises(ConfigError):
        load_config(path, env={"ROVER__ROBOT__NAME": "scout"})
    hotkey = write_config(tmp_path, MINIMAL_TOML + '\n[wake]\nbackend = "hotkey"\n')
    assert load_config(hotkey, env={"ROVER__ROBOT__NAME": "scout"}).robot.name == "scout"


def test_bus_source_names_match_the_message_source_enum():
    assert set(_SourceName.__args__) == {s.value for s in Source}
