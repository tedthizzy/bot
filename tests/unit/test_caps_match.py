"""The drift alarm between the host's caps and the firmware fork's constants.

``firmware/General_Driver/bot_config.h`` is what the controller compiles in.
The host restates four of those numbers -- the power cap, the heartbeat, the
time-of-flight stop distance and the low-battery cut-off -- as ceilings, bounds
and defaults.  A host bound wider than the firmware's would have robotd send
values the controller clamps without the host knowing, and a clamp the host did
not expect is a clamp it cannot report.  This test reads the header and compares.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from rover_contracts import CATALOG, CEILINGS, POWER_CAP, TWIST_BOUNDS, SafetyConfig

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "firmware" / "General_Driver" / "bot_config.h"

# `#define BOT_POWER_CAP 0.30f`: optional parentheses, a C suffix, a comment.
# Decimal only; a hex literal such as `0x29` is not a number we compare.
_DEFINE = re.compile(
    r"^[ \t]*#[ \t]*define[ \t]+(BOT_[A-Z0-9_]+)[ \t]+\(?[ \t]*"
    r"([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)(?![xX\d.])"
    r"[ \t]*[fFuUlL]*[ \t]*\)?",
    re.MULTILINE,
)


def parse_defines(text: str) -> dict[str, float]:
    return {name: float(value) for name, value in _DEFINE.findall(text)}


@pytest.fixture(scope="module")
def fw() -> dict[str, float]:
    if not HEADER.exists():
        pytest.skip(
            f"{HEADER.relative_to(REPO)} is not written yet; these tests compare "
            "the host's caps against the firmware fork and have nothing to read"
        )
    defines = parse_defines(HEADER.read_text(encoding="utf-8"))
    if not defines:
        pytest.fail(
            f"{HEADER.relative_to(REPO)} exists but defines no numeric BOT_* constant"
        )
    return defines


def constant(fw: dict[str, float], name: str) -> float:
    if name not in fw:
        pytest.fail(
            f"{HEADER.name} does not define {name}; the host restates it and has "
            "nothing to compare against"
        )
    return fw[name]


FW_BOUNDS = [(spec.name, b) for spec in CATALOG for b in spec.bounds if b.fw_cap]
FW_BOUNDS += [("twist", b) for b in TWIST_BOUNDS if b.fw_cap]


def test_parser_reads_the_lines_the_fork_writes():
    sample = (
        "#define BOT_POWER_CAP 0.30f   // sixty percent duty\n"
        "#define BOT_HEARTBEAT_MS 300\n"
        "#  define BOT_TOF_STOP_MM (250)\n"
        "#define BOT_LOWBAT_V 9.9f\n"
        '#define BOT_FW_TAG "bot-wr-1"\n'
        "#define BOT_RADIOS_OFF\n"
        "#define BOT_TOF_ADDR 0x29\n"
    )
    assert parse_defines(sample) == {
        "BOT_POWER_CAP": 0.30, "BOT_HEARTBEAT_MS": 300,
        "BOT_TOF_STOP_MM": 250, "BOT_LOWBAT_V": 9.9,
    }


def test_the_alarm_covers_the_three_power_bounds():
    assert {(skill, b.field) for skill, b in FW_BOUNDS} == {
        ("drive_for", "power"), ("twist", "lin"), ("twist", "ang")
    }


@pytest.mark.parametrize(
    ("skill", "bound"), FW_BOUNDS, ids=[f"{s}.{b.field}" for s, b in FW_BOUNDS]
)
def test_every_power_bound_sits_inside_the_firmware_cap(fw, skill, bound):
    cap = constant(fw, bound.fw_cap)
    assert abs(bound.hi) <= cap and abs(bound.lo) <= cap, (
        f"{skill}.{bound.field} spans [{bound.lo}, {bound.hi}] {bound.unit.value} "
        f"but {bound.fw_cap} = {cap} in {HEADER.name}"
    )


def test_power_ceiling_is_the_firmware_cap(fw):
    cap = constant(fw, "BOT_POWER_CAP")
    assert CEILINGS["limits.power_max"] == cap, (
        f"CEILINGS['limits.power_max'] = {CEILINGS['limits.power_max']} "
        f"but BOT_POWER_CAP = {cap}"
    )
    assert cap == POWER_CAP, (
        f"wave_proto.POWER_CAP = {POWER_CAP} but BOT_POWER_CAP = {cap}"
    )


def test_heartbeat_ceiling_is_the_firmware_heartbeat(fw):
    hb_ms = constant(fw, "BOT_HEARTBEAT_MS")
    assert CEILINGS["safety.heartbeat_ms"] == hb_ms, (
        f"CEILINGS['safety.heartbeat_ms'] = {CEILINGS['safety.heartbeat_ms']} "
        f"but BOT_HEARTBEAT_MS = {hb_ms}"
    )
    assert SafetyConfig().heartbeat_ms == hb_ms, (
        f"SafetyConfig().heartbeat_ms = {SafetyConfig().heartbeat_ms} "
        f"but BOT_HEARTBEAT_MS = {hb_ms}"
    )


@pytest.mark.parametrize(
    ("key", "name"),
    [("tof_stop_mm", "BOT_TOF_STOP_MM"), ("low_battery_v", "BOT_LOWBAT_V")],
)
def test_default_safety_mirrors_match_the_firmware(fw, key, name):
    value = constant(fw, name)
    default = getattr(SafetyConfig(), key)
    assert default == value, f"SafetyConfig().{key} = {default} but {name} = {value}"
    assert CEILINGS[f"safety.{key}"] == value, (
        f"CEILINGS['safety.{key}'] = {CEILINGS[f'safety.{key}']} but {name} = {value}"
    )
