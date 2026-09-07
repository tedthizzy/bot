"""The drift alarm between the operational bounds and the controller's caps.

Principle 1 splits the job three ways: the model proposes, robotd validates, the
MCU decides whether motion is safe.  That only holds while every bound robotd
enforces sits *inside* a cap the MCU compiled in -- otherwise robotd would be
the last line, and I-4's "compiled caps cannot be raised by any frame" would be
asserting against nothing.

This test reads ``firmware/core/rover_config.h`` and checks that claim from the
Python side.  It fails loudly, naming the bound and both numbers, because the
two layers are written by different people against one document.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.skills import CATALOG, mcu_cap_magnitude  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "firmware" / "core" / "rover_config.h"

_DEFINE = re.compile(
    r"^\s*#\s*define\s+([A-Za-z_]\w*)\s+\(?\s*([+-]?\d+)\s*[uUlL]*\s*\)?\s*(?://.*)?$",
    re.MULTILINE,
)
_ASSIGN = re.compile(
    r"\b([A-Za-z_]\w*)\s*=\s*\(?\s*([+-]?\d+)\s*[uUlL]*\s*\)?\s*[,;]"
)


def parse_caps(text: str) -> dict[str, int]:
    """Every integer constant the header defines, by name.

    Handles ``#define NAME 300``, ``static const int NAME = 300;`` and an
    ``enum { NAME = 300, }``, which is the whole vocabulary a freestanding C11
    header needs for a cap.
    """
    caps: dict[str, int] = {}
    for name, value in _DEFINE.findall(text):
        caps[name] = int(value)
    for name, value in _ASSIGN.findall(text):
        caps.setdefault(name, int(value))
    return caps


def lookup(caps: dict[str, int], wanted: str) -> int | None:
    """Find a cap, tolerating an added or missing ``ROVER_`` prefix."""
    for candidate in (wanted, wanted.removeprefix("ROVER_"), f"ROVER_{wanted}"):
        if candidate in caps:
            return caps[candidate]
    return None


BOUNDS = [
    (spec.name, bound)
    for spec in CATALOG
    for bound in spec.bounds
    if bound.mcu_cap is not None
]


def test_the_catalog_names_the_caps_it_depends_on():
    # Independent of the firmware: a motion bound with no named controller cap
    # would be enforced on the Pi alone, which principle 1 does not allow.
    named = {bound.mcu_cap for _, bound in BOUNDS}
    assert named == {"ROVER_MAX_V_MM_S", "ROVER_MAX_W_MRAD_S"}
    moving = {spec.name for spec in CATALOG if spec.moves}
    covered = {skill for skill, _ in BOUNDS}
    assert covered == moving


@pytest.mark.parametrize(
    ("skill", "bound"), BOUNDS, ids=[f"{s}.{b.field}" for s, b in BOUNDS]
)
def test_operational_bound_is_inside_its_controller_cap(skill, bound):
    if not HEADER.exists():
        pytest.skip(
            f"{HEADER} does not exist yet; the drift alarm arms itself as soon as "
            "the firmware core lands"
        )
    caps = parse_caps(HEADER.read_text())
    compiled = lookup(caps, bound.mcu_cap)
    assert compiled is not None, (
        f"{HEADER} defines no cap named {bound.mcu_cap} for {skill}.{bound.field}. "
        f"It defines: {sorted(caps)}"
    )
    operational = mcu_cap_magnitude(bound)
    assert operational <= compiled, (
        f"{skill}.{bound.field} is bounded at {operational} "
        f"(from {bound.lo}..{bound.hi} {bound.unit.value}) but the controller cap "
        f"{bound.mcu_cap} is {compiled}. The operational bound must sit inside the "
        "compiled cap: raise the firmware cap or lower the bound in "
        "packages/rover_contracts/skills.py, but they must not disagree."
    )


def test_bound_conversion_to_controller_units():
    # The comparison is only as good as the conversion, so pin it: ARCHITECTURE 6
    # states 0.30 m/s and 1.047 rad/s, and 6's turn row states the same angular
    # cap as 60 deg/s.
    by_field = {bound.field: bound for _, bound in BOUNDS}
    assert mcu_cap_magnitude(by_field["speed_mps"]) == 300
    assert mcu_cap_magnitude(by_field["rate_dps"]) == 1047
    assert mcu_cap_magnitude(by_field["linear_x_mps"]) == 300
    assert mcu_cap_magnitude(by_field["angular_z_radps"]) == 1047


def test_parser_reads_the_shapes_a_c_header_uses():
    text = """
    #define ROVER_MAX_V_MM_S      300
    #define ROVER_MAX_W_MRAD_S   (1200)   // +z up = CCW
    static const int ROVER_TOF_STOP_MM = 250;
    enum { ROVER_FRAME_TTL_MAX_MS = 500, };
    """
    caps = parse_caps(text)
    assert caps["ROVER_MAX_V_MM_S"] == 300
    assert caps["ROVER_MAX_W_MRAD_S"] == 1200
    assert caps["ROVER_TOF_STOP_MM"] == 250
    assert caps["ROVER_FRAME_TTL_MAX_MS"] == 500
    assert lookup(caps, "MAX_V_MM_S") == 300


def test_a_narrower_controller_cap_is_caught():
    # The alarm itself must work, or its silence means nothing.
    caps = {"ROVER_MAX_V_MM_S": 200}
    bound = next(b for _, b in BOUNDS if b.field == "speed_mps")
    compiled = lookup(caps, bound.mcu_cap)
    assert mcu_cap_magnitude(bound) > compiled


# ---------------------------------------------------------------------------
# The two sides of T.rx_drop
# ---------------------------------------------------------------------------

CODEC_C = REPO / "firmware" / "core" / "rover_codec.c"

_RX_DROPPED = re.compile(
    r"uint32_t\s+rover_rx_dropped\s*\([^)]*\)\s*\{(.*?)\n\}", re.S
)


def test_rx_drop_sums_the_same_wire_events_on_both_sides():
    """``rover_codec.c``'s own comment says the two sides "have to increment the
    same RxCounters field for the same wire event, or T.rx_drop and robotd's
    link stats cannot be compared".  Python omitted ``stale_seq``, so a
    duplicate or replayed down-frame burst raised the MCU's counter and nothing
    on the Pi -- and A8's list of what is dropped and counted names seq."""
    from rover_contracts.serial_codec import RxCounters  # noqa: PLC0415

    body = _RX_DROPPED.search(CODEC_C.read_text())
    assert body is not None, "rover_rx_dropped() not found in rover_codec.c"
    from_c = set(re.findall(r"c->(\w+)", body.group(1)))

    counters = RxCounters()
    from_python = set()
    for name in vars(counters):
        if name in ("ok", "overlong"):
            continue  # ok is not a drop; overlong is already counted as bad_length
        before = counters.dropped
        setattr(counters, name, getattr(counters, name) + 1)
        if counters.dropped == before + 1:
            from_python.add(name)

    assert from_c == from_python, (
        f"C sums {sorted(from_c)}; Python sums {sorted(from_python)}"
    )
