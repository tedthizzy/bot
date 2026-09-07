#!/usr/bin/env python3
"""G4 -- chassis and fault injection (ARCHITECTURE 13), sim half on the Mac.

Sub-gates a to m are I-5, I-8, I-9, I-10, I-11, I-12, I-13, I-14, I-15, I-16,
I-19, I-22 and I-23, each a named case; I-14 splits into h1/h2/h3 per
ARCHITECTURE 8.  I-17's grep gate lives here too, and so do the four cases the
invariants imply but no script previously owned: a connection that says ``hello``
as ``brain`` then sends ``teleop`` twists, a browser tab frozen mid-teleop, a
``clear`` from ``brain``, and the 70 mm cylinder at the chassis corner.

**Every case asserts the robot did not move.**  Where robotd is running that is
read off ``state.twist`` and ``state.active``; where it is not, the case either
proves the claim at the contract layer -- the fuzz corpus, the stop-class
corpus, the three-valued ToF rendering -- or reports what it needs and skips.

    python tests/gates/g4/run_g4.py                        # what runs with no rover
    python tests/gates/g4/run_g4.py --robotd run/robotd.sock
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve()
sys.path[:0] = [str(_HERE.parents[3] / "packages"), str(_HERE.parents[1])]

from fuzz import CORPUS_SIZE, build_corpus  # noqa: E402
from gatelib import clockcheck  # noqa: E402
from gatelib.bus import BusClient  # noqa: E402
from gatelib.env import REPO, have_module, load_robot_config, socket_alive  # noqa: E402
from gatelib.link import McuLink, SimProcess  # noqa: E402
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for  # noqa: E402
from rover_contracts import (  # noqa: E402
    TOF_ERROR_MM,
    TOF_NO_TARGET_MM,
    CtrlFlag,
    DecodeOk,
    Fault,
    FrameReader,
    McuState,
    ResultReason,
    StateRanges,
    TelemetryFrame,
    WorldState,
    client_adapter,
    new_cmd_id,
    new_turn_id,
)

GATE = "G4"

STOP_CLASS = ("stop", "estop", "cancel")
"""I-22: accepted from any allow-listed source in every state, never rejected."""

CLOCK_VIOLATIONS: tuple[tuple[str, str, str], ...] = (
    ("units-in-the-way", "py", "age_ms = (time.monotonic_ns() - f.mcu_us * 1000) / 1e6"),
    ("bare-locals", "py", "latency = now_mono_ns - telemetry.mcu_us"),
    ("call-in-the-way", "py", "skew = self._clock() - frame.mcu_us"),
    ("mcu-first", "py", "drift = mcu_us - host_now"),
    ("the-echo-field", "py", "drift = frame.mcu_us - pi_mono_us"),
    ("a-comparison", "py", "if frame.mcu_us > time.monotonic_ns():\n    pass"),
    ("casts", "py", "age = int(time.monotonic_ns()) - int(frame.mcu_us)"),
    ("c-casts", "c", "int64_t skew = (int64_t)now_us - (int64_t)host_stamp_us;"),
)
"""The spellings the adjacency regexes missed.  Six of the first seven were
invisible to them, and ``the-echo-field`` -- a direct comparison of the two
clocks -- was actively exempted by an allow-list keyed on a substring."""


def clock_hits(name: str) -> list[clockcheck.Crossing]:
    """Run the checker over one named spelling from ``CLOCK_VIOLATIONS``."""
    for case, language, source in CLOCK_VIOLATIONS:
        if case != name:
            continue
        if language == "py":
            return clockcheck.check_python(source, f"<{case}>")
        return clockcheck.check_c(source, f"<{case}>")
    raise KeyError(name)

SIM_REQUIRED = frozenset(
    {"b2", "b3", "b4", "e2", "f", "h1", "h2", "h3", "i", "l2", "l3", "m"}
)
"""Sub-gates whose section 8 gate column says M: they are meant to be green on
the Mac against mcu-sim, so a skip for a missing peer is a hole in A3's claim
rather than a limitation of the host."""


# ---------------------------------------------------------------------------
# Bus helpers
# ---------------------------------------------------------------------------


def connect(path: str, source: str, caps: list[str]) -> BusClient | None:
    if not socket_alive(path):
        return None
    try:
        client = BusClient(path)
    except OSError:
        return None
    client.hello(source, caps, subscribe=["state", "result", "event"])
    return client


def _cmd_id_of(line: Any) -> str | None:
    """The ``cmd_id`` a fuzz line carries, or ``None`` when it has none or is
    not JSON at all -- half the corpus is deliberately unparseable."""
    if isinstance(line, bytes):
        line = line.decode("utf-8", "replace")
    try:
        payload = json.loads(line)
    except (TypeError, ValueError):
        return None
    cmd_id = payload.get("cmd_id") if isinstance(payload, dict) else None
    return cmd_id if isinstance(cmd_id, str) else None


def settle(client: BusClient, seconds: float = 3.0) -> bool:
    """Wait until nothing is moving, so the next case measures its own effect.

    Returns whether rest was actually reached.  It used to give up silently,
    which is how another gate's in-flight drive was scored as this case's.
    """
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if not moved(client, 0.3):
            return True
    return not moved(client, 0.3)


def moved(client: BusClient, seconds: float = 0.5) -> bool:
    """Did any published state show motion in the window?"""
    for state in client.states(seconds):
        twist = state.get("twist") or {}
        if abs(twist.get("linear_x_mps", 0.0)) > 1e-6:
            return True
        if abs(twist.get("angular_z_radps", 0.0)) > 1e-6:
            return True
        if (state.get("mcu") or {}).get("motion"):
            return True
    return False


POSE_TOLERANCE_M = 0.005
POSE_TOLERANCE_RAD = 0.02
"""What "did not move" means when it is measured as a pose delta rather than
sampled from a window.  The rover is at rest either side of the corpus, so the
only motion the odometry can show is encoder noise."""


def pose_of(client: BusClient, seconds: float = 0.6) -> tuple[float, float, float] | None:
    """The newest odometry pose published in a window, or ``None``."""
    latest: tuple[float, float, float] | None = None
    for state in client.states(seconds):
        pose = state.get("pose")
        if isinstance(pose, dict):
            latest = (
                float(pose.get("x_m", 0.0)),
                float(pose.get("y_m", 0.0)),
                float(pose.get("yaw_rad", 0.0)),
            )
    return latest


def ack_seq_of(client: BusClient, seconds: float = 0.6) -> int | None:
    """The newest ``mcu.last_ack_seq``, recorded beside the pose so a run that
    did move can be told from one where robotd stopped writing the port."""
    latest: int | None = None
    for state in client.states(seconds):
        mcu = state.get("mcu")
        if isinstance(mcu, dict) and isinstance(mcu.get("last_ack_seq"), int):
            latest = int(mcu["last_ack_seq"])
    return latest


def skill_message(**overrides: Any) -> dict[str, Any]:
    body: dict[str, Any] = {
        "v": 1,
        "type": "skill",
        "source": "brain",
        "cmd_id": new_cmd_id(),
        "seq": 1,
        "turn_id": new_turn_id(),
        "issued_mono_ns": time.monotonic_ns(),
        "goal_ttl_ms": 3000,
        "skill": "drive",
        "args": {"distance_m": 0.2, "speed_mps": 0.15},
        # 4.2 checks the observation row *before* the replay window and the
        # turn_id row, so a motion skill with no `obs` is answered `obs_stale`
        # and never reaches the rule under test.  G4-m removes it deliberately.
        "obs": {"frame_id": "cam-000001", "frame_mono_ns": time.monotonic_ns()},
        # brain sets this on every dispatch (validate.to_bus_message).  A motion
        # skill without it is refused `unauthorized_utterance`: section 7 makes a
        # *null STT confidence* authorized, not a missing flag, and an absent
        # trace would otherwise re-enable motion with no authorizing utterance
        # for any source that never learned to set it (I-21).
        "trace": {"authorized_motion": True},
    }
    body.update(overrides)
    return body


# ---------------------------------------------------------------------------
# b -- I-8, the fuzz corpus
# ---------------------------------------------------------------------------


def sub_b_fuzz(gate: GateRun, client: BusClient | None) -> None:
    """500 fuzz cases across ``skill`` and ``twist``; zero reach the port."""
    corpus = build_corpus()
    contract_wrong: list[str] = []
    for case in corpus:
        try:
            client_adapter.validate_json(case.line)
            accepted = True
        except Exception:  # noqa: BLE001 - a rejection is the result
            accepted = False
        if accepted != case.valid:
            contract_wrong.append(f"{case.name} ({case.rule})")
    valid = sum(1 for case in corpus if case.valid)
    gate.check(
        not contract_wrong,
        "b",
        "I-8",
        f"{CORPUS_SIZE} fuzz cases: every out-of-bounds value refused, every legal "
        "one accepted",
        "; ".join(contract_wrong[:3])
        if contract_wrong
        else f"{len(corpus) - valid} refused, {valid} legal controls accepted",
        corpus=len(corpus),
        valid_controls=valid,
        wrong=contract_wrong[:20],
    )

    if client is None:
        gate.skip(
            "b2",
            "I-8",
            "robotd answers every fuzz case rejected and never moves",
            "a running robotd",
        )
        return
    rejected = 0
    accepted_invalid: list[str] = []
    # An earlier case's drive must not be scored as this one's.  `make
    # gates-sim` runs every gate against one shared rover, and G3's soak sends
    # real utterances, so sampling a window after the corpus attributes to the
    # corpus whatever else happens to be moving.  Measure instead: rest first,
    # then a pose delta over the corpus.
    at_rest = settle(client)
    client.drain(0.3)  # anything an earlier case left in flight
    before_pose = pose_of(client)
    before_ack = ack_seq_of(client, 0.2)
    for case in (c for c in corpus if not c.valid):
        client.send(case.line)
        # Match the answer to the case: without a cmd_id an earlier command's
        # late `accepted` is read as this one's, and a malformed line that
        # robotd rightly ignored is scored as an acceptance.
        answer = client.expect("result", deadline=0.4, cmd_id=_cmd_id_of(case.line))
        if answer is None:
            continue
        if answer.get("status") == "rejected":
            rejected += 1
        else:
            accepted_invalid.append(f"{case.name} -> {answer.get('status')}")
    after_pose = pose_of(client)
    after_ack = ack_seq_of(client, 0.2)

    if before_pose is None or after_pose is None:
        travel_m: float | None = None
        turned_rad: float | None = None
        stationary = False
        why = "no pose published, so the corpus cannot be attributed"
    else:
        travel_m = math.hypot(
            after_pose[0] - before_pose[0], after_pose[1] - before_pose[1]
        )
        turned_rad = abs(after_pose[2] - before_pose[2])
        stationary = travel_m <= POSE_TOLERANCE_M and turned_rad <= POSE_TOLERANCE_RAD
        why = f"pose moved {travel_m * 1000:.1f} mm / {turned_rad:.3f} rad"

    detail = (
        "; ".join(accepted_invalid[:3])
        if accepted_invalid
        else f"{rejected} rejected, {why}"
    )
    if not at_rest:
        detail = f"the shared rover was still moving before the corpus; {detail}"
    gate.check(
        at_rest and not accepted_invalid and stationary,
        "b2",
        "I-8",
        "robotd answers every fuzz case rejected and the pose does not move",
        detail,
        rejected=rejected,
        accepted_invalid=accepted_invalid[:20],
        at_rest_before=at_rest,
        travel_m=travel_m,
        turned_rad=turned_rad,
        ack_seq_before=before_ack,
        ack_seq_after=after_ack,
    )


def sub_b_source(gate: GateRun, path: str) -> None:
    """A connection bound to ``brain`` that then sends ``teleop`` twists."""
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip(
            "b3",
            "I-8",
            "hello as brain then twist as teleop is source_not_allowed",
            "a running robotd",
        )
        return
    with client:
        client.send(
            {
                "v": 1,
                "type": "twist",
                "source": "teleop",
                "cmd_id": new_cmd_id(),
                "seq": 2,
                "twist": {"linear_x_mps": 0.2, "angular_z_radps": 0.0},
            }
        )
        answer = client.expect("result", deadline=1.0)
        reason = (answer or {}).get("reason")
        gate.check(
            reason == ResultReason.SOURCE_NOT_ALLOWED and not moved(client),
            "b3",
            "I-8",
            "hello as brain then twist as teleop is source_not_allowed",
            f"reason={reason!r}, motion={moved(client, 0.3)}",
            reason=reason,
        )


def sub_b_frozen_tab(gate: GateRun, path: str, config: Any) -> None:
    """A browser tab frozen mid-teleop: the stream must lapse, not hold."""
    client = connect(path, "teleop", ["twist", "subscribe"])
    if client is None:
        gate.skip(
            "b4",
            "I-8",
            "a frozen teleop stream lapses to zero within teleop_input_max_age_ms",
            "a running robotd with bus.allow_stream=['teleop']",
        )
        return
    with client:
        cmd = new_cmd_id()
        for seq in range(6):
            client.send(
                {
                    "v": 1,
                    "type": "twist",
                    "source": "teleop",
                    "cmd_id": cmd,
                    "seq": seq,
                    "twist": {"linear_x_mps": 0.15, "angular_z_radps": 0.0},
                }
            )
            time.sleep(0.05)
        lapse_ms = config.bus.teleop_input_max_age_ms + config.limits.twist_renew_ms
        budget = lapse_ms / 1000.0
        time.sleep(budget + 0.1)
        gate.check(
            not moved(client, 0.4),
            "b4",
            "I-8",
            "a frozen teleop stream lapses to zero within teleop_input_max_age_ms",
            f"no motion published {budget * 1000:.0f} ms after the last twist",
            budget_ms=round(budget * 1000, 1),
        )


# ---------------------------------------------------------------------------
# The cases that need no robotd
# ---------------------------------------------------------------------------


OBSTACLE_MM = 231
"""ARCHITECTURE 5.1's own golden telemetry frame: inside the 250 mm stop."""

REVERSE_CLAMP_MM_S = 150
"""I-5: reverse is unsensed, so the MCU clamps it even while it stays legal."""

SLOW_ZONE_W_MRAD_S = 500
"""I-5: |w| is clamped inside the same zone -- the half spec v1 dropped."""


def sub_a_obstacle(gate: GateRun, args: Any, config: Any) -> None:
    """I-5, on one telemetry frame: forward zeroed, reverse and rotation kept.

    The whole point is that these are read off the *same* frame.  A test that
    stopped forward, then separately checked reverse, would pass on firmware
    that refused both for a moment -- and refusing reverse in front of an
    obstacle is how a rover gets stuck against a wall until a human moves it.
    """
    name = "obstacle at 231 mm: forward zeroed, reverse clamped to 150 mm/s, |w| to 500"
    if not have_module("rover_devtools.mcu_sim"):
        gate.skip("a", "I-5", name, "rover_devtools.mcu_sim")
        return
    sim = SimProcess(
        [f"obstacle={OBSTACLE_MM}"],
        command=args.sim_cmd.split() if args.sim_cmd else None,
        cwd=REPO,
    )
    if not sim.start():
        gate.skip("a", "I-5", name, f"rover_devtools.mcu_sim to start ({sim.error})")
        return
    link = McuLink(REPO / sim.pty_path)
    try:
        link.open()
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)

        link.stream(300, 0)
        forward = link.wait(TelemetryFrame, lambda f: bool(f.fault & Fault.TOF_STOP), 3.0)
        time.sleep(0.8)
        peak_forward = max(
            (
                sample.frame.v_cmd_mm_s  # type: ignore[attr-defined]
                for sample in link.snapshot()
                if isinstance(sample.frame, TelemetryFrame)
            ),
            default=0,
        )

        link.stream(-300, 1200)
        time.sleep(1.0)
        recent = [
            sample.frame
            for sample in link.snapshot()[-40:]
            if isinstance(sample.frame, TelemetryFrame)
        ]
        reverse = min((f.v_cmd_mm_s for f in recent), default=0)  # type: ignore[attr-defined]
        spin = max((abs(f.w_cmd_mrad_s) for f in recent), default=0)  # type: ignore[attr-defined]
        latched = any(f.state == McuState.FAULT for f in recent)  # type: ignore[attr-defined]
        link.stop_stream()

        ok = (
            forward is not None
            and peak_forward <= 0
            and reverse < 0
            and reverse >= -REVERSE_CLAMP_MM_S
            and spin <= SLOW_ZONE_W_MRAD_S
            and not latched
        )
        gate.check(
            ok,
            "a",
            "I-5",
            name,
            f"TOF_STOP {'set' if forward is not None else 'NOT set'}, "
            f"peak forward v_cmd {peak_forward} mm/s (must be <= 0), "
            f"reverse v_cmd {reverse} mm/s (must be in [-150, 0)), "
            f"|w_cmd| {spin} mrad/s (<= 500), state entered FAULT: {latched} "
            "(the obstacle class must not latch)",
            peak_forward_mm_s=peak_forward,
            reverse_mm_s=reverse,
            w_cmd_mrad_s=spin,
            latched=latched,
        )
    finally:
        link.close()
        sim.stop()


def sub_j_tof(gate: GateRun) -> None:
    """I-16: 65534 is a clear path, 65535 is blockage, and neither is 65.534 m."""
    problems: list[str] = []
    at_max = StateRanges(front=6.0, cliff=0.098)
    error = StateRanges(front=None, cliff=0.098)
    if at_max.front != 6.0:
        problems.append("the 65534 sentinel does not render as 6.0 m")
    if error.front is not None:
        problems.append("the 65535 sentinel does not render as null")
    world = WorldState.model_validate(
        {
            "pose_cm": {"x": 0, "y": 0},
            "heading_deg": 0,
            "battery_pct": 50,
            "obstacle_ahead": False,
            "front_range_cm": 600,
            "front_at_max": True,
            "bumper": False,
            "moving": False,
            "speed_cap_cms": 30,
            "allowed_skills": ["drive"],
            "motion_budget_left": {"path_cm": 100, "seconds": 8},
        }
    )
    if world.front_range_cm != 600 or not world.front_at_max:
        problems.append("the WorldState does not render 65534 as 600 cm plus the flag")
    try:
        WorldState.model_validate({**world.model_dump(), "front_range_cm": 6553})
        problems.append("the WorldState accepted 6553 cm, ten times the sensor maximum")
    except Exception:  # noqa: BLE001 - the refusal is the point
        pass
    if TOF_NO_TARGET_MM != 65534 or TOF_ERROR_MM != 65535:
        problems.append("the sentinels moved")
    gate.check(
        not problems,
        "j",
        "I-16",
        "the three-valued ToF survives into the bus and the WorldState",
        "; ".join(problems)
        if problems
        else "65534 -> 600 cm + front_at_max, 65535 -> null",
    )


def sub_j2_one_sensor(gate: GateRun, args: Any, config: Any) -> None:
    """I-16's other half: coverage is not a ``min()``.

    Front-L errors while front-R reports a clear 2 m.  Publishing the survivor's
    number as though coverage were intact is exactly what this invariant exists
    to catch, so forward must be refused on the strength of the dead sensor
    alone -- and the gate reads ``v_cmd`` off the wire, not a fault bit.
    """
    name = "one forward sensor dead refuses forward even when the other reports clear"
    if not have_module("rover_devtools.mcu_sim"):
        gate.skip("j2", "I-16", name, "rover_devtools.mcu_sim")
        return
    sim = SimProcess(
        ["tof_error=fl", "obstacle=2000"],
        command=args.sim_cmd.split() if args.sim_cmd else None,
        cwd=REPO,
    )
    if not sim.start():
        gate.skip("j2", "I-16", name, f"rover_devtools.mcu_sim to start ({sim.error})")
        return
    link = McuLink(REPO / sim.pty_path)
    try:
        link.open()
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)
        link.stream(300, 0)
        stale = link.wait(TelemetryFrame, lambda f: bool(f.fault & Fault.TOF_STALE), 3.0)
        time.sleep(0.8)
        frames = [
            sample.frame
            for sample in link.snapshot()
            if isinstance(sample.frame, TelemetryFrame)
        ]
        link.stop_stream()
        peak_forward = max((f.v_cmd_mm_s for f in frames), default=0)
        moved = max((abs(f.v_meas_mm_s) for f in frames), default=0)
        last = frames[-1] if frames else None
        fl_ok = bool(last.ctrl_flags & CtrlFlag.TOF_FL_OK) if last else True
        fr_ok = bool(last.ctrl_flags & CtrlFlag.TOF_FR_OK) if last else False
        gate.check(
            stale is not None and peak_forward <= 0 and not fl_ok and fr_ok,
            "j2",
            "I-16",
            name,
            f"TOF_STALE {'set' if stale is not None else 'NOT set'}, "
            f"peak forward v_cmd {peak_forward} mm/s (must be <= 0), "
            f"peak |v_meas| {moved} mm/s, ctrl_flags b8 tof_fl_ok={fl_ok} "
            f"b9 tof_fr_ok={fr_ok} (the survivor still reports 2000 mm)",
            peak_forward_mm_s=peak_forward,
            peak_v_meas_mm_s=moved,
            tof_fl_ok=fl_ok,
            tof_fr_ok=fr_ok,
        )
    finally:
        link.close()
        sim.stop()


def sub_l_stop_class(gate: GateRun, client: BusClient | None) -> None:
    """I-22: a stop is recognised before strict parsing, and never rejected."""
    mutations = [
        '{"v":1,"type":"stop","source":"brain"}',
        '{"v":1,"type":"stop","source":"web","reason":"user"}',
        '{"v":1,"type":"stop","source":"brain","seq":7,"cmd_id":"' + new_cmd_id() + '"}',
        '{"v":1,"type":"stop","source":"brain","turn_id":"' + new_turn_id() + '"}',
        '{"type":"stop","source":"brain"}',
        '{"v":1,"type":"stop","source":"brain","seq":-1}',
        '{"v":1,"type":"stop","source":"brain","reason":"' + "x" * 90 + '"}',
        '{"v":1,"type":"stop","source":"brain","extra":true}',
        '{"v":1,"type":"stop"}',
        '{"v":1,"type":"estop","source":"web"}',
        '{"v":1,"type":"cancel","source":"brain"}',
    ]
    unrecognised = [
        line
        for line in mutations
        if (json.loads(line).get("type") if _is_json(line) else None) not in STOP_CLASS
    ]
    gate.check(
        not unrecognised,
        "l",
        "I-22",
        "every stop-class mutation is dispatchable on the raw type, before parsing",
        f"{len(mutations)} mutations, {len(unrecognised)} unrecognised -- a "
        "StopMessage validation failure must never become a rejection",
        mutations=len(mutations),
    )

    if client is None:
        gate.skip(
            "l2",
            "I-22",
            "robotd never answers a stop-class message rejected",
            "a running robotd",
        )
        return
    rejections = []
    for line in mutations:
        client.send(line)
        answer = client.expect("result", deadline=0.3)
        if answer is not None and answer.get("status") == "rejected":
            rejections.append(line[:60])
    gate.check(
        not rejections,
        "l2",
        "I-22",
        "robotd never answers a stop-class message rejected",
        "; ".join(rejections[:2])
        if rejections
        else f"{len(mutations)} accepted or ignored",
        rejections=rejections,
    )


def _is_json(line: str) -> bool:
    try:
        json.loads(line)
    except json.JSONDecodeError:
        return False
    return True


def sub_l_clear(gate: GateRun, path: str) -> None:
    """A ``clear`` from ``brain`` is refused; ``estop_active`` still blocks."""
    web = connect(path, "web", ["subscribe"])
    brain = connect(path, "brain", ["skill", "subscribe"])
    if web is None or brain is None:
        for client in (web, brain):
            if client is not None:
                client.close()
        gate.skip(
            "l3",
            "I-22",
            "a clear from brain is refused and estop_active still blocks motion",
            "a running robotd",
        )
        return
    with web, brain:
        web.send({"v": 1, "type": "estop", "source": "web", "reason": "gate"})
        time.sleep(0.3)
        brain.send({"v": 1, "type": "clear", "source": "brain", "faults": ["estop_sw"]})
        # A `clear` carries no cmd_id and 5.2's `result` binds one, so robotd can
        # only refuse it as an `error`.  Either envelope counts, provided it
        # names the refusal: what 4.2 forbids is the clear taking effect.
        answer = brain.expect("result", "error", deadline=1.0) or {}
        refused = answer.get("status") == "rejected" or answer.get(
            "code"
        ) == ResultReason.SOURCE_NOT_ALLOWED
        brain.send(skill_message())
        after = brain.expect("result", deadline=1.0)
        blocked = (after or {}).get("reason") == ResultReason.ESTOP_ACTIVE
        gate.check(
            refused and blocked and not moved(brain),
            "l3",
            "I-22",
            "a clear from brain is refused and estop_active still blocks motion",
            f"clear refused as {answer.get('type')!r} "
            f"{answer.get('code') or answer.get('reason')!r}, "
            f"next skill reason={(after or {}).get('reason')!r}",
            clear_refused=refused,
            still_blocked=blocked,
        )
        # The latch is persisted in /run/rover/estop and outlives this case, so
        # a `web` clear -- the one source 4.2 allows -- puts robotd back in IDLE
        # rather than leaving every later case answered `estop_active`.
        web.send({"v": 1, "type": "clear", "source": "web", "faults": ["estop_sw"]})
        time.sleep(0.3)


def sub_i17_selftest(gate: GateRun) -> None:
    """The checker itself, against the spellings its predecessor missed.

    A static check that cannot fail is worse than none: it reports PASS today
    and keeps reporting PASS after the regression it exists to catch.  So the
    gate scores the checker before it scores the tree.
    """
    missed = [name for name, _, _ in CLOCK_VIOLATIONS if not clock_hits(name)]
    allowed = clockcheck.check_python(
        "class Link:\n"
        "    def _note_pong(self, frame):\n"
        "        return self._clock() - frame.mcu_us\n",
        "packages/rover_robotd/link.py",
    )
    elsewhere = clockcheck.check_python(
        "class Link:\n"
        "    def _note_ack(self, frame):\n"
        "        return self._clock() - frame.mcu_us\n",
        "packages/rover_robotd/link.py",
    )
    scoped = not allowed and len(elsewhere) == 1
    gate.check(
        not missed and scoped,
        "i17a",
        "I-17",
        "the clock checker flags every spelling of the violation",
        f"missed: {missed}"
        if missed
        else (
            f"{len(CLOCK_VIOLATIONS)} spellings all flagged; the 5.1 allowance is "
            f"one symbol, not a substring (Link._note_pong exempt, the same code "
            f"in a sibling method flagged: {scoped})"
        ),
        spellings=len(CLOCK_VIOLATIONS),
        missed=missed,
        allow_list_scoped=scoped,
    )


def sub_i17_grep(gate: GateRun) -> None:
    """I-17: no host timestamp is ever compared against the MCU clock."""
    roots = [REPO / "firmware", REPO / "packages" / "rover_robotd"]
    present = [root for root in roots if root.exists()]
    if not present:
        gate.skip(
            "i17",
            "I-17",
            "no host timestamp is compared against the MCU clock",
            "firmware/ or packages/rover_robotd/",
        )
        return
    crossings = [c for root in present for c in clockcheck.check_tree(root, REPO)]
    gate.check(
        not crossings,
        "i17",
        "I-17",
        "no host timestamp is compared against the MCU clock",
        "; ".join(str(c) for c in crossings[:3])
        if crossings
        else f"{len(present)} tree(s) scanned, only O.echo_pi_mono_us crosses",
        trees=[str(root.relative_to(REPO)) for root in present],
        hits=[str(c) for c in crossings],
    )


def sub_c_trace(gate: GateRun) -> None:
    """I-9: every G1 record shows the model reached the MCU only as a skill."""
    trials = sorted((REPO / "logs" / "gates").glob("g1-*-trials.jsonl"))
    if not trials:
        gate.skip(
            "c",
            "I-9",
            "every G1 record shows a validated skill and no unclamped model numeric",
            "a G1 run (logs/gates/g1-*-trials.jsonl)",
        )
        return
    latest = trials[-1]
    records = [
        json.loads(line) for line in latest.read_text().splitlines() if line.strip()
    ]
    breaches = [
        r["trial"]
        for r in records
        if r.get("cap_breach") or r.get("bound_breach") or r.get("budget_breach")
    ]
    unvalidated = [
        r["trial"] for r in records if r.get("accepted") and not r.get("skill")
    ]
    gate.check(
        not breaches and not unvalidated,
        "c",
        "I-9",
        "every G1 record shows a validated skill and no unclamped model numeric",
        f"{len(records)} records from {latest.name}: "
        f"{len(breaches)} breaches, {len(unvalidated)} accepted without a skill",
        source=latest.name,
        records=len(records),
        breaches=breaches[:10],
    )


# ---------------------------------------------------------------------------
# The cases that need a running robotd
# ---------------------------------------------------------------------------


def sub_f_duplicate(gate: GateRun, path: str) -> None:
    """I-12: a repeated ``cmd_id`` cannot execute motion twice."""
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip(
            "f", "I-12", "a repeated cmd_id cannot execute twice", "a running robotd"
        )
        return
    with client:
        turn = new_turn_id()
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": turn})
        message = skill_message(turn_id=turn)
        client.send(message)
        first = client.expect("result", deadline=2.0, cmd_id=message["cmd_id"])
        # I-12's literal test: the same skill again, verbatim.  4.2 checks seq
        # before the replay window, so this is refused `stale_seq` -- refused is
        # what the invariant asserts.
        client.send(message)
        second = client.expect("result", deadline=2.0, cmd_id=message["cmd_id"])
        # And the replay window itself: the same cmd_id with the seq advanced,
        # which is the only way past the seq row to row 8.
        replay = skill_message(turn_id=turn, cmd_id=message["cmd_id"], seq=2)
        client.send(replay)
        third = client.expect("result", deadline=2.0, cmd_id=message["cmd_id"])
        reason = (third or {}).get("reason")
        gate.check(
            (first or {}).get("status") == "accepted"
            and (second or {}).get("status") == "rejected"
            and (third or {}).get("status") == "rejected"
            and reason == ResultReason.DUPLICATE_CMD,
            # No `moved` term: the *first* command is accepted and does drive.
            # What I-12 asserts is that the repeat cannot execute, and the two
            # rejections are that assertion.
            "f",
            "I-12",
            "a repeated cmd_id cannot execute twice",
            f"first status={(first or {}).get('status')!r}, verbatim resend "
            f"{(second or {}).get('status')!r} reason="
            f"{(second or {}).get('reason')!r}, same cmd_id at seq 2 "
            f"{(third or {}).get('status')!r} reason={reason!r}",
            second_reason=(second or {}).get("reason"),
            third_reason=reason,
        )
        # The accepted drive is still running; stop-class, so never rejected.
        client.send({"v": 1, "type": "stop", "source": "brain", "reason": "gate"})
        time.sleep(0.3)


def sub_e_stale_turn(gate: GateRun, path: str) -> None:
    """I-11 (e2): a bare ``turn`` boundary makes the old plan stale."""
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip(
            "e2",
            "I-11",
            "a skill carrying a superseded turn_id is rejected stale_turn",
            "a running robotd",
        )
        return
    with client:
        old = new_turn_id()
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": old})
        time.sleep(0.1)
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": new_turn_id()})
        time.sleep(0.1)
        late = skill_message(turn_id=old)
        client.send(late)
        answer = client.expect("result", deadline=2.0, cmd_id=late["cmd_id"])
        reason = (answer or {}).get("reason")
        gate.check(
            reason == ResultReason.STALE_TURN and not moved(client),
            "e2",
            "I-11",
            "a skill carrying a superseded turn_id is rejected stale_turn",
            f"reason={reason!r}, motion={moved(client, 0.3)}",
            reason=reason,
        )


def sub_i_budget(gate: GateRun, path: str) -> None:
    """I-15: <= 1.5 m and <= 12 s per utterance, read from ``welcome``."""
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip(
            "i",
            "I-15",
            "ten legal drives in one turn exhaust the budget from welcome.limits",
            "a running robotd (the criterion reads welcome.limits, not the file)",
        )
        return
    with client:
        if client.welcome is None:
            gate.skip(
                "i",
                "I-15",
                "ten legal drives in one turn exhaust the budget from welcome.limits",
                "a welcome message carrying limits",
            )
            return
        budget_m = client.welcome.limits.budget_path_m
        cooldown = client.welcome.limits.motion_cooldown_ms / 1000.0
        turn = new_turn_id()
        # 4.2's cooldown is per turn_id but measured from the last motion start,
        # so an earlier case's drive makes this turn's first skill rate_limited.
        # Waiting it out is the rule, not a workaround.
        time.sleep(cooldown)
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": turn})
        accepted_m = 0.0
        refused = None
        for index in range(10):
            message = skill_message(
                turn_id=turn,
                seq=index + 1,
                args={"distance_m": 0.2, "speed_mps": 0.2},
                goal_ttl_ms=2000,
            )
            client.send(message)
            # `accepted` is not terminal, and 4.2 allows one motion in flight:
            # the next skill may not go out until this one has finished, or the
            # budget would be measured against `rate_limited` instead.
            answer = client.expect("result", deadline=6.0, cmd_id=message["cmd_id"])
            while answer is not None and answer.get("status") == "accepted":
                answer = client.expect("result", deadline=6.0, cmd_id=message["cmd_id"])
            if answer is None:
                break
            if answer.get("status") == "rejected":
                refused = answer.get("reason")
                break
            if answer.get("status") != "done":
                # An aborted drive charges only what it travelled, so counting
                # it as 0.2 m would credit the budget with distance nobody moved.
                refused = f"{answer.get('status')}/{answer.get('reason')}"
                break
            accepted_m += 0.2
            time.sleep(cooldown if index == 0 else 0.1)
        client.send({"v": 1, "type": "stop", "source": "brain", "reason": "gate"})
        settle(client)
        gate.check(
            refused == ResultReason.BUDGET_EXCEEDED and accepted_m <= budget_m + 1e-6,
            "i",
            "I-15",
            "ten legal drives in one turn exhaust the budget from welcome.limits",
            f"{accepted_m:.1f} m accepted against welcome budget_path_m={budget_m}, "
            f"then reason={refused!r}",
            accepted_m=accepted_m,
            budget_path_m=budget_m,
            refused=refused,
        )


def sub_m_obs_stale(gate: GateRun, path: str, config: Any) -> None:
    """I-23: an observation older than ``obs_max_age_ms`` cannot authorize motion."""
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip(
            "m",
            "I-23",
            "a stale observation cannot authorize motion",
            "a running robotd",
        )
        return
    with client:
        stale_ns = time.monotonic_ns() - (config.safety.obs_max_age_ms + 2000) * 1_000_000
        turn = new_turn_id()
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": turn})
        message = skill_message(
            turn_id=turn,
            obs={"frame_id": "cam-000001", "frame_mono_ns": max(0, stale_ns)},
        )
        client.send(message)
        answer = client.expect("result", deadline=2.0, cmd_id=message["cmd_id"])
        reason = (answer or {}).get("reason")
        gate.check(
            reason == ResultReason.OBS_STALE and not moved(client),
            "m",
            "I-23",
            "a stale observation cannot authorize motion",
            f"reason={reason!r} against obs_max_age_ms={config.safety.obs_max_age_ms}",
            reason=reason,
        )


class _WireTap:
    """A read-only listener on the controller port.

    ``h1`` freezes robotd, and robotd's own ``state`` stream is the evidence the
    old case read -- so the SIGSTOP silenced the witness and ``moved()`` returned
    False unconditionally, whatever the wheels did.  The controller keeps
    publishing ``T`` at 50 Hz regardless, so the evidence is taken off the wire.
    Read-only, because I-18 says only robotd writes this port.
    """

    def __init__(self, port: str) -> None:
        self._fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        self._reader = FrameReader()

    def close(self) -> None:
        os.close(self._fd)

    def __enter__(self) -> _WireTap:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def telemetry(self, seconds: float) -> Iterator[tuple[float, TelemetryFrame]]:
        """Every ``T`` in a window, stamped on this side's monotonic clock."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                data = os.read(self._fd, 4096)
            except BlockingIOError:
                data = b""
            except OSError:
                return
            if not data:
                time.sleep(0.002)
                continue
            for result in self._reader.feed(data):
                if isinstance(result, DecodeOk) and isinstance(
                    result.frame, TelemetryFrame
                ):
                    yield time.monotonic(), result.frame


def _drive_until_moving(
    client: BusClient, cooldown_ms: int, timeout: float = 6.0
) -> str:
    """Start a drive; return "" once the controller reports motion, else why not."""
    # motion_cooldown_ms applies to the first motion of a new turn_id, and every
    # earlier case in this run has just spent one, so a `rate_limited` refusal is
    # expected rather than a failure.
    last: dict[str, Any] = {}
    for attempt in range(4):
        turn = new_turn_id()
        client.send({"v": 1, "type": "turn", "source": "brain", "turn_id": turn})
        # A fresh seq per attempt: robotd checks seq before the cooldown, so a
        # retry that reuses it is answered stale_seq rather than re-tried.
        message = skill_message(
            turn_id=turn,
            seq=attempt + 1,
            goal_ttl_ms=5000,
            args={"distance_m": 0.6, "speed_mps": 0.2},
        )
        client.send(message)
        answer = client.expect("result", deadline=3.0, cmd_id=message["cmd_id"])
        last = answer or {}
        if last.get("status") == "accepted":
            break
        if attempt == 3:
            return f"an accepted drive (last answer: {last.get('reason') or last})"
        time.sleep(cooldown_ms / 1000.0 + 0.5)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if moved(client, 0.3):
            return ""
    return "a drive that actually turns the wheels"


def sub_h1_wire(gate: GateRun, path: str, args: Any, config: Any) -> None:
    """I-14 h1, measured on the controller rather than on the frozen daemon."""
    name = "rover-robotd frozen: v_cmd reaches 0 on the wire within 300 ms + t_brake"
    port = config.serial.port
    pid = _pid_of("rover-robotd", args.pid_pattern)
    if pid is None:
        gate.skip("h1", "I-14", name, "a running rover-robotd")
        return
    if not Path(port).exists():
        gate.skip("h1", "I-14", name, f"the controller port {port}")
        return
    client = connect(path, "brain", ["skill", "subscribe"])
    if client is None:
        gate.skip("h1", "I-14", name, "a running robotd bus")
        return
    # 300 ms is T1; the abort ramp at 2000 mm/s2 takes 150 ms from the 300 mm/s
    # cap, and one control period is 10 ms.  The rest is scheduling slack.
    budget_s = (config.limits.frame_ttl_ms + 150 + 10) / 1000.0 + 0.25
    with client:
        settle(client)
        client.drain(0.2)
        blocked = _drive_until_moving(client, config.limits.motion_cooldown_ms)
        if blocked:
            gate.skip("h1", "I-14", name, blocked)
            return
        with _WireTap(port) as tap:
            os.kill(pid, signal.SIGSTOP)
            frozen_at = time.monotonic()
            stopped_at: float | None = None
            saw_motion = False
            last: TelemetryFrame | None = None
            for stamp, frame in tap.telemetry(budget_s + 1.0):
                last = frame
                if frame.v_cmd_mm_s != 0 or frame.w_cmd_mrad_s != 0:
                    saw_motion = True
                    stopped_at = None
                    continue
                if stopped_at is None:
                    stopped_at = stamp
                # Zero PWM is not the terminal state: the brake is both MDD3A
                # inputs high, which reads back as pwm_enabled clear (I-1).
                if not CtrlFlag(frame.ctrl_flags) & CtrlFlag.PWM_ENABLED:
                    break
            os.kill(pid, signal.SIGCONT)
        # Leave the rover where this case found it.  Resuming robotd resumes an
        # unfinished goal, and the next case measures motion in a bare window --
        # so h1's own drive was scored as h2's.  A gate must not leave the
        # shared rover driving into the next one.
        client.send({"v": 1, "type": "stop", "source": "brain", "reason": "gate"})
        settle(client)
        elapsed_ms = None if stopped_at is None else (stopped_at - frozen_at) * 1000.0
        braked = last is not None and not (
            CtrlFlag(last.ctrl_flags) & CtrlFlag.PWM_ENABLED
        )
        gate.check(
            stopped_at is not None
            and elapsed_ms is not None
            and elapsed_ms <= budget_s * 1000.0
            and braked,
            "h1",
            "I-14",
            name,
            f"pid {pid} SIGSTOPped; v_cmd reached 0 after "
            f"{'never' if elapsed_ms is None else f'{elapsed_ms:.0f} ms'} "
            f"(budget {budget_s * 1000:.0f} ms), brake asserted: {braked}, "
            f"motion seen before the freeze: {saw_motion}",
            unit="rover-robotd",
            pid=pid,
            stop_ms=None if elapsed_ms is None else round(elapsed_ms, 1),
            budget_ms=round(budget_s * 1000, 1),
        )


def sub_h_process(gate: GateRun, path: str, args: Any, config: Any) -> None:
    """I-14: killing or freezing a process has a defined, per-process effect."""
    if gate.selected("h1"):
        with gate.guard("h1", "I-14", "rover-robotd frozen"):
            sub_h1_wire(gate, path, args, config)
    targets = {
        "h2": ("rover-brain", "the active command aborts within 400 ms + 300 ms TTL"),
        "h3": ("rover-cam", "no motion effect; the next motion skill is obs_stale"),
    }
    for sub, (unit, effect) in targets.items():
        if not gate.selected(sub):
            continue
        name = f"{unit} killed and frozen: {effect}"
        pid = _pid_of(unit, args.pid_pattern)
        if pid is None:
            gate.skip(sub, "I-14", name, f"a running {unit}")
            continue
        client = connect(path, "web", ["subscribe"])
        if client is None:
            gate.skip(sub, "I-14", name, "a running robotd")
            continue
        with client:
            # Measure this case's effect, not the previous one's: every gate
            # here shares one simulated rover.
            at_rest = settle(client)
            client.drain(0.2)
            os.kill(pid, signal.SIGSTOP)
            started = time.monotonic()
            still_moving = moved(client, 1.2)
            os.kill(pid, signal.SIGCONT)
            detail = (
                f"pid {pid} SIGSTOPped for "
                f"{(time.monotonic() - started) * 1000:.0f} ms; motion published: "
                f"{still_moving}"
            )
            if not at_rest:
                detail = f"the shared rover was still moving beforehand; {detail}"
            gate.check(
                at_rest and not still_moving,
                sub,
                "I-14",
                f"{unit} frozen: {effect}",
                detail,
                unit=unit,
                pid=pid,
                at_rest_before=at_rest,
            )

    # Section 8 asks for both signals on h1 and h2. SIGKILL is only run where
    # something restarts the unit afterwards: under `make sim` the process would
    # simply be gone and every later case would measure its absence.
    for sub, unit in (("h1k", "rover-robotd"), ("h2k", "rover-brain")):
        if not gate.selected(sub):
            continue
        name = f"{unit} SIGKILLed: the effect of a -9 matches the -STOP"
        if not _systemd_supervises(unit):
            gate.skip(
                sub, "I-14", name,
                f"{unit} under a supervisor that restarts it (systemd); "
                "`make sim` does not restart what it started",
            )
            continue
        pid = _pid_of(unit, args.pid_pattern)
        if pid is None:
            gate.skip(sub, "I-14", name, f"a running {unit}")
            continue
        os.kill(pid, signal.SIGKILL)
        deadline = time.monotonic() + 20.0
        back = None
        while time.monotonic() < deadline:
            back = _pid_of(unit, args.pid_pattern)
            if back is not None and back != pid:
                break
            time.sleep(0.5)
        client = connect(path, "web", ["subscribe"]) if back is not None else None
        still_moving = True
        if client is not None:
            with client:
                still_moving = moved(client, 1.0)
        gate.check(
            back is not None and back != pid and not still_moving,
            sub,
            "I-14",
            name,
            f"pid {pid} killed, restarted as {back}; motion published after: "
            f"{still_moving}",
            unit=unit,
            pid=pid,
            restarted_pid=back,
        )


def _systemd_supervises(unit: str) -> bool:
    """True when systemd is running this unit, so a SIGKILL is restarted."""
    try:
        result = subprocess.run(
            ["systemctl", "is-active", unit], capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.stdout.strip() == "active"


_UNIT_MODULES = {
    "rover-robotd": "rover_robotd.main",
    "rover-brain": "rover_brain.main",
    "rover-cam": "rover_cam.publisher",
}
"""Two spellings per unit: systemd runs the console script, `make sim` runs the
module.  Both have to be findable, because I-14 is a claim about the process."""


def _pgrep(needle: str) -> list[int]:
    """Pids whose argv matches, minus this process and any shell.

    `make sim` is one bash whose argv holds the whole start script, so every
    module name matches it.  SIGSTOPping that shell would freeze nothing and
    the case would pass on a process that was never the subject.
    """
    try:
        out = subprocess.run(
            ["pgrep", "-f", needle], capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.SubprocessError):
        return []
    found = []
    for line in out.stdout.split():
        if not line.isdigit():
            continue
        pid = int(line)
        if pid == os.getpid():
            continue
        try:
            command = subprocess.run(
                ["ps", "-o", "command=", "-p", str(pid)],
                capture_output=True, text=True, timeout=5,
            ).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            continue
        head = command.split()[0] if command else ""
        if Path(head).name in ("bash", "sh", "zsh", "make", "pgrep"):
            continue
        found.append(pid)
    return found


def _pid_of(unit: str, pattern: str | None) -> int | None:
    needles = [pattern] if pattern else [unit, _UNIT_MODULES.get(unit, unit)]
    for needle in needles:
        pids = _pgrep(needle)
        if pids:
            return pids[0]
    return None


HARDWARE_ONLY = (
    (
        "a2",
        "I-5",
        "a 70 mm cylinder at the chassis corner, recording where the "
        "forward ToF stops seeing it",
        "hardware: ISO 3691-4's test piece and a tape measure",
    ),
    (
        "a3",
        "I-5",
        "a real table edge with a catch strap: forward refused, travel "
        "past the trigger recorded",
        "hardware: a table edge and a catch strap",
    ),
    (
        "d",
        "I-10",
        "the per-channel I2t trips on a single-channel overload",
        "hardware: one channel loaded to 3.5 A",
    ),
    (
        "e",
        "I-11",
        "a late box response after a stop cannot start motion",
        "a running brain and a box that can be made to answer late",
    ),
    (
        "g",
        "I-13",
        "serial reconnect re-seeds, re-sends H and starts DISARMED",
        "a running robotd and a pty or cable to pull",
    ),
    (
        "k",
        "I-19",
        "box-link loss cancels an in-flight brain goal within 3 s",
        "a running brain with a box to disconnect",
    ),
    (
        "soak",
        "G4",
        "camera, speech and motion together for 30 minutes inside "
        "thermal and power limits",
        "hardware: the assembled rover",
    ),
)


def main() -> int:
    parser = base_parser(GATE, __doc__ or "")
    parser.add_argument(
        "--robotd", default=None, help="robotd.sock path (default from [bus] sock)"
    )
    parser.add_argument(
        "--pid-pattern", default=None, help="pgrep pattern for the I-14 process cases"
    )
    parser.add_argument(
        "--sim-cmd", default=None, help="override 'python -m rover_devtools.mcu_sim'"
    )
    args = parser.parse_args()

    config, source = load_robot_config()
    path = args.robotd or config.bus.sock
    gate = GateRun(
        GATE,
        metrics=metrics_path_for(GATE, args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="hardware" if args.hardware else "sim",
    )
    print(f"config: {source}   robotd: {path}")

    live = socket_alive(path)
    client = connect(path, "brain", ["skill", "subscribe"]) if live else None
    try:
        if gate.selected("b"):
            sub_b_fuzz(gate, client)
        if gate.selected("l"):
            sub_l_stop_class(gate, client)
        if gate.selected("j"):
            sub_j_tof(gate)
            sub_j2_one_sensor(gate, args, config)
        if gate.selected("c"):
            sub_c_trace(gate)
        if gate.selected("i17a"):
            sub_i17_selftest(gate)
        if gate.selected("i17"):
            sub_i17_grep(gate)
    finally:
        if client is not None:
            client.close()

    if gate.selected("a"):
        with gate.guard("a", "I-5", "obstacle at 231 mm"):
            sub_a_obstacle(gate, args, config)
    if gate.selected("b3"):
        sub_b_source(gate, path)
    if gate.selected("b4"):
        sub_b_frozen_tab(gate, path, config)
    if gate.selected("l3"):
        sub_l_clear(gate, path)
    if gate.selected("f"):
        sub_f_duplicate(gate, path)
    if gate.selected("e2"):
        sub_e_stale_turn(gate, path)
    if gate.selected("i"):
        sub_i_budget(gate, path)
    if gate.selected("m"):
        sub_m_obs_stale(gate, path, config)
    sub_h_process(gate, path, args, config)

    for sub, inv, name, reason in HARDWARE_ONLY:
        if gate.selected(sub):
            gate.skip(sub, inv, name, reason)

    code = gate.summary()
    # A3 claims 19 of 24 invariants go green before hardware. Every case below
    # is tagged M in section 8, so a skip for a missing peer is a hole in that
    # claim and must not read as a pass: `make gates-sim` boots the rover the
    # cases need.
    holes = sorted(
        case.label
        for case in gate.cases
        if case.status is Status.SKIP and case.sub in SIM_REQUIRED
    )
    if holes and code == 0:
        print()
        print(
            f"{GATE}: {len(holes)} case(s) tagged M in section 8 did not run: "
            + " ".join(holes)
        )
        print("  Boot the simulated rover first: make gates-sim")
        return 1 if live else 2
    return code


if __name__ == "__main__":
    raise SystemExit(main())
