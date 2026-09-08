#!/usr/bin/env python3
"""G4: named fault-injection boundaries, isolated from any running rover.

Runtime bus/TCP/process checks, brain FSM checks, and source scans are labeled
separately. A missing required software test is FAIL, not a hardware skip.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gatelib import clockcheck
from gatelib.checks import pytest_case, validate_subsets
from gatelib.env import REPO
from gatelib.runner import GateRun, base_parser, metrics_path_for

MOTION = "tests/unit/test_robotd_motion.py"
RUNTIME = "tests/gates/test_runtime.py"
CASES = {
    "a": (
        "I-5",
        "forward obstacle abort and reverse positive control",
        [RUNTIME + "::test_obstacle_aborts_forward_but_reverse_remains_available"],
    ),
    "b": (
        "I-8",
        "500 schema cases; rejected lines cross real bus without motion",
        [
            RUNTIME
            + "::test_five_hundred_fuzz_lines_match_schema_and_invalid_lines_cannot_move"
        ],
    ),
    "b2": (
        "I-8",
        "malformed bus lines preserve connection; raw commands stay capped",
        [
            MOTION
            + "::test_parse_rejection_reaches_unsubscribed_sender_and_connection_survives"
        ],
    ),
    "b3": (
        "I-8",
        "source spoof and exhausted budget refuse raw motion",
        [RUNTIME + "::test_source_spoof_and_exhausted_budget_cannot_emit_motion"],
    ),
    "b4": (
        "I-14",
        "teleop renewal expiry and preemption on actual daemon",
        [MOTION + "::test_teleop_preempts_then_expires_and_records_power_episode"],
    ),
    "c": (
        "I-9",
        "motion authorization metadata through actual daemon",
        [MOTION, "-k", "rejected_motion and unauthorized"],
    ),
    "e": (
        "I-11",
        "actual brain FSM drops late plans after stop/new utterance",
        [
            "tests/unit/test_brain_fsm.py",
            "-k",
            "plan_landing or plan_for_an_older or result_for_an_older",
        ],
    ),
    "e2": (
        "I-11",
        "stale turn refused after successful motion",
        [MOTION, "-k", "rejected_motion and stale_turn"],
    ),
    "f": (
        "I-12",
        "command replay refused after successful motion",
        [MOTION, "-k", "rejected_motion and replay"],
    ),
    "g": (
        "I-13",
        "daemon aborts on reconnect/reboot and sends only zeros",
        [MOTION, "-k", "motion_faults and (disconnect or reboot)"],
    ),
    "h1": (
        "I-14",
        "owned robotd process freezes; heartbeat expires; expired goal never resumes",
        [
            RUNTIME
            + "::test_frozen_owned_robotd_stops_stream_and_expired_goal_does_not_resume"
        ],
    ),
    "h2": (
        "I-14",
        "missing brain pings abort active motion",
        [MOTION, "-k", "motion_faults and silence"],
    ),
    "h3": (
        "I-14",
        "missing rover feedback aborts active motion",
        [MOTION, "-k", "motion_faults and stale_feedback"],
    ),
    "i": (
        "I-15",
        "shared time ledger, runtime refusal, published remaining budget",
        [
            RUNTIME + "::test_source_spoof_and_exhausted_budget_cannot_emit_motion",
            MOTION + "::test_state_and_logs_report_current_feedback_and_shared_budget",
            "tests/unit/test_robotd_budget.py",
        ],
    ),
    "j": (
        "I-16",
        "unknown range is not clear; independent stub sensor gates",
        [
            "tests/unit/test_brain_world.py",
            "tests/unit/test_rover_stub.py",
            "-k",
            "range or tof or bumper",
        ],
    ),
    "j2": (
        "I-16",
        "single current ToF absent/error representation",
        ["tests/unit/test_wave_proto.py", "-k", "tof or feedback"],
    ),
    "k": (
        "I-19",
        "actual brain FSM cancels on box loss; HTTP client reports outages",
        [
            "tests/unit/test_brain_fsm.py",
            "tests/unit/test_brain_box.py",
            "-k",
            "box_loss or transport_failure or late_response",
        ],
    ),
    "l": (
        "I-22",
        "stop-class malformed fields cannot suppress actual stopping",
        [MOTION, "-k", "motion_faults and (stop or estop or cancel)"],
    ),
    "l2": (
        "I-22",
        "stop before hello and unknown fields at real bus",
        ["tests/unit/test_robotd_bus.py", "-k", "stop"],
    ),
    "l3": (
        "I-22",
        "only web clears persistent estop",
        [
            "tests/unit/test_robotd_bus.py::test_clear_from_web_lifts_the_latch_and_brain_may_not"
        ],
    ),
    "m": (
        "I-23",
        "stale camera observation cannot command motion",
        [MOTION, "-k", "rejected_motion and stale_obs"],
    ),
}
HARDWARE = {
    "a2": "cylinder/corner visibility measurement",
    "a3": "table-edge test with catch strap",
    "d": "real overcurrent/thermal behavior; no encoder/current model is claimed",
    "soak": "30-minute assembled camera/speech/motion thermal and power soak",
}


def check_clocks(gate):
    if gate.selected("i17a"):
        samples = [
            "age = time.monotonic_ns() - frame.mcu_us * 1000",
            "age = now_mono_ns - telemetry.mcu_us",
            "age = self._clock() - frame.mcu_us",
            "age = mcu_us - host_now",
            "age = frame.mcu_us - pi_mono_us",
            "if frame.mcu_us > time.monotonic_ns(): pass",
            "age = int(time.monotonic_ns()) - int(frame.mcu_us)",
        ]
        hits = all(clockcheck.check_python(source, "<selftest>") for source in samples)
        hits = hits and bool(
            clockcheck.check_c(
                "int64_t skew = (int64_t)now_us - (int64_t)host_stamp_us;", "<selftest>"
            )
        )
        gate.check(
            hits, "i17a", "I-17", "clock checker rejects all eight regression spellings"
        )
    if gate.selected("i17"):
        roots = [REPO / "hosts/pi", REPO / "packages", REPO / "firmware/General_Driver"]
        missing = [str(root) for root in roots if not root.exists()]
        hits = [str(hit) for root in roots for hit in clockcheck.check_tree(root, REPO)]
        gate.check(
            not missing and not hits,
            "i17",
            "I-17",
            "source scan: no mixed host/MCU timestamp arithmetic",
            str(missing or hits),
            trees=[str(root.relative_to(REPO)) for root in roots],
        )


def main():
    parser = base_parser("G4", __doc__)
    for option in ("robotd", "pid-pattern", "sim-cmd"):
        parser.add_argument(
            "--" + option,
            default=None,
            help="retired shared-process override; isolated tests own their processes",
        )
    args = parser.parse_args()
    gate = GateRun(
        "G4",
        metrics=metrics_path_for("G4", args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="isolated runtime / FSM / source",
    )
    validate_subsets(gate, [*CASES, *HARDWARE, "i17a", "i17"])
    if args.robotd or args.pid_pattern or args.sim_cmd:
        gate.fail(
            "CLI",
            "CLI",
            "shared process overrides are not supported",
            "the gate never signals or drives an existing service",
        )
    for sub, (invariant, name, selectors) in CASES.items():
        pytest_case(gate, sub, invariant, name, *selectors)
    check_clocks(gate)
    for sub, procedure in HARDWARE.items():
        if gate.selected(sub):
            gate.skip(
                sub,
                "hardware",
                procedure,
                "physical assembled rover and human bench procedure",
            )
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
