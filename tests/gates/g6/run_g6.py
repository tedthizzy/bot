#!/usr/bin/env python3
"""G6: opt-in teleop and wheel-power recordings on the current bus.

Default runs isolated recording checks. --episodes validates at least 20 real
recordings without altering them. --only convert reports the still-unimplemented
offline conversion path as FAIL. S3 SERVO_EN is superseded, not verified.
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gatelib.checks import pytest_case, validate_subsets
from gatelib.env import REPO
from gatelib.runner import GateRun, base_parser, metrics_path_for


def episode_keys(gate, directory):
    from rover_robotd.episodes import ACTION_KEYS

    files = sorted(directory.rglob("*.jsonl")) if directory.is_dir() else []
    problems, frames = [], 0
    for path in files:
        rows = [
            json.loads(line) for line in path.read_text().splitlines() if line.strip()
        ]
        if not rows:
            problems.append(f"{path}: empty episode")
        for index, row in enumerate(rows):
            action = row.get("action", {})
            if tuple(action) != ACTION_KEYS or row.get("index") != index:
                problems.append(f"{path}:{index + 1}: wrong action keys or sequence")
            if not all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and -0.3 <= value <= 0.3
                for value in action.values()
            ):
                problems.append(f"{path}:{index + 1}: invalid wheel power")
            frames += 1
    gate.check(
        len(files) >= 20 and frames > 0 and not problems,
        "keys",
        "A34",
        "at least 20 recorded episodes with exact wheel-power keys and bounded actions",
        "; ".join(problems[:5]) or f"{len(files)} files; {frames} frames",
        files=len(files),
        frames=frames,
    )


def main():
    parser = base_parser("G6", __doc__)
    parser.add_argument(
        "--episodes",
        default=None,
        help="existing robotd episodes directory to validate (20 minimum)",
    )
    parser.add_argument(
        "--sim-cmd", default=None, help="retired override; isolated tests own transports"
    )
    args = parser.parse_args()
    gate = GateRun(
        "G6",
        metrics=metrics_path_for("G6", args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="isolated recording; optional supplied dataset",
    )
    validate_subsets(gate, ["pre", "keys", "convert", "servo"])
    if args.sim_cmd:
        gate.fail(
            "CLI",
            "CLI",
            "retired simulator override",
            "the current recording test owns its transport",
        )
    pytest_case(
        gate,
        "pre",
        "A35",
        "production teleop opt-in enforced through web and validator",
        "tests/unit/test_web.py::test_teleop_answers_source_not_allowed_until_allow_stream_is_set",
        "tests/unit/test_robotd_validator.py::test_a_twist_is_refused_while_allow_stream_is_empty",
    )
    if gate.selected("keys"):
        if args.episodes:
            with gate.guard("keys", "A34", "recorded episode validation"):
                episode_keys(gate, Path(args.episodes))
        else:
            pytest_case(
                gate,
                "keys",
                "A34",
                "actual teleop records current action schema and syncs only at close",
                "tests/unit/test_robotd_motion.py::test_teleop_preempts_then_expires_and_records_power_episode",
                "tests/unit/test_robotd_motion.py::test_recording_does_not_fsync_until_episode_closes",
            )
    if gate.only and gate.selected("convert"):
        gate.fail(
            "convert",
            "A34",
            "offline LeRobot conversion is not implemented",
            "tools/to_lerobot.py remains planned; JSONL recording is implemented",
        )
    if (gate.only and gate.selected("servo")) or args.hardware:
        gate.skip(
            "servo",
            "superseded",
            "S3 SERVO_EN has no current WAVE actuator channel",
            "topology decision and physical STOP confirmation before adding actuators",
        )
    print(
        "Pending: 20 real episodes, offline conversion, physical STOP. No SERVO_EN claim."
    )
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
