#!/usr/bin/env python3
"""G6 -- the merge track (ARCHITECTURE 13), separate and after G4 is green.

Three things, in the order they block each other:

  1. **The precondition.**  Recording teleop needs ``bus.allow_stream=["teleop"]``
     and production ships it empty, so ``/teleop`` answers ``source_not_allowed``
     until it is set.  Checking it first is what stops a G6 run failing twenty
     times for a reason nobody reads.
  2. **The episode key set.**  A34: the recorded keys must equal LeKiwi's
     *exactly* -- ``x.vel``, ``y.vel`` (constant 0.0) and ``theta.vel`` in deg/s.
     Dropping ``y.vel`` is the day-one mistake that turns every episode into
     scrap, so this asserts equality rather than containment, and it asserts
     ``y.vel`` really is zero.
  3. **``SERVO_EN``.**  It follows ``V.flags`` b1 ANDed with heartbeat age,
     ``fault == 0`` and e-stop released, and drops within 10 ms of any of those
     going away.  The sim half runs over the link; the LED is the hardware half.

    python tests/gates/g6/run_g6.py --episodes data/episodes
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve()
sys.path[:0] = [str(_HERE.parents[3] / "packages"), str(_HERE.parents[1])]

from gatelib.env import REPO, have_module, load_robot_config  # noqa: E402
from gatelib.link import McuLink, SimProcess  # noqa: E402
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for  # noqa: E402
from rover_contracts import TelemetryFrame, VFlag  # noqa: E402

GATE = "G6"

LEKIWI_KEYS = frozenset({"x.vel", "y.vel", "theta.vel"})
"""A34, and the whole point of the contract test: equal, not a superset."""

EPISODES_WANTED = 20
"""ARCHITECTURE 13: 20 teleop episodes."""


def sub_precondition(gate: GateRun, config: Any) -> None:
    allowed = list(config.bus.allow_stream)
    gate.check(
        "teleop" in allowed,
        "pre",
        "A35",
        "bus.allow_stream contains 'teleop', without which /teleop refuses",
        f"allow_stream={allowed}. Production ships it empty on purpose; set it "
        "before recording and clear it after",
        allow_stream=allowed,
    )


def sub_episode_keys(gate: GateRun, root: Path) -> None:
    """A34: the recorded key set equals LeKiwi's exactly."""
    if not root.exists():
        gate.skip(
            "keys",
            "A34",
            f"{EPISODES_WANTED} LeKiwi-keyed episodes, key set equal to LeKiwi's",
            f"recorded episodes under {root} (record them with "
            "bus.allow_stream=['teleop'])",
        )
        return
    files = sorted(root.rglob("*.jsonl"))
    if not files:
        gate.skip(
            "keys",
            "A34",
            f"{EPISODES_WANTED} LeKiwi-keyed episodes, key set equal to LeKiwi's",
            f"at least one .jsonl episode under {root}",
        )
        return
    problems: list[str] = []
    frames = 0
    nonzero_y = 0
    for path in files:
        for number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            record = json.loads(line)
            action = record.get("action") or record.get("action_features") or record
            keys = {key for key in action if key.endswith(".vel")}
            if keys != LEKIWI_KEYS:
                problems.append(f"{path.name}:{number} keys {sorted(keys)}")
                break
            frames += 1
            if abs(float(action["y.vel"])) > 1e-9:
                nonzero_y += 1
    gate.check(
        not problems and len(files) >= EPISODES_WANTED and nonzero_y == 0,
        "keys",
        "A34",
        f"{EPISODES_WANTED} LeKiwi-keyed episodes, key set equal to LeKiwi's",
        "; ".join(problems[:3])
        if problems
        else f"{len(files)} episodes, {frames} frames, y.vel non-zero in "
        f"{nonzero_y} (must be 0), theta.vel in deg/s per the plugin",
        episodes=len(files),
        frames=frames,
        nonzero_y=nonzero_y,
        problems=problems[:10],
    )


def sub_lerobot(gate: GateRun, root: Path) -> None:
    converter = REPO / "tools" / "to_lerobot.py"
    if not converter.exists():
        gate.skip(
            "convert",
            "A34",
            "tools/to_lerobot.py converts and dataset.finalize() succeeds",
            "tools/to_lerobot.py",
        )
        return
    if not have_module("lerobot"):
        gate.skip(
            "convert",
            "A34",
            "tools/to_lerobot.py converts and the dataset loads off-robot",
            "LeRobot >= 0.6.1, which is deliberately not a rover dependency",
        )
        return
    gate.skip(
        "convert",
        "A34",
        "tools/to_lerobot.py converts and the dataset loads off-robot",
        f"a run of tools/to_lerobot.py over {root} on the workstation, not the Pi",
    )


def sub_servo_en(gate: GateRun, args: Any, config: Any) -> None:
    """``SERVO_EN`` follows ``V.flags`` b1, and drops within 10 ms."""
    if args.hardware:
        gate.skip(
            "servo",
            "A35",
            "SERVO_EN asserts on V.flags b1 and drops within 10 ms",
            "hardware: the SERVO_EN FET fitted at this gate, and an LED to watch",
        )
        return
    if not have_module("rover_devtools.mcu_sim"):
        gate.skip(
            "servo",
            "A35",
            "SERVO_EN asserts on V.flags b1 and drops within 10 ms",
            "rover_devtools.mcu_sim (the LED half is hardware)",
        )
        return
    sim = SimProcess(command=args.sim_cmd.split() if args.sim_cmd else None, cwd=REPO)
    if not sim.start():
        gate.skip(
            "servo",
            "A35",
            "SERVO_EN asserts on V.flags b1 and drops within 10 ms",
            f"rover_devtools.mcu_sim to start ({sim.error})",
        )
        return
    link = McuLink(REPO / sim.pty_path)
    try:
        link.open()
        link.bring_up(config.serial.reseed_wait_ms / 1000.0)
        link.stream(0, 0, 300, int(VFlag.SERVO_RAIL_EN))
        asserted = link.wait(TelemetryFrame, lambda f: bool(f.rails & 1), 2.0)
        dropped = None
        if asserted is not None:
            link.stop_stream()
            cut = time.monotonic()
            dropped = link.wait(
                TelemetryFrame, lambda f: not (f.rails & 1), 2.0, since=cut
            )
        gate.record(
            "servo",
            "A35",
            "SERVO_EN asserts on V.flags b1 and drops when the heartbeat lapses",
            Status.PASS if asserted is not None and dropped is not None else Status.FAIL,
            f"rails b0 asserted={asserted is not None}, dropped "
            f"{(dropped.mono - cut) * 1000:.0f} ms after the last V "
            f"(the MCU ANDs b1 with heartbeat age < 500 ms, fault == 0 and e-stop "
            "released)"
            if dropped is not None
            else "rails b0 never asserted or never dropped",
            drop_ms=round((dropped.mono - cut) * 1000, 1) if dropped else None,
        )
    finally:
        link.close()
        sim.stop()


def main() -> int:
    parser = base_parser(GATE, __doc__ or "")
    parser.add_argument(
        "--episodes",
        default="data/episodes",
        help="where robotd writes LeKiwi-keyed episodes",
    )
    parser.add_argument("--sim-cmd", default=None)
    args = parser.parse_args()

    config, source = load_robot_config()
    root = Path(args.episodes)
    if not root.is_absolute():
        root = REPO / root
    gate = GateRun(
        GATE,
        metrics=metrics_path_for(GATE, args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="hardware" if args.hardware else "sim",
    )
    print(f"config: {source}   episodes: {root}")

    if gate.selected("pre"):
        sub_precondition(gate, config)
    if gate.selected("keys"):
        sub_episode_keys(gate, root)
    if gate.selected("convert"):
        sub_lerobot(gate, root)
    if gate.selected("servo"):
        sub_servo_en(gate, args, config)
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
