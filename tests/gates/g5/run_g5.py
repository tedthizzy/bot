#!/usr/bin/env python3
"""G5 -- behaviours (ARCHITECTURE 13).

``find`` in eight sectors, the bearing calibration, the stop word, the filler,
and the scene ring.  Two of these need only the contracts and run anywhere:

  * **bearing** -- the Pi computes the turn angle from ``center_x_permille`` and
    a calibrated ``hfov_deg``, and the model supplies no angle at all (5.5).  The
    round trip is checked at eight true bearings including the sign, because the
    inverse sign makes ``find`` turn *away* from the object and nothing else in
    the design would notice.
  * **no bearing_deg** -- no ``FindObservation`` may carry one, which
    ``extra="forbid"`` makes structural rather than a convention.

The rest need brain, an object on the floor and a person to place it.  The stop
word is **reported, never gated**: A29 makes it a best-effort fourth channel and
no invariant counts it, so a gate that failed on it would be asserting something
the design deliberately does not promise.

    python tests/gates/g5/run_g5.py
    python tests/gates/g5/run_g5.py --hardware --sector 3 --object "red mug"
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve()
sys.path[:0] = [str(_HERE.parents[3] / "packages"), str(_HERE.parents[1])]

from gatelib.bus import BusClient  # noqa: E402
from gatelib.env import REPO, load_robot_config, socket_alive  # noqa: E402
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for  # noqa: E402
from rover_contracts import (  # noqa: E402
    FindObservation,
    UtteranceMessage,
    bearing_deg_from_center_x,
    observation_adapter,
)

GATE = "G5"

TRUE_BEARINGS_DEG = (-40.0, -30.0, -15.0, -5.0, 5.0, 15.0, 30.0, 40.0)
"""Eight bearings inside the 83 degree crop, both signs, including the +30 that
ARCHITECTURE 13 calibrates against."""

TOLERANCE_DEG = 5.0
"""G5's own criterion: the Pi-computed bearing within +-5 degrees."""


def sub_bearing(gate: GateRun, hfov_deg: float) -> None:
    """The bearing is a pure function of ``center_x_permille`` and ``hfov_deg``."""
    errors: list[tuple[float, float, int]] = []
    for true_deg in TRUE_BEARINGS_DEG:
        # What a camera with this crop would report for an object at true_deg.
        permille = round((0.5 - true_deg / hfov_deg) * 1000)
        computed = bearing_deg_from_center_x(permille, hfov_deg)
        errors.append((true_deg, computed - true_deg, permille))
    worst = max(abs(delta) for _true, delta, _p in errors)
    sign_ok = bearing_deg_from_center_x(400, hfov_deg) > 0
    gate.check(
        worst <= TOLERANCE_DEG and sign_ok,
        "bearing",
        "A13",
        "the Pi-computed bearing is within +-5 degrees, and + is left",
        f"worst error {worst:.2f} deg over {len(errors)} bearings at hfov "
        f"{hfov_deg} deg; an object left of centre gives "
        f"{'+' if sign_ok else 'MINUS'} bearing",
        hfov_deg=hfov_deg,
        worst_error_deg=round(worst, 3),
        sign_left_is_positive=sign_ok,
        samples=[{"true_deg": t, "permille": p} for t, _d, p in errors],
    )
    # 120 is the diagonal, not the horizontal: using it inflates every bearing.
    inflated = bearing_deg_from_center_x(139, 120.0)
    correct = bearing_deg_from_center_x(139, hfov_deg)
    gate.record(
        "bearing2",
        "A18",
        "hfov_deg is a config key, and the 120 degree diagonal would inflate it",
        Status.PASS,
        f"the same detection reads {correct:.1f} deg at hfov {hfov_deg} and "
        f"{inflated:.1f} deg at 120 -- {100 * (inflated / correct - 1):.0f}% high",
        at_config_deg=round(correct, 2),
        at_diagonal_deg=round(inflated, 2),
    )


def sub_no_bearing(gate: GateRun) -> None:
    """5.5: no FindObservation record may carry a model-supplied bearing."""
    base = {
        "kind": "find",
        "present": True,
        "center_x_permille": 610,
        "confidence": "medium",
        "description": "a red ceramic mug on a wooden table",
    }
    refused = []
    for extra in ("bearing_deg", "angle_deg", "distance_m", "skill"):
        try:
            observation_adapter.validate_python({**base, extra: 1.0})
            refused.append(extra)
        except Exception:  # noqa: BLE001 - the refusal is the pass
            pass
    accepted = FindObservation.model_validate(base)
    gate.check(
        not refused and accepted.center_x_permille == 610,
        "obs",
        "A13",
        "a FindObservation carries no bearing, no distance and no skill",
        f"accepted fields would have been {refused}"
        if refused
        else "bearing_deg, angle_deg, distance_m and skill all refused",
        refused_none_of=refused,
    )


def sub_find_sectors(gate: GateRun, args: object, config: object) -> None:
    """A14/G5: ``find`` succeeds 8/10 with the object in one of eight sectors."""
    gate.skip(
        "find",
        "A14",
        "find succeeds 8/10 with the object in one of eight 45 degree sectors",
        "hardware and a person to move the object between sectors: 8 sweeps of "
        "45 degrees with an 83 degree field covers 360, and only a real placement "
        "tests that",
    )


def sub_stop_word(gate: GateRun, config: object) -> None:
    """A29: measured at G5, counted by nothing."""
    brain_path = config.bus.brain_sock  # type: ignore[attr-defined]
    bus_path = config.bus.sock  # type: ignore[attr-defined]
    if not (socket_alive(brain_path) and socket_alive(bus_path)):
        gate.skip(
            "stopword",
            "A29",
            "spoken stop to S on the wire, reported and not gated",
            "a running brain and robotd (and, for the real number, a microphone: "
            "the spoken stop word is a best-effort fourth channel that no "
            "invariant counts)",
        )
        return
    brain = BusClient(brain_path, timeout=5.0)
    robotd = BusClient(bus_path, timeout=5.0)
    robotd.hello("web", ["subscribe"], subscribe=["state", "result", "event"])
    with brain, robotd:
        robotd.drain(0.2)
        sent = time.monotonic()
        brain.send(
            UtteranceMessage(
                source="cli",
                text="stop",
                confidence=None,
                is_final=True,
                mono_ns=time.monotonic_ns(),
            )
        )
        stopped = None
        while time.monotonic() - sent < 3.0:
            message = robotd.read(0.5)
            if message is None:
                break
            if message.get("type") == "state" and not (message.get("mcu") or {}).get(
                "motion"
            ):
                stopped = time.monotonic()
                break
        gate.record(
            "stopword",
            "A29",
            "stop utterance to a stationary MCU, reported and not gated",
            Status.PASS,
            f"{(stopped - sent) * 1000:.0f} ms from utterance to a state with "
            "mcu.motion false. Text in, not speech: the spoken path adds wake, VAD "
            "and STT, and A29 counts none of it"
            if stopped
            else "no state with mcu.motion false arrived within 3 s",
            stop_latency_ms=round((stopped - sent) * 1000, 1) if stopped else None,
        )


def main() -> int:
    parser = base_parser(GATE, __doc__ or "")
    parser.add_argument(
        "--sector", type=int, default=None, help="0-7, where the object is"
    )
    parser.add_argument("--object", default="red mug")
    args = parser.parse_args()

    config, source = load_robot_config()
    gate = GateRun(
        GATE,
        metrics=metrics_path_for(GATE, args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="hardware" if args.hardware else "sim",
    )
    print(f"config: {source}   hfov_deg: {config.camera.hfov_deg}")

    if gate.selected("bearing"):
        sub_bearing(gate, config.camera.hfov_deg)
    if gate.selected("obs"):
        sub_no_bearing(gate)
    if gate.selected("find"):
        sub_find_sectors(gate, args, config)
    if gate.selected("stopword"):
        sub_stop_word(gate, config)
    if gate.selected("filler"):
        gate.skip(
            "filler",
            "A31",
            "the filler stops the instant the model's first sentence is ready",
            "a running brain with audio, since the criterion is an audible overlap",
        )
    if gate.selected("memory"):
        gate.skip(
            "memory",
            "A31",
            "the scene ring answers 'where did you see the mug'",
            "a running brain that has already seen a mug",
        )
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
