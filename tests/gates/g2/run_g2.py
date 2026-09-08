#!/usr/bin/env python3
"""G2: WAVE protocol, firmware headers, and actual host link boundaries.

Native firmware checks mock GPIO/clock, and rover-stub is an independent model.
Neither supplies physical stopping, reset-current, or sensor-placement proof.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gatelib.checks import command_case, pytest_case, validate_subsets
from gatelib.env import REPO
from gatelib.runner import GateRun, base_parser, metrics_path_for

LINK = "tests/unit/test_robotd_link.py"
STUB = "tests/unit/test_rover_stub.py"
CASES = {
    "a": (
        "I-1",
        "stopped stream expires; no autonomous host keepalive",
        [
            LINK + "::test_command_streams_what_the_caller_asks_and_stops_when_it_stops",
            STUB + "::test_a_stopped_speed_stream_zeroes_the_motors_within_300ms",
        ],
    ),
    "b": (
        "I-2",
        "malformed lines renew nothing; bounded reassembly",
        [LINK, "-k", "dropped or reassembled or buffer_stays"],
    ),
    "b-codec": (
        "I-2",
        "current JSON codec, not retired CRC",
        ["tests/unit/test_wave_proto.py"],
    ),
    "c": (
        "I-3",
        "bring-up and stock-firmware refusal",
        [LINK, "-k", "bring_up or stock or banner or fork or firmware"],
    ),
    "c2": (
        "I-13",
        "link restart and reconnect discard old command state",
        [LINK, "-k", "restart or reconnect or connection or reopen"],
    ),
    "d": (
        "I-4",
        "raw host cap and independent board clamping",
        [
            LINK + "::test_command_clamps_to_power_max_and_zeroes_non_finite_values",
            STUB + "::test_a_speed_over_the_cap_is_applied_as_the_cap_and_counted",
        ],
    ),
    "d2": (
        "I-4,I-5",
        "stop flags block forward, preserve reverse/rotation",
        [STUB, "-k", "tof or bumper or low_battery or coast"],
    ),
    "d3": (
        "I-4",
        "host caps match firmware source constants",
        ["tests/unit/test_caps_match.py", "tests/unit/test_config.py"],
    ),
    "d4": ("I-4", "full current protocol/stub cases; no obsolete V.flags byte", [STUB]),
    "trapezoid": (
        "T2",
        "real robotd drive ramps and completes",
        [
            "tests/unit/test_robotd_motion.py::test_drive_completes_ramps_and_charges_only_streamed_motion"
        ],
    ),
}
HARDWARE = {
    "brake": "measure stopping distance at configured power; no encoders are fitted",
    "e": "verify physical emergency motor-power isolation",
    "f": "sweep the real battery supply and verify cutoff",
    "g": "measure stalled-wheel current and thermal behavior; no encoder slip claim",
    "halt": "measure obstacle-to-halt distance with the fitted ToF and chassis",
    "rpack": "characterize the supply; current-based sag compensation is not implemented",
    "bumper": "verify fitted bumper wiring, including a disconnected harness",
    "rail": "verify host supply behavior while the controller resets",
    "i2": "measure motor output through reset, power-cycle, and bootloader",
    "h-hardware": "verify real controller watchdog behavior and reset outputs",
}


def main():
    parser = base_parser("G2", __doc__)
    parser.add_argument(
        "--full", action="store_true", help="include full current protocol/stub sweep"
    )
    for name in ("port", "sim-cmd", "sim-flags", "hang-ms", "surface"):
        parser.add_argument(
            "--" + name,
            default=None,
            help="legacy bench option; not used by isolated software checks",
        )
    args = parser.parse_args()
    gate = GateRun(
        "G2",
        metrics=metrics_path_for("G2", args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="isolated software + native mocked hardware",
    )
    validate_subsets(gate, [*CASES, *HARDWARE, "h", "i"])
    if any(
        getattr(args, name) is not None
        for name in ("port", "sim_cmd", "sim_flags", "hang_ms")
    ):
        gate.fail(
            "CLI",
            "CLI",
            "retired direct-controller override",
            "software checks own their transports; hardware needs a separate bench run",
        )
    for sub, (invariant, name, selectors) in CASES.items():
        if sub == "d4" and not args.full and not gate.only:
            continue
        pytest_case(gate, sub, invariant, name, *selectors)
    for sub in ("h", "i"):
        command_case(
            gate,
            sub,
            "I-20,I-24",
            "firmware stops, bounded intake, cooperative waits; GPIO/clock mocked",
            ["bash", "firmware/tests/run.sh"],
        )
    for sub, procedure in HARDWARE.items():
        if gate.selected(sub):
            gate.skip(
                sub,
                "hardware",
                procedure,
                "bench evidence; --hardware does not turn source into measurements",
            )
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
