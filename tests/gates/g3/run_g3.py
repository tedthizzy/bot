#!/usr/bin/env python3
"""G3 -- the desk gate (ARCHITECTURE 13): the soak, then the speech bake-off.

G3a is 30 minutes continuous with the MCU on the bench and ``audio.input="ptt"``:
PTT to STT to box to TTS to motion, camera streaming, telemetry logging.  Its
criteria are Pi measurements -- ``vcgencmd get_throttled``, the ARM clock under
load, SoC temperature, header voltage, RSS, ``fuser`` on the port -- so on a Mac
every one of them reports "requires hardware" and the run carries no result
rather than a false green.

Two things here do run wherever brain and robotd run, and they are what makes
the soak a soak rather than a wait: the turn loop itself, and the latency it
measures.  The metric ARCHITECTURE 9 gates is end-of-speech to first audio; a
gate outside the process cannot hear audio, so this measures **utterance
accepted to FSM SPEAKING_INTENT**, names it that in every record, and reports it
beside the 2.8 s p95 rather than claiming to be it.

G3b is the bake-off: the STT corpus, and Piper's first-audio latency on a short
reply, which is open item 4 and open item 10.  Neither can be synthesized --
speech recognition needs recorded speech -- so both say what they need.

    python tests/gates/g3/run_g3.py --duration 1800 --hardware   # on the Pi
    python tests/gates/g3/run_g3.py --duration 120               # what runs in sim
"""

from __future__ import annotations

import json
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

_HERE = Path(__file__).resolve()
sys.path[:0] = [str(_HERE.parents[3] / "packages"), str(_HERE.parents[1])]

from gatelib.bus import BusClient  # noqa: E402
from gatelib.env import (  # noqa: E402
    REPO,
    have_binary,
    is_pi,
    load_robot_config,
    socket_alive,
)
from gatelib.runner import GateRun, Status, base_parser, metrics_path_for  # noqa: E402
from rover_contracts import StopMessage, UtteranceMessage  # noqa: E402

GATE = "G3"

SOAK_UTTERANCES = (
    "what do you see",
    "turn left ninety degrees",
    "say hello",
    "drive forward twenty centimeters",
    "describe the room",
)
"""One turn each, cycled: a vision call, a motion profile, speech, a short drive."""

BAKEOFF_DIR = REPO / "tests" / "fixtures" / "audio" / "bakeoff"


# ---------------------------------------------------------------------------
# Pi measurements
# ---------------------------------------------------------------------------


def vcgencmd(*args: str) -> str | None:
    if not have_binary("vcgencmd"):
        return None
    try:
        out = subprocess.run(
            ["vcgencmd", *args], capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None


def rss_kb(pattern: str) -> int | None:
    """Resident set of the first process matching ``pattern``, in KiB."""
    try:
        pids = subprocess.run(
            ["pgrep", "-f", pattern], capture_output=True, text=True, timeout=5
        ).stdout.split()
        total = 0
        for pid in pids:
            out = subprocess.run(
                ["ps", "-o", "rss=", "-p", pid],
                capture_output=True,
                text=True,
                timeout=5,
            ).stdout.strip()
            if out.isdigit():
                total += int(out)
        return total or None
    except (OSError, subprocess.SubprocessError):
        return None


class PiSamples:
    """The under-load samples G3a's clock criterion is scored on.

    ARCHITECTURE 13 is explicit that an idle-inclusive sample fails on a healthy
    board, because the ondemand governor idles the A72 at 600 MHz.  Samples are
    therefore taken only when the caller says the rover is doing something.
    """

    def __init__(self) -> None:
        self.throttled: list[int] = []
        self.arm_hz: list[int] = []
        self.temp_c: list[float] = []
        self.rss_kb: list[int] = []

    def take(self) -> None:
        throttled = vcgencmd("get_throttled")
        if throttled:
            match = re.search(r"0x([0-9a-fA-F]+)", throttled)
            if match:
                self.throttled.append(int(match.group(1), 16))
        clock = vcgencmd("measure_clock", "arm")
        if clock and "=" in clock:
            self.arm_hz.append(int(clock.split("=")[1]))
        temp = vcgencmd("measure_temp")
        if temp:
            match = re.search(r"([\d.]+)", temp)
            if match:
                self.temp_c.append(float(match.group(1)))
        total = rss_kb("rover-")
        if total:
            self.rss_kb.append(total)

    @property
    def any(self) -> bool:
        return bool(self.throttled or self.arm_hz or self.temp_c)


def arm_freq_hz() -> int | None:
    config = vcgencmd("get_config", "arm_freq")
    if config and "=" in config:
        return int(config.split("=")[1]) * 1_000_000
    return None


# ---------------------------------------------------------------------------
# The soak loop
# ---------------------------------------------------------------------------


def soak(
    gate: GateRun, config: Any, duration_s: float, sample_every: float = 5.0
) -> dict[str, Any]:
    """Drive turns for ``duration_s`` and collect everything observable."""
    brain_path = config.bus.brain_sock
    bus_path = config.bus.sock
    brain = None
    robotd = None
    if not socket_alive(brain_path) and not socket_alive(bus_path):
        gate.skip(
            "a",
            "A1",
            f"the {duration_s:.0f} s soak loop",
            f"a running brain on {brain_path} or robotd on {bus_path} -- "
            "there is nothing to soak",
        )
        return {
            "brain": False,
            "robotd": False,
            "samples": PiSamples(),
            "elapsed_s": 0.0,
            "turns": 0,
            "turn_failures": 0,
            "latencies": [],
            "mcu_age_ms": [],
            "arm_cycles": 0,
            "arm_cycles_per_hour": 0.0,
        }
    if socket_alive(brain_path):
        brain = BusClient(brain_path, timeout=15.0)
    if socket_alive(bus_path):
        robotd = BusClient(bus_path, timeout=5.0)
        robotd.hello("web", ["subscribe"], subscribe=["state", "result", "event"])

    samples = PiSamples()
    latencies: list[float] = []
    turns = 0
    failures = 0
    ages: list[int] = []
    arm_cycles = 0
    armed = None
    started = time.monotonic()
    next_sample = started

    while time.monotonic() - started < duration_s:
        if brain is not None:
            text = SOAK_UTTERANCES[turns % len(SOAK_UTTERANCES)]
            sent = time.monotonic()
            brain.send(
                UtteranceMessage(
                    source="cli",
                    text=text,
                    confidence=None,
                    is_final=True,
                    mono_ns=time.monotonic_ns(),
                )
            )
            speaking = None
            while time.monotonic() - sent < 12.0:
                # A quiet second is not an answer: brain publishes nothing while
                # a sentence is being spoken, and `say` on a four-word reply is
                # over a second.  The 12 s deadline is the bound.
                message = brain.read(1.0)
                if message is None:
                    continue
                if message.get("type") == "fsm" and message.get("state") in (
                    "SPEAKING_INTENT",
                    "SPEAKING_RESULT",
                ):
                    speaking = time.monotonic()
                    break
            turns += 1
            if speaking is None:
                failures += 1
            else:
                latencies.append(speaking - sent)
        else:
            time.sleep(0.5)

        if robotd is not None:
            for state in robotd.states(0.2):
                mcu = state.get("mcu") or {}
                if isinstance(mcu.get("age_ms"), int):
                    ages.append(mcu["age_ms"])
                now_armed = state.get("armed")
                if armed is not None and now_armed != armed:
                    arm_cycles += 1
                armed = now_armed

        if time.monotonic() >= next_sample:
            samples.take()
            next_sample = time.monotonic() + sample_every

    # Drain to a stop before letting go of the bus.  The loop breaks out of
    # each turn at SPEAKING_INTENT, so the last drive is still in flight; every
    # gate after this one shares the same simulated rover, and one that is
    # still driving is scored as the next gate's motion (G4-b2 measured exactly
    # that).  A gate must leave the rover where it found it.
    if robotd is not None:
        robotd.send(StopMessage(source="web", reason="gate_teardown"))
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            still = any(
                (state.get("mcu") or {}).get("motion")
                for state in robotd.states(0.3)
            )
            if not still:
                break

    for client in (brain, robotd):
        if client is not None:
            client.close()

    elapsed = time.monotonic() - started
    per_hour = round(arm_cycles * 3600.0 / elapsed, 2) if elapsed else 0.0
    return {
        "elapsed_s": round(elapsed, 1),
        "turns": turns,
        "turn_failures": failures,
        "latencies": latencies,
        "mcu_age_ms": ages,
        "arm_cycles": arm_cycles,
        "arm_cycles_per_hour": per_hour,
        "samples": samples,
        "brain": brain is not None,
        "robotd": robotd is not None,
    }


def score_soak(gate: GateRun, result: dict[str, Any], config: Any) -> None:
    samples: PiSamples = result["samples"]

    if result["brain"]:
        latencies = sorted(result["latencies"])
        if latencies:
            p50 = statistics.median(latencies)
            p95 = latencies[min(len(latencies) - 1, int(0.95 * len(latencies)))]
            gate.record(
                "a-latency",
                "A31",
                "utterance accepted to SPEAKING_INTENT (a proxy for EOS to first audio)",
                Status.PASS if result["turn_failures"] == 0 else Status.FAIL,
                f"{len(latencies)} turns, p50 {p50:.2f} s, p95 {p95:.2f} s "
                f"(ARCHITECTURE 9 gates 2.8 s p95 on the real metric, which needs "
                f"the audio device); {result['turn_failures']} turns produced no FSM "
                "transition",
                turns=result["turns"],
                turn_failures=result["turn_failures"],
                proxy_p50_s=round(p50, 3),
                proxy_p95_s=round(p95, 3),
            )
        else:
            gate.fail(
                "a-latency",
                "A31",
                "utterance accepted to SPEAKING_INTENT",
                f"{result['turns']} utterances sent, no FSM transition came back",
            )
    else:
        gate.skip(
            "a-latency",
            "A31",
            "utterance accepted to SPEAKING_INTENT",
            f"a running brain on {config.bus.brain_sock}",
        )

    if result["robotd"]:
        ages = result["mcu_age_ms"]
        if ages:
            ages_sorted = sorted(ages)
            p99 = ages_sorted[min(len(ages_sorted) - 1, int(0.99 * len(ages_sorted)))]
            gate.check(
                p99 < config.safety.link_alive_max_age_ms,
                "a-link",
                "T0",
                "telemetry age stays inside link_alive_max_age_ms",
                f"{len(ages)} samples, p99 {p99} ms "
                f"(< {config.safety.link_alive_max_age_ms})",
                mcu_age_p99_ms=p99,
                samples=len(ages),
            )
        gate.record(
            "a-relay",
            "A24",
            "arm/disarm cycles per hour, the upper bound on relay actuations",
            Status.PASS if result["arm_cycles_per_hour"] < 10 else Status.FAIL,
            f"{result['arm_cycles_per_hour']}/hour over {result['elapsed_s']} s. "
            "MOTOR_EN must not follow arm state (4.1), so the real relay count is "
            "lower; confirm by ear or scope",
            arm_cycles=result["arm_cycles"],
            arm_cycles_per_hour=result["arm_cycles_per_hour"],
        )
    else:
        for sub, name in (
            ("a-link", "telemetry age stays inside link_alive_max_age_ms"),
            ("a-relay", "arm/disarm cycles per hour"),
        ):
            gate.skip(sub, "T0", name, f"a running robotd on {config.bus.sock}")

    if not samples.any:
        for sub, inv, name in (
            ("a-throttle", "A1", "vcgencmd get_throttled == 0x0, no bit exclusions"),
            ("a-clock", "A1", "the ARM clock is at arm_freq for >= 99% of load samples"),
            ("a-temp", "A1", "SoC below 70 C"),
            ("a-rss", "A1", "RSS below 1.6 GB"),
        ):
            gate.skip(sub, inv, name, "hardware: vcgencmd on a Raspberry Pi")
        return

    worst = max(samples.throttled) if samples.throttled else 0
    gate.check(
        worst == 0,
        "a-throttle",
        "A1",
        "vcgencmd get_throttled == 0x0, no bit exclusions",
        f"worst 0x{worst:X} over {len(samples.throttled)} samples. "
        "0x2/0x20000 is the Pi 5 trigger and is never excluded",
        get_throttled_worst=worst,
    )
    target = arm_freq_hz()
    if target and samples.arm_hz:
        at_speed = sum(1 for hz in samples.arm_hz if hz >= target)
        share = at_speed / len(samples.arm_hz)
        gate.check(
            share >= 0.99,
            "a-clock",
            "A1",
            "the ARM clock is at arm_freq for >= 99% of load samples",
            f"{at_speed}/{len(samples.arm_hz)} = {share * 100:.1f}% at "
            f"{target / 1e6:.0f} MHz",
            arm_freq_hz=target,
            at_speed_share=round(share, 4),
        )
    else:
        gate.skip("a-clock", "A1", "the ARM clock is at arm_freq", "vcgencmd get_config")
    if samples.temp_c:
        peak = max(samples.temp_c)
        gate.check(
            peak < 70.0,
            "a-temp",
            "A1",
            "SoC below 70 C",
            f"peak {peak:.1f} C over {len(samples.temp_c)} samples",
            soc_peak_c=peak,
        )
    if samples.rss_kb:
        peak_kb = max(samples.rss_kb)
        gate.check(
            peak_kb < 1_600_000,
            "a-rss",
            "A1",
            "RSS below 1.6 GB",
            f"peak {peak_kb / 1024:.0f} MiB across the rover units",
            rss_peak_kb=peak_kb,
        )


# ---------------------------------------------------------------------------
# G3b -- the bake-off
# ---------------------------------------------------------------------------


def word_error_rate(reference: str, hypothesis: str) -> float:
    """Levenshtein over words, the standard WER."""
    ref = reference.lower().split()
    hyp = hypothesis.lower().split()
    if not ref:
        return 0.0 if not hyp else 1.0
    previous = list(range(len(hyp) + 1))
    for i, word in enumerate(ref, 1):
        current = [i]
        for j, other in enumerate(hyp, 1):
            current.append(
                min(
                    previous[j] + 1,
                    current[j - 1] + 1,
                    previous[j - 1] + (word != other),
                )
            )
        previous = current
    return previous[-1] / len(ref)


def bakeoff_corpus() -> list[tuple[Path, str]]:
    """Recorded WAVs with a sibling ``.txt`` transcript."""
    if not BAKEOFF_DIR.exists():
        return []
    out = []
    for wav in sorted(BAKEOFF_DIR.glob("*.wav")):
        transcript = wav.with_suffix(".txt")
        if transcript.exists():
            out.append((wav, transcript.read_text(encoding="utf-8").strip()))
    return out


def sub_b_stt(gate: GateRun, out_path: Path) -> None:
    """The bake-off: the corpus is checked here, the WER is computed here.

    ``tools/bakeoff_stt.py`` owns the four sherpa-onnx backends and is asked for
    one JSONL line per (backend, recording) carrying at least ``backend``,
    ``wav``, ``hypothesis``, and optionally ``rtf``, ``eos_to_final_s`` and
    ``rss_kb``.  Scoring stays in the gate so the number is computed by the
    thing that reports it.
    """
    corpus = bakeoff_corpus()
    name = "STT bake-off: WER, RTF, EOS to final and RSS per backend"
    if not corpus:
        gate.skip(
            "b-stt",
            "A27",
            name,
            f"recorded speech in {BAKEOFF_DIR.relative_to(REPO)}/ "
            "(one .wav plus a .txt transcript each) -- WER cannot be synthesized",
        )
        return
    runner = REPO / "tools" / "bakeoff_stt.py"
    if not runner.exists():
        gate.skip(
            "b-stt",
            "A27",
            name,
            "tools/bakeoff_stt.py, which owns the four backends "
            f"({len(corpus)} recordings are present and ready)",
            recordings=len(corpus),
        )
        return
    try:
        run = subprocess.run(
            [
                sys.executable,
                str(runner),
                "--corpus",
                str(BAKEOFF_DIR),
                "--out",
                str(out_path),
            ],
            capture_output=True,
            text=True,
            timeout=3600,
            cwd=REPO,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        gate.fail("b-stt", "A27", name, f"{type(exc).__name__}: {exc}")
        return
    if run.returncode != 0 or not out_path.exists():
        gate.fail(
            "b-stt", "A27", name, f"exit {run.returncode}: {run.stderr.strip()[-160:]}"
        )
        return

    truth = {wav.name: text for wav, text in corpus}
    per_backend: dict[str, list[float]] = {}
    extras: dict[str, dict[str, float]] = {}
    for line in out_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        reference = truth.get(Path(record.get("wav", "")).name)
        if reference is None:
            continue
        backend = record.get("backend", "?")
        per_backend.setdefault(backend, []).append(
            word_error_rate(reference, record.get("hypothesis", ""))
        )
        for key in ("rtf", "eos_to_final_s", "rss_kb"):
            if isinstance(record.get(key), (int, float)):
                extras.setdefault(backend, {}).setdefault(key, 0.0)
                extras[backend][key] = max(extras[backend][key], float(record[key]))
    if not per_backend:
        gate.fail("b-stt", "A27", name, f"{out_path.name} matched no recording")
        return
    summary = ", ".join(
        f"{backend} WER {100 * statistics.mean(wers):.1f}%"
        for backend, wers in sorted(per_backend.items())
    )
    gate.record(
        "b-stt",
        "A27",
        name,
        Status.PASS,
        f"{len(corpus)} recordings, {len(per_backend)} backends: {summary}. "
        "Reported, not gated: A27 picks the default at G3b",
        recordings=len(corpus),
        wer_by_backend={
            backend: round(statistics.mean(wers), 4)
            for backend, wers in per_backend.items()
        },
        extras_by_backend=extras,
    )


def sub_b_tts(gate: GateRun, config: Any) -> None:
    """Open items 4 and 10: does ``piper --output-raw`` stream, and how fast?"""
    binary = Path(config.tts.bin)
    if config.tts.backend != "piper" or not binary.exists():
        gate.skip(
            "b-tts",
            "A28",
            "piper --output-raw streams, and first audio on a four-word reply",
            f"[tts] backend=piper with a binary at {config.tts.bin} "
            "(the Mac default is backend='say')",
        )
        return
    started = time.monotonic()
    try:
        process = subprocess.Popen(
            [str(binary), "-m", config.tts.voice, "--output-raw"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        gate.fail("b-tts", "A28", "piper --output-raw streams", str(exc))
        return
    assert process.stdin is not None and process.stdout is not None
    process.stdin.write(b"Turning left now.\n")
    process.stdin.flush()
    first = process.stdout.read(4096)
    first_audio_s = time.monotonic() - started
    process.stdin.close()
    process.terminate()
    gate.record(
        "b-tts",
        "A28",
        "piper --output-raw streams, and first audio on a four-word reply",
        Status.PASS if first and first_audio_s < 1.0 else Status.FAIL,
        f"{len(first)} bytes after {first_audio_s * 1000:.0f} ms. If this is the "
        "whole utterance rather than a prefix, --output-raw buffers and the "
        "fallback is wyoming-piper as its own unit (open item 10)",
        first_audio_ms=round(first_audio_s * 1000, 1),
        first_chunk_bytes=len(first),
        voice=config.tts.voice,
        threads=config.tts.threads,
    )


HARDWARE_ONLY = (
    (
        "a-volt",
        "A1",
        "header voltage >= 5.0 V under the ReSpeaker's peak draw",
        "hardware: a USB-C inline power meter, which is what makes this measurable",
    ),
    (
        "a-fuser",
        "I-18",
        "fuser -v /dev/rover-mcu shows exactly one pid",
        "hardware: /dev/rover-mcu and psmisc",
    ),
    (
        "a-vgap",
        "OI-3",
        "p99 V inter-frame gap below 100 ms on a loaded Pi",
        "robotd's own writer metric, or a logic analyser on UART5 -- robotd owns "
        "the port, so nothing outside it can time the frames",
    ),
    (
        "a-cores",
        "A1",
        "rover-cam against A1's core budget, from top -H",
        "hardware: a Pi under the real camera load",
    ),
    (
        "b-wake",
        "A30",
        "the wake word runs and its false-accept rate is recorded",
        "hardware: a microphone, and a self-trained rover.tflite (open item 11)",
    ),
)


def sub_fuser(gate: GateRun, config: Any) -> None:
    """I-18: only robotd writes the port."""
    port = config.serial.port
    if not (is_pi() and Path(port).exists() and have_binary("fuser")):
        gate.skip(
            "a-fuser",
            "I-18",
            "fuser -v /dev/rover-mcu shows exactly one pid",
            "hardware: /dev/rover-mcu and psmisc",
        )
        return
    out = subprocess.run(["fuser", "-v", port], capture_output=True, text=True, timeout=5)
    pids = re.findall(r"\b(\d+)\b", out.stderr + out.stdout)
    gate.check(
        len(set(pids)) == 1,
        "a-fuser",
        "I-18",
        "fuser -v /dev/rover-mcu shows exactly one pid",
        f"{sorted(set(pids))} hold {port}",
        holders=sorted(set(pids)),
    )


def main() -> int:
    parser = base_parser(GATE, __doc__ or "")
    parser.add_argument(
        "--duration",
        type=float,
        default=1800.0,
        help="soak seconds (ARCHITECTURE 13 says 30 minutes)",
    )
    parser.add_argument("--sample-every", type=float, default=5.0)
    args = parser.parse_args()

    config, source = load_robot_config()
    gate = GateRun(
        GATE,
        metrics=metrics_path_for(GATE, args.metrics, REPO),
        strict=args.strict,
        hardware=args.hardware,
        only=args.only,
        mode="pi" if is_pi() else "mac",
    )
    print(f"config: {source}   soak: {args.duration:.0f} s   pi: {is_pi()}")

    if gate.selected("a"):
        result = soak(gate, config, args.duration, args.sample_every)
        score_soak(gate, result, config)
    if gate.selected("a-fuser"):
        sub_fuser(gate, config)
    if gate.selected("b-stt"):
        sub_b_stt(gate, gate.path.with_name(gate.path.stem + "-bakeoff.jsonl"))
    if gate.selected("b-tts"):
        sub_b_tts(gate, config)

    for sub, inv, name, reason in HARDWARE_ONLY:
        if sub == "a-fuser":
            continue
        if gate.selected(sub):
            gate.skip(sub, inv, name, reason)
    return gate.summary()


if __name__ == "__main__":
    raise SystemExit(main())
