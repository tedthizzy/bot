"""roverctl speaks the bus the architecture describes, and nothing else.

The stand-in robotd here validates every line it receives with
``client_adapter`` -- the same adapter the real robotd uses -- so the test
fails if roverctl invents a field, drops one, or gets the order wrong.
"""

from __future__ import annotations

import json
import socket
import sys
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts import (  # noqa: E402
    DriveForBusArgs,
    LimitsConfig,
    SayBusArgs,
    StateMessage,
    TurnToBusArgs,
    brain_client_adapter,
    client_adapter,
)
from rover_devtools.roverctl import (  # noqa: E402
    goal_ttl_ms,
    main,
    render_state,
    stop_flag_names,
)

STATE = {
    "v": 1,
    "type": "state",
    "t_utc_ns": 1757260800120000000,
    "t_mono_ns": 98765432100,
    "rover": {
        "fw": "bot-wr-1", "hb_ok": True, "stop_flags": 6, "feedback_age_ms": 18,
        "cmd_left": 0.2, "cmd_right": 0.2, "heading_deg": 87.3, "yaw_rate_dps": 0.0,
        "roll_deg": 0.5, "pitch_deg": -1.2, "temp_c": 36.5, "clamp_count": 3,
        "motion": True,
    },
    "twist": {"lin": 0.2, "ang": 0.0},
    "front_m": 1.204,
    "bumper": False, "estop_sw": False,
    "battery": {"pack_v": 11.62, "pct": 62},
    "active": {
        "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD", "skill": "drive_for", "source": "brain",
        "progress": 0.34, "deadline_in_ms": 1500,
    },
    "budget": {"motion_s": 9.0},
    "ready": True, "reason": "",
}


@contextmanager
def fake_robotd(tmp_path: Path, replies: list[dict[str, object]]
                ) -> Iterator[tuple[str, list[dict[str, object]]]]:
    """A socket that records validated client lines and sends ``replies``.

    Bound relative to a chdir'ed tmp dir: an absolute pytest tmp path overruns
    macOS's 104-byte ``sun_path``.
    """
    received: list[dict[str, object]] = []
    Path("robotd.sock").unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind("robotd.sock")
    server.listen(1)

    def serve() -> None:
        try:
            conn, _ = server.accept()
        except OSError:
            return  # the client exited before connecting; nothing to record
        with conn:
            for reply in replies:
                conn.sendall((json.dumps(reply) + "\n").encode())
            buf = bytearray()
            conn.settimeout(5.0)
            try:
                while True:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, _, rest = bytes(buf).partition(b"\n")
                        buf = bytearray(rest)
                        if line.strip():
                            client_adapter.validate_json(line)
                            received.append(json.loads(line))
            except (TimeoutError, OSError):
                pass

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield "robotd.sock", received
    finally:
        server.close()
        thread.join(timeout=3.0)
        Path("robotd.sock").unlink(missing_ok=True)


@pytest.fixture()
def config_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.chdir(tmp_path)
    path = tmp_path / "robot.toml"
    path.write_text('[robot]\nname = "rover"\n')
    return path


def result(status: str, reason: str = "") -> dict[str, object]:
    return {
        "v": 1, "type": "result", "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD",
        "status": status, "reason": reason, "t_utc_ns": 1757260802230000000,
    }


# --------------------------------------------------------------------------
# The T2 estimate
# --------------------------------------------------------------------------


def test_goal_ttl_is_the_catalogs_deadline_formula() -> None:
    limits = LimitsConfig()
    one = DriveForBusArgs(duration_s=1.0, power=0.2)
    two = DriveForBusArgs(duration_s=2.0, power=0.2)
    assert goal_ttl_ms("drive_for", one, limits) == 2000
    assert goal_ttl_ms("drive_for", two, limits) == 3500


def test_a_turn_uses_its_timeout_and_is_capped_at_the_maximum() -> None:
    limits = LimitsConfig()
    # 4.0 * 1.5 + 0.5 = 6.5 s, over goal_ttl_ms_max, so robotd sees the cap and
    # answers goal_ttl_too_long itself rather than roverctl hiding it.
    assert goal_ttl_ms("turn_to", TurnToBusArgs(heading_deg=90.0), limits) == (
        limits.goal_ttl_ms_max
    )
    quick = TurnToBusArgs(heading_deg=90.0, timeout_s=2.0)
    assert goal_ttl_ms("turn_to", quick, limits) == 3500


def test_a_non_motion_skill_gets_the_configured_maximum() -> None:
    limits = LimitsConfig()
    assert goal_ttl_ms("say", SayBusArgs(text="hi"), limits) == limits.goal_ttl_ms_max


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def test_render_state_shows_the_rover_fields_an_operator_watches() -> None:
    line = render_state(StateMessage.model_validate(STATE))
    assert "fw=bot-wr-1" in line
    assert "hb=ok" in line
    assert "flags=tof|bumper" in line
    assert "heading=+87deg" in line
    assert "front=1.20m" in line
    assert "bat=11.6V/62%" in line
    assert "budget=9.0s" in line
    assert "active=drive_for:34%/1500ms" in line


def test_render_state_shows_stock_firmware_a_lost_heartbeat_and_no_range() -> None:
    stock = {
        **STATE,
        "rover": {**STATE["rover"], "fw": None, "hb_ok": False, "stop_flags": 1},
        "front_m": None, "active": None, "ready": False, "reason": "unpatched_firmware",
    }
    line = render_state(StateMessage.model_validate(stock))
    assert "fw=stock hb=LOST flags=heartbeat" in line
    assert "front=none" in line
    assert "ready=False (unpatched_firmware)" in line
    assert "active=" not in line


def test_stop_flags_are_named_in_bit_order() -> None:
    assert stop_flag_names(0) == "-"
    assert stop_flag_names(31) == "heartbeat|tof|bumper|lowbat|coast"
    assert stop_flag_names(8) == "lowbat"


# --------------------------------------------------------------------------
# The wire
# --------------------------------------------------------------------------


def test_drive_for_sends_hello_subscribe_turn_then_skill(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock, "--source", "web",
                "drive-for", "1.0", "0.2", "--wait", "2",
            ]
        )
    assert code == 0
    assert [message["type"] for message in received] == [
        "hello", "subscribe", "turn", "skill"
    ]
    skill = received[-1]
    assert skill["skill"] == "drive_for"
    assert skill["args"] == {"duration_s": 1.0, "power": 0.2}
    assert skill["goal_ttl_ms"] == 2000
    assert skill["turn_id"] == received[2]["turn_id"]
    assert skill["source"] == "web"


def test_turn_to_carries_the_configured_timeout_and_tolerance(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            ["--config", str(config_file), "--sock", sock,
             "turn-to", "270", "--wait", "2"]
        )
    assert code == 0
    skill = received[-1]
    assert skill["skill"] == "turn_to"
    assert skill["args"] == {"heading_deg": 270.0, "timeout_s": 4.0, "tolerance_deg": 5.0}
    assert skill["goal_ttl_ms"] == 5000


def test_turn_to_options_override_the_config(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "turn-to", "90", "--timeout", "2", "--tolerance", "10", "--wait", "2",
            ]
        )
    assert code == 0
    skill = received[-1]
    assert skill["args"] == {"heading_deg": 90.0, "timeout_s": 2.0, "tolerance_deg": 10.0}
    assert skill["goal_ttl_ms"] == 3500


def test_say_is_the_say_skill(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            ["--config", str(config_file), "--sock", sock,
             "say", "hello", "there", "--wait", "2"]
        )
    assert code == 0
    assert received[-1]["skill"] == "say"
    assert received[-1]["args"] == {"text": "hello there"}


def test_a_rejected_result_is_a_nonzero_exit(config_file: Path) -> None:
    with fake_robotd(
        config_file.parent, [result("rejected", "goal_ttl_too_long")]
    ) as (sock, _):
        code = main(
            ["--config", str(config_file), "--sock", sock, "drive-for", "1.0", "0.2",
             "--wait", "2"]
        )
    assert code == 1


def test_stop_carries_no_seq_cmd_id_or_turn_id(config_file: Path) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        assert main(["--config", str(config_file), "--sock", sock, "stop"]) == 0
    stop = received[-1]
    assert stop["type"] == "stop"
    # The loosest message on the bus: roverctl supplies none of the three, so a
    # stop needs no turn to exist and never has to pass strict parsing (I-22).
    assert stop["seq"] is None
    assert stop["cmd_id"] is None
    assert stop["turn_id"] is None


def test_estop_is_its_own_message(config_file: Path) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        assert main(
            ["--config", str(config_file), "--sock", sock, "estop", "--reason", "user"]
        ) == 0
    assert received[-1] == {
        "v": 1, "type": "estop", "source": "web", "reason": "user"
    }


def test_clear_names_the_faults(config_file: Path) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        assert main(
            ["--config", str(config_file), "--sock", sock,
             "clear", "estop_sw", "low_battery"]
        ) == 0
    assert received[-1]["faults"] == ["estop_sw", "low_battery"]


def test_an_unknown_clearable_fault_is_refused_locally(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        code = main(["--config", str(config_file), "--sock", sock, "clear", "gremlins"])
    assert code == 2
    assert "clearable faults" in capsys.readouterr().err
    assert [m["type"] for m in received] == ["hello"] or received == []


def test_watch_stops_after_the_requested_count(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    with fake_robotd(config_file.parent, [STATE, STATE]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "watch", "--count", "1", "--seconds", "3",
            ]
        )
    assert code == 0
    assert [m["type"] for m in received] == ["hello", "subscribe"]
    out = capsys.readouterr().out
    assert out.count("state   ") == 1
    assert "fw=bot-wr-1 hb=ok flags=tof|bumper" in out


def test_bad_skill_args_never_reach_the_socket(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    for argv in (
        ["drive-for", "5", "0.2"],  # over drive_for_max_s
        ["drive-for", "1", "0.9"],  # over the 0.30 cap
        ["drive-for", "1", "0"],  # zero power is not a drive
        ["turn-to", "360"],  # heading is [0, 360)
    ):
        with fake_robotd(config_file.parent, []) as (sock, received):
            code = main(["--config", str(config_file), "--sock", sock, *argv])
        assert code == 2, argv
        assert "bad args for" in capsys.readouterr().err
        assert received == []


def test_skill_dispatches_any_bus_skill_from_json(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "skill", "find", "--args", '{"object":"red mug","max_sweeps":3}',
                "--wait", "2",
            ]
        )
    assert code == 0
    assert received[-1]["skill"] == "find"
    assert received[-1]["args"] == {"object": "red mug", "max_sweeps": 3}
    assert received[-1]["goal_ttl_ms"] == LimitsConfig().goal_ttl_ms_max


def test_nothing_listening_is_reported(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    code = main(["--config", str(config_file), "--sock", "no-such.sock", "stop"])
    assert code == 2
    assert "roverctl:" in capsys.readouterr().err


# --------------------------------------------------------------------------
# brain.sock and the log
# --------------------------------------------------------------------------


@contextmanager
def fake_brain(replies: list[dict[str, object]]
               ) -> Iterator[tuple[str, list[dict[str, object]]]]:
    """A stand-in brain: validates with ``brain_client_adapter`` (5.9)."""
    received: list[dict[str, object]] = []
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind("brain.sock")
    server.listen(1)

    def serve() -> None:
        try:
            conn, _ = server.accept()
        except OSError:
            return
        with conn:
            for reply in replies:
                conn.sendall((json.dumps(reply) + "\n").encode())
            conn.settimeout(3.0)
            try:
                for line in conn.recv(65536).split(b"\n"):
                    if line.strip():
                        brain_client_adapter.validate_json(line)
                        received.append(json.loads(line))
            except (TimeoutError, OSError):
                pass

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield "brain.sock", received
    finally:
        server.close()
        thread.join(timeout=3.0)


def test_utter_is_the_5_9_utterance_with_a_null_confidence(config_file: Path) -> None:
    face = {"v": 1, "type": "face", "expr": "thinking"}
    with fake_brain([face]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--brain-sock", sock,
                "utter", "turn", "left", "ninety", "degrees", "--watch", "1",
            ]
        )
    assert code == 0
    utterance = received[-1]
    assert utterance["type"] == "utterance"
    assert utterance["source"] == "cli"
    assert utterance["text"] == "turn left ninety degrees"
    # A null confidence counts as authorized in ARCHITECTURE 7's rule, which is
    # what lets the whole text-mode Mac test plan move the wheels.
    assert utterance["confidence"] is None
    assert utterance["is_final"] is True


def test_tail_reads_the_newest_jsonl_in_the_configured_directory(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    logs = config_file.parent / "logs"
    logs.mkdir()
    (logs / "robotd-2026-09-07.jsonl").write_text(
        '{"type":"result","status":"done"}\n{"type":"event","kind":"link_up"}\n'
    )
    config_file.write_text(f'[log]\ndir = "{logs}"\n')
    assert main(["--config", str(config_file), "tail", "-n", "1"]) == 0
    captured = capsys.readouterr()
    assert captured.out == '{"type":"event","kind":"link_up"}\n'
    assert "robotd-2026-09-07.jsonl" in captured.err


def test_tail_says_which_directory_is_empty(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    empty = config_file.parent / "logs"
    empty.mkdir()
    config_file.write_text(f'[log]\ndir = "{empty}"\n')
    assert main(["--config", str(config_file), "tail"]) == 1
    assert "no *.jsonl" in capsys.readouterr().err
