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
import time
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
        "fw": "bot-wr-1",
        "hb_ok": True,
        "stop_flags": 6,
        "feedback_age_ms": 18,
        "cmd_left": 0.2,
        "cmd_right": 0.2,
        "heading_deg": 87.3,
        "yaw_rate_dps": 0.0,
        "roll_deg": 0.5,
        "pitch_deg": -1.2,
        "temp_c": 36.5,
        "clamp_count": 3,
        "motion": True,
    },
    "twist": {"lin": 0.2, "ang": 0.0},
    "front_m": 1.204,
    "bumper": False,
    "estop_sw": False,
    "battery": {"pack_v": 11.62, "pct": 62},
    "active": {
        "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD",
        "skill": "drive_for",
        "source": "brain",
        "progress": 0.34,
        "deadline_in_ms": 1500,
    },
    "budget": {"motion_s": 9.0},
    "ready": True,
    "reason": "",
}


@contextmanager
def fake_robotd(
    tmp_path: Path, replies: list[dict[str, object]], *, brain: bool = False
) -> Iterator[tuple[str, list[dict[str, object]]]]:
    """A socket that records validated client lines and sends ``replies``.

    Bound relative to a chdir'ed tmp dir: an absolute pytest tmp path overruns
    macOS's 104-byte ``sun_path``.
    """
    received: list[dict[str, object]] = []
    path = "brain.sock" if brain else "robotd.sock"
    adapter = brain_client_adapter if brain else client_adapter
    Path(path).unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)
    server.settimeout(0.1)
    stopped = threading.Event()
    errors: list[Exception] = []

    def serve() -> None:
        while not stopped.is_set():
            try:
                conn, _ = server.accept()
                break
            except TimeoutError:
                continue
            except OSError:
                return
        else:
            return
        with conn:
            for reply in replies:
                if reply.get("cmd_id") != "request":
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
                            adapter.validate_json(line)
                            message = json.loads(line)
                            received.append(message)
                            if message["type"] in {"skill", "request_skill"}:
                                cmd_id = message.get("cmd_id", message.get("request_id"))
                                for reply in replies:
                                    if reply.get("cmd_id") == "request":
                                        response = {**reply, "cmd_id": cmd_id}
                                        conn.sendall(
                                            (json.dumps(response) + "\n").encode()
                                        )
            except (TimeoutError, OSError):
                pass
            except Exception as exc:
                errors.append(exc)

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield path, received
    finally:
        stopped.set()
        server.close()
        thread.join(timeout=1.0)
        Path(path).unlink(missing_ok=True)
        assert not errors, errors
        assert not thread.is_alive()


@pytest.fixture()
def config_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[Path]:
    monkeypatch.chdir(tmp_path)
    path = tmp_path / "robot.toml"
    path.write_text('[robot]\nname = "rover"\n[bus]\nframes_sock = "frames.sock"\n')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind("frames.sock")
        server.listen(10)
        server.settimeout(0.1)
        stopped = threading.Event()

        def serve_frames() -> None:
            while not stopped.is_set():
                try:
                    conn, _ = server.accept()
                except TimeoutError:
                    continue
                with conn:
                    conn.settimeout(1)
                    assert json.loads(conn.recv(1024)) == {
                        "type": "still",
                        "plane": "lores",
                    }
                    header = {
                        "frame_id": "cam-000123",
                        "kind": "still",
                        "frame_mono_ns": time.monotonic_ns(),
                        "frame_wallclock_ns": time.time_ns(),
                        "w": 2,
                        "h": 2,
                        "fmt": "jpeg",
                        "quality": 80,
                        "bytes": 4,
                    }
                    conn.sendall(json.dumps(header).encode() + b"\nJPEG")

        thread = threading.Thread(target=serve_frames, daemon=True)
        thread.start()
        try:
            yield path
        finally:
            stopped.set()
            thread.join(timeout=2)
            assert not thread.is_alive()


def result(status: str, reason: str = "") -> dict[str, object]:
    return {
        "v": 1,
        "type": "result",
        "cmd_id": "request",
        "status": status,
        "reason": reason,
        "t_utc_ns": 1757260802230000000,
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
    assert goal_ttl_ms("turn_to", TurnToBusArgs(heading_deg=90.0), limits) == 4000
    quick = TurnToBusArgs(heading_deg=90.0, timeout_s=2.0)
    assert goal_ttl_ms("turn_to", quick, limits) == 2000


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
        "front_m": None,
        "active": None,
        "ready": False,
        "reason": "unpatched_firmware",
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
                "--config",
                str(config_file),
                "--sock",
                sock,
                "--source",
                "web",
                "drive-for",
                "1.0",
                "0.2",
                "--wait",
                "2",
            ]
        )
    assert code == 0
    assert [message["type"] for message in received if message["type"] != "ping"] == [
        "hello",
        "subscribe",
        "turn",
        "skill",
    ]
    skill = next(m for m in received if m["type"] == "skill")
    assert skill["skill"] == "drive_for"
    assert skill["args"] == {"duration_s": 1.0, "power": 0.2}
    assert skill["goal_ttl_ms"] == 2000
    assert skill["turn_id"] == received[2]["turn_id"]
    assert skill["source"] == "web"
    assert skill["obs"]["frame_id"] == "cam-000123"
    assert skill["trace"]["authorized_motion"] is True


def test_turn_to_carries_the_configured_timeout_and_tolerance(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            [
                "--config",
                str(config_file),
                "--sock",
                sock,
                "turn-to",
                "270",
                "--wait",
                "2",
            ]
        )
    assert code == 0
    skill = next(m for m in received if m["type"] == "skill")
    assert skill["skill"] == "turn_to"
    assert skill["args"] == {"heading_deg": 270.0, "timeout_s": 4.0, "tolerance_deg": 5.0}
    assert skill["goal_ttl_ms"] == 4000


def test_turn_to_options_override_the_config(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("done")]) as (sock, received):
        code = main(
            [
                "--config",
                str(config_file),
                "--sock",
                sock,
                "turn-to",
                "90",
                "--timeout",
                "2",
                "--tolerance",
                "10",
                "--wait",
                "2",
            ]
        )
    assert code == 0
    skill = next(m for m in received if m["type"] == "skill")
    assert skill["args"] == {"heading_deg": 90.0, "timeout_s": 2.0, "tolerance_deg": 10.0}
    assert skill["goal_ttl_ms"] == 2000


def test_say_is_the_say_skill(config_file: Path) -> None:
    with fake_brain([result("done")]) as (sock, received):
        code = main(
            [
                "--config",
                str(config_file),
                "--brain-sock",
                sock,
                "say",
                "stop",
                "there",
                "--wait",
                "2",
            ]
        )
    assert code == 0
    assert received[-1]["type"] == "request_skill"
    assert received[-1]["call"]["skill"] == "say"
    assert received[-1]["call"]["args"] == {"text": "stop there"}


def test_a_rejected_result_is_a_nonzero_exit(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [result("rejected", "goal_ttl_too_long")]) as (
        sock,
        _,
    ):
        code = main(
            [
                "--config",
                str(config_file),
                "--sock",
                sock,
                "drive-for",
                "1.0",
                "0.2",
                "--wait",
                "2",
            ]
        )
    assert code == 1


@pytest.mark.parametrize("reply", [None, "accepted", "unrelated_done"])
def test_only_own_terminal_result_completes_motion_and_waiting_renews_liveness(
    config_file: Path,
    reply: str | None,
) -> None:
    replies = (
        [] if reply is None else [result("accepted" if reply == "accepted" else "done")]
    )
    if reply == "unrelated_done":
        replies[0]["cmd_id"] = "01J9ZC7K3QF2M8XR4V6T0YAHBD"
    with fake_robotd(config_file.parent, replies) as (sock, received):
        assert (
            main(
                [
                    "--config",
                    str(config_file),
                    "--sock",
                    sock,
                    "drive-for",
                    "1",
                    "0.2",
                    "--wait",
                    "0.5",
                ]
            )
            == 1
        )
    skill = next(m for m in received if m["type"] == "skill")
    assert len([m for m in received if m["type"] == "ping"]) >= 2
    assert received[-1]["type"] == "cancel"
    assert received[-1]["cmd_id"] == skill["cmd_id"]


def test_unfinished_brain_request_cancels_only_itself(config_file: Path) -> None:
    with fake_brain([result("accepted")]) as (sock, received):
        assert (
            main(
                [
                    "--config",
                    str(config_file),
                    "--brain-sock",
                    sock,
                    "say",
                    "hello",
                    "--wait",
                    "0.1",
                ]
            )
            == 1
        )
    assert received[-1]["type"] == "cancel"
    assert received[-1]["request_id"] == received[0]["request_id"]


def test_motion_without_camera_never_reaches_robotd(config_file: Path) -> None:
    config_file.write_text('[bus]\nframes_sock = "absent-frames.sock"\n')
    with fake_robotd(config_file.parent, []) as (sock, received):
        assert (
            main(
                [
                    "--config",
                    str(config_file),
                    "--sock",
                    sock,
                    "drive-for",
                    "1",
                    "0.2",
                    "--wait",
                    "0.1",
                ]
            )
            == 2
        )
    assert received == []


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
        assert (
            main(
                [
                    "--config",
                    str(config_file),
                    "--sock",
                    sock,
                    "estop",
                    "--reason",
                    "user",
                ]
            )
            == 0
        )
    assert received[-1] == {"v": 1, "type": "estop", "source": "web", "reason": "user"}


def test_clear_names_the_faults(config_file: Path) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        assert (
            main(
                [
                    "--config",
                    str(config_file),
                    "--sock",
                    sock,
                    "clear",
                    "estop_sw",
                    "low_battery",
                ]
            )
            == 0
        )
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
                "--config",
                str(config_file),
                "--sock",
                sock,
                "watch",
                "--count",
                "1",
                "--seconds",
                "3",
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
    with fake_brain([result("done")]) as (sock, received):
        code = main(
            [
                "--config",
                str(config_file),
                "--brain-sock",
                sock,
                "skill",
                "find",
                "--args",
                '{"object":"red mug","max_sweeps":3}',
                "--wait",
                "2",
            ]
        )
    assert code == 0
    assert received[-1]["call"]["skill"] == "find"
    assert received[-1]["call"]["args"] == {"object": "red mug", "max_sweeps": 3}


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
def fake_brain(
    replies: list[dict[str, object]],
) -> Iterator[tuple[str, list[dict[str, object]]]]:
    with fake_robotd(Path("."), replies, brain=True) as server:
        yield server


def test_utter_is_the_5_9_utterance_with_a_null_confidence(config_file: Path) -> None:
    face = {"v": 1, "type": "face", "expr": "thinking"}
    with fake_brain([face]) as (sock, received):
        code = main(
            [
                "--config",
                str(config_file),
                "--brain-sock",
                sock,
                "utter",
                "turn",
                "left",
                "ninety",
                "degrees",
                "--watch",
                "1",
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
