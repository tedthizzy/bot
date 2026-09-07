"""roverctl speaks the bus the architecture describes, and refuses the port.

The stand-in robotd here validates every line it receives with
``client_adapter`` -- the same adapter the real robotd will use -- so the test
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
    DriveBusArgs,
    LimitsConfig,
    StateMessage,
    TurnBusArgs,
    brain_client_adapter,
    client_adapter,
)
from rover_devtools.roverctl import goal_ttl_ms, main, render_state  # noqa: E402

STATE = {
    "v": 1,
    "type": "state",
    "t_utc_ns": 1757260800120000000,
    "t_mono_ns": 98765432100,
    "mcu": {
        "state": "ARMED_MOVING", "fault": 0, "session": 40010, "age_ms": 18,
        "last_ack_seq": 3, "loop_late_pct": 0, "rx_drop": 0, "motion": True,
    },
    "armed": True,
    "pose": {"frame_id": "odom", "x_m": 1.42, "y_m": -0.30, "yaw_rad": 1.518},
    "twist": {"linear_x_mps": 0.248, "angular_z_radps": 0.208},
    "wheels": {
        "left_ticks": 204411, "right_ticks": 203877, "ticks_per_rev": 2200,
        "wheel_radius_m": 0.045, "track_m": 0.150,
    },
    "ranges_m": {"front": 1.204, "cliff": 0.098},
    "front_at_max": False,
    "tof": {"front_l_ok": True, "front_r_ok": True},
    "bumper": False, "estop_hw": False, "estop_sw": False,
    "battery": {"pack_v": 11.62, "oc_v": 11.70, "current_a": 0.41, "pct": 62},
    "rails": {"servo": False},
    "active": {
        "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD", "skill": "drive", "source": "brain",
        "progress": 0.34, "deadline_in_ms": 3500,
    },
    "budget": {"path_m": 1.1, "motion_s": 9.0},
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


@pytest.fixture()
def config_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.chdir(tmp_path)
    path = tmp_path / "robot.toml"
    path.write_text('[robot]\nname = "rover"\n')
    return path


# --------------------------------------------------------------------------
# The T2 estimate
# --------------------------------------------------------------------------


def test_goal_ttl_is_5_2s_worked_example() -> None:
    limits = LimitsConfig()
    args = DriveBusArgs(distance_m=0.40, speed_mps=0.15)
    assert goal_ttl_ms("drive", args, limits) == 4500


def test_an_unexecutable_but_legal_drive_is_capped_not_hidden() -> None:
    # drive(100 cm, 5 cm/s) is 30.5 s: capped at goal_ttl_ms_max here so robotd
    # answers goal_ttl_too_long, which is the reason the operator needs to see.
    limits = LimitsConfig()
    args = DriveBusArgs(distance_m=1.0, speed_mps=0.05)
    assert goal_ttl_ms("drive", args, limits) == limits.goal_ttl_ms_max


def test_turn_uses_its_own_profile() -> None:
    limits = LimitsConfig()
    args = TurnBusArgs(angle_deg=90.0, rate_dps=60.0)
    assert goal_ttl_ms("turn", args, limits) == 2750


def test_a_non_motion_skill_gets_the_configured_maximum() -> None:
    limits = LimitsConfig()
    assert goal_ttl_ms("say", DriveBusArgs(distance_m=0.1, speed_mps=0.1), limits) == (
        limits.goal_ttl_ms_max
    )


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def test_render_state_shows_the_fields_an_operator_watches() -> None:
    line = render_state(StateMessage.model_validate(STATE))
    assert "ARMED_MOVING" in line
    assert "armed=True" in line
    assert "front=1.20m" in line
    assert "bat=62%" in line
    assert "active=drive:34%/3500ms" in line


def test_render_state_shows_a_stale_range_as_stale() -> None:
    stale = {**STATE, "ranges_m": {"front": None, "cliff": 0.098}}
    assert "front=stale" in render_state(StateMessage.model_validate(stale))


# --------------------------------------------------------------------------
# The wire
# --------------------------------------------------------------------------


def test_a_skill_sends_hello_subscribe_turn_then_skill(config_file: Path) -> None:
    result = {
        "v": 1, "type": "result", "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD",
        "status": "done", "reason": "", "t_utc_ns": 1757260802230000000,
    }
    with fake_robotd(config_file.parent, [result]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock, "--source", "web",
                "skill", "drive", "--args", '{"distance_m":0.4,"speed_mps":0.15}',
                "--wait", "2",
            ]
        )
    assert code == 0
    assert [message["type"] for message in received] == [
        "hello", "subscribe", "turn", "skill"
    ]
    skill = received[-1]
    assert skill["skill"] == "drive"
    assert skill["args"] == {"distance_m": 0.4, "speed_mps": 0.15}
    assert skill["goal_ttl_ms"] == 4500
    assert skill["turn_id"] == received[2]["turn_id"]
    assert skill["source"] == "web"


def test_a_rejected_result_is_a_nonzero_exit(config_file: Path) -> None:
    result = {
        "v": 1, "type": "result", "cmd_id": "01J9ZC7K3QF2M8XR4V6T0YAHBD",
        "status": "rejected", "reason": "goal_ttl_too_long",
        "t_utc_ns": 1757260802230000000,
    }
    with fake_robotd(config_file.parent, [result]) as (sock, _):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "skill", "drive", "--args", '{"distance_m":0.4,"speed_mps":0.15}',
                "--wait", "2",
            ]
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
            ["--config", str(config_file), "--sock", sock, "clear", "estop_sw"]
        ) == 0
    assert received[-1]["faults"] == ["estop_sw"]


def test_an_unknown_clearable_fault_is_refused_locally(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        code = main(["--config", str(config_file), "--sock", sock, "clear", "gremlins"])
    assert code == 2
    assert "clearable faults" in capsys.readouterr().err
    assert [m["type"] for m in received] == ["hello"] or received == []


def test_state_stops_after_the_requested_count(config_file: Path) -> None:
    with fake_robotd(config_file.parent, [STATE, STATE]) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "state", "--count", "1", "--seconds", "3",
            ]
        )
    assert code == 0
    assert [m["type"] for m in received] == ["hello", "subscribe"]


def test_bad_skill_args_never_reach_the_socket(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    with fake_robotd(config_file.parent, []) as (sock, received):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "skill", "drive", "--args", '{"distance_m":9.0,"speed_mps":0.15}',
            ]
        )
    assert code == 2
    assert "bad args for drive" in capsys.readouterr().err
    assert received == []


# --------------------------------------------------------------------------
# I-18
# --------------------------------------------------------------------------


def test_arm_refuses_while_robotd_holds_the_port(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    with fake_robotd(config_file.parent, []) as (sock, _):
        code = main(
            [
                "--config", str(config_file), "--sock", sock,
                "arm", "--device", "/dev/null",
            ]
        )
    assert code == 2
    assert "I-18" in capsys.readouterr().err


def test_arm_reports_a_missing_device(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    code = main(
        [
            "--config", str(config_file), "--sock", "no-such.sock",
            "arm", "--device", str(config_file.parent / "no-such-device"),
        ]
    )
    assert code == 2
    assert "cannot open" in capsys.readouterr().err


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


def test_log_tails_the_newest_jsonl_in_the_configured_directory(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    logs = config_file.parent / "logs"
    logs.mkdir()
    (logs / "robotd-2026-09-07.jsonl").write_text(
        '{"type":"result","status":"done"}\n{"type":"event","kind":"arm_ok"}\n'
    )
    config_file.write_text(f'[log]\ndir = "{logs}"\n')
    assert main(["--config", str(config_file), "log", "-n", "1"]) == 0
    captured = capsys.readouterr()
    assert captured.out == '{"type":"event","kind":"arm_ok"}\n'
    assert "robotd-2026-09-07.jsonl" in captured.err


def test_log_says_which_directory_is_empty(
    config_file: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    empty = config_file.parent / "logs"
    empty.mkdir()
    config_file.write_text(f'[log]\ndir = "{empty}"\n')
    assert main(["--config", str(config_file), "log"]) == 1
    assert "no *.jsonl" in capsys.readouterr().err
