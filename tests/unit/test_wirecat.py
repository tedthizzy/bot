"""wirecat renders what the codec decoded, and nothing it invented.

The frames come from ``tests/contract/serial_vectors.jsonl`` -- the same golden
lines the C firmware core is built against -- so a rendering that drifts from
the wire is caught here rather than at deploy step 10, where the ``B`` banner is
the only thing standing between an operator and a wrongly flashed MCU.
"""

from __future__ import annotations

import json
import os
import pty
import shutil
import socket
import sys
import tempfile
import tty
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.serial_codec import (  # noqa: E402
    AckReason,
    DecodeErr,
    DecodeOk,
    decode_line,
)
from rover_devtools.wirecat import main, open_serial, render  # noqa: E402

VECTORS = Path(__file__).resolve().parents[1] / "contract" / "serial_vectors.jsonl"


def frames() -> dict[str, list[str]]:
    """Every golden frame line, grouped by type letter in document order.

    There are two ``T`` frames and three ``K`` frames in ARCHITECTURE 5.1, and
    they say different things, so the index matters.
    """
    lines: dict[str, list[str]] = {}
    for raw in VECTORS.read_text().splitlines():
        row = json.loads(raw)
        if row.get("kind") == "frame":
            lines.setdefault(row["type"], []).append(row["line"])
    return lines


@pytest.fixture(scope="module")
def golden() -> dict[str, list[str]]:
    return frames()


def rendered(line: str) -> str:
    result = decode_line(line.encode())
    assert isinstance(result, DecodeOk), result
    return render(result)


def test_the_boot_banner_shows_what_deploy_step_10_reads(
    golden: dict[str, list[str]],
) -> None:
    text = rendered(golden["B"][0])
    assert "fw=0.1.0" in text
    assert "proto=2" in text
    assert "caps=0x3[DEBUG_BUILD|CLIFF_SENSOR]" in text
    assert "safety_hash=3381018647" in text


def test_telemetry_names_the_state_the_flags_and_the_faults(
    golden: dict[str, list[str]],
) -> None:
    text = rendered(golden["T"][0])
    assert "ARMED_MOVING" in text
    assert "cap_clamped" in text
    assert "TOF_FL_OK" in text and "TOF_FR_OK" in text
    assert "v=248/255mm/s" in text


def test_an_ack_names_the_acked_type_and_the_reason(
    golden: dict[str, list[str]],
) -> None:
    arm_ack, velocity_ack = rendered(golden["K"][0]), rendered(golden["K"][1])
    assert "acks A seq=2 OK reason=NONE echo=90210" in arm_ack
    assert "acks V seq=4 CLAMPED reason=CAP_EXCEEDED echo=0" in velocity_ack


def test_an_event_names_its_code(golden: dict[str, list[str]]) -> None:
    assert "CAL_STORED" in rendered(golden["E"][0])


def test_a_velocity_frame_shows_its_ttl_and_flags(
    golden: dict[str, list[str]],
) -> None:
    text = rendered(golden["V"][0])
    assert "ttl=300ms" in text
    assert "flags=0x0[-]" in text


def test_a_clear_names_the_latched_bits(golden: dict[str, list[str]]) -> None:
    assert "overcurrent|stall" in rendered(golden["C"][0])


def test_every_golden_frame_renders(golden: dict[str, list[str]]) -> None:
    for group in golden.values():
        for line in group:
            assert rendered(line).strip()


def test_a_rejected_line_shows_its_reason_rather_than_vanishing() -> None:
    result = decode_line(b"$V,2,3,40010,250,210,300,0*0000")
    assert isinstance(result, DecodeErr)
    text = render(result)
    assert AckReason.BAD_CRC.name in text
    assert "CRC mismatch" in text


def test_raw_round_trips_the_frame_bytes(golden: dict[str, list[str]]) -> None:
    result = decode_line(golden["S"][0].encode())
    assert isinstance(result, DecodeOk)
    assert golden["S"][0].strip() in render(result, raw=True)


def test_open_serial_reads_a_pty_slave_raw() -> None:
    master, slave = pty.openpty()
    tty.setraw(slave)
    device = os.ttyname(slave)
    fd = open_serial(device, 921600)
    try:
        os.write(master, b"$D,2,8,40010*DFCC\n")
        assert os.read(fd, 4096) == b"$D,2,8,40010*DFCC\n"
    finally:
        os.close(fd)
        os.close(slave)
        os.close(master)


def test_a_missing_device_is_reported_not_raised(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main([str(tmp_path / "nope")]) == 2
    assert "cannot open" in capsys.readouterr().err


def test_count_stops_and_the_config_hash_is_asserted(
    tmp_path: Path, golden: dict[str, list[str]], capsys: pytest.CaptureFixture[str]
) -> None:
    device = tmp_path / "frames"
    device.write_bytes(golden["B"][0].encode())
    config = tmp_path / "robot.toml"
    config.write_text('[robot]\nname = "rover"\n')
    assert main([str(device), "--count", "1", "--config", str(config)]) == 0
    captured = capsys.readouterr()
    assert "safety_hash MATCH" in captured.out
    assert "1 ok, 0 dropped" in captured.err


# ---------------------------------------------------------------------------
# I-18: only robotd writes the port
# ---------------------------------------------------------------------------


def test_the_default_fd_cannot_write(tmp_path: Path) -> None:
    """Read-only is the default, and it is the fd itself rather than a promise
    in the help string: a second writer advancing the MCU's last_down makes
    robotd's next V stale, and at 20 Hz roughly every other V is dropped."""
    master, slave = pty.openpty()
    tty.setraw(master)
    try:
        fd = open_serial(os.ttyname(slave), 921600)
        try:
            with pytest.raises(OSError):
                os.write(fd, b"$P,2,1,40010,1*0000\n")
        finally:
            os.close(fd)
    finally:
        os.close(slave)
        os.close(master)


def test_ping_refuses_while_something_is_listening_on_the_bus(
    tmp_path: Path, golden: dict[str, list[str]], capsys: pytest.CaptureFixture[str]
) -> None:
    """The guard roverctl's arm/disarm already has (I-18).  Without it a
    `wirecat --ping` run during a drive is a second writer into the motor
    controller, and a diagnostic tool stops the robot."""
    # A short directory: an AF_UNIX path is capped at ~104 bytes and pytest's
    # tmp_path is longer than that on macOS.
    run_dir = Path(tempfile.mkdtemp(prefix="wc"))
    sock_path = run_dir / "robotd.sock"
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(sock_path))
    listener.listen(1)
    config = tmp_path / "robot.toml"
    config.write_text(f'[bus]\nsock = "{sock_path}"\n')
    device = tmp_path / "frames"
    device.write_bytes(golden["B"][0].encode())
    try:
        assert main([str(device), "--ping", "--config", str(config)]) == 2
        err = capsys.readouterr().err
        assert "Only robotd writes the port" in err
        # ...and read-only still works against the same live bus.
        assert main([str(device), "--count", "1", "--config", str(config)]) == 0
    finally:
        listener.close()
        shutil.rmtree(run_dir, ignore_errors=True)
