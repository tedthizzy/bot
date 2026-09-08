"""wirecat renders what ``wave_proto`` decoded, and nothing it invented.

The lines are the protocol document's own examples, so a rendering that drifts
from the wire is caught here rather than on the bench, where the banner is the
only thing standing between an operator and a controller running the wrong
firmware.
"""

from __future__ import annotations

import os
import pty
import shutil
import socket
import sys
import tempfile
import termios
import threading
import tty
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.wave_proto import (  # noqa: E402
    Banner,
    Dropped,
    Feedback,
    banner_request,
    decode_line,
    feedback_flow,
)
from rover_devtools.wirecat import (  # noqa: E402
    Counters,
    LineSplitter,
    banner_verdict,
    main,
    open_serial,
    render,
)

FEEDBACK = (
    '{"T":1001,"L":0.2,"R":0.2,"r":0.5,"p":-1.2,"y":87.3,"temp":36.5,"v":11.62,'
    '"hb":1,"st":6,"tf":1204,"bp":1,"cc":3}'
)
STOCK = '{"T":1001,"L":0,"R":0,"r":0,"p":0,"y":12.0,"temp":35.0,"v":11.9}'
BANNER = '{"T":1006,"fw":"bot-wr-1","hb_ms":300,"cap":0.3,"proto":1}'
IMU = (
    '{"T":1002,"r":0.1,"p":0.2,"y":3.0,"ax":0,"ay":0,"az":9.8,"gx":1.5,"gy":0,'
    '"gz":-2.0,"mx":0,"my":0,"mz":0,"temp":30.0}'
)
UNKNOWN = '{"T":1003,"x":1}'
GARBAGE = "UGV started."
CAPTURE = "\n".join([BANNER, FEEDBACK, STOCK, IMU, UNKNOWN, GARBAGE]) + "\n"


def rendered(line: str) -> str:
    return render(decode_line(line.encode()), raw=line.encode())


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def test_patched_feedback_shows_the_fork_fields_by_name() -> None:
    text = rendered(FEEDBACK)
    assert "feedback" in text
    assert "L=+0.200 R=+0.200" in text
    assert "yaw=+87.3" in text
    assert "v=11.62V" in text
    assert "hb=1" in text
    assert "st=0x6[tof|bumper]" in text
    assert "tf=1204mm" in text
    assert "bp=1" in text
    assert "cc=3" in text
    assert "stock" not in text


def test_stock_feedback_is_marked_stock() -> None:
    decoded = decode_line(STOCK.encode())
    assert isinstance(decoded, Feedback) and not decoded.patched
    text = rendered(STOCK)
    assert text.rstrip().endswith("stock")
    assert "hb=" not in text


def test_the_banner_shows_what_robotd_compares() -> None:
    text = rendered(BANNER)
    assert "banner" in text
    assert "fw=bot-wr-1 hb_ms=300 cap=0.3 proto=1" in text


def test_an_imu_line_shows_its_rates() -> None:
    text = rendered(IMU)
    assert "imu" in text
    assert "gyro=(+1.5,+0.0,-2.0)dps" in text


def test_an_unknown_type_shows_its_number() -> None:
    text = rendered(UNKNOWN)
    assert "unknown" in text and "T=1003" in text


def test_garbage_is_dropped_with_its_reason_and_bytes_not_hidden() -> None:
    decoded = decode_line(GARBAGE.encode())
    assert isinstance(decoded, Dropped)
    text = rendered(GARBAGE)
    assert " ! " in text
    assert "dropped" in text
    assert "not_json" in text
    assert "UGV started." in text


def test_raw_appends_the_line_bytes() -> None:
    text = render(decode_line(BANNER.encode()), raw=BANNER.encode(), show_raw=True)
    assert BANNER in text


def test_banner_verdict_is_robotds_test() -> None:
    banner = decode_line(BANNER.encode())
    assert isinstance(banner, Banner)
    matched, text = banner_verdict(banner, 300)
    assert matched and "MATCH" in text
    matched, text = banner_verdict(banner, 250)
    assert not matched and "MISMATCH" in text and "config 250" in text
    matched, _ = banner_verdict(Banner("x", 300, 0.5, 1), 300)
    assert not matched


# --------------------------------------------------------------------------
# Splitting and counting
# --------------------------------------------------------------------------


def test_the_splitter_reassembles_partial_lines_and_skips_blank_ones() -> None:
    splitter = LineSplitter()
    assert splitter.feed(b'{"T":10') == []
    assert splitter.feed(b'06}\r\n\r\n{"T":1') == [b'{"T":1006}\r']
    assert splitter.feed(b"}\n") == [b'{"T":1}']


def test_an_endless_line_is_handed_on_whole_so_decode_drops_it() -> None:
    splitter = LineSplitter()
    lines = splitter.feed(b"x" * 600)
    assert len(lines) == 1
    assert decode_line(lines[0]) == Dropped("oversize")
    assert splitter.feed(b"\n") == []


def test_counters_tell_stock_from_patched() -> None:
    counters = Counters()
    for line in (BANNER, FEEDBACK, STOCK, IMU, UNKNOWN, GARBAGE):
        counters.count(decode_line(line.encode()))
    assert counters.summary() == (
        "2 feedback (1 stock), 1 imu, 1 banner, 1 unknown, 1 dropped"
    )


# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------


def test_open_serial_reads_a_pty_slave_raw() -> None:
    master, slave = pty.openpty()
    tty.setraw(slave)
    fd = open_serial(os.ttyname(slave))
    try:
        os.write(master, BANNER.encode() + b"\n")
        assert os.read(fd, 4096) == BANNER.encode() + b"\n"
    finally:
        os.close(fd)
        os.close(slave)
        os.close(master)


def test_the_default_fd_cannot_write() -> None:
    """Read-only is the fd itself rather than a promise in the help string:
    only robotd writes the port (I-18)."""
    master, slave = pty.openpty()
    tty.setraw(master)
    try:
        fd = open_serial(os.ttyname(slave))
        try:
            with pytest.raises(OSError):
                os.write(fd, banner_request())
        finally:
            os.close(fd)
    finally:
        os.close(slave)
        os.close(master)


def test_a_missing_device_is_reported_not_raised(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main([str(tmp_path / "nope")]) == 2
    assert "cannot open" in capsys.readouterr().err


def test_a_capture_file_is_decoded_counted_and_the_banner_judged(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    capture = tmp_path / "capture.txt"
    capture.write_text(CAPTURE)
    config = tmp_path / "robot.toml"
    config.write_text('[robot]\nname = "rover"\n')
    assert main([str(capture), "--config", str(config)]) == 0
    captured = capsys.readouterr()
    assert "banner MATCH: hb_ms 300 (config 300), cap 0.3 (ceiling 0.3)" in captured.out
    shown = [line for line in captured.out.splitlines() if line[:1] == " "]
    assert len(shown) == 6
    assert "2 feedback (1 stock), 1 imu, 1 banner, 1 unknown, 1 dropped" in captured.err


def test_a_banner_that_disagrees_with_the_config_is_a_mismatch_exit(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    capture = tmp_path / "capture.txt"
    capture.write_text(BANNER + "\n")
    config = tmp_path / "robot.toml"
    config.write_text("[safety]\nheartbeat_ms = 250\n")
    assert main([str(capture), "--config", str(config)]) == 1
    assert "banner MISMATCH" in capsys.readouterr().out


def test_kind_filters_hide_or_keep_and_count_stops(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    capture = tmp_path / "capture.txt"
    capture.write_text(CAPTURE)
    assert main([str(capture), "--exclude", "feedback,dropped"]) == 0
    out = capsys.readouterr().out
    assert "feedback" not in out and "not_json" not in out
    assert "banner" in out and "T=1003" in out

    assert main([str(capture), "--only", "banner"]) == 0
    shown = [line for line in capsys.readouterr().out.splitlines() if line[:1] == " "]
    assert len(shown) == 1 and "fw=bot-wr-1" in shown[0]

    assert main([str(capture), "--count", "2"]) == 0
    shown = [line for line in capsys.readouterr().out.splitlines() if line[:1] == " "]
    assert len(shown) == 2


def test_an_unknown_kind_is_a_usage_error(tmp_path: Path) -> None:
    with pytest.raises(SystemExit):
        main([str(tmp_path / "x"), "--only", "frames"])


def test_tcp_reads_a_rover_stub_port(capsys: pytest.CaptureFixture[str]) -> None:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]

    def serve() -> None:
        conn, _ = listener.accept()
        with conn:
            conn.sendall(CAPTURE.encode())

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        assert main(["--tcp", f"127.0.0.1:{port}", "--only", "banner,feedback"]) == 0
    finally:
        thread.join(timeout=3.0)
        listener.close()
    captured = capsys.readouterr()
    assert "fw=bot-wr-1" in captured.out
    assert "1 banner" in captured.err


def test_a_device_and_tcp_together_is_a_usage_error() -> None:
    with pytest.raises(SystemExit):
        main(["/dev/null", "--tcp", "127.0.0.1:1"])
    with pytest.raises(SystemExit):
        main([])


# --------------------------------------------------------------------------
# I-18: only robotd writes the port
# --------------------------------------------------------------------------


def test_request_refuses_while_something_is_listening_on_the_bus(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Without this a `wirecat --request` run during a drive is a second writer
    into the motor controller, and a diagnostic tool changes the robot."""
    # A short directory: an AF_UNIX path is capped at ~104 bytes and pytest's
    # tmp_path is longer than that on macOS.
    run_dir = Path(tempfile.mkdtemp(prefix="wc"))
    sock_path = run_dir / "robotd.sock"
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(sock_path))
    listener.listen(1)
    config = tmp_path / "robot.toml"
    config.write_text(f'[bus]\nsock = "{sock_path}"\n')
    capture = tmp_path / "capture.txt"
    capture.write_text(BANNER + "\n")
    try:
        assert main([str(capture), "--request", "--config", str(config)]) == 2
        err = capsys.readouterr().err
        assert "Only robotd writes the port" in err
        # ...and read-only still works against the same live bus.
        assert main([str(capture), "--config", str(config)]) == 0
    finally:
        listener.close()
        shutil.rmtree(run_dir, ignore_errors=True)


def test_request_sends_feedback_on_and_a_banner_request_once(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    config = tmp_path / "robot.toml"
    config.write_text(f'[bus]\nsock = "{tmp_path / "absent.sock"}"\n')
    master, slave = pty.openpty()
    tty.setraw(master)
    os.set_blocking(master, False)
    try:
        os.write(master, BANNER.encode() + b"\n")
        code = main(
            [os.ttyname(slave), "--request", "--count", "1", "--config", str(config)]
        )
        assert code == 0
        assert "fw=bot-wr-1" in capsys.readouterr().out
        assert os.read(master, 4096) == feedback_flow(True) + banner_request()
    finally:
        os.close(slave)
        os.close(master)


@pytest.mark.parametrize("send_request", [False, True])
def test_serial_access_refuses_live_robotd_even_without_writes(
    send_request: bool,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr("rover_devtools.wirecat._socket_is_live", lambda _: True)
    master, slave = pty.openpty()
    try:
        original = termios.tcgetattr(slave)
        args = [os.ttyname(slave)] + (["--request"] if send_request else [])
        assert main(args) == 2
        assert termios.tcgetattr(slave) == original
        assert "another serial reader" in capsys.readouterr().err
    finally:
        os.close(slave)
        os.close(master)
