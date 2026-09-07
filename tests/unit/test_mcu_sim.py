"""The simulator wrapper reports a device path robotd can actually open.

The firmware agent's C binary does not exist yet, so these tests drive the
wrapper with a stub that speaks just enough of ARCHITECTURE 5.1 to prove the
plumbing: it emits the golden ``B`` banner and echoes what it is sent.  What is
under test is the wrapper -- the pty, the device path, the symlink and the
byte pump -- not the firmware.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_contracts.serial_codec import (  # noqa: E402
    BootFrame,
    DecodeOk,
    FrameReader,
)
from rover_devtools.mcu_sim import (  # noqa: E402
    BINARY_ENV,
    FAULT_FLAGS,
    McuSim,
    SimulatorMissing,
    find_binary,
    main,
    parse_flags,
)
from rover_devtools.wirecat import open_serial  # noqa: E402

BANNER = b"$B,2,1,40010,256,2,0003,1,3381018647*3230\n"

STUB = """#!{python}
import os, sys
sys.stdout.buffer.write({banner!r})
sys.stdout.buffer.flush()
while True:
    data = os.read(0, 4096)
    if not data:
        break
    os.write(1, data)
"""


@pytest.fixture()
def stub(tmp_path: Path) -> Path:
    """A stand-in for the C simulator: banner, then echo."""
    path = tmp_path / "mcu-sim"
    path.write_text(STUB.format(python=sys.executable, banner=BANNER))
    path.chmod(0o755)
    return path


def _read_until(fd: int, wanted: bytes, seconds: float = 5.0) -> bytes:
    deadline = time.monotonic() + seconds
    seen = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""
        if chunk:
            seen += chunk
            if wanted in seen:
                return bytes(seen)
        else:
            time.sleep(0.01)
    raise AssertionError(f"never saw {wanted!r}; got {bytes(seen)!r}")


# --------------------------------------------------------------------------
# Finding the binary
# --------------------------------------------------------------------------


def test_a_missing_binary_names_the_make_target(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv(BINARY_ENV, raising=False)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(SimulatorMissing) as caught:
        find_binary()
    message = str(caught.value)
    assert "make sim" in message
    assert BINARY_ENV in message
    assert "firmware/build/host/mcu-sim" in message


def test_an_explicit_path_that_is_not_executable_is_named(tmp_path: Path) -> None:
    plain = tmp_path / "not-a-binary"
    plain.write_text("")
    with pytest.raises(SimulatorMissing, match="not an executable file"):
        find_binary(plain)


def test_the_env_var_wins(stub: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(BINARY_ENV, str(stub))
    assert find_binary() == stub


def test_main_reports_a_missing_binary_without_a_traceback(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.delenv(BINARY_ENV, raising=False)
    monkeypatch.chdir(tmp_path)
    assert main([]) == 2
    assert "make sim" in capsys.readouterr().err


# --------------------------------------------------------------------------
# Fault flags
# --------------------------------------------------------------------------


def test_every_flag_of_ARCHITECTURE_10_is_accepted() -> None:
    documented = {
        "ttl_drop", "garbage", "crc_flip", "reset_mid_drive", "hang", "obstacle",
        "no_target", "tof_error", "cliff", "bumper", "estop", "seq_replay",
        "seq_desync", "no_t_before_h", "lag", "vbat", "stall", "stall_one_channel",
        "slip_one_channel",
    }
    assert set(FAULT_FLAGS) == documented


def test_flags_pass_through_verbatim() -> None:
    tokens = ["obstacle=231", "tof_error=fl", "no_t_before_h", "hang=1200",
              "lag=200ms", "vbat=9.5"]
    assert parse_flags(tokens) == tokens


@pytest.mark.parametrize(
    ("token", "message"),
    [
        ("wobble", "unknown fault flag"),
        ("obstacle", "needs a value"),
        ("cliff=1", "takes no value"),
        ("tof_error=rear", "must be fl, fr or both"),
    ],
)
def test_a_bad_flag_fails_at_the_wrapper(token: str, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        parse_flags([token])


# --------------------------------------------------------------------------
# The device path
# --------------------------------------------------------------------------


def test_the_device_path_is_a_usable_serial_device(stub: Path, tmp_path: Path) -> None:
    link = tmp_path / "run" / "mcu.pty"
    sim = McuSim(binary=stub, link=link)
    device = sim.start()
    pump = threading.Thread(target=sim.pump, daemon=True)
    pump.start()
    try:
        assert Path(device).exists()
        assert link.is_symlink()
        assert os.path.realpath(link) == os.path.realpath(device)

        # Opened read-write, as robotd opens the real port; wirecat's own
        # default is read-only (I-18) and would not be able to write here.
        fd = open_serial(link, 921600, write=True)
        try:
            reader = FrameReader()
            results = reader.feed(_read_until(fd, b"\n"))
            assert [type(r.frame) for r in results if isinstance(r, DecodeOk)] == [
                BootFrame
            ]
            os.write(fd, b"$D,2,8,40010*DFCC\n")
            assert b"$D,2,8,40010*DFCC" in _read_until(fd, b"DFCC")
        finally:
            os.close(fd)
    finally:
        sim.close()
        pump.join(timeout=3.0)
    assert not link.exists()


def test_the_symlink_is_replaced_not_appended(stub: Path, tmp_path: Path) -> None:
    link = tmp_path / "mcu.pty"
    link.symlink_to("/dev/null")
    with McuSim(binary=stub, link=link) as sim:
        assert os.path.realpath(link) == os.path.realpath(sim.device)


def test_no_link_leaves_the_filesystem_alone(stub: Path, tmp_path: Path) -> None:
    with McuSim(binary=stub) as sim:
        assert sim.device.startswith("/dev/")
    assert not (tmp_path / "run").exists()
