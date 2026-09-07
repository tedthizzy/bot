"""``roverctl`` -- the one operator entry point.

ARCHITECTURE 12: "The CLI is ``roverctl`` everywhere".  It speaks the three
sockets of 5.2 and 5.9 and, for the two commands that have no bus message, the
serial link of 5.1:

``utter``   an ``Utterance`` on ``brain.sock`` -- deploy step 13's first turn
``say`` / ``skill``   a ``skill`` on ``robotd.sock``
``state``   subscribe and print what the robot thinks it is doing
``stop`` / ``estop`` / ``clear``   the stop authorities and the recovery
``arm`` / ``disarm``   ``A``/``D`` straight at the MCU, for bench bring-up
``log``     tail the JSONL robotd writes

``arm`` and ``disarm`` exist nowhere on the bus: 4.2 makes arming robotd's own
policy, and the only place the words exist is the serial link.  So they write
the port -- which I-18 says only robotd may do -- and refuse to run while
anything is listening on ``[bus] sock``.  They are the preflight step 11 check
("``A`` accepted"), not a way to drive.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import socket
import sys
import time
from collections.abc import Iterator, Sequence
from pathlib import Path
from types import TracebackType
from typing import Any, Generic, TypeVar

from pydantic import BaseModel, TypeAdapter, ValidationError
from rover_contracts.config import LimitsConfig, RobotConfig
from rover_contracts.ids import new_cmd_id, new_turn_id
from rover_contracts.jsonl import to_json_line
from rover_contracts.messages import (
    BUS_SKILL_ARGS,
    BusCap,
    ClearableFault,
    ClearMessage,
    ErrorMessage,
    EstopMessage,
    EventMessage,
    FaceMessage,
    HelloMessage,
    ResultMessage,
    ResultStatus,
    ServerMessage,
    SkillMessage,
    Source,
    StateMessage,
    StopMessage,
    SubscribeMessage,
    SubscribeTopic,
    TurnMessage,
    UtteranceMessage,
    UtteranceSource,
    WelcomeMessage,
    brain_server_adapter,
    server_adapter,
)
from rover_contracts.serial_codec import (
    AckFrame,
    AckReason,
    AckResult,
    ArmFrame,
    DecodeOk,
    DisarmFrame,
    FrameReader,
    HelloFrame,
    TelemetryFrame,
    ack_type_code,
    encode_frame,
)
from rover_contracts.units import rad_to_deg

from rover_devtools import load_config_or_default
from rover_devtools.wirecat import open_serial

__all__ = ["BusClient", "goal_ttl_ms", "main", "render_state"]

_M = TypeVar("_M")


# --------------------------------------------------------------------------
# NDJSON over a Unix socket
# --------------------------------------------------------------------------


class BusClient(Generic[_M]):
    """One connection to ``robotd.sock`` or ``brain.sock``.

    The adapter decides what comes back -- ``server_adapter`` for robotd,
    ``brain_server_adapter`` for brain -- so the two sockets share one reader
    without sharing a message set.  ``source`` is bound to the connection at
    ``hello`` (4.2), so a client sends exactly one and every later message
    repeats it.
    """

    def __init__(self, path: str | Path, adapter: TypeAdapter[_M]) -> None:
        self.path = str(path)
        self._adapter = adapter
        self._sock: socket.socket | None = None
        self._buf = bytearray()

    def connect(self, timeout: float = 2.0) -> BusClient[_M]:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(self.path)
        self._sock = sock
        return self

    def send(self, message: BaseModel) -> None:
        assert self._sock is not None, "connect() first"
        self._sock.sendall(to_json_line(message).encode("utf-8"))

    def readline(self, deadline: float) -> bytes | None:
        """One NDJSON line, or ``None`` at the deadline or on EOF."""
        assert self._sock is not None, "connect() first"
        while True:
            index = self._buf.find(b"\n")
            if index >= 0:
                line = bytes(self._buf[:index])
                del self._buf[: index + 1]
                if line.strip():
                    return line
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._sock.settimeout(remaining)
            try:
                chunk = self._sock.recv(65536)
            except TimeoutError:
                return None
            if not chunk:
                return None
            self._buf += chunk

    def messages(self, seconds: float) -> Iterator[_M]:
        """Every message that arrives inside ``seconds``.  A line that does not
        validate is reported on stderr rather than dropped: an operator tool
        that hides a protocol mismatch is worse than none."""
        deadline = time.monotonic() + seconds
        while True:
            line = self.readline(deadline)
            if line is None:
                return
            try:
                yield self._adapter.validate_json(line)
            except ValidationError as exc:
                print(f"roverctl: undecodable line: {exc.error_count()} errors",
                      file=sys.stderr)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self) -> BusClient[_M]:
        return self.connect()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


def _hello(client: BusClient[Any], source: Source, caps: Sequence[BusCap]) -> None:
    client.send(HelloMessage(source=source, pid=os.getpid(), caps=list(caps)))


def _socket_is_live(path: str) -> bool:
    """True when something is accepting on ``path``.  A stale socket file with
    no listener answers ECONNREFUSED and is not a reason to refuse."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(0.3)
    try:
        sock.connect(path)
    except OSError:
        return False
    finally:
        sock.close()
    return True


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def render_state(state: StateMessage) -> str:
    """One compact line for one ``state`` message."""
    front = state.ranges_m.front
    front_text = (
        "stale" if front is None else f"{front:.2f}m{'+' if state.front_at_max else ''}"
    )
    active = (
        f" active={state.active.skill}:{state.active.progress:.0%}"
        f"/{state.active.deadline_in_ms}ms"
        if state.active is not None
        else ""
    )
    return (
        f"{state.mcu.state.name} fault=0x{state.mcu.fault:X} age={state.mcu.age_ms}ms "
        f"armed={state.armed} pose=({state.pose.x_m:+.2f},{state.pose.y_m:+.2f}) "
        f"yaw={rad_to_deg(state.pose.yaw_rad):+.0f}deg "
        f"v={state.twist.linear_x_mps:+.3f} w={state.twist.angular_z_radps:+.3f} "
        f"front={front_text} bat={state.battery.pct}% "
        f"bumper={state.bumper} estop={state.estop_hw}/{state.estop_sw} "
        f"ready={state.ready}{f' ({state.reason})' if state.reason else ''}{active}"
    )


def _render(message: ServerMessage) -> str:
    if isinstance(message, StateMessage):
        return "state   " + render_state(message)
    if isinstance(message, ResultMessage):
        detail = message.detail.model_dump(exclude_none=True) if message.detail else {}
        return (
            f"result  {message.status.value} {message.cmd_id} "
            f"reason={message.reason.value or '-'} {detail or ''}".rstrip()
        )
    if isinstance(message, EventMessage):
        return f"event   {message.kind.value} {message.detail or ''}".rstrip()
    if isinstance(message, ErrorMessage):
        return f"error   {message.code} {message.detail}".rstrip()
    if isinstance(message, WelcomeMessage):
        return (
            f"welcome session={message.session} mcu_session={message.mcu_session} "
            f"speed_mps={message.limits.speed_mps} "
            f"budget_path_m={message.limits.budget_path_m}"
        )
    return f"?       {message!r}"


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def goal_ttl_ms(skill: str, args: BaseModel, limits: LimitsConfig) -> int:
    """The T2 estimate, in milliseconds, as ARCHITECTURE 6 states it.

    brain computes this from the profile and never from a constant; so does
    this CLI, or a legal ``drive(100, 5)`` would be answered
    ``goal_ttl_too_short`` for the wrong reason.
    """
    values = args.model_dump()
    if skill == "drive":
        seconds = abs(float(values["distance_m"])) / float(values["speed_mps"])
    elif skill == "turn":
        seconds = abs(float(values["angle_deg"])) / float(values["rate_dps"])
    else:
        return limits.goal_ttl_ms_max
    seconds = seconds * 1.5 + 0.5
    return max(100, min(limits.goal_ttl_ms_max, math.ceil(seconds * 1000)))


def _send_skill(
    args: argparse.Namespace, config: RobotConfig, skill: str, payload: dict[str, object]
) -> int:
    model = BUS_SKILL_ARGS.get(skill)
    if model is None:
        known = ", ".join(sorted(BUS_SKILL_ARGS))
        print(f"roverctl: {skill!r} does not cross the bus; known: {known}",
              file=sys.stderr)
        return 2
    try:
        skill_args = model.model_validate(payload)
    except ValidationError as exc:
        print(f"roverctl: bad args for {skill}:\n{exc}", file=sys.stderr)
        return 2

    ttl = args.goal_ttl_ms or goal_ttl_ms(skill, skill_args, config.limits)
    turn_id = args.turn_id or new_turn_id()
    with BusClient(args.sock, server_adapter) as client:
        _hello(client, args.source, [BusCap.SKILL, BusCap.SUBSCRIBE])
        client.send(
            SubscribeMessage(topics=[SubscribeTopic.RESULT, SubscribeTopic.EVENT])
        )
        client.send(TurnMessage(source=args.source, turn_id=turn_id))
        client.send(
            SkillMessage(
                source=args.source,
                cmd_id=new_cmd_id(),
                seq=1,
                turn_id=turn_id,
                issued_mono_ns=time.monotonic_ns(),
                goal_ttl_ms=ttl,
                skill=skill,  # type: ignore[arg-type]
                args=skill_args,  # type: ignore[arg-type]
            )
        )
        status = 0
        for message in client.messages(args.wait):
            print(_render(message))
            if isinstance(message, ResultMessage):
                if message.status is ResultStatus.DONE:
                    return 0
                if message.status is not ResultStatus.ACCEPTED:
                    status = 1
        return status


def _cmd_skill(args: argparse.Namespace, config: RobotConfig) -> int:
    try:
        payload = json.loads(args.args)
    except ValueError as exc:
        print(f"roverctl: --args is not JSON: {exc}", file=sys.stderr)
        return 2
    return _send_skill(args, config, args.name, payload)


def _cmd_say(args: argparse.Namespace, config: RobotConfig) -> int:
    return _send_skill(args, config, "say", {"text": " ".join(args.text)})


def _cmd_utter(args: argparse.Namespace, config: RobotConfig) -> int:
    text = " ".join(args.text)
    with BusClient(args.brain_sock, brain_server_adapter) as client:
        client.send(
            UtteranceMessage(
                source=UtteranceSource.CLI,
                text=text,
                confidence=None,
                is_final=True,
                mono_ns=time.monotonic_ns(),
            )
        )
        for message in client.messages(args.watch):
            value = message.expr if isinstance(message, FaceMessage) else message.state
            print(f"{message.type:8s}{value.value}")
    return 0


def _cmd_state(args: argparse.Namespace, config: RobotConfig) -> int:
    topics = [SubscribeTopic.STATE, SubscribeTopic.RESULT, SubscribeTopic.EVENT]
    with BusClient(args.sock, server_adapter) as client:
        _hello(client, args.source, [BusCap.SUBSCRIBE])
        client.send(SubscribeMessage(topics=topics, state_hz=config.bus.state_hz))
        shown = 0
        for message in client.messages(args.seconds):
            if args.json:
                print(message.model_dump_json())
            else:
                print(_render(message))
            if isinstance(message, StateMessage):
                shown += 1
                if args.count and shown >= args.count:
                    return 0
    return 0


def _cmd_stop(args: argparse.Namespace, config: RobotConfig) -> int:
    with BusClient(args.sock, server_adapter) as client:
        _hello(client, args.source, [BusCap.SKILL])
        if args.command == "estop":
            client.send(EstopMessage(source=args.source, reason=args.reason))
        else:
            client.send(StopMessage(source=args.source, reason=args.reason))
        for message in client.messages(0.5):
            print(_render(message))
    return 0


def _cmd_clear(args: argparse.Namespace, config: RobotConfig) -> int:
    try:
        faults = [ClearableFault(name) for name in args.faults]
    except ValueError as exc:
        known = ", ".join(f.value for f in ClearableFault)
        print(f"roverctl: {exc}; clearable faults: {known}", file=sys.stderr)
        return 2
    with BusClient(args.sock, server_adapter) as client:
        _hello(client, args.source, [BusCap.SKILL])
        client.send(ClearMessage(source=args.source, faults=faults))
        for message in client.messages(1.0):
            print(_render(message))
    return 0


# --------------------------------------------------------------------------
# arm / disarm -- the serial link, because the bus has no such message
# --------------------------------------------------------------------------


def _read_telemetry(fd: int, reader: FrameReader, seconds: float
                    ) -> TelemetryFrame | None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b""
        except OSError:
            return None
        if not data:
            time.sleep(0.005)
            continue
        for result in reader.feed(data):
            if isinstance(result, DecodeOk) and isinstance(result.frame, TelemetryFrame):
                return result.frame
    return None


def _await_ack(fd: int, reader: FrameReader, letter: str, seconds: float
               ) -> AckFrame | None:
    wanted = ack_type_code(letter)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            data = b""
        except OSError:
            return None
        if not data:
            time.sleep(0.005)
            continue
        for result in reader.feed(data):
            if (
                isinstance(result, DecodeOk)
                and isinstance(result.frame, AckFrame)
                and result.frame.ack_type == wanted
            ):
                return result.frame
    return None


def _cmd_arm(args: argparse.Namespace, config: RobotConfig) -> int:
    arming = args.command == "arm"
    if _socket_is_live(args.sock):
        print(
            f"roverctl: {args.sock} has a listener, so robotd owns "
            f"{args.device}. Only robotd writes the port (I-18); stop "
            "rover-robotd before arming by hand.",
            file=sys.stderr,
        )
        return 2
    try:
        # write=True: this is the one roverctl path that transmits, and the
        # guard above has just established that robotd is not running (I-18).
        fd = open_serial(args.device, config.serial.baud, write=True)
    except OSError as exc:
        print(f"roverctl: cannot open {args.device}: {exc}", file=sys.stderr)
        return 2

    reader = FrameReader()
    wait = config.serial.reseed_wait_ms / 1000.0
    try:
        # A8's re-seed: adopt down_seq from T.ack_seq, or 1 for a fresh MCU.
        telemetry = _read_telemetry(fd, reader, wait)
        session = telemetry.session if telemetry else 0
        seq = ((telemetry.ack_seq + 1) & 0xFFFF) if telemetry else 1
        os.write(
            fd,
            encode_frame(HelloFrame(seq, session, secrets.randbits(32))),
        )
        if session == 0:
            # no_t_before_h: telemetry starts only once the hello lands.
            telemetry = _read_telemetry(fd, reader, wait)
            if telemetry is None:
                print(
                    f"roverctl: no telemetry from {args.device} after H. Is the "
                    "MCU flashed and wired? `python -m rover_devtools.wirecat "
                    f"{args.device}` shows the raw link.",
                    file=sys.stderr,
                )
                return 1
            session = telemetry.session
        seq = (seq + 1) & 0xFFFF
        frame = (
            ArmFrame(seq, session, secrets.randbits(32))
            if arming
            else DisarmFrame(seq, session)
        )
        os.write(fd, encode_frame(frame))
        ack = _await_ack(fd, reader, "A" if arming else "D", 1.0)
        if ack is None:
            print(f"roverctl: no ack for {'A' if arming else 'D'}", file=sys.stderr)
            return 1
        result, reason = AckResult(ack.result), AckReason(ack.reason)
        print(
            f"roverctl: {'arm' if arming else 'disarm'} {result.name} "
            f"reason={reason.name} session={session} echo={ack.echo}"
        )
        return 0 if result is AckResult.OK else 1
    finally:
        os.close(fd)


# --------------------------------------------------------------------------
# log
# --------------------------------------------------------------------------


def _newest_log(directory: Path) -> Path | None:
    candidates = sorted(directory.glob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def _cmd_log(args: argparse.Namespace, config: RobotConfig) -> int:
    path = Path(args.path) if args.path else None
    if path is None:
        directory = Path(config.log.dir)
        if not directory.is_dir():
            print(f"roverctl: no log directory {directory}", file=sys.stderr)
            return 1
        path = _newest_log(directory)
        if path is None:
            print(f"roverctl: no *.jsonl in {directory}", file=sys.stderr)
            return 1
    print(f"roverctl: {path}", file=sys.stderr)
    with path.open("r", encoding="utf-8") as handle:
        tail = handle.readlines()[-args.lines :] if args.lines else handle.readlines()
        for line in tail:
            sys.stdout.write(line)
        sys.stdout.flush()
        if not args.follow:
            return 0
        try:
            while True:
                line = handle.readline()
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                else:
                    time.sleep(0.2)
        except KeyboardInterrupt:
            return 130


# --------------------------------------------------------------------------
# argv
# --------------------------------------------------------------------------


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="roverctl", description="The rover's operator CLI."
    )
    parser.add_argument("--config", metavar="PATH", help="config/robot.toml")
    parser.add_argument(
        "--source",
        type=Source,
        default=Source.WEB,
        choices=list(Source),
        help="the bus source to bind at hello (default: web)",
    )
    parser.add_argument("--sock", help="override [bus] sock")
    parser.add_argument("--brain-sock", help="override [bus] brain_sock")
    sub = parser.add_subparsers(dest="command", required=True)

    utter = sub.add_parser("utter", help="send an Utterance on brain.sock")
    utter.add_argument("text", nargs="+")
    utter.add_argument("--watch", type=float, default=0.0, metavar="SECONDS")
    utter.set_defaults(run=_cmd_utter)

    say = sub.add_parser("say", help="speak a sentence (the say skill)")
    say.add_argument("text", nargs="+")
    say.add_argument("--wait", type=float, default=8.0, help="seconds to watch")
    say.add_argument("--turn-id", metavar="ULID")
    say.add_argument("--goal-ttl-ms", type=int)
    say.set_defaults(run=_cmd_say)

    skill = sub.add_parser("skill", help="dispatch one skill")
    skill.add_argument("name", choices=sorted(BUS_SKILL_ARGS))
    skill.add_argument("--args", default="{}", metavar="JSON")
    skill.add_argument("--turn-id", metavar="ULID")
    skill.add_argument("--goal-ttl-ms", type=int, help="default: the T2 estimate")
    skill.add_argument("--wait", type=float, default=8.0, help="seconds to watch")
    skill.set_defaults(run=_cmd_skill)

    state = sub.add_parser("state", help="watch state, result and event")
    state.add_argument("--count", type=int, default=0, help="stop after N states")
    state.add_argument("--seconds", type=float, default=5.0)
    state.add_argument("--json", action="store_true")
    state.set_defaults(run=_cmd_state)

    for name, helptext in (
        ("stop", "stop the active command"),
        ("estop", "software e-stop: S then D, and every skill refused after"),
    ):
        stop = sub.add_parser(name, help=helptext)
        stop.add_argument("--reason", default=None)
        stop.set_defaults(run=_cmd_stop)

    clear = sub.add_parser("clear", help="clear latched faults, or estop_sw")
    clear.add_argument("faults", nargs="+", metavar="FAULT")
    clear.set_defaults(run=_cmd_clear)

    for name, helptext in (
        ("arm", "send A to the MCU (bench bring-up only)"),
        ("disarm", "send D to the MCU (bench bring-up only)"),
    ):
        arm = sub.add_parser(name, help=helptext)
        arm.add_argument("--device", help="default: [serial] port")
        arm.set_defaults(run=_cmd_arm)

    log = sub.add_parser("log", help="tail robotd's JSONL")
    log.add_argument("path", nargs="?", help="default: newest *.jsonl in [log] dir")
    log.add_argument("-n", "--lines", type=int, default=20)
    log.add_argument("-f", "--follow", action="store_true")
    log.set_defaults(run=_cmd_log)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        config, origin = load_config_or_default(args.config)
    except (OSError, ValueError) as exc:
        print(f"roverctl: {exc}", file=sys.stderr)
        return 2
    if args.config is None and origin.startswith("built-in"):
        print(f"roverctl: {origin}", file=sys.stderr)
    args.sock = args.sock or config.bus.sock
    args.brain_sock = args.brain_sock or config.bus.brain_sock
    if getattr(args, "device", None) is None and args.command in ("arm", "disarm"):
        args.device = config.serial.port
    try:
        return int(args.run(args, config))
    except FileNotFoundError as exc:
        print(f"roverctl: {exc}", file=sys.stderr)
        return 2
    except ConnectionRefusedError:
        print(f"roverctl: nothing listening on {args.sock}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
