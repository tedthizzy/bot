"""``roverctl`` -- the one operator entry point.

ARCHITECTURE 12: "The CLI is ``roverctl`` everywhere".  It speaks the two
sockets of 5.2 and 5.9 and nothing else -- there is no path from here to the
serial port, because I-18 says only robotd writes it:

``utter``      an ``Utterance`` on ``brain.sock`` -- deploy step 13's first turn
``drive-for``  a ``drive_for`` skill: a power for a number of seconds
``turn-to``    a ``turn_to`` skill: an absolute heading in degrees
``say``        the ``say`` skill
``skill``      any bus skill, with its arguments as JSON
``watch``      subscribe and print what the robot thinks it is doing
``stop`` / ``estop`` / ``clear``   the stop authorities and the recovery
``tail``       the JSONL robotd writes
"""

from __future__ import annotations

import argparse
import json
import math
import os
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
from rover_contracts.skills import goal_deadline_s
from rover_contracts.wave_proto import StopFlag

from rover_devtools import load_config_or_default

__all__ = ["BusClient", "goal_ttl_ms", "main", "render_state", "stop_flag_names"]

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


# --------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------


def stop_flag_names(mask: int) -> str:
    """``st`` by name: ``tof|bumper``, or ``-`` when clear."""
    return "|".join(flag.name.lower() for flag in StopFlag if mask & flag) or "-"


def render_state(state: StateMessage) -> str:
    """One compact line for one ``state`` message: the StateRover fields an
    operator watches, then the rest of the message."""
    rover = state.rover
    front = "none" if state.front_m is None else f"{state.front_m:.2f}m"
    active = (
        f" active={state.active.skill}:{state.active.progress:.0%}"
        f"/{state.active.deadline_in_ms}ms"
        if state.active is not None
        else ""
    )
    return (
        f"fw={rover.fw or 'stock'} hb={'ok' if rover.hb_ok else 'LOST'} "
        f"flags={stop_flag_names(rover.stop_flags)} age={rover.feedback_age_ms}ms "
        f"heading={rover.heading_deg:+.0f}deg "
        f"cmd={rover.cmd_left:+.2f}/{rover.cmd_right:+.2f} "
        f"front={front} bat={state.battery.pack_v:.1f}V/{state.battery.pct}% "
        f"bumper={state.bumper} estop={state.estop_sw} "
        f"budget={state.budget.motion_s:.1f}s "
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
            f"welcome session={message.session} rover_fw={message.rover_fw or 'none'} "
            f"power_max={message.limits.power_max} "
            f"budget_motion_s={message.limits.budget_motion_s} "
            f"heartbeat_ms={message.safety.heartbeat_ms}"
        )
    return f"?       {message!r}"


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def goal_ttl_ms(skill: str, args: BaseModel, limits: LimitsConfig) -> int:
    """The T2 estimate, in milliseconds, from the catalog's own formula.

    brain computes this from the skill's duration and never from a constant;
    so does this CLI, or a legal two-second drive would be answered
    ``goal_ttl_too_short`` for the wrong reason.
    """
    values = args.model_dump()
    if skill == "drive_for":
        seconds = goal_deadline_s(float(values["duration_s"]))
    elif skill == "turn_to":
        seconds = goal_deadline_s(float(values["timeout_s"]))
    else:
        return limits.goal_ttl_ms_max
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


def _cmd_drive_for(args: argparse.Namespace, config: RobotConfig) -> int:
    payload = {"duration_s": float(args.seconds), "power": float(args.power)}
    return _send_skill(args, config, "drive_for", payload)


def _cmd_turn_to(args: argparse.Namespace, config: RobotConfig) -> int:
    payload = {
        "heading_deg": float(args.heading),
        "timeout_s": float(
            config.limits.turn_timeout_max_s if args.timeout is None else args.timeout
        ),
        "tolerance_deg": float(
            config.limits.turn_tolerance_deg if args.tolerance is None else args.tolerance
        ),
    }
    return _send_skill(args, config, "turn_to", payload)


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


def _cmd_watch(args: argparse.Namespace, config: RobotConfig) -> int:
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
# tail
# --------------------------------------------------------------------------


def _newest_log(directory: Path) -> Path | None:
    candidates = sorted(directory.glob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def _cmd_tail(args: argparse.Namespace, config: RobotConfig) -> int:
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


def _add_dispatch_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--wait", type=float, default=8.0, help="seconds to watch")
    parser.add_argument("--turn-id", metavar="ULID")
    parser.add_argument("--goal-ttl-ms", type=int, help="default: the T2 estimate")


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

    drive = sub.add_parser(
        "drive-for", help="drive_for: both sides at POWER for SECONDS (open loop)"
    )
    drive.add_argument("seconds", type=float, help="0 < s <= [limits] drive_for_max_s")
    drive.add_argument(
        "power", type=float,
        help="Waveshare units, |p| <= [limits] power_max (0.30); negative reverses",
    )
    _add_dispatch_options(drive)
    drive.set_defaults(run=_cmd_drive_for)

    turn = sub.add_parser("turn-to", help="turn_to: an absolute heading in degrees")
    turn.add_argument("heading", type=float, help="0 <= deg < 360, the WorldState frame")
    turn.add_argument(
        "--timeout", type=float, default=None, help="default: [limits] turn_timeout_max_s"
    )
    turn.add_argument(
        "--tolerance", type=float, default=None,
        help="degrees; default: [limits] turn_tolerance_deg",
    )
    _add_dispatch_options(turn)
    turn.set_defaults(run=_cmd_turn_to)

    say = sub.add_parser("say", help="speak a sentence (the say skill)")
    say.add_argument("text", nargs="+")
    _add_dispatch_options(say)
    say.set_defaults(run=_cmd_say)

    skill = sub.add_parser("skill", help="dispatch any bus skill")
    skill.add_argument("name", choices=sorted(BUS_SKILL_ARGS))
    skill.add_argument("--args", default="{}", metavar="JSON")
    _add_dispatch_options(skill)
    skill.set_defaults(run=_cmd_skill)

    watch = sub.add_parser("watch", help="watch state, result and event")
    watch.add_argument("--count", type=int, default=0, help="stop after N states")
    watch.add_argument("--seconds", type=float, default=5.0)
    watch.add_argument("--json", action="store_true")
    watch.set_defaults(run=_cmd_watch)

    for name, helptext in (
        ("stop", "stop the active command"),
        ("estop", "software e-stop: zeros on the wire, every skill refused after"),
    ):
        stop = sub.add_parser(name, help=helptext)
        stop.add_argument("--reason", default=None)
        stop.set_defaults(run=_cmd_stop)

    clear = sub.add_parser("clear", help="clear a latched fault, or estop_sw")
    clear.add_argument(
        "faults", nargs="+", metavar="FAULT",
        help="one of " + ", ".join(f.value for f in ClearableFault),
    )
    clear.set_defaults(run=_cmd_clear)

    tail = sub.add_parser("tail", help="tail robotd's JSONL")
    tail.add_argument("path", nargs="?", help="default: newest *.jsonl in [log] dir")
    tail.add_argument("-n", "--lines", type=int, default=20)
    tail.add_argument("-f", "--follow", action="store_true")
    tail.set_defaults(run=_cmd_tail)

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
