"""rover-web: the face, the teleop pad, the text box and the STOP button.

ARCHITECTURE 4.5.  It is the only WebSocket speaker (A10) and it holds three
NDJSON clients: two robotd sessions -- ``web`` for the stop authorities and
``teleop`` for the joystick, so emptying ``[bus] allow_stream`` removes the
joystick without touching STOP -- and one ``brain.sock`` client for utterances,
PTT and the ``face`` broadcasts that are the whole of ``set_face``'s executor
path.

The rover is open loop (ADR-0013), so the joystick is scaled into
``TwistPayload(lin, ang)`` in power units, both bounded by ``[limits]
twist_power``; robotd's mixer turns that into left and right.  It never touches
the serial port, it is in no stop path except as an additional stop authority,
and every send is best effort: with robotd or brain absent the pages still load
and the rover behaves identically.  Lines from robotd and brain are relayed to
the browser verbatim; rover-web does not interpret them.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
import time
from collections.abc import Awaitable, Callable, Iterable
from pathlib import Path
from typing import cast

from aiohttp import WSMsgType, web
from pydantic import BaseModel
from rover_contracts.config import RobotConfig, load_config
from rover_contracts.ids import new_cmd_id
from rover_contracts.messages import (
    BusCap,
    ClearableFault,
    ClearMessage,
    EstopMessage,
    HelloMessage,
    PttEndMessage,
    PttStartMessage,
    Source,
    StopMessage,
    SubscribeMessage,
    SubscribeTopic,
    TwistMessage,
    TwistPayload,
    UtteranceMessage,
    UtteranceSource,
)
from rover_contracts.wave_proto import StopFlag

__all__ = [
    "BRAIN",
    "MAX_BODY_BYTES",
    "MAX_UTTERANCE_CHARS",
    "ROBOTD_TELEOP",
    "ROBOTD_WEB",
    "STOP_FLAG_BITS",
    "LineClient",
    "TeleopStream",
    "create_app",
    "main",
]

log = logging.getLogger("rover_web")

STATIC_DIR = Path(__file__).resolve().parent / "static"
MAX_BODY_BYTES = 64 * 1024
"""The robotd bus line cap of ARCHITECTURE 5.2, applied to browser uploads too."""
MAX_WS_MESSAGE_BYTES = 8 * 1024
MAX_UTTERANCE_CHARS = 500
STOP_FLAG_BITS: dict[str, int] = {
    cast(str, flag.name).lower(): int(flag) for flag in StopFlag
}
"""``state.rover.stop_flags`` by name, for the pages' readout.  Served from
``rover_contracts`` rather than restated in JavaScript: the bit table has one
home, ``docs/protocol.md``."""
_LINE_LIMIT = 1 << 17
_RELAY_DEPTH = 256
_RECONNECT_S = 1.0
_TELEOP_HZ = 20.0


class LineClient:
    """One NDJSON Unix-socket peer, reconnecting for ever, never blocking.

    ``send`` reports whether the line reached the socket buffer.  It never
    raises and never awaits: a dead peer must not be able to stall a page or a
    STOP button.
    """

    def __init__(
        self,
        path: str | Path,
        name: str,
        *,
        greeting: Callable[[], Iterable[BaseModel]] = tuple,
        on_line: Callable[[str], None] | None = None,
        reconnect_s: float = _RECONNECT_S,
    ) -> None:
        self.name = name
        self._path = str(path)
        self._greeting = greeting
        self._on_line = on_line
        self._reconnect_s = reconnect_s
        self._writer: asyncio.StreamWriter | None = None
        self._task: asyncio.Task[None] | None = None
        self._seq = 0

    @property
    def connected(self) -> bool:
        writer = self._writer
        return writer is not None and not writer.is_closing()

    def next_seq(self) -> int:
        """Strictly increasing per (source, client-session), as robotd requires."""
        self._seq += 1
        return self._seq

    def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name=f"rover-web:{self.name}")

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        await self._disconnect()

    def send(self, message: BaseModel) -> bool:
        writer = self._writer
        if writer is None or writer.is_closing():
            log.debug("%s down, dropping %s", self.name, type(message).__name__)
            return False
        try:
            writer.write(message.model_dump_json(exclude_none=True).encode() + b"\n")
        except (OSError, ConnectionError):
            return False
        return True

    async def _run(self) -> None:
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(
                    self._path, limit=_LINE_LIMIT
                )
            except (OSError, ConnectionError):
                await asyncio.sleep(self._reconnect_s)
                continue
            self._writer = writer
            self._seq = 0
            log.info("%s connected to %s", self.name, self._path)
            try:
                for message in self._greeting():
                    writer.write(
                        message.model_dump_json(exclude_none=True).encode() + b"\n"
                    )
                await writer.drain()
                async for line in reader:
                    if self._on_line is not None:
                        self._on_line(line.decode("utf-8", "replace").rstrip("\n"))
            except (OSError, ConnectionError, ValueError):
                pass
            finally:
                await self._disconnect()
                log.info("%s disconnected", self.name)
            await asyncio.sleep(self._reconnect_s)

    async def _disconnect(self) -> None:
        writer, self._writer = self._writer, None
        if writer is None:
            return
        writer.close()
        # CancelledError is deliberately not suppressed: swallowing it here
        # leaves the supervising loop running after task.cancel(), and the
        # shutdown that cancelled it then waits for a task that never ends.
        with contextlib.suppress(OSError, ConnectionError):
            await writer.wait_closed()


class TeleopStream:
    """The browser-input TTL of ARCHITECTURE 4.5.

    The browser sends a normalised stick at >=10 Hz, centred included; this
    scales it by ``[limits] twist_power`` into ``TwistPayload(lin, ang)``, so a
    browser cannot name a value out of bounds.  Once the last stick message
    ages past ``teleop_input_max_age_ms`` the stream sends **one** zero twist
    and stops entirely: a backgrounded tab must not renew the last non-zero
    value.
    """

    def __init__(
        self,
        client: LineClient,
        *,
        twist_power: float,
        input_max_age_ms: int,
        hz: float = _TELEOP_HZ,
    ) -> None:
        self._client = client
        self._power = twist_power
        self._max_age_ns = input_max_age_ms * 1_000_000
        self._period = 1.0 / hz
        self._x = 0.0
        self._y = 0.0
        self._last_input_ns = 0
        self._streaming = False
        self._task: asyncio.Task[None] | None = None

    @property
    def streaming(self) -> bool:
        return self._streaming

    def update(self, x: float, y: float) -> None:
        """One joystick sample: ``x`` turn, ``y`` forward, each in [-1, 1]."""
        self._x = max(-1.0, min(1.0, x))
        self._y = max(-1.0, min(1.0, y))
        self._last_input_ns = time.monotonic_ns()
        self._streaming = True
        if self._task is None:
            self._task = asyncio.create_task(self._run(), name="rover-web:teleop")

    def tick(self) -> None:
        """One send decision, called at ``hz`` and directly by the tests."""
        if not self._streaming:
            return
        if time.monotonic_ns() - self._last_input_ns > self._max_age_ns:
            self._send(0.0, 0.0)
            self._streaming = False
            log.info("teleop input stale: zero twist sent, stream stopped")
            return
        self._send(self._x, self._y)

    async def close(self) -> None:
        """A WebSocket close is an immediate zero."""
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        if self._streaming:
            self._send(0.0, 0.0)
            self._streaming = False

    async def _run(self) -> None:
        while True:
            await asyncio.sleep(self._period)
            self.tick()

    def _send(self, x: float, y: float) -> None:
        self._client.send(
            TwistMessage(
                source=Source.TELEOP,
                cmd_id=new_cmd_id(),
                seq=self._client.next_seq(),
                twist=TwistPayload(
                    lin=round(y * self._power, 4),
                    ang=round(x * self._power, 4),
                ),
            )
        )


CONFIG = web.AppKey("config", RobotConfig)
ROBOTD_WEB = web.AppKey("robotd_web", LineClient)
# None whenever ``[bus] allow_stream`` omits ``teleop``, which production does.
ROBOTD_TELEOP: web.AppKey[LineClient | None] = web.AppKey("robotd_teleop")
BRAIN = web.AppKey("brain", LineClient)
WS_CLIENTS: web.AppKey[set[web.WebSocketResponse]] = web.AppKey("ws_clients")
RELAY: web.AppKey[asyncio.Queue[str]] = web.AppKey("relay")
RELAY_TASK: web.AppKey[asyncio.Task[None]] = web.AppKey("relay_task")

_Handler = Callable[[web.Request], Awaitable[web.StreamResponse]]


def _page(name: str) -> _Handler:
    path = STATIC_DIR / name

    async def handler(_request: web.Request) -> web.StreamResponse:
        return web.FileResponse(path)

    return handler


async def _relay_loop(app: web.Application) -> None:
    queue = app[RELAY]
    clients = app[WS_CLIENTS]
    while True:
        line = await queue.get()
        for ws in list(clients):
            if ws.closed:
                clients.discard(ws)
                continue
            with contextlib.suppress(ConnectionError, RuntimeError):
                await ws.send_str(line)


async def _post_estop(request: web.Request) -> web.Response:
    """The STOP button.  Ahead of the model and ahead of the validator (I-22)."""
    delivered = request.app[ROBOTD_WEB].send(
        EstopMessage(source=Source.WEB, reason="user")
    )
    log.warning("STOP pressed; delivered=%s", delivered)
    return web.json_response({"ok": True, "delivered": delivered})


async def _post_stop(request: web.Request) -> web.Response:
    delivered = request.app[ROBOTD_WEB].send(
        StopMessage(source=Source.WEB, reason="user")
    )
    return web.json_response({"ok": True, "delivered": delivered})


async def _post_clear(request: web.Request) -> web.Response:
    """Recovery from a latched fault; ``[bus] clear_sources`` defaults to ``web``.

    The body is optional and names the faults to clear, defaulting to the
    software e-stop.  ``obstacle_latched`` and ``low_battery`` are the other
    two names robotd accepts; a clear never re-arms anything, and robotd
    refuses one whose cause is still present.
    """
    faults = [ClearableFault.ESTOP_SW]
    if request.can_read_body:
        try:
            body = await request.json()
        except ValueError:
            raise web.HTTPBadRequest(reason="body is not JSON") from None
        named = body.get("faults") if isinstance(body, dict) else None
        if named is not None:
            if not isinstance(named, list) or not named:
                raise web.HTTPBadRequest(reason="faults must be a non-empty list")
            try:
                faults = [ClearableFault(str(name)) for name in named]
            except ValueError as exc:
                raise web.HTTPBadRequest(reason=f"unknown fault: {exc}") from None
    delivered = request.app[ROBOTD_WEB].send(
        ClearMessage(source=Source.WEB, faults=faults)
    )
    log.warning("clear %s; delivered=%s", [str(f) for f in faults], delivered)
    return web.json_response(
        {"ok": True, "delivered": delivered, "faults": [str(f) for f in faults]}
    )


async def _post_utter(request: web.Request) -> web.Response:
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(reason="body is not JSON") from None
    text = body.get("text") if isinstance(body, dict) else None
    if not isinstance(text, str) or not text.strip():
        raise web.HTTPBadRequest(reason="text is required")
    if len(text) > MAX_UTTERANCE_CHARS:
        raise web.HTTPBadRequest(reason="text is too long")
    delivered = _send_utterance(request.app, text)
    return web.json_response({"ok": True, "delivered": delivered})


async def _status(request: web.Request) -> web.Response:
    """What the pages show in their status strip; never a control path.

    ``stop_flags`` is the bit table the readout decodes ``state.rover.stop_flags``
    with, and ``clearable`` the names ``POST /clear`` accepts, both served from
    ``rover_contracts`` rather than restated in the page.
    """
    teleop = request.app[ROBOTD_TELEOP]
    return web.json_response(
        {
            "robotd": request.app[ROBOTD_WEB].connected,
            "brain": request.app[BRAIN].connected,
            "teleop_allowed": teleop is not None,
            "teleop": teleop.connected if teleop is not None else False,
            "stop_flags": STOP_FLAG_BITS,
            "clearable": [str(fault) for fault in ClearableFault],
        }
    )


def _send_utterance(app: web.Application, text: str) -> bool:
    return app[BRAIN].send(
        UtteranceMessage(
            source=UtteranceSource.WEB,
            text=text,
            confidence=None,
            is_final=True,
            mono_ns=time.monotonic_ns(),
        )
    )


async def _websocket(request: web.Request) -> web.WebSocketResponse:
    app = request.app
    ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=MAX_WS_MESSAGE_BYTES)
    await ws.prepare(request)
    app[WS_CLIENTS].add(ws)
    teleop = app[ROBOTD_TELEOP]
    stream = (
        None
        if teleop is None
        else TeleopStream(
            teleop,
            twist_power=app[CONFIG].limits.twist_power,
            input_max_age_ms=app[CONFIG].bus.teleop_input_max_age_ms,
        )
    )
    try:
        async for msg in ws:
            if msg.type is not WSMsgType.TEXT:
                continue
            try:
                payload = json.loads(msg.data)
            except ValueError:
                continue
            if isinstance(payload, dict):
                await _handle_ws_message(app, ws, stream, payload)
    finally:
        app[WS_CLIENTS].discard(ws)
        if stream is not None:
            await stream.close()
    return ws


async def _handle_ws_message(
    app: web.Application,
    ws: web.WebSocketResponse,
    stream: TeleopStream | None,
    payload: dict[str, object],
) -> None:
    kind = payload.get("type")
    if kind == "twist":
        if stream is None:
            await ws.send_str(json.dumps({"type": "error", "code": "source_not_allowed"}))
            return
        x, y = payload.get("x", 0.0), payload.get("y", 0.0)
        if isinstance(x, int | float) and isinstance(y, int | float):
            stream.update(float(x), float(y))
    elif kind == "utterance":
        text = payload.get("text")
        if isinstance(text, str) and text.strip() and len(text) <= MAX_UTTERANCE_CHARS:
            _send_utterance(app, text)
    elif kind == "ptt_start":
        app[BRAIN].send(PttStartMessage(source=UtteranceSource.WEB))
    elif kind == "ptt_end":
        app[BRAIN].send(PttEndMessage(source=UtteranceSource.WEB))


def create_app(config: RobotConfig) -> web.Application:
    """Build the app.  Nothing connects until startup and nothing raises if a
    peer is missing: rover-web must come up with robotd and brain dead."""
    app = web.Application(client_max_size=MAX_BODY_BYTES)
    relay: asyncio.Queue[str] = asyncio.Queue(maxsize=_RELAY_DEPTH)
    app[CONFIG] = config
    app[WS_CLIENTS] = set()
    app[RELAY] = relay

    def on_line(line: str) -> None:
        with contextlib.suppress(asyncio.QueueFull):
            relay.put_nowait(line)

    app[ROBOTD_WEB] = LineClient(
        config.bus.sock,
        "robotd:web",
        greeting=lambda: (
            HelloMessage(source=Source.WEB, pid=os.getpid(), caps=[BusCap.SUBSCRIBE]),
            SubscribeMessage(
                topics=[
                    SubscribeTopic.STATE,
                    SubscribeTopic.RESULT,
                    SubscribeTopic.EVENT,
                ],
                state_hz=config.bus.state_hz,
            ),
        ),
        on_line=on_line,
    )
    app[ROBOTD_TELEOP] = (
        LineClient(
            config.bus.sock,
            "robotd:teleop",
            greeting=lambda: (
                HelloMessage(source=Source.TELEOP, pid=os.getpid(), caps=[BusCap.TWIST]),
            ),
            on_line=on_line,
        )
        if "teleop" in config.bus.allow_stream
        else None
    )
    app[BRAIN] = LineClient(config.bus.brain_sock, "brain", on_line=on_line)

    app.add_routes(
        [
            web.get("/", _page("face.html")),
            web.get("/teleop", _page("teleop.html")),
            web.get("/ptt", _page("ptt.html")),
            web.get("/status", _status),
            web.get("/ws", _websocket),
            web.post("/estop", _post_estop),
            web.post("/stop", _post_stop),
            web.post("/clear", _post_clear),
            web.post("/utter", _post_utter),
            web.static("/static", STATIC_DIR),
        ]
    )
    app.on_startup.append(_on_startup)
    app.on_cleanup.append(_on_cleanup)
    return app


def _clients(app: web.Application) -> list[LineClient]:
    return [c for c in (app[ROBOTD_WEB], app[ROBOTD_TELEOP], app[BRAIN]) if c is not None]


async def _on_startup(app: web.Application) -> None:
    task: asyncio.Task[None] = asyncio.create_task(
        _relay_loop(app), name="rover-web:relay"
    )
    app[RELAY_TASK] = task
    for client in _clients(app):
        client.start()


async def _on_cleanup(app: web.Application) -> None:
    task = app.get(RELAY_TASK)
    if task is not None:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
    clients = app[WS_CLIENTS]
    for ws in list(clients):
        with contextlib.suppress(ConnectionError, RuntimeError):
            await ws.close()
    clients.clear()
    for client in _clients(app):
        await client.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rover-web")
    parser.add_argument("--config", default="config/robot.toml")
    # Both default to [web], which defaults to loopback: /utter reaches a motion
    # skill and /clear clears the software e-stop, neither authenticated, so
    # reaching the house Wi-Fi has to be something someone typed.
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    config = load_config(args.config)
    host = args.host if args.host is not None else config.web.bind
    port = args.port if args.port is not None else config.web.port
    if host not in ("127.0.0.1", "localhost", "::1"):
        log.warning(
            "serving on %s:%d -- /utter and /clear are unauthenticated, so every "
            "device on this network can drive the robot and clear a software "
            "e-stop",
            host,
            port,
        )
    web.run_app(create_app(config), host=host, port=port, print=None)
    return 0


if __name__ == "__main__":  # `python -m rover_web.app`
    raise SystemExit(main())
