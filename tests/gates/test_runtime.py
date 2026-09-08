"""Gate-only runtime boundaries, using the same isolated motion fixture."""

from __future__ import annotations

import asyncio
import base64
import json
import os
import signal
import sys
import tempfile
import threading
from pathlib import Path

import pytest
from aiohttp import ClientSession, web

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "unit"))
from g4.fuzz import CORPUS_SIZE, build_corpus
from gatelib.checks import child_env
from rover_cam.fake_backend import jpeg_size
from rover_contracts import client_adapter
from rover_contracts.config import BusConfig, LogConfig, load_config
from rover_devtools.fakebox import FakeBox, make_server
from rover_devtools.rover_stub import RoverStub, StubServer
from rover_web.app import create_app
from test_robotd_link import FakeRover, link_config, speeds_of, until
from test_robotd_motion import (
    CMD2,
    TURN,
    Client,
    assert_quiet,
    client,
    drive_message,
    rover,
    start,
)

pytestmark = pytest.mark.timeout(60)


async def test_cli_brain_robotd_camera_box_and_web_stop(tmp_path, monkeypatch):
    """Public process entry points, real software servers, no hardware or audio.

    The WAVE stub supplies the simulated applied power and heading. This is
    software integration evidence, not a firmware or physical braking test.
    """
    board = StubServer(RoverStub(), tcp_port=0)
    box = FakeBox()
    received_images = []
    record = box.record

    def record_image(body):
        entry = record(body)
        for message in body["messages"]:
            content = message.get("content")
            for part in content if isinstance(content, list) else []:
                if part.get("type") == "image_url":
                    encoded = part["image_url"]["url"].split(",", 1)[1]
                    size = jpeg_size(base64.b64decode(encoded))
                    received_images.append((entry.schema, size))
        return entry

    monkeypatch.setattr(box, "record", record_image)
    endpoint = make_server(port=0, box=box)
    worker = threading.Thread(target=endpoint.serve_forever, daemon=True)
    worker.start()
    processes = []
    runner = connection = ping = None
    env = {k: v for k, v in child_env().items() if not k.startswith("ROVER__")}
    env["ROVER_BOX_API_KEY"] = ""
    with tempfile.TemporaryDirectory(prefix="rover-e2e-", dir="/tmp") as directory:
        config_path = Path(directory) / "robot.toml"

        async def launch(module, *args):
            output = tmp_path / f"{len(processes)}-{module}.log"
            with output.open("wb") as stream:
                process = await asyncio.create_subprocess_exec(
                    sys.executable,
                    "-m",
                    module,
                    "--config",
                    str(config_path),
                    *args,
                    env=env,
                    cwd=directory,
                    stdin=asyncio.subprocess.DEVNULL,
                    stdout=stream,
                    stderr=asyncio.subprocess.STDOUT,
                )
            processes.append((process, output))
            return process, output

        async def cli(*args, expected=0):
            process, output = await launch("rover_devtools.roverctl", *args)
            await asyncio.wait_for(process.wait(), 12)
            text = output.read_text()
            assert process.returncode == expected, text
            return text

        try:
            await board.start()
            config_path.write_text(
                f'[link]\nbackend = "tcp"\ntcp_port = {board.tcp_port}\n'
                f'[bus]\nsock = "{directory}/robotd.sock"\n'
                f'frames_sock = "{directory}/frames.sock"\n'
                f'brain_sock = "{directory}/brain.sock"\nsource_uids = {{}}\n'
                "[limits]\nmotion_cooldown_ms = 0\n"
                '[camera]\nbackend = "fake"\n'
                '[audio]\ninput = "text"\n[tts]\nbackend = "null"\n'
                f'[box]\nurl = "http://127.0.0.1:{endpoint.server_port}/v1"\n'
                f"[log]\ndir = {json.dumps(str(tmp_path / 'events'))}\n"
            )
            config = load_config(config_path, env={})
            await launch("rover_robotd.main")
            await launch("rover_cam.publisher", "--still-period-s", "0.1")
            await until(lambda: Path(config.bus.sock).exists(), timeout=10)
            reader, writer = await asyncio.open_unix_connection(config.bus.sock)
            connection = Client(reader, writer, "web")
            await connection.send(type="hello", pid=os.getpid(), caps=["subscribe"])
            await connection.receive("welcome")
            await connection.send(type="subscribe", topics=["state"], state_hz=20)
            ping = asyncio.create_task(connection.ping())
            async with asyncio.timeout(5):
                while not (await connection.receive("state"))["ready"]:
                    pass
            await until(lambda: Path(config.bus.frames_sock).exists(), timeout=10)
            await launch("rover_brain.main")
            await until(lambda: Path(config.bus.brain_sock).exists(), timeout=10)

            # describe_scene must ask the camera for main, not use lores cache.
            assert "result  done" in await cli("skill", "describe_scene")
            assert ("scene", (896, 672)) in received_images
            assert "result  done" in await cli("say", "Software integration check.")
            assert "result  done" in await cli(
                "skill", "find", "--args", '{"object":"mug","max_sweeps":1}'
            )
            assert ("find", (896, 672)) in received_images
            assert abs(board.stub.plant.heading_deg) > 2
            assert "result  done" in await cli(
                "skill", "set_face", "--args", '{"expr":"happy"}'
            )

            # Direct CLI motion also carries real observation metadata and pings.
            direct, direct_log = await launch(
                "rover_devtools.roverctl", "drive-for", "0.6", "0.1"
            )
            await until(lambda: board.stub.applied_l > 0 and board.stub.applied_r > 0)
            await asyncio.wait_for(direct.wait(), 5)
            assert direct.returncode == 0, direct_log.read_text()
            assert "result  done" in direct_log.read_text()
            await until(lambda: board.stub.applied_l == board.stub.applied_r == 0)

            # An utterance goes through brain, then robotd, then WAVE JSON.
            await connection.send(
                type="subscribe", topics=["state", "result"], state_hz=20
            )
            movement, movement_log = await launch(
                "rover_devtools.roverctl", "utter", "advance", "--watch", "3"
            )
            await until(lambda: board.stub.applied_l > 0 and board.stub.applied_r > 0)
            accepted = await connection.receive("result", status="accepted")
            await until(lambda: board.stub.applied_l == board.stub.applied_r == 0)
            await connection.receive("result", cmd_id=accepted["cmd_id"], status="done")
            await asyncio.wait_for(movement.wait(), 5)
            assert movement.returncode == 0, movement_log.read_text()
            assert "fsm     IDLE" in movement_log.read_text()
            assert any(request.schema == "skill_call" for request in box.requests)
            assert board.stub.clamp_count == 0

            runner = web.AppRunner(create_app(config))
            await runner.setup()
            site = web.TCPSite(runner, "127.0.0.1", 0)
            await site.start()
            web_port = runner.addresses[0][1]
            web_url = f"http://127.0.0.1:{web_port}"
            async with ClientSession() as http:
                async with asyncio.timeout(5):
                    while True:
                        async with http.get(f"{web_url}/status") as reply:
                            status = await reply.json()
                        if status["robotd"] and status["brain"]:
                            break
                        await asyncio.sleep(0.02)
                await cli("utter", "forward")
                await until(lambda: board.stub.applied_l > 0 and board.stub.applied_r > 0)
                async with http.post(f"{web_url}/estop") as reply:
                    assert await reply.json() == {"ok": True, "delivered": True}
                await until(
                    lambda: board.stub.applied_l == board.stub.applied_r == 0,
                    timeout=0.5,
                )
                async with asyncio.timeout(5):
                    while not (await connection.receive("state"))["estop_sw"]:
                        pass
                assert "reason=estop_active" in await cli(
                    "drive-for", "0.2", "0.1", expected=1
                )
                assert board.stub.applied_l == board.stub.applied_r == 0
        except BaseException:
            for _, output in processes:
                print(f"\n{output.name}:\n{output.read_text()[-6000:]}")
            raise
        finally:
            if runner:
                await runner.cleanup()
            if ping:
                ping.cancel()
                await asyncio.gather(ping, return_exceptions=True)
            if connection:
                connection.writer.close()
                await connection.writer.wait_closed()
            for process, _ in reversed(processes):
                if process.returncode is None:
                    process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), 3)
                    except TimeoutError:
                        process.kill()
                        await process.wait()
            await board.stop()
            await asyncio.to_thread(endpoint.shutdown)
            endpoint.server_close()
            await asyncio.to_thread(worker.join, 3)
            assert not worker.is_alive()


async def test_five_hundred_fuzz_lines_match_schema_and_invalid_lines_cannot_move(
    tmp_path,
):
    corpus = build_corpus()
    assert len(corpus) == CORPUS_SIZE
    assert sum(case.valid for case in corpus) > 50
    for case in corpus:
        try:
            client_adapter.validate_json(case.line)
            accepted = True
        except ValueError:
            accepted = False
        assert accepted == case.valid, case.name
    async with rover(tmp_path, teleop=True) as (robotd, board), client(robotd) as brain:
        await start(brain, board)
        await brain.result("done")
        await assert_quiet(board)
        boundary = len(speeds_of(board))
        for case in (case for case in corpus if not case.valid):
            brain.writer.write(case.line.encode() + b"\n")
            await brain.writer.drain()
            answer = json.loads(await asyncio.wait_for(brain.reader.readline(), 2))
            assert answer["type"] == "error" or (
                answer["type"] == "result" and answer["status"] == "rejected"
            ), (case.name, answer)
        await assert_quiet(board)
        assert all(pair == (0, 0) for pair in speeds_of(board)[boundary:])
        await brain.send(**drive_message(cmd_id=CMD2, seq=2))
        await brain.result("accepted", CMD2)
        await brain.result("done", CMD2)
        assert board.clamp_count == 0


async def test_obstacle_aborts_forward_but_reverse_remains_available(tmp_path):
    async with rover(tmp_path) as (robotd, board), client(robotd) as brain:
        await start(brain, board, duration=2.0)
        board.tof_mm = 231
        assert (await brain.result("aborted"))["reason"] == "obstacle"
        await assert_quiet(board)
        await brain.send(**drive_message(cmd_id=CMD2, seq=2, power=-0.15))
        await brain.result("accepted", CMD2)
        await brain.result("done", CMD2)
        assert any(left < 0 and right < 0 for left, right in speeds_of(board))


async def test_source_spoof_and_exhausted_budget_cannot_emit_motion(tmp_path):
    async with rover(tmp_path, teleop=True) as (robotd, board), client(robotd) as brain:
        await start(brain, board)
        await brain.result("done")
        await brain.send(
            type="twist",
            source="teleop",
            cmd_id=CMD2,
            seq=2,
            twist={"lin": 0.2, "ang": 0.0},
        )
        assert (await brain.result("rejected", CMD2))["reason"] == "source_not_allowed"
        robotd.budget.charge(TURN, robotd.config.limits.budget_motion_s)
        await brain.send(**drive_message(cmd_id=CMD2, seq=3))
        assert (await brain.result("rejected", CMD2))["reason"] == "budget_exceeded"
        await assert_quiet(board)


@pytest.mark.skipif(
    not hasattr(signal, "SIGSTOP"), reason="requires POSIX process suspension"
)
async def test_frozen_owned_robotd_stops_stream_and_expired_goal_does_not_resume(
    tmp_path,
):
    """Real process freeze; independent observer and board heartbeat keep running.

    The goal expires while suspended. This proves the owned process emits no
    keepalive and does not replay the expired goal, not physical motor braking.
    """
    board = FakeRover()
    port = await board.start_tcp()
    with tempfile.TemporaryDirectory(prefix="rover-freeze-", dir="/tmp") as directory:
        config = link_config(
            port,
            bus=BusConfig(sock=f"{directory}/bus.sock", source_uids={}),
            log=LogConfig(dir=str(tmp_path)),
        )
        code = (
            "import asyncio,sys; from rover_contracts.config import RobotConfig; "
            "from rover_robotd.main import serve; "
            "asyncio.run(serve(RobotConfig.model_validate_json(sys.argv[1])))"
        )
        process = await asyncio.create_subprocess_exec(
            sys.executable,
            "-c",
            code,
            config.model_dump_json(),
            env=child_env(),
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.PIPE,
        )
        connection = ping = None
        try:
            await until(
                lambda: Path(config.bus.sock).exists() or process.returncode is not None
            )
            assert process.returncode is None, (await process.stderr.read()).decode()
            reader, writer = await asyncio.open_unix_connection(config.bus.sock)
            connection = Client(reader, writer, "brain")
            await connection.send(
                type="hello", pid=os.getpid(), caps=["skill", "subscribe"]
            )
            await connection.receive("welcome")
            await connection.send(type="subscribe", topics=["state"], state_hz=20)
            async with asyncio.timeout(5):
                while not (await connection.receive("state"))["ready"]:
                    pass
            ping = asyncio.create_task(connection.ping())
            await start(connection, board, duration=0.6)
            process.send_signal(signal.SIGSTOP)
            await until(lambda: not board.hb_alive)
            boundary = len(speeds_of(board))
            await asyncio.sleep(0.65)  # expire the 600 ms profile while suspended
            assert len(speeds_of(board)) == boundary
            assert board.left == board.right == 0
            process.send_signal(signal.SIGCONT)
            await until(lambda: len(speeds_of(board)) > boundary)
            await assert_quiet(board)
            assert all(pair == (0, 0) for pair in speeds_of(board)[boundary:])
        finally:
            if ping:
                ping.cancel()
                await asyncio.gather(ping, return_exceptions=True)
            if process.returncode is None:
                process.send_signal(signal.SIGCONT)
                process.terminate()
                try:
                    await asyncio.wait_for(process.wait(), 3)
                except TimeoutError:
                    process.kill()
                    await process.wait()
            if connection:
                connection.writer.close()
                await connection.writer.wait_closed()
            await board.stop()
