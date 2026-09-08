# rover

An indoor AI robot on WAVE ROVER hardware, with a Pi host and a separate GPU model service.
The model proposes one bounded skill. `robotd` owns motion. Firmware enforces actuator limits and command expiry. A physical STOP cuts driver-board power.

Start with [debloat.md](debloat.md) for the end-to-end walkthrough, audit, cleanup plan, and verification status.
The Pi-to-Android migration is deliberate: prove the chassis with the existing host before replacing that host.
Android remains a skeleton. Physical performance and stop latency still require hardware checks.

## Run locally

```bash
uv sync --frozen --group dev
make test
make sim
```

If the checkout sits inside a synced folder (iCloud Drive's Desktop & Documents, Dropbox), keep the interpreter outside it: `export UV_PROJECT_ENVIRONMENT=~/.venvs/bot` before `uv sync`, and every `make` target follows it, putting the bytecode cache there too. iCloud evicts rarely-read files and re-fetches them on demand, and a test that imports the OpenAI SDK then blocks for minutes on a single `.pyc`; it also mints `name 2` duplicates that have already corrupted one git ref here. Moving the repository out of the synced folder is the better fix.

`make sim` starts fakebox, rover-stub, robotd, camera, brain, and web using `config/robot.mac.toml`.
It stops only the processes it started when you press Ctrl-C.
In another terminal:

```bash
uv run roverctl --config config/robot.mac.toml utter "turn left ninety degrees"
uv run roverctl --config config/robot.mac.toml drive-for 1.0 0.15
uv run roverctl --config config/robot.mac.toml say "hello"
uv run roverctl --config config/robot.mac.toml watch
uv run roverctl --config config/robot.mac.toml stop
open http://127.0.0.1:8080/
```

The CLI sends motion to robotd with a current camera still and client liveness.
It sends speech, scene description, search, and face requests to brain's real executors.
Only a matching terminal execution result counts as success.
Text instructions use the router and model path instead.

## Understand the code

| Location | Responsibility |
| --- | --- |
| `packages/rover_contracts/` | Typed messages, units, bounds, configuration, wire codec |
| `hosts/pi/rover_robotd/` | Only serial owner; admission, active goal, deadlines, budget, output |
| `hosts/pi/rover_brain/` | Instruction FSM, inference, speech, observations, composed skills |
| `hosts/pi/rover_cam/` | Camera and bounded frame publication |
| `hosts/pi/rover_web/` | Browser controls, telemetry, face, STOP |
| `packages/rover_devtools/` | CLI, diagnostics, simulated controller and model |
| `firmware/` | Pinned vendor source, reviewable patches, build, native regression tests |
| `tests/` | Unit/integration checks, gate runners, shared stimuli |
| `hosts/android/` | Planned replacement host; generated contracts and scaffolding |
| `legacy/firmware-s3/` | Retired controller reference, not an active runtime dependency |

The robot has no wheel encoders, measured-speed controller, map, or odometry.
`drive_for` applies power for time. `turn_to` follows IMU heading with a timeout.
Host headings increase leftward. Wire power `0.30` means 60% duty at the vendor's `0.5` full scale, not 30% duty.
Speech is half-duplex. Spoken STOP is best effort, not a guaranteed safety mechanism.

Use `tree --gitignore -L 3` or `git ls-files` to inspect source without build caches and runtime logs.
`make clean` removes generated caches/build output, not logs, sockets, models, or local configuration.

## Verify and deploy

`make help` lists commands. The main checks are `make test`, `make lint`, `make typecheck`, `make gates-full`, `make schema-check`, `make kotlin-check`, and `make firmware`.
The [gate map](docs/gates.md) separates simulated checks from physical measurements.
Passing simulated gates does not qualify the robot for unattended operation.

Before applying power, read [wiring.md](docs/wiring.md).
Build and verify the fork using [firmware/README.md](firmware/README.md).
Set up inference using [box/README.md](box/README.md).
Install the Pi with `hosts/pi/deploy/install.sh`, reboot for UART configuration, and run `hosts/pi/deploy/preflight.sh`.
The installer enables services without starting the complete target.
Inspect `hosts/pi/deploy/sync.sh HOST --dry-run` before syncing; the real command uses `rsync --delete` within the selected deployment directory.

Keep the wheels raised for initial motion tests.
Measure firmware expiry, STOP, sensor blocks, battery cutoff, stalled-wheel behavior, and heading direction on the real board before floor tests.
Keep the physical STOP accessible. Do not operate unattended near children, pets, or stairs.

Configuration lives in `config/robot.example.toml` and the local ignored `config/robot.toml`.
`ROVER__SECTION__KEY` overrides machine-specific settings. Keep credentials in the environment variables named by configuration.

## Document authority and licences

[ADR-0013](docs/adr/0013-wave-rover-open-loop.md) amends the frozen original [ARCHITECTURE.md](ARCHITECTURE.md).
[protocol.md](docs/protocol.md) specifies the current link.
[debloat.md](debloat.md) records what the current implementation actually does and which claims remain unverified.
Older research and verification transcripts are historical evidence, not current passing results.

First-party code uses [Apache-2.0](LICENSE). Vendor firmware and Piper have separate licence boundaries described in [THIRD_PARTY.md](THIRD_PARTY.md).
The repository does not ship wake-word or speech-recognition models.
