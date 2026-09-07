# Rover build and test entry points (ARCHITECTURE 10 and 12).
#
# Everything here runs on a MacBook Air M4 with no hardware attached, which is
# directive 3: nothing ships that cannot run on the MacBook first, and the fakes
# are configuration (config/robot.mac.toml), never a code branch.
#
#   make sim      the whole rover, no hardware
#   make test     unit + contract, Python and the C core
#   make gates    the fast gate subset
#
# `make help` lists the rest.

SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.DEFAULT_GOAL := help

UV       ?= uv
VENV     ?= .venv
PY       ?= $(VENV)/bin/python
CONFIG   ?= config/robot.mac.toml
RUN_DIR  ?= run
WEB_PORT ?= 8080
BOX_PORT ?= 8000
# Extra pytest arguments for `make gates-sim`, e.g. -k "g4".
GATES_SIM_ARGS ?=
# robotd and brain both take --log-level.  `make sim LOG_LEVEL=debug` puts every
# encoded down-frame in run/robotd.log, which is the only way to see the wire
# while I-18 keeps robotd the sole owner of the port.
LOG_LEVEL ?= info
IDF_TAG  ?= v5.5.5
PORT     ?=
# The host build drops the simulator here under ARCHITECTURE 10's spelling,
# `mcu-sim`, which is the name rover_devtools.mcu_sim searches for; the export
# is belt-and-braces for an out-of-tree HOST_BUILD.
HOST_BUILD ?= firmware/host/build
MCU_SIM    ?= $(HOST_BUILD)/mcu-sim
export ROVER_MCU_SIM = $(abspath $(MCU_SIM))
# cmake and ninja are pip-installable and the dev group brings them in, so a
# bare macOS with no Homebrew toolchain still builds the C core.
CMAKE ?= $(if $(shell command -v cmake),cmake,$(abspath $(VENV)/bin/cmake))
CTEST ?= $(if $(shell command -v ctest),ctest,$(abspath $(VENV)/bin/ctest))

# Every component takes --config and defaults to config/robot.toml, so the Mac
# profile is passed explicitly rather than left to an environment variable.
# packages/ on the path, so an uninstalled clone behaves like an installed one.
export PYTHONPATH := packages$(if $(PYTHONPATH),:$(PYTHONPATH),)

.PHONY: help venv test test-py test-firmware host-build lint format typecheck \
        schemas schema-check sim dev gates gates-sim gates-full gates-pi gate-g1 firmware \
        firmware-debug flash docker-dev box secrets deps-check doctor clean distclean

help:
	@echo "rover — make targets"
	@echo
	@echo "  sim / dev     whole rover on this Mac: fakebox, mcu-sim on a pty,"
	@echo "                robotd, cam (fake), brain (text), web on :$(WEB_PORT)"
	@echo "  test          unit + contract tests, Python and the C core"
	@echo "  lint          ruff check + shell syntax"
	@echo "  typecheck     mypy over packages/"
	@echo "  schemas       regenerate box/schema/*.json from rover_contracts"
	@echo "  schema-check  fail if box/schema/*.json has drifted"
	@echo "  gates         fast gate subset (G2-sim, G4-sim fuzz and races)"
	@echo "  gates-sim     boots the six sim processes, runs the gates, tears down"
	@echo "  gates-full    adds the 256-flag sweep, the budget drives, the races"
	@echo "  gates-pi      the G3a soak; only meaningful on the Pi (G3_SOAK_S=1800)"
	@echo "  gate-g1       model + validator gate; BOX=real to use the real box"
	@echo "  firmware      build the ESP-IDF app in the espressif/idf:$(IDF_TAG) image"
	@echo "  flash         PORT=/dev/tty.usbserial-XXXX; native esptool via uvx"
	@echo "  docker-dev    linux/arm64 parity image for the Pi environment"
	@echo "  box           start vLLM from docker/compose.box.yml"
	@echo "  secrets       scan the tree for anything that must not be published"
	@echo "  deps-check    assert robotd's runtime closure has no ML package"
	@echo "  doctor        resolved config, limits, devices, sockets, import map"
	@echo "  clean         remove caches, run/ and build output"

# --- environment ------------------------------------------------------------

venv: $(PY)

$(PY):
	$(UV) sync --frozen --group dev

# --- tests ------------------------------------------------------------------

test: test-py test-firmware

test-py: venv
	$(PY) -m pytest tests/unit tests/contract

# The C core builds under host clang with ASan and UBSan and runs the same
# tests/contract/serial_vectors.jsonl the Python codec does (A3).  Skipped with
# a notice while firmware/host/ does not exist, so the Python half of `make
# test` is usable before the firmware lands.
test-firmware: host-build
	@if [ -d $(HOST_BUILD) ]; then \
	  $(CTEST) --test-dir $(HOST_BUILD) --output-on-failure; \
	else \
	  echo "skip: no host build"; \
	fi

# Builds both the core's host tests and the mcu_sim binary `make sim` drives.
host-build:
	@if [ -f firmware/host/CMakeLists.txt ]; then \
	  $(CMAKE) -S firmware/host -B $(HOST_BUILD) -DROVER_HOST_TEST=ON >/dev/null && \
	  $(CMAKE) --build $(HOST_BUILD); \
	else \
	  echo "skip: firmware/host/CMakeLists.txt not present yet"; \
	fi

# --- lint -------------------------------------------------------------------

# The gate is the selected rule set in pyproject.toml (E, F, I, UP, B, SIM at
# 90 columns), which is what packages/rover_contracts was written against.
# `ruff format` is a convenience below, deliberately not a gate: adding a
# formatter check mid-build would fail every component for reasons unrelated to
# whether it is correct.
lint: venv
	$(PY) -m ruff check .
	@for f in $$(find deploy -name '*.sh' 2>/dev/null); do bash -n "$$f" || exit 1; done
	@echo "shell syntax ok"

format: venv
	$(PY) -m ruff format .
	$(PY) -m ruff check --fix .

typecheck: venv
	$(PY) -m mypy packages

# --- schemas ----------------------------------------------------------------
# box/schema/*.json is generated, never hand-edited: the model's output schema
# and the two Observation schemas have exactly one definition, in
# rover_contracts, and CI fails if the committed copies drift from it.

define GEN_SCHEMAS
import json, pathlib
from rover_brain.box import SKILL_CALL_SCHEMA
from rover_contracts.observations import FindObservation, SceneObservation
out = pathlib.Path("box/schema")
out.mkdir(parents=True, exist_ok=True)
# skillcall.json is the schema brain actually puts in response_format, not a
# second rendering of the same models: a back end that constrains a flat oneOf
# and stalls on $ref-with-discriminator would otherwise pass G1 and fail in
# production, with nothing in the gate able to see it.
for name, schema in (
    ("skillcall", SKILL_CALL_SCHEMA),
    ("find", FindObservation.model_json_schema()),
    ("scene", SceneObservation.model_json_schema()),
):
    (out / f"{name}.json").write_text(json.dumps(schema, indent=2, sort_keys=True) + "\n")
    print("wrote", out / f"{name}.json")
endef
export GEN_SCHEMAS

schemas: venv
	@$(PY) -c "$$GEN_SCHEMAS"

schema-check: venv
	@tmp=$$(mktemp -d); cp -R box/schema "$$tmp/before" 2>/dev/null || true; \
	$(PY) -c "$$GEN_SCHEMAS" >/dev/null; \
	if ! diff -ru "$$tmp/before" box/schema; then \
	  echo "box/schema/*.json is stale — run 'make schemas' and commit"; \
	  rm -rf "$$tmp"; exit 1; \
	fi; \
	rm -rf "$$tmp"; echo "schemas up to date"

# --- the simulated rover ----------------------------------------------------

# The boot half, shared by `make sim` (which then waits) and `make gates-sim`
# (which then runs the gates and tears down).  A3 claims 19 of 24 invariants go
# green before hardware; that claim is only true of a run that actually has a
# rover up, so the target CI runs has to start one.
define SIM_BOOT
set -uo pipefail

RUN=$(RUN_DIR)
mkdir -p "$$RUN" data/logs data/models/stt
pids=()

# Import main() and call it, rather than `python -m`: every component defines
# main(), not every one has a __main__ guard, and this form needs no install --
# a bare clone with packages/ on PYTHONPATH behaves like a deployed one.
start() {                 # start <name> <module> [args...]
  local name=$$1 module=$$2; shift 2
  echo "  start $$name"
  $(PY) -c "import sys; from $$module import main; sys.exit(main())" "$$@" \
    >"$$RUN/$$name.log" 2>&1 &
  pids+=($$!)
}

die() {
  echo
  echo "FAILED: $$1"
  [ -n "$${2:-}" ] && { echo "--- last 20 lines of $$RUN/$$2.log ---"; tail -20 "$$RUN/$$2.log" || true; }
  exit 1
}

wait_for() {              # wait_for <description> <seconds> <shell test> [logname]
  local what=$$1 limit=$$2 probe=$$3 logname=$${4:-}
  local i=0
  while ! eval "$$probe" >/dev/null 2>&1; do
    i=$$((i + 1))
    [ "$$i" -gt "$$((limit * 10))" ] && die "$$what did not come up in $${limit}s" "$$logname"
    sleep 0.1
  done
  echo "  ok    $$what"
}

cleanup() {
  echo
  echo "stopping..."
  for p in "$${pids[@]:-}"; do [ -n "$$p" ] && kill "$$p" 2>/dev/null || true; done
  for p in "$${pids[@]:-}"; do [ -n "$$p" ] && wait "$$p" 2>/dev/null || true; done
  rm -f "$$RUN/mcu.pty"
  echo "stopped."
}
trap cleanup EXIT INT TERM

echo "rover simulation — config $(CONFIG), logs in $$RUN/"
echo

# The fake box and the simulated MCU first: neither depends on anything.
start fakebox rover_devtools.fakebox --host 127.0.0.1 --port $(BOX_PORT)
start mcu-sim rover_devtools.mcu_sim --link "$$RUN/mcu.pty" $(MCU_SIM_FLAGS)
wait_for "fakebox on :$(BOX_PORT)" 20 "curl -sf http://127.0.0.1:$(BOX_PORT)/v1/models" fakebox
wait_for "mcu-sim pty at $$RUN/mcu.pty" 20 "[ -e '$$RUN/mcu.pty' ]" mcu-sim

# robotd retries open() every [serial] open_retry_ms, so it is not order-bound.
start robotd rover_robotd.main --config $(CONFIG) --log-level $(LOG_LEVEL)
wait_for "robotd bus at $$RUN/robotd.sock" 20 "[ -S '$$RUN/robotd.sock' ]" robotd

start cam rover_cam.publisher --config $(CONFIG)
wait_for "cam frames at $$RUN/frames.sock" 20 "[ -S '$$RUN/frames.sock' ]" cam

start brain rover_brain.main --config $(CONFIG) --log-level $(LOG_LEVEL)
wait_for "brain bus at $$RUN/brain.sock" 30 "[ -S '$$RUN/brain.sock' ]" brain

start web rover_web.app --config $(CONFIG) --host 127.0.0.1 --port $(WEB_PORT)
wait_for "web on :$(WEB_PORT)" 20 "curl -sf http://127.0.0.1:$(WEB_PORT)/ -o /dev/null" web
endef
export SIM_BOOT

define SIM_SH
# eval, not $$SIM_BOOT on its own line: bash would word-split the variable and
# run its first word as a command.  eval runs it in this shell, so the traps,
# the functions and the pid list all survive into what follows.
eval "$$SIM_BOOT"

cat <<'EOF'

  All six up. Try:

    face + STOP button   http://127.0.0.1:$(WEB_PORT)/
    teleop joystick      http://127.0.0.1:$(WEB_PORT)/teleop
    speak to it          $(PY) -m rover_devtools.roverctl --config $(CONFIG) utter "turn left ninety degrees"
    watch the wire       $(PY) -m rover_devtools.wirecat --config $(CONFIG) --exclude T $(RUN_DIR)/mcu.pty
    resolved config      make doctor

  Inject a fault without restarting anything:

    FAKEBOX_FAULT=out_of_range   on the fakebox environment
    make sim MCU_SIM_FLAGS="obstacle=231"        ToF at 231 mm (I-5)
    make sim LOG_LEVEL=debug                     every down-frame in robotd.log

  Logs: $(RUN_DIR)/{fakebox,mcu-sim,robotd,cam,brain,web}.log
  Ctrl-C stops everything.

EOF

wait
endef
export SIM_SH

# The same six processes, then the gates against them, then teardown.  Without
# this every gate case whose precondition is "something is listening" skips,
# run_gate maps "nothing ran" to a skip and `make gates` is green with eleven
# enforcement cases never executed.
define GATES_SIM_SH
eval "$$SIM_BOOT"

echo
echo "running the gates against the simulated rover"
set +e
$(PY) -m pytest tests/gates -m gate $(GATES_SIM_ARGS)
status=$$?
set -e
exit $$status
endef
export GATES_SIM_SH

sim: venv host-build
	@bash -c "$$SIM_SH"

dev: sim

# --- gates ------------------------------------------------------------------
# Every gate writes its result as JSON into logs/gates/ so the tracker links
# evidence rather than assertions (docs/gates.md).  No wall-clock claim is made
# for any of these: G1 alone is 510 fakebox requests and I-15's ten drives sit
# behind mandated cooldowns.


# run_gate <path> [extra pytest args] — a gate directory that does not exist
# yet is a skip with a notice, never a silent pass and never a CI wall for
# every other component.
run_gate = @if [ -d "$(1)" ]; then $(PY) -m pytest "$(1)" -m gate $(2); \
	   else echo "skip: $(1) not present yet"; fi

gates: venv
	$(call run_gate,tests/gates,-k "g2 or g4")

# What CI runs: a rover is up, so nothing skips for want of a peer.
gates-sim: venv host-build
	@bash -c "$$GATES_SIM_SH"

gates-full: venv
	$(call run_gate,tests/gates)

# The Pi's venv carries no test tooling: install.sh runs `uv pip install -e .`
# with no dependency group, and ARCHITECTURE 12 says the production venv must
# never carry it -- so `pytest` is absent and `uv run --group dev` would install
# it into the environment robotd, cam and web execute from.  The gate script
# puts packages/ and tests/gates on sys.path itself, so it runs directly.
# G3_SOAK_S is the soak length: 1800 is G3a's 30 minutes.
G3_SOAK_S ?= 1800
gates-pi: venv
	$(PY) tests/gates/g3/run_g3.py --duration $(G3_SOAK_S) --hardware

gate-g1: venv
	$(call run_gate,tests/gates/g1)

# --- firmware ---------------------------------------------------------------
# Check `docker manifest inspect espressif/idf:$(IDF_TAG) | grep architecture`
# before build day: whether that image publishes a linux/arm64 manifest is
# unverified.  If it is amd64-only, add --platform linux/amd64 (works under
# QEMU, several times slower).  firmware/core/ builds under host clang either
# way, so `make test` does not depend on this.

# Through build.sh, never a bare `idf.py build`: build.sh is what passes
# -DSDKCONFIG per profile, and a bare build reuses whatever sdkconfig the last
# `build.sh debug` left in firmware/, shipping a console in a release image.
firmware:
	ROVER_IDF_IMAGE=espressif/idf:$(IDF_TAG) firmware/docker/build.sh release

firmware-debug:
	ROVER_IDF_IMAGE=espressif/idf:$(IDF_TAG) firmware/docker/build.sh debug

# Native macOS esptool: Docker Desktop has no USB passthrough, so the IDF
# container cannot see the port.  flash_args holds paths relative to the build
# directory, which is why this cds there first.
flash:
	@[ -n "$(PORT)" ] || { echo "usage: make flash PORT=/dev/tty.usbserial-XXXX"; exit 2; }
	cd firmware/build && uvx --from 'esptool==5.*' esptool \
	  --chip esp32s3 -p $(PORT) -b 460800 write_flash "@flash_args"

# --- containers -------------------------------------------------------------

docker-dev:
	docker buildx build --platform linux/arm64 -f docker/Dockerfile.services -t rover-services:dev .
	docker compose -f docker/compose.dev.yml run --rm services

box:
	docker compose -f docker/compose.box.yml up -d vllm

# --- checks -----------------------------------------------------------------

secrets:
	./deploy/check-secrets.sh

# robotd must never import an ML library (4.2).  The base runtime closure is
# resolved from uv.lock, so this is offline and exact.
deps-check:
	@closure=$$(mktemp); \
	 $(UV) export --frozen --no-dev --no-emit-project --no-emit-workspace \
	   --format requirements.txt \
	 | sed -n 's/^\([A-Za-z0-9._-]*\)==.*/\1/p' | tr 'A-Z_' 'a-z-' | sort -u > "$$closure"; \
	 bad=$$(grep -E '^(numpy|scipy|torch|torchaudio|torchvision|tensorflow|onnx|onnxruntime.*|sherpa-onnx|transformers|sentencepiece|tflite-runtime|pyopen-wakeword|opencv-.*|sounddevice)$$' "$$closure" || true); \
	 rm -f "$$closure"; \
	 if [ -n "$$bad" ]; then echo "ML package in the base runtime closure:"; echo "$$bad"; exit 1; fi; \
	 echo "runtime closure clean: no ML package"

# doctor is the one target README tells a Pi user to run when a step fails, and
# CONFIG is the Mac profile: on the Pi it described the simulator -- pty serial,
# sockets under run/, fake camera, a loopback box -- and never looked at
# /dev/rover-mcu or /run/rover, so the real fault stayed invisible behind three
# false warnings.  It resolves the real file when one exists, which is what
# every component's own --config default already does.
DOCTOR_CONFIG ?= $(if $(wildcard config/robot.toml),config/robot.toml,$(CONFIG))

doctor: venv
	$(PY) -m rover_devtools.doctor --config $(DOCTOR_CONFIG)

# --- housekeeping -----------------------------------------------------------

clean:
	rm -rf $(RUN_DIR) .pytest_cache .ruff_cache .mypy_cache .hypothesis \
	       $(HOST_BUILD) firmware/build
	find . -name __pycache__ -type d -prune -exec rm -rf {} +

distclean: clean
	rm -rf $(VENV) data
