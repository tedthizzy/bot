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
STUB_PORT ?= 7777
STUB_FLAGS ?=
GATES_SIM_ARGS ?=
LOG_LEVEL ?= info
PORT     ?=

# A bare clone behaves like an installed one: every package under packages/ and
# hosts/pi/ imports without `uv sync` having run.
export PYTHONPATH := packages:hosts/pi$(if $(PYTHONPATH),:$(PYTHONPATH),)

.PHONY: help venv test lint format typecheck schemas schema-check sim dev gates \
        gates-sim gates-full gates-pi gate-g1 firmware flash docker-dev box \
        secrets deps-check doctor kotlin kotlin-check clean distclean

help:
	@echo "rover — make targets"
	@echo
	@echo "  sim / dev     whole rover on this Mac: fakebox, rover-stub on TCP :$(STUB_PORT),"
	@echo "                robotd, cam (fake), brain (text), web on :$(WEB_PORT)"
	@echo "  test          unit tests (contracts, robotd, brain, cam, web, devtools)"
	@echo "  lint          ruff check + shell syntax"
	@echo "  typecheck     mypy over packages/ and hosts/pi/"
	@echo "  schemas       regenerate box/schema/*.json from the contracts"
	@echo "  schema-check  fail if box/schema/*.json has drifted"
	@echo "  kotlin        regenerate the Android data classes from the contracts"
	@echo "  gates         fast gate subset (G2 patch verification, G4 fuzz and races)"
	@echo "  gates-sim     alias for gates-full; all test services are isolated"
	@echo "  gates-full    software gates and end-to-end simulation; hardware skips stay visible"
	@echo "  gates-pi      the G3 soak; only meaningful on the Pi (G3_SOAK_S=1800)"
	@echo "  gate-g1       model + validator gate; BOX=real to use the real box"
	@echo "  firmware      compile the Waveshare fork in the arduino-cli container"
	@echo "  flash         PORT=/dev/tty.usbserial-XXXX; esptool via uvx"
	@echo "  docker-dev    linux/arm64 parity image for the Pi environment"
	@echo "  box           start vLLM from docker/compose.box.yml"
	@echo "  secrets       scan the tree for anything that must not be published"
	@echo "  deps-check    assert robotd's runtime closure has no ML package"
	@echo "  doctor        resolved config, limits, devices, sockets, firmware fork"
	@echo "  clean         remove caches and firmware build output; keep runtime data"

venv: $(PY)
$(PY):
	$(UV) sync --frozen --group dev

# --- tests --------------------------------------------------------------------

test: venv
	$(PY) -m pytest tests/unit

lint: venv
	$(PY) -m ruff check .
	@while IFS= read -r f; do bash -n "$$f"; done < <(rg --files --hidden hosts/pi/deploy firmware -g '*.sh')
	@echo "shell syntax ok"

format: venv
	$(PY) -m ruff format .
	$(PY) -m ruff check --fix .

typecheck: venv
	$(PY) -m mypy packages hosts/pi

# --- generated artefacts ------------------------------------------------------

# skillcall.json is the schema brain actually puts in response_format, not a
# second rendering of the same models: a back end that constrains a flat oneOf
# and stalls on $ref-with-discriminator would otherwise pass G1 and fail in
# production, with nothing in the gate able to see it.
define GEN_SCHEMAS
import json, pathlib
from rover_brain.box import SKILL_CALL_SCHEMA
from rover_contracts.observations import FindObservation, SceneObservation
out = pathlib.Path("box/schema")
out.mkdir(parents=True, exist_ok=True)
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

kotlin: venv
	$(PY) tools/gen_kotlin.py

kotlin-check: venv
	$(PY) tools/gen_kotlin.py --check

# --- the simulated rover ------------------------------------------------------

define SIM_BOOT
set -uo pipefail
RUN=$(RUN_DIR)
mkdir -p "$$RUN" data/logs data/models/stt
pids=()
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
  echo "stopped."
}
trap cleanup EXIT INT TERM
echo "rover simulation — config $(CONFIG), logs in $$RUN/"
echo
start fakebox rover_devtools.fakebox --host 127.0.0.1 --port $(BOX_PORT)
start rover-stub rover_devtools.rover_stub --tcp $(STUB_PORT) $(STUB_FLAGS)
wait_for "fakebox on :$(BOX_PORT)" 20 "curl -sf http://127.0.0.1:$(BOX_PORT)/v1/models" fakebox
wait_for "rover-stub on :$(STUB_PORT)" 20 "$(PY) -c 'import socket; socket.create_connection((\"127.0.0.1\", $(STUB_PORT)), 0.2).close()'" rover-stub
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
eval "$$SIM_BOOT"
cat <<'EOF'
  All six up. Try:
    face + STOP button   http://127.0.0.1:$(WEB_PORT)/
    teleop pad           http://127.0.0.1:$(WEB_PORT)/teleop
    speak to it          $(PY) -m rover_devtools.roverctl --config $(CONFIG) utter "turn left ninety degrees"
    drive it directly    $(PY) -m rover_devtools.roverctl --config $(CONFIG) drive-for 1.0 0.15
    watch the wire       $(PY) -m rover_devtools.wirecat --tcp 127.0.0.1:$(STUB_PORT)
    resolved config      make doctor
  Inject a fault without restarting anything:
    FAKEBOX_FAULT=out_of_range                   on the fakebox environment
    make sim STUB_FLAGS="--tof-mm 200"           obstacle at 200 mm: forward refused
    make sim STUB_FLAGS="--stock"                unpatched firmware: robotd refuses to move
    make sim LOG_LEVEL=debug                     every command line in robotd.log
  Logs: $(RUN_DIR)/{fakebox,rover-stub,robotd,cam,brain,web}.log
  Ctrl-C stops everything.
EOF
wait
endef
export SIM_SH

sim: venv
	@bash -c "$$SIM_SH"
dev: sim

# --- gates --------------------------------------------------------------------

gates: venv
	$(PY) -m pytest tests/gates -m gate -k "g2 or g4"

gates-sim: gates-full

gates-full: venv
	$(PY) -m pytest tests/gates $(GATES_SIM_ARGS)

G3_SOAK_S ?= 1800
gates-pi: venv
	$(PY) tests/gates/g3/run_g3.py --duration $(G3_SOAK_S) --hardware

gate-g1: venv
	$(PY) -m pytest tests/gates/test_gates.py -m gate -k g1

# --- firmware (Waveshare fork, Arduino) ---------------------------------------

firmware:
	firmware/build.sh

flash:
	@[ -n "$(PORT)" ] || { echo "usage: make flash PORT=/dev/tty.usbserial-XXXX"; exit 2; }
	firmware/flash.sh $(PORT)

# --- box and containers -------------------------------------------------------

docker-dev:
	docker buildx build --platform linux/arm64 -f docker/Dockerfile.services -t rover-services:dev .
	docker compose -f docker/compose.dev.yml run --rm services

box:
	docker compose -f docker/compose.box.yml up -d vllm

# --- hygiene ------------------------------------------------------------------

secrets:
	./hosts/pi/deploy/check-secrets.sh

deps-check:
	@closure=$$(mktemp); \
	 $(UV) export --frozen --no-dev --no-emit-project --no-emit-workspace \
	   --format requirements.txt \
	 | sed -n 's/^\([A-Za-z0-9._-]*\)==.*/\1/p' | tr 'A-Z_' 'a-z-' | sort -u > "$$closure"; \
	 bad=$$(grep -E '^(numpy|scipy|torch|torchaudio|torchvision|tensorflow|onnx|onnxruntime.*|sherpa-onnx|transformers|sentencepiece|tflite-runtime|pyopen-wakeword|opencv-.*|sounddevice)$$' "$$closure" || true); \
	 rm -f "$$closure"; \
	 if [ -n "$$bad" ]; then echo "ML package in the base runtime closure:"; echo "$$bad"; exit 1; fi; \
	 echo "runtime closure clean: no ML package"

DOCTOR_CONFIG ?= $(if $(wildcard config/robot.toml),config/robot.toml,$(CONFIG))
doctor: venv
	$(PY) -m rover_devtools.doctor --config $(DOCTOR_CONFIG)

clean:
	rm -rf .pytest_cache .ruff_cache .mypy_cache .hypothesis firmware/build
	find packages hosts/pi tests -name __pycache__ -type d -prune -exec rm -rf {} +

distclean: clean
	@test "$(VENV)" = .venv || { echo "distclean only removes the default .venv"; exit 2; }
	rm -rf .venv
