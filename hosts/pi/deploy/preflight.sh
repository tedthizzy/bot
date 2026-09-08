#!/usr/bin/env bash
# hosts/pi/deploy/preflight.sh — everything that must be true before `systemctl start
# rover.target`. ARCHITECTURE 11 step 11.
#
#   sudo /opt/rover/hosts/pi/deploy/preflight.sh
#
# It asserts; it never writes. The cliff baseline in particular is derived on
# the MCU and only checked here, which is what keeps the down-link's
# integer-only, narrowing-only story intact (there is no config_set on the wire,
# by design).
#
# Every check prints ok/FAIL/skip and the script exits non-zero if anything
# failed. A skip is a check that could not run, not a check that passed.

set -uo pipefail

ROOT=${ROVER_ROOT:-/opt/rover}
VENV="$ROOT/.venv"
PY="$VENV/bin/python"
CONFIG=${ROVER_CONFIG:-$ROOT/config/robot.toml}
DEV=/dev/serial0

pass=0; fail=0; skipped=0
ok()   { printf '  ok    %s\n' "$*"; pass=$((pass+1)); }
bad()  { printf '  FAIL  %s\n' "$*"; fail=$((fail+1)); }
skip() { printf '  skip  %s\n' "$*"; skipped=$((skipped+1)); }
head_() { printf '\n== %s\n' "$*"; }

[ -x "$PY" ] || { echo "no venv at $VENV; run deploy/install.sh first" >&2; exit 2; }
if ! grep -qi 'raspberry pi' /proc/device-tree/model 2>/dev/null && [ "${ROVER_FORCE:-0}" != "1" ]; then
  echo "preflight checks Pi hardware and is meaningless elsewhere." >&2
  echo "For the simulated rover on a Mac: make sim, then make gates." >&2
  echo "Override with ROVER_FORCE=1." >&2
  exit 2
fi

export ROVER_CONFIG="$CONFIG"
# brain has its own venv (its [speech] extra carries numpy, which must not
# shadow apt's for picamera2); the rest run from $VENV.
BRAIN_PY="$ROOT/.venv-brain/bin/python"

# --- nothing may already be running -----------------------------------------
# rover-cam holds the camera and robotd holds /dev/serial0. With either up,
# the Picamera2 probe below fails as "FrameWallClock absent" and the banner read
# fails as "no B banner in 8 s", neither of which names the real cause.

head_ "units"
running=""
for u in rover-cam rover-brain rover-web; do
  systemctl is-active --quiet "$u" 2>/dev/null && running="$running $u"
done
if [ -n "$running" ]; then
  bad "stop rover.target before running preflight —$running running"
  echo "    rover-cam holds the camera; run: sudo systemctl stop rover.target"
else
  ok "rover-cam, rover-brain and rover-web are not running"
fi
# robotd is allowed to be up: the bus section below needs it, and it is the one
# unit whose device the checks here never open.
systemctl is-active --quiet rover-robotd 2>/dev/null &&
  printf '  note  rover-robotd is running; the banner read is skipped\n'

# --- configuration ----------------------------------------------------------

head_ "configuration"
if "$PY" - "$CONFIG" <<'PY'
import sys
from rover_contracts.config import load_config
c = load_config(sys.argv[1])
print(f"    speed_mps={c.limits.speed_mps} tof_stop_mm={c.safety.tof_stop_mm} "
      f"hfov_deg={c.camera.hfov_deg} port={c.serial.port}")
PY
then ok "$CONFIG loads and every ceiling holds"
else bad "$CONFIG does not validate"; fi

# --- camera -----------------------------------------------------------------

head_ "camera"
if "$PY" -c 'import picamera2' 2>/dev/null; then
  ok "the venv imports apt's picamera2"
else
  bad "the venv cannot import picamera2 — --system-site-packages, or open item 6's fallback"
fi
# FrameWallClock is what every frame stamps for the logs; frame_mono_ns is what
# robotd's freshness gate reads. Its absence is open item 6.
if "$PY" - <<'PY'
from picamera2 import Picamera2
p = Picamera2()
p.configure(p.create_still_configuration())
p.start()
md = p.capture_metadata()
p.stop(); p.close()
assert "FrameWallClock" in md, sorted(md)
PY
then ok 'md["FrameWallClock"] present'
else bad 'md["FrameWallClock"] absent or the camera did not start'; fi

# hfov_deg is what bearing_deg is computed from, and it is a config key rather
# than a literal precisely because it is wrong by 45% if anyone reaches for the
# 120 deg on the lens box. Logged, not asserted: recalibrate at G3a.
"$PY" - "$CONFIG" <<'PY' || skip "could not read ScalerCrop"
import math, sys
from picamera2 import Picamera2
from rover_contracts.config import load_config
cfg = load_config(sys.argv[1])
p = Picamera2()
props = p.camera_properties
p.configure(p.create_still_configuration(main={"size": tuple(cfg.camera.main)}))
p.start(); md = p.capture_metadata(); p.stop(); p.close()
full = props.get("PixelArrayActiveAreas", [(0, 0, 0, 0)])[0]
crop = md.get("ScalerCrop")
print(f"    configured hfov_deg={cfg.camera.hfov_deg}")
print(f"    ScalerCrop={crop} of active area {full}")
if crop and full[2]:
    frac = crop[2] / full[2]
    print(f"    crop is {frac:.3f} of the sensor width -> implied hfov "
          f"{2 * math.degrees(math.atan(frac * math.tan(math.radians(102.0 / 2)))):.1f} deg "
          "(against the Wide lens's 102 deg horizontal)")
PY

# rover-cam is the unit that actually opens the camera, and it runs as
# rover-cam with only SupplementaryGroups=video.  A root probe passes on a card
# whose udev rules leave /dev/dma_heap/linux,cma or a /dev/media* node outside
# group video, and rover-cam then restart-loops every 2 s after the target
# starts, with a libcamera permission traceback preflight could have named.
if getent passwd rover-cam >/dev/null; then
  cam_probe=$(mktemp); chmod 0644 "$cam_probe"
  cat > "$cam_probe" <<'PY'
from picamera2 import Picamera2
p = Picamera2()
p.configure(p.create_still_configuration())
p.start()
p.capture_metadata()
p.stop(); p.close()
print("    rover-cam opened the camera")
PY
  if runuser -u rover-cam -- "$PY" "$cam_probe"; then
    ok "rover-cam itself can open the camera (group video)"
  else
    bad "rover-cam cannot open the camera — check group video and /dev/dma_heap permissions"
  fi
  rm -f "$cam_probe"
else
  skip "no rover-cam user yet"
fi

# --- audio ------------------------------------------------------------------

head_ "audio"
match=$("$PY" -c "import os;from rover_contracts.config import load_config;print(load_config(os.environ['ROVER_CONFIG']).audio.device_match)" 2>/dev/null || echo ReSpeaker)
if "$PY" - "$match" <<'PY'
import sys
import sounddevice as sd
want = sys.argv[1].lower()
hits = [(i, d["name"]) for i, d in enumerate(sd.query_devices())
        if want in d["name"].lower() and d["max_input_channels"] > 0]
if not hits:
    print("    no input device matching", sys.argv[1])
    for i, d in enumerate(sd.query_devices()):
        print(f"    [{i}] {d['name']}  in={d['max_input_channels']} out={d['max_output_channels']}")
    raise SystemExit(1)
for i, name in hits:
    print(f"    resolved [{i}] {name}")
PY
then ok "PortAudio resolves [audio] device_match (an ALSA string would not)"
else bad "no PortAudio input device matches [audio] device_match"; fi

# The same probe as rover-brain, because /dev/snd/* is root:audio 0660: running
# it only as root passes on a box where brain cannot open the device at all,
# and the failure then surfaces at deploy step 14 as an ALSA permission error.
if [ -x "$BRAIN_PY" ] && getent passwd rover-brain >/dev/null; then
  brain_probe=$(mktemp); chmod 0644 "$brain_probe"
  cat > "$brain_probe" <<'PY'
import sys
import sounddevice as sd
want = sys.argv[1].lower()
hits = [d["name"] for d in sd.query_devices()
        if want in d["name"].lower() and d["max_input_channels"] > 0]
if not hits:
    print("    rover-brain sees no input device matching", sys.argv[1])
    raise SystemExit(1)
print("    rover-brain resolves", hits[0])
PY
  if runuser -u rover-brain -- "$BRAIN_PY" "$brain_probe" "$match"; then
    ok "rover-brain itself can enumerate the capture device (group audio)"
  else
    bad "rover-brain cannot open the audio device — usermod -a -G audio rover-brain"
  fi
  rm -f "$brain_probe"
else
  skip "no .venv-brain or no rover-brain user yet"
fi

if command -v arecord >/dev/null; then
  if arecord -l 2>/dev/null | grep -qi respeaker; then ok "arecord -l finds the ReSpeaker Lite"
  else bad "arecord -l does not list a ReSpeaker"; fi
else skip "arecord not installed"; fi

# Playback goes to ALSA's default without -D, which on a Pi 4 is card 0:
# vc4-hdmi or the headphone jack, not the ReSpeaker Lite's amp.
out=$("$PY" -c "import os;from rover_contracts.config import load_config;print(load_config(os.environ['ROVER_CONFIG']).audio.output_device)" 2>/dev/null || echo "")
tts_backend=$("$PY" -c "import os;from rover_contracts.config import load_config;print(load_config(os.environ['ROVER_CONFIG']).tts.backend)" 2>/dev/null || echo "")
if [ -z "$out" ] && [ "$tts_backend" = "piper" ]; then
  # Card 0 on a headless Pi 4 is vc4-hdmi or the headphone jack, not the
  # ReSpeaker Lite's amp, so every turn runs correctly and is inaudible with
  # nothing in the journal naming the cause. That is a FAIL, not a skip.
  bad "[audio] output_device is empty and [tts] backend is piper — playback would go to ALSA card 0, not the ReSpeaker Lite"
  echo "    pick a card below and add it to [audio] in $CONFIG, e.g."
  echo "        output_device = \"plughw:CARD=Lite,DEV=0\""
  aplay -l 2>/dev/null | sed 's/^/    /' || true
elif [ -z "$out" ]; then
  skip "[audio] output_device empty — playback goes to ALSA's default (card 0), and [tts] backend is $tts_backend"
  aplay -l 2>/dev/null | sed 's/^/    /' || true
elif aplay -L 2>/dev/null | grep -qxF "$out" || aplay -l 2>/dev/null | grep -qi "${out#plughw:CARD=}"; then
  ok "[audio] output_device $out is a device aplay knows"
else
  bad "[audio] output_device $out is not in aplay -L / aplay -l"
fi

# The recogniser the input mode needs.  [audio] input="ptt"|"wake" with [stt]
# backend="text" captures audio and throws every frame away: the FSM sits in
# TRANSCRIBING until its 8 s timeout and reports a timeout, not "no STT model".
# The four filenames are the ones SherpaStt opens (packages/rover_brain/audio/
# stt.py); the model itself is the G3b bake-off's, not something install.sh
# picks for you.
read -r audio_input stt_backend stt_dir <<EOF
$("$PY" -c "import os;from rover_contracts.config import load_config;c=load_config(os.environ['ROVER_CONFIG']);print(c.audio.input, c.stt.backend, c.stt.model_dir)" 2>/dev/null || echo "text text -")
EOF
if [ "$audio_input" = "text" ]; then
  skip "[audio] input=text — no recogniser is needed (A30: text is the default until G3b)"
elif [ "$stt_backend" = "text" ] || [ "$stt_backend" = "mock" ]; then
  bad "[audio] input=$audio_input with [stt] backend=$stt_backend — audio is captured and discarded, and the turn ends in a TRANSCRIBING timeout"
  echo "    set [stt] backend=\"sherpa\" and put a streaming Zipformer int8 in $stt_dir,"
  echo "    or leave [audio] input=\"text\" until the G3b bake-off picks the model"
elif [ "$stt_backend" = "sherpa" ]; then
  missing=""
  for f in tokens.txt encoder.onnx decoder.onnx joiner.onnx; do
    [ -s "$stt_dir/$f" ] || missing="$missing $f"
  done
  if [ -z "$missing" ]; then
    ok "[stt] sherpa model complete in $stt_dir"
  else
    bad "[stt] backend=sherpa but $stt_dir is missing:$missing"
    echo "    SherpaStt opens exactly tokens.txt, encoder.onnx, decoder.onnx and joiner.onnx"
  fi
else
  skip "[stt] backend=$stt_backend — nothing local to check"
fi

# The Piper voice: a bare name resolves against the process cwd, where nothing
# is, and piper then exits with a message nobody reads.
voice=$("$PY" -c "import os;from rover_contracts.config import load_config;c=load_config(os.environ['ROVER_CONFIG']);print(c.tts.voice if c.tts.backend=='piper' else '')" 2>/dev/null || echo "")
if [ -z "$voice" ]; then
  skip "[tts] backend is not piper"
elif [ -s "$voice" ] && [ -s "$voice.json" ]; then
  ok "[tts] voice $voice and its .json are present and non-empty"
else
  bad "[tts] voice $voice (or $voice.json) missing or empty — install.sh downloads it"
fi

# --- host health ------------------------------------------------------------

head_ "host"
if command -v vcgencmd >/dev/null; then
  t=$(vcgencmd get_throttled)
  # 0x0 with no bit exclusions: on a Pi 4, 0x1/0x10000, 0x2/0x20000 and
  # 0x4/0x40000 all apply, and 0x2/0x20000 is the bit the Pi 5 trigger fires on.
  [ "$t" = "throttled=0x0" ] && ok "$t" || bad "$t — fix the rail or the cooling before anything else"
  printf '    %s / %s\n' "$(vcgencmd measure_temp)" "$(vcgencmd measure_clock arm)"
else skip "vcgencmd not present"; fi

[ -c /dev/watchdog ] && ok "/dev/watchdog present" || bad "/dev/watchdog absent — dtparam=watchdog=on, then reboot"
w=$(systemctl show -p RuntimeWatchdogUSec --value 2>/dev/null || echo "")
[ "$w" = "14s" ] && ok "RuntimeWatchdogSec=14" || bad "RuntimeWatchdogUSec=$w, expected 14s"

# --- the controller port ----------------------------------------------------

head_ "controller port"
if [ -e "$DEV" ]; then
  ok "$DEV exists"
  # I-18: exactly one writer. Before the target is started that is zero, which
  # is also correct; more than one is never correct.
  if command -v fuser >/dev/null; then
    holders=$(fuser "$DEV" 2>/dev/null | wc -w | tr -d ' ')
    case "$holders" in
      0) ok "no process holds $DEV yet (the target is not started)";;
      1) ok "exactly one pid holds $DEV";;
      *) bad "$holders processes hold $DEV — only robotd may (I-18)";;
    esac
  else skip "fuser not installed (apt install psmisc)"; fi
else
  bad "$DEV missing"
  udevadm info -q all -n "$DEV" 2>&1 | sed 's/^/    /' || true
  echo "    check enable_uart=1 and dtoverlay=disable-bt in /boot/firmware/config.txt, then the rule:"
  ls -l /sys/class/tty/ttyAMA*/device 2>/dev/null | sed 's/^/    /' || true
fi

# --- the controller -------------------------------------------------------------
# The firmware fork announces itself with a JSON banner and adds five fields to
# its feedback. Stock Waveshare firmware does neither, and robotd will refuse to
# move against it. This block reads one banner and one feedback line and
# asserts the compiled heartbeat and cap against [safety] and [limits].

head_ "controller banner and safety cross-check"
if systemctl is-active --quiet rover-robotd 2>/dev/null; then
  skip "rover-robotd holds $DEV — a second reader splits the byte stream (I-18)"
elif [ -e "$DEV" ]; then
  if "$PY" - "$DEV" "$CONFIG" <<'PY'
import sys, time
import serial
from rover_contracts import (
    CEILINGS, Banner, Feedback, banner_request, decode_line, echo, feedback_flow,
    feedback_interval, load_config, quiet,
)

dev, cfg_path = sys.argv[1], sys.argv[2]
cfg = load_config(cfg_path)
banner = feedback = None
deadline = time.monotonic() + 8.0
with serial.Serial(dev, cfg.link.baud, timeout=0.2) as port:
    port.write(quiet() + echo(False) + feedback_interval(cfg.link.feedback_interval_ms)
               + feedback_flow(True) + banner_request())
    buf = b""
    while time.monotonic() < deadline and not (banner and feedback):
        buf += port.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            got = decode_line(line)
            if isinstance(got, Banner):
                banner = got
            elif isinstance(got, Feedback) and feedback is None:
                feedback = got

bad = 0
if banner is None:
    print("    no banner in 8 s — is the board flashed with the fork in firmware/?")
    print("    (stock Waveshare firmware never answers T:1007; robotd refuses to move against it)")
    raise SystemExit(1)
print(f"    fw={banner.fw} hb_ms={banner.hb_ms} cap={banner.cap} proto={banner.proto}")
if banner.hb_ms != cfg.safety.heartbeat_ms:
    bad = 1
    print(f"    heartbeat MISMATCH: firmware {banner.hb_ms} ms, [safety] heartbeat_ms {cfg.safety.heartbeat_ms}")
    print("    The firmware's value is compiled in (BOT_HEARTBEAT_MS); edit firmware/, not the TOML.")
if abs(banner.cap - CEILINGS["limits.power_max"]) > 1e-6:
    bad = 1
    print(f"    cap MISMATCH: firmware {banner.cap}, ceiling {CEILINGS['limits.power_max']}")
if feedback is None:
    bad = 1
    print("    no feedback line — the fork streams T:1001 once T:131 is sent")
elif not feedback.patched:
    bad = 1
    print("    feedback lacks the fork's fields (hb st tf bp cc) — stock firmware")
else:
    print(f"    feedback ok: hb={feedback.hb} st={int(feedback.st)} tf={feedback.tof_mm} "
          f"bp={feedback.bumper} v={feedback.bus_v:.2f} y={feedback.yaw_deg:.1f}")
    if feedback.left or feedback.right:
        bad = 1
        print("    motors are being driven with robotd stopped — nothing else may write this port")
    if feedback.bus_v < cfg.safety.low_battery_v:
        print(f"    pack {feedback.bus_v:.2f} V is below low_battery_v; charge before G2")
raise SystemExit(bad)
PY
  then ok "banner and feedback agree with [safety] and [limits]"
  else bad "controller cross-check failed (details above)"; fi
else
  skip "$DEV absent; nothing to cross-check"
fi

# --- the bus ----------------------------------------------------------------

head_ "bus"
SOCK=$("$PY" -c "import os;from rover_contracts.config import load_config;print(load_config(os.environ['ROVER_CONFIG']).bus.sock)")
if systemctl is-active --quiet rover-robotd; then
  # 0660 group rover is not decoration: it is how "only group rover can command
  # motion" stops being a self-declared field and becomes a kernel check (A10).
  perms=$(stat -c '%a %G' "$SOCK" 2>/dev/null || echo "missing")
  [ "$perms" = "660 rover" ] && ok "$SOCK is 0660 group rover" || bad "$SOCK is '$perms', expected '660 rover'"

  # As rover-web, because [bus] source_uids binds the declared source to the
  # peer's uid at hello and root is not one of them.
  probe=$(mktemp); chmod 0644 "$probe"
  cat > "$probe" <<'PY'
import json, os, socket, sys
from rover_contracts.config import load_config
from rover_contracts.messages import server_adapter
cfg = load_config(os.environ["ROVER_CONFIG"])
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(2.0)
s.connect(cfg.bus.sock)
s.sendall(json.dumps({"v": 1, "type": "hello", "source": "web",
                      "pid": os.getpid(), "caps": ["subscribe"]}).encode() + b"\n")
line = s.makefile("rb").readline()
msg = server_adapter.validate_json(line)
if msg.type != "welcome":
    print("    robotd answered", msg.type, "not welcome"); sys.exit(1)
# A33: a gate reads the running limits from welcome, never from the file.
print(f"    welcome session={msg.session} mcu_session={msg.mcu_session} "
      f"speed_mps={msg.limits.speed_mps} budget_path_m={msg.limits.budget_path_m} "
      f"tof_stop_mm={msg.safety.tof_stop_mm}")
PY
  if runuser -u rover-web -- env ROVER_CONFIG="$CONFIG" "$PY" "$probe"; then
    ok "robotd hello round-trip; welcome carries the running limits"
  else
    bad "robotd is running but did not answer a hello with a welcome"
  fi
  rm -f "$probe"
else
  skip "rover-robotd not started yet — that is the expected state here"
  bus_skipped=1
fi

# --- verdict ----------------------------------------------------------------

printf '\n%s\n' "preflight: $pass ok, $fail failed, $skipped skipped"
if [ "$fail" -gt 0 ]; then
  echo "Do not start rover.target until these are green."
  exit 1
fi
echo "Ready:  sudo systemctl start rover.target"
if [ "${bus_skipped:-0}" = "1" ]; then
  # The two halves are mutually exclusive by design -- the banner read needs
  # robotd stopped, the bus checks need it running -- and the documented path
  # runs preflight once, before the target. So the socket's 0660 rover mode and
  # the hello/welcome round-trip never execute unless this says to run it again.
  echo "Then:   sudo deploy/preflight.sh   # re-run for the bus checks skipped above"
fi
