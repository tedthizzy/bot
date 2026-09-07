#!/usr/bin/env bash
# deploy/install.sh — bring a clean Raspberry Pi OS Trixie 64-bit Lite card to
# the point where deploy/preflight.sh can run. ARCHITECTURE 11 step 9.
#
#   sudo /opt/rover/deploy/install.sh
#
# Idempotent: run it as often as you like. It creates nothing twice, never
# overwrites config/robot.toml, and only ever `systemctl enable`s — the target
# is started by hand after preflight passes, so robotd does not restart-loop
# against an MCU that is not yet wired.
#
# It refuses to run on anything that is not a Raspberry Pi. Override with
# ROVER_FORCE=1 if you know what you are doing; the apt packages, the udev rule
# keyed on a device-tree node and the /boot/firmware/config.txt edits are all
# Pi-specific and will do nothing useful elsewhere.
#
# No Docker on the Pi (A36): bridge networking costs 10.1 ms against host's
# 2.5 ms and 1-10 ms per peripheral I/O on a Pi 4. Docker lives on the Mac, for
# arm64 parity and the box compose.

set -euo pipefail

ROOT=${ROVER_ROOT:-/opt/rover}
VENV="$ROOT/.venv"
BRAIN_VENV="$ROOT/.venv-brain"
TTS_VENV="$ROOT/.venv-tts"
DATA=${ROVER_DATA:-/data}
CONFIG_TXT=${ROVER_CONFIG_TXT:-/boot/firmware/config.txt}
UNITS=(rover-robotd rover-cam rover-brain rover-web)
SERVICE_USERS=(rover-robotd rover-cam rover-brain rover-web)

say()  { printf '\n== %s\n' "$*"; }
info() { printf '   %s\n' "$*"; }
die()  { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

# --- 0. refuse to run in the wrong place ------------------------------------

[ "$(id -u)" -eq 0 ] || die "run me with sudo"

is_pi() {
  [ -e /proc/device-tree/model ] &&
    grep -qi 'raspberry pi' /proc/device-tree/model 2>/dev/null
}
if ! is_pi; then
  if [ "${ROVER_FORCE:-0}" != "1" ]; then
    die "this is not a Raspberry Pi.
  install.sh installs apt's camera stack, a udev rule keyed on the Pi's UART5
  device-tree node, and Pi boot overlays. None of that means anything here.
  For a simulated rover on a Mac, run 'make sim' instead.
  To override anyway: ROVER_FORCE=1 sudo $0"
  fi
  info "ROVER_FORCE=1: continuing on a non-Pi. You are on your own."
fi

[ -d "$ROOT" ] || die "$ROOT does not exist; clone the repo there first"
[ -f "$ROOT/pyproject.toml" ] || die "$ROOT does not look like the rover repo"

model=$(tr -d '\0' </proc/device-tree/model 2>/dev/null || echo unknown)
info "host: $model"
info "root: $ROOT"

# --- 1. system packages -----------------------------------------------------

say "apt packages"
packages=$(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$ROOT/deploy/apt-packages.txt" | tr -s '[:space:]' ' ')
info "$packages"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
# shellcheck disable=SC2086
apt-get install -y --no-install-recommends $packages

# Hold the camera stack together. An upgrade that moves libcamera under a held
# picamera2 is the commonest way this breaks.
#
# One package at a time: libcameraN.M is version-suffixed and its name changes
# with every libcamera bump, so a single apt-mark call naming all three failed
# entirely on a card that shipped a different one -- leaving python3-picamera2
# and python3-libcamera unheld while the note read like a partial success. The
# suffixed name is derived from the installed module rather than hard-coded.
libcamera_pkg=$(dpkg -S "$(python3 -c 'import libcamera, os; print(os.path.realpath(libcamera.__file__))' 2>/dev/null)" 2>/dev/null | cut -d: -f1 | head -1)
[ -n "$libcamera_pkg" ] || info "could not resolve the libcameraN.M package name; hold it by hand after 'dpkg -l | grep libcamera'"
for p in python3-picamera2 python3-libcamera ${libcamera_pkg:-}; do
  if apt-mark hold "$p" >/dev/null 2>&1; then
    info "held $p"
  else
    info "not present, not held: $p"
  fi
done

# --- 2. users, groups and directories ---------------------------------------

say "users and directories"
getent group rover >/dev/null || { groupadd --system rover; info "created group rover"; }
for u in "${SERVICE_USERS[@]}"; do
  if ! getent passwd "$u" >/dev/null; then
    useradd --system --gid rover --no-create-home --shell /usr/sbin/nologin "$u"
    info "created user $u"
  fi
done
# The camera needs the video group; brain needs audio. /dev/snd/* is
# root:audio 0660 on Debian, so without this brain can open neither the
# ReSpeaker for capture nor aplay for playback -- and preflight's own probe
# runs as root and would not notice.
usermod -a -G video rover-cam
usermod -a -G audio rover-brain

# The login user drives the robot from a shell, so it needs the group too.
login_user=${SUDO_USER:-}
if [ -n "$login_user" ] && ! id -nG "$login_user" | tr ' ' '\n' | grep -qx rover; then
  usermod -a -G rover "$login_user"
  info "added $login_user to group rover — log out and back in for it to take effect"
fi

install -d -o rover-robotd -g rover -m 2775 "$DATA/logs"
install -d -o root -g rover -m 2755 "$DATA/models" "$DATA/models/stt" \
        "$DATA/models/tts" "$DATA/models/wake"

# /run/rover is shared by four units with four users, so a robotd restart must
# not delete brain's and cam's sockets. tmpfiles owns it, not RuntimeDirectory=.
# 0770 rover: the 0660 sockets inside it are how "only group rover can command
# motion" becomes a filesystem permission (A10).
cat > /etc/tmpfiles.d/rover.conf <<'EOF'
# /run/rover holds robotd.sock, frames.sock, brain.sock and the persisted
# estop_sw flag. Group rover, no world access.
d /run/rover 0770 rover-robotd rover -
EOF
systemd-tmpfiles --create /etc/tmpfiles.d/rover.conf

# --- 3. configuration -------------------------------------------------------

say "configuration"
if [ ! -f "$ROOT/config/robot.toml" ]; then
  install -o root -g rover -m 0640 "$ROOT/config/robot.example.toml" "$ROOT/config/robot.toml"
  info "created config/robot.toml from the example — edit it, or override with ROVER__SECTION__KEY"
else
  info "config/robot.toml exists; left alone"
fi

# Credentials live here and nowhere else. Names, never values.
install -d -m 0750 -o root -g rover /etc/rover
if [ ! -f /etc/rover/box.env ]; then
  # Both assignments stay commented. An empty ROVER__BOX__URL= is not "unset":
  # the override scheme applies any ROVER__* variable it finds, so the empty
  # string would override [box] url and brain would build a client with no
  # scheme and fail with an httpx protocol error naming neither key nor file.
  cat > /etc/rover/box.env <<'EOF'
# Read by rover-brain.service. Values only; this file is not in git.
# Uncomment and fill in; an empty value here OVERRIDES config/robot.toml.
#ROVER__BOX__URL=http://<your-box>:8000/v1
#ROVER_BOX_API_KEY=
EOF
  chown root:rover /etc/rover/box.env
  chmod 0640 /etc/rover/box.env
  info "created /etc/rover/box.env — put the box address and token there"
fi

# --- 4. uv and the runtime venv ---------------------------------------------

say "uv and the virtual environment"
if ! command -v /usr/local/bin/uv >/dev/null; then
  curl -LsSf https://astral.sh/uv/install.sh | UV_INSTALL_DIR=/usr/local/bin sh
fi
UV=/usr/local/bin/uv
info "$("$UV" --version)"

# --system-site-packages so rover-cam sees apt's python3-libcamera and
# python3-picamera2 and can still import rover_contracts. --python /usr/bin/python3
# so it is apt's interpreter those packages were built for, never a uv download.
[ -x "$VENV/bin/python" ] || "$UV" venv --python /usr/bin/python3 --system-site-packages "$VENV"

# The base set only, into the --system-site-packages venv rover-cam runs from.
# A venv's own site-packages precedes the system dist-packages, so a PyPI numpy
# installed here shadows apt's python3-numpy -- which apt's python3-picamera2
# and python3-kms++ were built against, and which pyproject.toml and
# ARCHITECTURE 12 both say must never be shadowed. The [speech] extra carries
# numpy, so it does not belong in this environment.
VIRTUAL_ENV="$VENV" "$UV" pip install --python "$VENV/bin/python" -e "$ROOT"

# brain's own environment, with no system site-packages: it is the only unit
# that needs [speech], and it never touches the camera stack.
[ -x "$BRAIN_VENV/bin/python" ] || "$UV" venv --python /usr/bin/python3 "$BRAIN_VENV"
VIRTUAL_ENV="$BRAIN_VENV" "$UV" pip install --python "$BRAIN_VENV/bin/python" \
  -e "$ROOT[speech]"

# piper-tts is GPL-3.0-or-later and this repo is Apache-2.0. It goes in its own
# venv, is invoked as a subprocess, and is never imported and never a dependency
# of the published package (A28). [tts] bin points at this absolute path.
[ -x "$TTS_VENV/bin/python" ] || "$UV" venv --python /usr/bin/python3 "$TTS_VENV"
VIRTUAL_ENV="$TTS_VENV" "$UV" pip install --python "$TTS_VENV/bin/python" 'piper-tts>=1.8.0'

chown -R root:rover "$VENV" "$BRAIN_VENV" "$TTS_VENV"

# --- 5. models --------------------------------------------------------------

say "models"
# The Piper voice. lessac-low @1 thread is A26's consequence; its first-audio
# latency on a short reply is open item 4 and is measured at G3b.
# [tts] voice is now an absolute path to the .onnx, so the download target is
# read straight off it rather than reconstructed.
voice_path=$(sed -n 's/^[[:space:]]*voice[[:space:]]*=[[:space:]]*"\([^"]*\)".*/\1/p' "$ROOT/config/robot.toml" | head -1)
voice_path=${voice_path:-$DATA/models/tts/en_US-lessac-low.onnx}
voice_dir=$(dirname "$voice_path")
voice=$(basename "$voice_path" .onnx)
install -d -o root -g rover -m 2755 "$voice_dir"
# en_US-lessac-low -> en/en_US/lessac/low/en_US-lessac-low.{onnx,onnx.json}
locale=${voice%%-*}; rest=${voice#*-}
base="https://huggingface.co/rhasspy/piper-voices/resolve/main/${locale%%_*}/$locale/${rest%-*}/${rest##*-}/$voice"
for ext in onnx onnx.json; do
  if [ ! -f "$voice_dir/$voice.$ext" ]; then
    info "fetching $voice.$ext"
    curl -fsSL -o "$voice_dir/$voice.$ext" "$base.$ext" || \
      info "could not fetch $voice.$ext — download it into $voice_dir before G3b"
  fi
done
# The STT models are chosen by the G3b bake-off, not here. [stt] backend is
# "text" until then, which is what makes the whole Pi test plan runnable with
# no model at all.
# The two assets nothing downloads, both named by absolute path in [wake] and
# [vad] so a missing one is a clear error rather than a lookup in the cwd.
info "STT models: none yet by design; the G3b bake-off picks them (make gate)"
[ -f "$DATA/models/stt/silero_vad.onnx" ] || \
  info "MISSING $DATA/models/stt/silero_vad.onnx — [vad] needs it before G3b"
[ -f "$DATA/models/wake/rover.tflite" ] || \
  info "MISSING $DATA/models/wake/rover.tflite — no wake-word model ships:
     openWakeWord's pre-trained models are CC BY-NC-SA, so train your own and
     put it there before setting [audio] input=\"wake\""
chown -R root:rover "$DATA/models"

# --- 6. udev, systemd, logrotate --------------------------------------------

say "device rule and units"
install -m 0644 "$ROOT/deploy/udev/99-rover.rules" /etc/udev/rules.d/99-rover.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty

for u in "${UNITS[@]}"; do
  install -m 0644 "$ROOT/deploy/systemd/$u.service" "/etc/systemd/system/$u.service"
done
install -m 0644 "$ROOT/deploy/systemd/rover.target" /etc/systemd/system/rover.target

# The CLI is `roverctl` everywhere (ARCHITECTURE 12) and nothing else puts it on
# PATH, so README step 13's bare `roverctl utter "..."` was `command not found`
# on a fresh Pi.
ln -sfn "$VENV/bin/roverctl" /usr/local/bin/roverctl
info "roverctl -> $VENV/bin/roverctl"

# A wedged kernel is not something T1 or the Task-WDT can catch. 14 s against
# bcm2835_wdt, whose cap is 15 s.
install -d /etc/systemd/system.conf.d
cat > /etc/systemd/system.conf.d/rover-watchdog.conf <<'EOF'
[Manager]
RuntimeWatchdogSec=14
EOF

retain=$(sed -n 's/^[[:space:]]*retain_days[[:space:]]*=[[:space:]]*\([0-9]*\).*/\1/p' "$ROOT/config/robot.toml" | head -1)
retain=${retain:-7}
cat > /etc/logrotate.d/rover <<EOF
$DATA/logs/*.jsonl $DATA/logs/*.log {
    daily
    rotate $retain
    missingok
    notifempty
    compress
    delaycompress
    copytruncate
    su rover-robotd rover
}
EOF
info "logrotate: $retain days, from [log] retain_days"

systemctl daemon-reload
# daemon-reload does not re-read system.conf.d; RuntimeWatchdogSec needs a
# re-exec of pid 1, which is safe and does not restart services.
systemctl daemon-reexec
# The four units only. rover.target carries WantedBy=multi-user.target, so
# enabling it here would start all four on the next boot -- and this script
# edits config.txt and then tells you to reboot. They would be running before
# preflight, which is exactly what steps 8-10 of this file promise cannot
# happen: rover-cam would hold the camera and robotd would hold /dev/rover-mcu,
# and both preflight probes would fail with a message naming the wrong cause.
systemctl enable "${UNITS[@]/%/.service}" >/dev/null
info "units enabled, NOT started, and rover.target NOT enabled — preflight first"

# --- 7. Wi-Fi power save ----------------------------------------------------

say "Wi-Fi power save"
if command -v nmcli >/dev/null; then
  con=$(nmcli -t -f NAME,TYPE connection show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')
  if [ -n "$con" ]; then
    nmcli connection modify "$con" 802-11-wireless.powersave 2
    info "powersave 2 (disabled) on '$con'"
  else
    info "no active Wi-Fi connection; set 802-11-wireless.powersave 2 when there is one"
  fi
else
  info "nmcli not present; skipped"
fi

# --- 8. boot configuration --------------------------------------------------
# A4's serial change and the fan overlay. gpio-poweroff is deliberately NOT
# here: it goes in at deploy step 17, only after G2-f, on battery, with the
# MCU's rail control wired. Installed now it would brick every reboot on the
# bench, because nothing pulls PI_RAIL_EN low outside UNDERVOLT_D.

say "boot configuration ($CONFIG_TXT)"
changed=0
add_line() {
  if [ ! -f "$CONFIG_TXT" ]; then info "no $CONFIG_TXT; skipping '$1'"; return; fi
  if grep -qxF "$1" "$CONFIG_TXT"; then
    info "have  $1"
  else
    printf '%s\n' "$1" >> "$CONFIG_TXT"
    info "added $1"
    changed=1
  fi
}
# [all] first: a stock Trixie config.txt happens to end in [all], but a card
# whose file ends inside a [pi5]/[cm4]/board-specific section would otherwise
# scope every line below to that filter, and the failure looks like the
# overlays not existing.
if [ -f "$CONFIG_TXT" ] && ! grep -q '^# --- rover ---' "$CONFIG_TXT"; then
  printf '\n# --- rover --- (deploy/install.sh; gpio-poweroff is step 17, not here)\n[all]\n' >> "$CONFIG_TXT"
fi
add_line 'dtoverlay=uart5'
add_line 'dtoverlay=gpio-fan,gpiopin=18,temp=60000'
add_line 'dtoverlay=gpio-shutdown,gpio_pin=17,active_low=0,gpio_pull=down'
add_line 'camera_auto_detect=1'
add_line 'dtparam=watchdog=on'
# A rev 1.4 board only. G3a's clock criterion has an 1800 MHz branch that is
# otherwise unreachable, and it raises the thermal load the fan sizing assumes.
# The Model line carries "Rev 1.4" on most images; the revision decode is the
# fallback for one that does not. A Pi 4B rev 1.4 revision code ends "3114".
if grep -qi 'rev.*1\.4' /proc/cpuinfo 2>/dev/null || \
   [ "$(sed -n 's/^Revision.*: //p' /proc/cpuinfo 2>/dev/null | tr -d '[:space:]' | tail -c 4)" = "3114" ]; then
  add_line 'arm_boost=1'
else
  info "not a rev 1.4 board; arm_boost left alone"
fi

# --- 9. self-check ----------------------------------------------------------

say "self-check"
fail=0
check() { if eval "$2" >/dev/null 2>&1; then info "ok   $1"; else info "FAIL $1"; fail=1; fi; }

check "venv imports rover_contracts" "$VENV/bin/python -c 'import rover_contracts'"
check "venv imports pydantic"        "$VENV/bin/python -c 'import pydantic'"
check "config validates"             "$VENV/bin/python -c \"from rover_contracts.config import load_config; load_config('$ROOT/config/robot.toml')\""
check "piper binary present"         "test -x $TTS_VENV/bin/piper"
check "brain venv imports rover_brain" "$BRAIN_VENV/bin/python -c 'import rover_brain'"
check "piper voice present"          "test -s $voice_dir/$voice.onnx"
# The four units invoke these by absolute path; a missing one is a unit that
# will restart-loop rather than fail visibly.
for s in rover-robotd rover-cam rover-web roverctl; do
  check "$s entry point"             "test -x $VENV/bin/$s"
done
check "rover-brain entry point"      "test -x $BRAIN_VENV/bin/rover-brain"
check "rover-brain in group audio"   "id -nG rover-brain | tr ' ' '\n' | grep -qx audio"
check "group rover exists"           "getent group rover"
check "/run/rover exists"            "test -d /run/rover"
check "$DATA/logs writable by robotd" "runuser -u rover-robotd -- test -w $DATA/logs"
check "udev rule installed"          "test -f /etc/udev/rules.d/99-rover.rules"
for u in "${UNITS[@]}"; do
  check "$u enabled" "systemctl is-enabled $u.service"
done
check "RuntimeWatchdogSec=14" "systemctl show -p RuntimeWatchdogUSec | grep -q '14s'"
# picamera2 lives in apt's site-packages; this is the one import the venv's
# --system-site-packages flag exists for, and open item 6's early warning.
if "$VENV/bin/python" -c 'import picamera2' >/dev/null 2>&1; then
  info "ok   venv sees apt's picamera2"
else
  info "WARN venv cannot import picamera2 — check --system-site-packages, then"
  info "     the fallback in open item 6: run rover-cam on /usr/bin/python3 with"
  info "     PYTHONPATH=$ROOT/packages"
fi

say "done"
if [ "$changed" = 1 ]; then
  echo "  $CONFIG_TXT changed — REBOOT before going further."
fi
cat <<EOF
  Next, in order:
    1. wire UART5, then read the banner:
         $VENV/bin/python -m rover_devtools.wirecat /dev/rover-mcu
    2. sudo $ROOT/deploy/preflight.sh
    3. sudo systemctl enable --now rover.target
       (enable is here and not above on purpose: nothing starts before
        preflight has passed once)
EOF
[ "$fail" = 0 ] || die "self-check found problems above"
