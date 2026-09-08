#!/usr/bin/env bash
# hosts/pi/deploy/sync.sh — the edit loop. Push the working tree to the rover and
# restart the two units that hold no hardware state.
#
#   hosts/pi/deploy/sync.sh rover.local
#   ROVER_HOST=rover.local hosts/pi/deploy/sync.sh --dry-run
#
# This is the edit loop and NOT `reboot`: rsync plus a restart of brain and
# web, the two units that hold no hardware state.
#
# robotd and cam are NOT restarted by default. robotd owns the serial link and
# streams the zeros the controller's heartbeat watches, and restarting it
# re-runs the firmware bring-up; cam restarts cost a libcamera reinitialise.
# Pass --all when you have actually changed them.
#
# No host is baked in: this repo is public and a hostname on someone's LAN is
# not something to publish. The argument or ROVER_HOST is the only source.

set -euo pipefail

HOST=
REMOTE_ROOT=${ROVER_ROOT:-/opt/rover}
UNITS=(rover-brain rover-web)
RSYNC_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --all) UNITS=(rover-robotd rover-cam rover-brain rover-web) ;;
    --dry-run) RSYNC_ARGS+=(--dry-run --itemize-changes) ;;
    -*) echo "unknown option: $arg" >&2; exit 2 ;;
    *) [ -z "$HOST" ] || { echo "only one host may be specified" >&2; exit 2; }
       HOST=$arg ;;
  esac
done
HOST=${HOST:-${ROVER_HOST:-}}
[ -n "$HOST" ] || {
  echo "usage: hosts/pi/deploy/sync.sh <user@host> [--all] [--dry-run] (or set ROVER_HOST)" >&2
  exit 2
}

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
[ -f "$here/pyproject.toml" ] && [ -d "$here/packages/rover_contracts" ] || {
  echo "not a rover checkout: $here" >&2; exit 2;
}
# This path is passed through SSH and is also a destructive rsync destination.
[[ "$REMOTE_ROOT" =~ ^/[a-zA-Z0-9_./-]+$ &&
   "$REMOTE_ROOT" != / && "$REMOTE_ROOT" != *//* &&
   "$REMOTE_ROOT" != */./* && "$REMOTE_ROOT" != */. &&
   "$REMOTE_ROOT" != */../* && "$REMOTE_ROOT" != */.. ]] || {
  echo "ROVER_ROOT must be an absolute deployment directory without shell characters, duplicate slashes, '.' or '..' components" >&2
  exit 2
}

# --delete, because a file that stopped existing here must stop existing there:
# a stale module left behind is how a fix appears to have no effect. The
# excludes are the things that belong to the robot and not to this checkout --
# above all config/robot.toml, which holds local settings and calibration.
# --rsync-path="sudo rsync": /opt is root:root 0755, so the tree there is
# root-owned however it was cloned, and a login-user rsync cannot write into it.
# --no-owner --no-group --chown=root:rover: the receiving rsync runs as root, so
# -a's -o -g would stamp this Mac's numeric uid/gid onto the Pi -- a macOS
# checkout is typically 501:20, and gid 20 on Debian is `dialout`, which leaves
# /opt/rover owned by a uid that does not exist and unwritable by the login user.
rsync -az --delete --info=stats1 "${RSYNC_ARGS[@]}" \
  --rsync-path="sudo rsync" \
  --no-owner --no-group --chown=root:rover \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude '.venv-tts/' \
  --exclude '.venv-brain/' \
  --exclude '.cache/' \
  --exclude '.env' \
  --exclude '.env.*' \
  --exclude '*.local.md' \
  --exclude 'rasberrypi.md' \
  --exclude 'raspberrypi.md' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.pytest_cache/' \
  --exclude '.mypy_cache/' \
  --exclude '.ruff_cache/' \
  --exclude '.hypothesis/' \
  --exclude 'run/' \
  --exclude 'data/' \
  --exclude 'logs/' \
  --exclude 'build/' \
  --exclude 'config/robot.toml' \
  "$here/" "$HOST:$REMOTE_ROOT/"

# A preview must not reinstall dependencies or restart any service.
[ "${#RSYNC_ARGS[@]}" -eq 0 ] || exit 0

# The venv is editable, so a pure-Python change needs no reinstall. A change to
# the dependency set does, and that is what this line catches cheaply.
ssh "$HOST" "sudo -n /usr/local/bin/uv pip install --python $REMOTE_ROOT/.venv/bin/python \
  -q -e '$REMOTE_ROOT'"
ssh "$HOST" "sudo -n /usr/local/bin/uv pip install \
  --python $REMOTE_ROOT/.venv-brain/bin/python -q -e '$REMOTE_ROOT[speech]'"

ssh "$HOST" "sudo -n systemctl restart ${UNITS[*]}"
ssh "$HOST" "systemctl --no-pager --lines=0 status ${UNITS[*]}" || true

echo
echo "restarted: ${UNITS[*]}"
echo "logs:      ssh $HOST journalctl -fu ${UNITS[0]}"
