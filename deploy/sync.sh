#!/usr/bin/env bash
# deploy/sync.sh — the edit loop. Push the working tree to the rover and
# restart the two units that hold no hardware state.
#
#   deploy/sync.sh rover.local
#   ROVER_HOST=rover.local deploy/sync.sh
#
# This is the edit loop and NOT `reboot`, deliberately. After deploy step 17
# adds dtoverlay=gpio-poweroff, a reboot requires the MCU to cycle PI_RAIL_EN:
# the overlay "interferes with the normal power-down sequence, preventing the
# kernel from resetting the SoC (a necessary step in a normal power-off or
# reboot)". rsync plus a restart of brain and web never touches that.
#
# robotd and cam are NOT restarted by default. robotd owns the serial session
# and the arm state, and restarting it re-runs the seq re-seed and drops to
# IDLE; cam restarts cost a libcamera reinitialise. Pass --all when you have
# actually changed them.
#
# No host is baked in: this repo is public and a hostname on someone's LAN is
# not something to publish. The argument or ROVER_HOST is the only source.

set -euo pipefail

HOST=${1:-${ROVER_HOST:-}}
[ -n "$HOST" ] || {
  echo "usage: deploy/sync.sh <user@host>   (or set ROVER_HOST)" >&2
  exit 2
}
shift || true

REMOTE_ROOT=${ROVER_ROOT:-/opt/rover}
UNITS=(rover-brain rover-web)
for arg in "$@"; do
  [ "$arg" = "--all" ] && UNITS=(rover-robotd rover-cam rover-brain rover-web)
done

here=$(cd -- "$(dirname -- "$0")/.." && pwd)

# --delete, because a file that stopped existing here must stop existing there:
# a stale module left behind is how a fix appears to have no effect. The
# excludes are the things that belong to the robot and not to this checkout --
# above all config/robot.toml, which holds the box address and the measured
# cliff baseline.
# --rsync-path="sudo rsync": /opt is root:root 0755, so the tree there is
# root-owned however it was cloned, and a login-user rsync cannot write into it.
# --no-owner --no-group --chown=root:rover: the receiving rsync runs as root, so
# -a's -o -g would stamp this Mac's numeric uid/gid onto the Pi -- a macOS
# checkout is typically 501:20, and gid 20 on Debian is `dialout`, which leaves
# /opt/rover owned by a uid that does not exist and unwritable by the login user.
rsync -az --delete --info=stats1 \
  --rsync-path="sudo rsync" \
  --no-owner --no-group --chown=root:rover \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude '.venv-tts/' \
  --exclude '.venv-brain/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.pytest_cache/' \
  --exclude '.mypy_cache/' \
  --exclude '.ruff_cache/' \
  --exclude '.hypothesis/' \
  --exclude 'run/' \
  --exclude 'data/' \
  --exclude 'logs/' \
  --exclude 'firmware/build/' \
  --exclude 'firmware/host/build/' \
  --exclude 'config/robot.toml' \
  "$here/" "$HOST:$REMOTE_ROOT/"

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
