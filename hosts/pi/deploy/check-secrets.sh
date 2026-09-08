#!/usr/bin/env bash
# hosts/pi/deploy/check-secrets.sh — this repo is public. Fail the build if anything that
# must not be published is in it.
#
#   make secrets            # or ./deploy/check-secrets.sh
#   ./deploy/check-secrets.sh --all   # scan untracked files too
#
# Scans the files git would publish, not the working tree: an ignored file is
# not a leak, and scanning the working tree makes the check fail on things that
# were never going to ship. Outside a git checkout it falls back to find with
# the same exclusions.
#
# Two kinds of finding:
#   secret   a credential shape or a literal assignment
#   private  something true only of one person's house — a LAN address, a MAC,
#            a home directory path
# Both fail. A LAN name is not a security incident and is still not publishable.

set -uo pipefail

cd -- "$(cd -- "$(dirname -- "$0")/.." && pwd)"
self=deploy/check-secrets.sh          # holds every pattern below, by definition
findings=0

# Documented placeholders, not anyone's network. ARCHITECTURE.md's [box] url
# example and the Imager's default mDNS name; both are in the frozen document
# and neither resolves to a real host. config/*.toml uses localhost instead.
ALLOW='box\.lan|rover\.local|example\.com|localhost'

files() {
  if [ "${1:-}" = "--all" ] || ! git rev-parse --git-dir >/dev/null 2>&1; then
    find . \
      \( -name .git -o -name .venv -o -name '.venv-tts' -o -name node_modules \
         -o -name __pycache__ -o -name build -o -name '.*_cache' \
         -o -name '.hypothesis' -o -name 'hf-cache' \) -prune -o \
      -type f -print | sed 's|^\./||'
  else
    # Tracked plus untracked-but-not-ignored: exactly the set that would be
    # published on the next `git add -A`, which is what needs checking before
    # a commit rather than after one.
    git ls-files --cached --others --exclude-standard
  fi
}

report() {                            # report <kind> <what> <matches>
  printf '\n%s: %s\n' "$1" "$2"
  printf '%s\n' "$3" | sed 's/^/    /'
  findings=$((findings + 1))
}

scan() {                              # scan <kind> <description> <regex>
  local hits
  [ -n "$LIST" ] || return 0
  hits=$(printf '%s\n' "$LIST" | tr '\n' '\0' |
         xargs -0 grep -InE -- "$3" 2>/dev/null |
         grep -vE "^$self:" | grep -vE "$ALLOW")
  [ -n "$hits" ] && report "$1" "$2" "$hits"
  return 0
}

LIST=$(files "${1:-}")
count=$(printf '%s\n' "$LIST" | grep -c . || true)
echo "scanning $count files"

# --- credentials ------------------------------------------------------------

scan secret "private key block" \
  '-----BEGIN [A-Z ]*PRIVATE KEY-----'
scan secret "GitHub token" \
  '(ghp|gho|ghu|ghs|ghr)_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}'
scan secret "OpenAI-style key" \
  'sk-[A-Za-z0-9_-]{20,}'
scan secret "Hugging Face token" \
  'hf_[A-Za-z0-9]{20,}'
scan secret "AWS access key id" \
  'AKIA[0-9A-Z]{16}'
scan secret "Slack token" \
  'xox[abprs]-[A-Za-z0-9-]{10,}'
# A literal beside a credential-shaped name. The empty string and a $VAR
# reference are fine: box/.env.example and the systemd EnvironmentFile are
# exactly that shape.
scan secret "credential assigned a literal" \
  '(api[_-]?key|secret|password|passwd|passphrase|auth[_-]?token|bearer)[[:space:]]*[:=][[:space:]]*["'"'"'][^"'"'"'$][^"'"'"']{7,}'

# --- private facts ----------------------------------------------------------

scan private "RFC1918 address" \
  '(^|[^0-9.])(10\.[0-9]{1,3}|192\.168|172\.(1[6-9]|2[0-9]|3[01]))\.[0-9]{1,3}\.[0-9]{1,3}([^0-9.]|$)'
scan private "MAC address" \
  '([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}'
scan private "LAN hostname" \
  '[A-Za-z0-9]\.(lan|home|internal|localdomain)([^A-Za-z0-9.-]|$)'
scan private "home directory path" \
  '/(Users|home)/[A-Za-z0-9._-]+/'
scan private "personal ssh target" \
  'ssh[[:space:]]+[A-Za-z0-9._-]+@[A-Za-z0-9._-]+'

# --- what must and must not be tracked --------------------------------------

if git rev-parse --git-dir >/dev/null 2>&1; then
  # Must be ignored: the local configuration and every environment file.
  for f in config/robot.toml box/.env .env; do
    if [ -e "$f" ] && ! git check-ignore -q "$f"; then
      report secret "file that must stay out of git is not ignored" "$f"
    fi
  done
  # Must NOT be ignored, and this is the direction that bites: a bare *.jsonl
  # in .gitignore silently excludes the golden serial vectors, which the C
  # firmware core and the Python codec are both written against. A missing
  # contract artefact is as much a build failure as a leaked secret.
  for f in tests/contract/serial_vectors.jsonl tests/gates/g1/utterances.jsonl; do
    if [ -e "$f" ] && git check-ignore -q "$f"; then
      report secret "contract artefact excluded by .gitignore" \
        "$f exists but .gitignore hides it — the *.jsonl rule needs its negation"
    fi
  done
fi

# --- verdict ----------------------------------------------------------------

echo
if [ "$findings" -gt 0 ]; then
  echo "$findings finding(s). Nothing here may be committed."
  echo "A credential belongs in an environment variable named by config; a LAN"
  echo "address belongs in ROVER__BOX__URL or /etc/rover/box.env, not a file."
  exit 1
fi
echo "clean: no secret, LAN address, MAC address or home path in what git publishes"
