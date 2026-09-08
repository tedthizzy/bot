#!/usr/bin/env bash
# hosts/pi/deploy/check-secrets.sh — this repo is public. Fail the build if anything that
# must not be published is in it.
#
#   make secrets
#   ./hosts/pi/deploy/check-secrets.sh --all   # include ignored files too
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

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.."
self=hosts/pi/deploy/check-secrets.sh  # holds every pattern below
findings=0

# Python is already required by deployment. Do not silently skip the filter.
command -v python3 >/dev/null || { echo 'secret scan requires python3' >&2; exit 2; }

filter_hits() {
  python3 -c '
import hashlib
import re
import sys

# Exact source-line SHA256 values, scoped to their original repository paths.
# Every allowed line was compared with public ugv_base_general commit
# d308df91b333a513f45a238bef9ba9a0a5edf64c (firmware/UPSTREAM.md).
# The patch allowance is an unchanged context line from ugv_config.h.
# Changing or extending one of these lines invalidates its allowance.
public_lines = {
    "firmware/General_Driver/wifi_ctrl.h": {
        "4b2cc62fc602c08cc98bb5a0810b6deaa0242543156a2bf7ac2c44bb64f15fb3",
        "e4f6b25bf3899ce38898614db1aac4d9c045c4eca1639536923bb258b9d822f3",
    },
    "firmware/General_Driver/web_page.h": {
        "88b1f483a4ed5d84370f2ffdb273a54015e812ea5aff5c77fa26ac2bbd9af74a",
        "945274ff3cd0229776d3beca3a4ba4ea7ba8fbffde893cc831b4b18dc1b92582",
        "b3a15537769d0abf4a1ec8d778f292339236c866023150d06681502b384bdc9e",
        "c3f1ad8dfbd567c1692207ccb26de3c34fce244e6b0a4258fa10656bc1b83d9d",
        "e1e840228912d1aaf0d3917039ef80415acb26ae8842069398e86134cddfc150",
    },
    "firmware/General_Driver/esp_now_ctrl.h": {
        "119ed718ab5f155d135618867759d502251db186e8ffd3b251f78d174484a55f",
    },
    "firmware/General_Driver/json_cmd.h": {
        "08ad14b8c904de8545274363f4696c6e7804cebe9149fc243668e26beed39298",
        "14d9c195b23ff9f0e4cd5fd6463ce22f596c7a3a270572d1b45f38906078b845",
        "15fe1e7b7483b2fd39f55412a595cc8dd9fe1cbc1c4fca0bc4693a2a16221635",
        "22b9946b82f42903cf9d4b0f7cb7ff8ab9e83170dab6c2c44e675fe4e56339f7",
        "285e4722ac96f78659f9655e4ee6d51bc18098097c51ddc6662d8c0795ddf08a",
        "431493bc25fc42b5624550a104b98de1c06562f7791e80d6b8835c3e50f4dfc0",
        "49deef17d3d4524e7c40a9ae081da4f00d034d76ec87892deb82ff6dc69a6e9b",
        "7a936c914c0db9ef7d05212ae546c74916456e3fe05806f298ba0c420fc8bcc8",
        "8d00acf59bea3d0e16faa58060b850ed2ea233e1e21450e1a440dba9a5151be1",
        "b4457dd9f4e5cdc309f38b5f303f6ec961f91c3a594e0a3c8fd8eb5d30f64d0b",
        "b9906069d92f717ade0c44ea1bf047516f2a31a79713abf74906d27052f5a973",
        "b9e33aa744b849139a39a7c0054a211569aa8d3e8ca5b1b87ff19c9ab6f94aa3",
        "be59aba91fd4ca0e87ccc6536f564b8252ffd775e0bd3ae5ffdd091ba8bc2073",
        "c563713d55d9664237488fc0a33544db138aebe279d09c53b3cef879a7e04703",
        "c879ce54c7e19de44bd44f9437e898dfcc3d11fac60e8f3704b7de0f626c6783",
        "c8812e47aea2c51861df91abaf59e9be70060745725be0d9bc60ac9618fff83c",
        "df35682fe1ed58d9dfe1a558a461852d38225c76dbc804a6e04e8502292ec28b",
    },
    "firmware/General_Driver/ugv_config.h": {
        "6ab6a45dbf02debb161dbc6be50940c6d166caa9d089bee97ede159cc34bf284",
    },
    "firmware/patches/0003-radios-off.patch": {
        "2a13bebbd730700cc6c2028f077822091e7137cc47d31d9447edb7bac9cb27ed",
    },
}
placeholder_hosts = {"box.lan", "rover.local", "example.com", "localhost"}
sys.stdin.reconfigure(errors="surrogateescape")
for raw in sys.stdin:
    match = re.match(r"^(.+?):([0-9]+):(.*)$", raw.rstrip("\r\n"))
    if not match:
        print("[unparseable match redacted]")
        continue
    filename, number, line = match.groups()
    if filename == sys.argv[1]:
        continue
    digest = hashlib.sha256(line.encode(errors="surrogateescape")).hexdigest()
    if digest in public_lines.get(filename, ()):
        continue
    # Exempt only the matched host, never unrelated secrets on the same line.
    hosts = None
    if sys.argv[2] == "LAN hostname":
        hosts = re.findall(r"[A-Za-z0-9][A-Za-z0-9.-]*\.(?:lan|home|internal|localdomain)(?![A-Za-z0-9.-])", line)
    elif sys.argv[2] == "personal ssh target":
        hosts = re.findall(r"ssh\s+[A-Za-z0-9._-]+@([A-Za-z0-9._-]+)", line)
    if hosts and all(host in placeholder_hosts for host in hosts):
        continue
    print(f"{filename}:{number}: [match redacted]")
' "$self" "$1"
}

files() {
  if [ "${1:-}" = "--all" ] || ! git rev-parse --git-dir >/dev/null 2>&1; then
    find . \
      \( -name .git -o -name .venv -o -name '.venv-*' -o -name .cache -o -name node_modules \
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
         filter_hits "$2")
  # Name the file and line without copying credentials into CI/tool logs.
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
  for f in legacy/firmware-s3/contracts/serial_vectors.jsonl tests/gates/g1/utterances.jsonl; do
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
