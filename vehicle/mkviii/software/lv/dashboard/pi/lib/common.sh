# shellcheck shell=bash
# Helpers shared by the scripts in ~/startup on the Pi. Sourced, not run.
# Needs bash, jq, curl, git, diff, patch, flock (pi-setup.sh installs them).

STARTUP_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)   # ~/startup
CONFIG=${OEM_STARTUP_CONFIG:-$STARTUP_DIR/startup_config.json}

# Where deploys land. Must match deploy/lib/common.sh on the laptop.
# OEM_DEPLOY_DIR is only for testing these scripts somewhere other than a Pi.
DEPLOY_DIR=${OEM_DEPLOY_DIR:-$HOME/deploy}
PENDING_DIR=$DEPLOY_DIR/pending              # a deploy/rollback waiting for the next boot
MANIFEST=$DEPLOY_DIR/deploy-manifest.json    # the files the Pi keeps in sync
VERSIONS_DIR=$DEPLOY_DIR/versions            # the version store (versions.sh)

# Whoever changes pending/ holds this lock: startup.sh while it claims a
# pending deploy, and the laptop (deploy.sh, reboot.sh) or versions.sh restore
# while they replace it and reboot. So a deploy that arrives during a boot
# either waits for the next boot or is applied whole, never half-applied or
# applied and then synced away by the reboot that was meant to apply it.
# Must match REMOTE_PENDING_LOCK in deploy/lib/common.sh.
PENDING_LOCK=$DEPLOY_DIR/.pending.lock

log()  { echo "$*" >&2; }
warn() { echo "WARNING: $*" >&2; }
die()  { echo "ERROR: $*" >&2; exit 1; }

# Print the comment block at the top of the calling script (its usage text).
usage() {
  sed -n '2,/^[^#]/{/^#/!q;s/^# \{0,1\}//p}' "$0"
  exit "${1:-0}"
}

# cfg KEY [DEFAULT]: a top-level value from startup_config.json (false and 0
# are returned as-is; only a missing or null key gives DEFAULT).
cfg() {
  jq -r --arg k "$1" --arg d "${2-}" 'if .[$k] == null then $d else .[$k] end' "$CONFIG"
}

# lock_pending [SECONDS]: take PENDING_LOCK on file descriptor 8 (waiting up
# to SECONDS, default 30). It's released when the script exits, or with
# unlock_pending.
lock_pending() {
  mkdir -p "$DEPLOY_DIR" && exec 8>"$PENDING_LOCK" && flock -w "${1:-30}" 8
}

unlock_pending() {
  exec 8>&-
}

# install_file SRC DEST: copy SRC to DEST through a temp file and a rename, so
# losing power mid-copy never leaves a half-written DEST. Chained with && so a
# failed copy never gets renamed into place, even when called as `if install_file`
# (set -e doesn't apply inside a function used as a condition).
install_file() {
  mkdir -p "$(dirname "$2")" &&
    cp --preserve=mode,timestamps "$1" "$2.oem-tmp" &&
    mv -fT "$2.oem-tmp" "$2" ||
    { rm -f "$2.oem-tmp"; return 1; }
}

# write_json_atomic DEST: write stdin to DEST through a temp file and a rename.
write_json_atomic() {
  cat >"$1.tmp" && mv -f "$1.tmp" "$1" || { rm -f "$1.tmp"; return 1; }
}

# version_hash MANIFEST FILES_DIR: the hash that identifies a version.
# MANIFEST is a JSON list of {file_name, file_path, repo_path}; each file's
# content is FILES_DIR/<file_name>. It's the sha256 of one line per file,
# "<sha256 of content>  <file_path>  <repo_path>", sorted. Must match
# version_hash in deploy/lib/common.sh on the laptop.
version_hash() {
  local manifest=$1 dir=$2 name path repo sum
  jq -r '.[] | [.file_name, .file_path, (.repo_path // "")] | @tsv' "$manifest" |
    while IFS=$'\t' read -r name path repo; do
      sum=$(sha256sum <"$dir/$name") || exit 1
      printf '%s  %s  %s\n' "${sum%% *}" "$path" "$repo"
    done | LC_ALL=C sort | sha256sum | cut -c1-64
}
