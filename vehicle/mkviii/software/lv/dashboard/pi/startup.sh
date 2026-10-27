#!/usr/bin/env bash
# Boot script for the dashboard Pi. oem-startup.service runs it on every boot.
#
#   startup.sh
#
# 1. If a deploy or rollback is pending (~/deploy/pending/, staged from the
#    laptop by deploy.sh or versions.sh), installs its files. Otherwise
#    downloads each file in ~/deploy/deploy-manifest.json from the GitHub
#    branch in startup_config.json; without network it keeps the files it has.
# 2. In the background: refreshes ~/.ssh/authorized_keys from GitHub
#    (sync-keys.sh) and records the installed files as a version (versions.sh).
# 3. Runs pre_run_command, if set (with bash, in dashboard_dir), e.g. to
#    compile the dashboard. If it fails, the dashboard is NOT started: broken
#    code shows up as no dashboard, not as an older one still running.
# 4. Runs dashboard_command (with bash, in dashboard_dir), restarting it
#    whenever it exits.
#
# The dashboard starts even if step 1 fails. Logs: journalctl -u oem-startup -b

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

case $# in
  0) ;;
  *) [[ $1 == -h || $1 == --help ]] && usage; usage 2 ;;
esac

err() { echo "ERROR: $*" >&2; }

# What runs the dashboard when startup_config.json doesn't say (or can't be read).
DEFAULT_DASH_CMD="/usr/bin/python3 dashboard.py"

# Read startup_config.json in one jq call (jq is slow to start on a Pi Zero).
# A broken config must not stop the dashboard: fall back to the defaults and
# skip the GitHub sync.
read_config() {
  local out c
  REPO_URL="" BRANCH=main NET_TIMEOUT=10 DASH_CMD=$DEFAULT_DASH_CMD DASH_DIR=$HOME
  RESTART=true RESTART_DELAY=3 PRE_RUN="" CONFIG_OK=0
  # Commands are one line (newlines -> spaces) so each stays one config value.
  if ! out=$(jq -r --arg cmd "$DEFAULT_DASH_CMD" '
      def get(k; d): if .[k] == null then d else .[k] end;
      get("repo_url"; ""), get("branch"; "main"), get("network_timeout_s"; 10),
      (get("dashboard_command"; $cmd) | tostring | gsub("[\\r\\n]+"; " ")), get("dashboard_dir"; "~"),
      (get("pre_run_command"; "") | tostring | gsub("[\\r\\n]+"; " ")),
      get("restart_dashboard_on_exit"; true), get("restart_delay_s"; 3)' "$CONFIG" 2>&1); then
    err "Can't read $CONFIG (${out:-no output}); running the dashboard with default settings, without a GitHub sync"
    return 0
  fi
  mapfile -t c <<<"$out"
  if [ "${#c[@]}" -ne 8 ]; then
    err "Unexpected values in $CONFIG; running the dashboard with default settings, without a GitHub sync"
    return 0
  fi
  REPO_URL=${c[0]} BRANCH=${c[1]} NET_TIMEOUT=${c[2]} DASH_CMD=${c[3]}
  DASH_DIR=${c[4]} PRE_RUN=${c[5]} RESTART=${c[6]} RESTART_DELAY=${c[7]} CONFIG_OK=1
  [[ -n ${PRE_RUN//[[:space:]]/} ]] || PRE_RUN=""
  # dashboard_dir may start with ~ or be relative to the home folder.
  case $DASH_DIR in
    "~") DASH_DIR=$HOME ;;
    "~/"*) DASH_DIR=$HOME/${DASH_DIR#"~/"} ;;
    /*) ;;
    *) DASH_DIR=$HOME/$DASH_DIR ;;
  esac
  if [[ -z ${DASH_CMD//[[:space:]]/} ]]; then
    warn "dashboard_command is empty; using \"$DEFAULT_DASH_CMD\""
    DASH_CMD=$DEFAULT_DASH_CMD
  fi
  # 0 would mean "no timeout" to timeout and curl, so a dead network could hold
  # up the dashboard forever.
  if [[ ! $NET_TIMEOUT =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ $NET_TIMEOUT =~ ^0*([.]0*)?$ ]]; then
    warn "network_timeout_s must be a number above 0 (it's $NET_TIMEOUT); using 10"
    NET_TIMEOUT=10
  fi
  # At least 1 s, so a dashboard that crashes at once can't spin the CPU.
  if [[ ! $RESTART_DELAY =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ $RESTART_DELAY =~ ^0*([.][0-9]*)?$ ]]; then
    warn "restart_delay_s must be at least 1 (it's $RESTART_DELAY); using 3"
    RESTART_DELAY=3
  fi
}

# tagged TAG COMMAND...: run COMMAND, prefixing every line it prints with
# "TAG: ", so background jobs are easy to pick out in the journal.
tagged() {
  local tag=$1 line rc=0
  shift
  "$@" 2>&1 | while IFS= read -r line || [ -n "$line" ]; do log "$tag: $line"; done || rc=$?
  [ "$rc" -eq 0 ] || warn "$tag: exited with code $rc"
  return "$rc"
}

# files/ is a deploy staged for the old startup.py. (Half-finished uploads,
# .incoming-*, are left for the next deploy.sh to clear: one could be arriving
# right now.)
clean_leftovers() {
  if [ -e "$DEPLOY_DIR/files" ]; then
    log "Removing $DEPLOY_DIR/files (a deploy staged for the old startup.py)"
    rm -rf -- "$DEPLOY_DIR/files" || warn "Couldn't remove $DEPLOY_DIR/files"
  fi
}

# claim_pending: if a deploy or rollback is pending, take it by renaming
# pending/ to .applying/ while holding PENDING_LOCK (see lib/common.sh), so
# one that arrives from now on waits for the next boot. A .applying/ that's
# already there was cut short (power loss mid-boot): apply it again, unless a
# newer deploy is pending.
APPLYING=$DEPLOY_DIR/.applying
claim_pending() {
  lock_pending || warn "Couldn't lock $PENDING_LOCK; going ahead without it"
  if [ -d "$PENDING_DIR" ]; then
    { rm -rf -- "$APPLYING" && mv -- "$PENDING_DIR" "$APPLYING"; } || err "Couldn't move $PENDING_DIR to $APPLYING"
  elif [ -d "$APPLYING" ]; then
    log "Found a deploy whose install was cut short; applying it again"
  fi
  unlock_pending
  [ -d "$APPLYING" ]
}

# apply_pending DIR: install a deploy/rollback claimed by claim_pending and
# make its manifest the Pi's. Sets SOURCE/DETAIL for the snapshot. If it's
# unusable it's discarded and SOURCE stays empty, so the boot syncs with GitHub
# instead.
apply_pending() {
  local dir=$1 m=$1/deploy-manifest.json entries name path msg ok=0 failed=0
  local -a info=()
  if ! entries=$(jq -r 'if type == "array" then .[] else error("not a list") end |
      [.file_name // "", .file_path // ""] | @tsv' "$m" 2>&1); then
    err "The pending deploy has no usable deploy-manifest.json (${entries:-empty}); discarding it"
    rm -rf -- "$dir" || err "Couldn't remove $dir"
    return 0
  fi
  mapfile -t info < <(jq -r '.source // "deploy", .detail // "deploy", .time // ""' \
    "$dir/deploy-info.json" 2>/dev/null)
  SOURCE=${info[0]:-deploy} DETAIL=${info[1]:-deploy}
  case $SOURCE in deploy | rollback) ;; *) SOURCE=deploy ;; esac
  log "Applying the pending $SOURCE: $DETAIL${info[2]:+ (staged ${info[2]})}"

  while IFS=$'\t' read -r name path; do
    [ -n "$name$path" ] || continue
    if [ -z "$name" ] || [[ $path != /* ]]; then
      err "Bad manifest entry (file_name '$name', file_path '$path'); skipping it"
      failed=$((failed + 1))
    elif msg=$(install_file "$dir/files/$name" "$path" 2>&1); then
      log "Deployed $name -> $path"
      ok=$((ok + 1))
    else
      err "Couldn't deploy $name -> $path: $msg"
      failed=$((failed + 1))
    fi
  done <<<"$entries"

  # From now on GitHub syncs keep the files in this manifest up to date. If
  # even this small file can't be written, the SD card is full or read-only:
  # keep the deploy to try again next boot rather than lose it.
  if ! write_json_atomic "$MANIFEST" <"$m"; then
    err "Couldn't save $MANIFEST (is the SD card full?); keeping the $SOURCE to try again next boot"
    return 0
  fi
  # Make sure the installed files are on the SD card before the staged copy is
  # deleted: a power cut right after a deploy is normal in a car.
  sync
  # Removed even if files failed, so a bad deploy can't block later GitHub syncs.
  rm -rf -- "$dir" || err "Couldn't remove $dir; it will be applied again next boot"
  if [ "$failed" -eq 0 ]; then
    log "Applied the $SOURCE: $ok file(s) installed"
  elif [ "$ok" -eq 0 ]; then
    err "None of the $SOURCE's files could be installed (see above)"
    DETAIL="$DETAIL (none of its files could be installed)"
  else
    err "Applied the $SOURCE with errors: $ok file(s) installed, $failed failed (see above)"
    DETAIL="$DETAIL ($failed file(s) failed to install)"
  fi
}

# github_sync: overwrite each manifest file that has a repo_path with its
# version on BRANCH. Every file comes from the one commit ls-remote names
# (raw.githubusercontent.com pinned to its sha, so the CDN can't serve a stale
# mix), and if any download fails nothing is installed, so the Pi never mixes
# commits. Sets SOURCE/DETAIL.
github_sync() {
  local url gh_owner gh_repo entries out s ref sha="" code why path enc repo_path msg i rc
  local updated=0 unchanged=0 failed=0
  local -a paths=() encs=() repo_paths=() got=()
  SOURCE=local DETAIL="files on the Pi (no deploy; GitHub sync skipped)"
  [ "$CONFIG_OK" = 1 ] || return 0

  url=${REPO_URL%/}
  url=${url%.git}
  if [[ ! $url =~ (^|[/@])github\.com[:/]+([^/]+)/([^/]+)$ ]]; then
    warn "repo_url ($REPO_URL) isn't a GitHub repo; skipping the GitHub sync"
    return 0
  fi
  gh_owner=${BASH_REMATCH[2]} gh_repo=${BASH_REMATCH[3]}

  # repo_path is URL-encoded segment by segment, keeping the "/" separators.
  # The possibly-empty fields go last (read merges consecutive tabs).
  if ! entries=$(jq -r 'if type == "array" then .[] else error("not a list") end |
      [.file_path // "", ((.repo_path // "") | split("/") | map(@uri) | join("/")), .repo_path // ""] |
      @tsv' "$MANIFEST" 2>&1); then
    err "Can't read $MANIFEST (${entries:-empty}); skipping the GitHub sync"
    return 0
  fi
  while IFS=$'\t' read -r path enc repo_path; do
    [ -n "$path$repo_path" ] || continue
    if [[ $path != /* ]]; then
      err "Bad file_path '$path' in $MANIFEST; not syncing it"
    elif [ -z "$repo_path" ]; then
      warn "No repo_path for $path in the manifest; not syncing it"
    else
      paths+=("$path") encs+=("$enc") repo_paths+=("$repo_path")
    fi
  done <<<"$entries"
  if [ "${#paths[@]}" -eq 0 ]; then
    log "Nothing in $MANIFEST comes from GitHub; keeping the existing files"
    return 0
  fi

  log "Checking GitHub for the latest commit on $BRANCH"
  rc=0
  out=$(GIT_TERMINAL_PROMPT=0 timeout -k 2 "$NET_TIMEOUT" \
    git ls-remote "$REPO_URL" "refs/heads/$BRANCH" 2>"$WORK/err") || rc=$?
  if [ "$rc" -ne 0 ]; then
    if [ "$rc" -eq 124 ]; then why="timed out after $NET_TIMEOUT s"; else why=$(tail -n 1 "$WORK/err"); fi
    # GitHub asks for a login when a repo doesn't exist (or is private).
    case $why in *"could not read Username"*) why="no public repo at $REPO_URL" ;; esac
    warn "Couldn't reach GitHub (${why:-git exited with code $rc}); keeping the existing files"
    DETAIL="files on the Pi (no deploy; GitHub unreachable)"
    return 0
  fi
  while read -r s ref; do
    if [ "$ref" = "refs/heads/$BRANCH" ]; then sha=$s; fi
  done <<<"$out"
  if [ -z "$sha" ]; then
    warn "Branch $BRANCH isn't on GitHub ($REPO_URL); keeping the existing files. Change it with deploy/set-branch.sh."
    DETAIL="files on the Pi (no deploy; branch $BRANCH not on GitHub)"
    return 0
  fi

  log "Downloading ${#paths[@]} file(s) from $BRANCH @ ${sha:0:7}"
  mkdir -p "$WORK/dl"
  for i in "${!paths[@]}"; do
    rc=0
    code=$(curl -sS -L --max-time "$NET_TIMEOUT" -o "$WORK/dl/$i" -w '%{http_code}' \
      "https://raw.githubusercontent.com/$gh_owner/$gh_repo/$sha/${encs[i]}" 2>"$WORK/err") || rc=$?
    if [ "$rc" -ne 0 ]; then
      warn "Couldn't download ${repo_paths[i]} ($(tail -n 1 "$WORK/err")); keeping all the existing files"
      DETAIL="files on the Pi (no deploy; GitHub download failed)"
      return 0
    fi
    case $code in
      200) got+=("$i") ;;
      404) warn "${repo_paths[i]} isn't on $BRANCH; keeping the existing ${paths[i]}" ;;
      *)
        warn "Couldn't download ${repo_paths[i]} (HTTP $code); keeping all the existing files"
        DETAIL="files on the Pi (no deploy; GitHub download failed)"
        return 0
        ;;
    esac
  done
  if [ "${#got[@]}" -eq 0 ]; then
    DETAIL="files on the Pi (none of them on $BRANCH @ ${sha:0:7})"
    return 0
  fi

  local -a changed=()
  for i in "${got[@]}"; do
    if [ -e "${paths[i]}" ] && cmp -s "$WORK/dl/$i" "${paths[i]}"; then
      log "Unchanged ${paths[i]}"
      unchanged=$((unchanged + 1))
    else
      changed+=("$i")
    fi
  done
  # Save what's installed before overwriting it, so edits made by hand on the
  # Pi can be rolled back to. Usually it's already a version (a cache hit,
  # well under a second); only runs when GitHub changes something.
  if [ "${#changed[@]}" -gt 0 ] && [ -f "$STARTUP_DIR/versions.sh" ]; then
    tagged versions "$BASH" "$STARTUP_DIR/versions.sh" snapshot --source local \
      --detail "files on the Pi before syncing $BRANCH @ ${sha:0:7}" || true
  fi

  for i in "${changed[@]}"; do
    path=${paths[i]}
    # A download gets default permissions; keep the installed file's (e.g. +x).
    if [ -e "$path" ]; then chmod --reference="$path" "$WORK/dl/$i" 2>/dev/null || true; fi
    if msg=$(install_file "$WORK/dl/$i" "$path" 2>&1); then
      log "Updated $path (from ${repo_paths[i]})"
      updated=$((updated + 1))
    else
      err "Couldn't update $path: $msg"
      failed=$((failed + 1))
    fi
  done
  [ "$updated" -eq 0 ] || sync
  msg="$updated updated, $unchanged unchanged"
  [ "$failed" -eq 0 ] || msg+=", $failed failed"
  log "GitHub sync done ($BRANCH @ ${sha:0:7}): $msg"
  SOURCE=github DETAIL="github $BRANCH @ ${sha:0:7}"
}

# prepare_files: apply a pending deploy or sync with GitHub. Writes the
# snapshot's source and detail to $WORK/result.
prepare_files() {
  SOURCE="" DETAIL=""
  if ! command -v jq >/dev/null; then
    err "jq isn't installed (pi-setup.sh installs it); leaving any pending deploy for later and skipping the GitHub sync"
    return 0
  fi
  if claim_pending; then
    apply_pending "$APPLYING"
  fi
  if [ -z "$SOURCE" ]; then
    if [ -f "$MANIFEST" ]; then
      github_sync
    else
      warn "No $MANIFEST yet, so there's nothing to sync. Run deploy/deploy.sh from the laptop once."
    fi
  fi
  printf '%s\n%s\n' "$SOURCE" "${DETAIL//$'\n'/ }" >"$WORK/result"
}

# pre_run: run pre_run_command once, in dashboard_dir. Its output goes to the
# journal tagged "pre-run". Returns its exit code.
pre_run() {
  log "Running the pre-run command (in $DASH_DIR): $PRE_RUN"
  tagged pre-run /bin/bash -c "cd -- \"\$1\" && exec /bin/bash -c \"\$2\"" pre-run "$DASH_DIR" "$PRE_RUN"
}

# run_dashboard: run dashboard_command with bash in dashboard_dir, restarting
# it RESTART_DELAY s after it exits unless restart_dashboard_on_exit is false.
# Returns the dashboard's exit code.
run_dashboard() {
  local code
  while true; do
    log "Starting the dashboard (in $DASH_DIR): $DASH_CMD"
    code=0
    # exec, so the dashboard replaces the subshell and gets systemd's signals.
    (cd -- "$DASH_DIR" && exec /bin/bash -c "$DASH_CMD") || code=$?
    if [ "$code" -eq 0 ]; then log "The dashboard exited (code 0)"; else warn "The dashboard exited with code $code"; fi
    if [ "$RESTART" = false ]; then
      return "$code"
    fi
    log "Restarting the dashboard in $RESTART_DELAY s"
    sleep "$RESTART_DELAY"
  done
}

main() {
  local start rc=0 code=0 now
  local -a result=()
  # Time since boot, not the wall clock: the Pi has no clock battery, so its
  # clock jumps when it gets the time from the network during boot.
  read -r start _ </proc/uptime
  read_config
  clean_leftovers

  # Run as a background job and waited for: unlike `prepare_files || ...`, that
  # keeps set -e working inside it, and a bug (a failed command, an unset
  # variable) ends only prepare_files, never the boot.
  if WORK=$(mktemp -d -t oem-startup.XXXXXX); then
    prepare_files &
    wait "$!" || rc=$?
    [ "$rc" -eq 0 ] || err "Applying the deploy / GitHub sync failed unexpectedly (exit $rc); starting the dashboard with the files that are there"
    [ ! -s "$WORK/result" ] || mapfile -t result <"$WORK/result"
    rm -rf -- "$WORK" || true
  else
    err "Couldn't create a temp folder; skipping the deploy / GitHub sync"
  fi

  # Background jobs stay children of this script, so they aren't cut short
  # while the dashboard runs; stopping the service stops them too. The key sync
  # starts only now so it doesn't slow down the GitHub sync the dashboard waits
  # for (the Pi Zero has one core, and each HTTPS connection costs seconds).
  if [ -f "$STARTUP_DIR/sync-keys.sh" ]; then
    tagged keys "$BASH" "$STARTUP_DIR/sync-keys.sh" &
  else
    warn "No $STARTUP_DIR/sync-keys.sh; not updating authorized_keys"
  fi

  # Snapshot what's installed, also when nothing came in (that records hand
  # edits on the Pi; otherwise it's a cache hit). In the background: storing a
  # new version can take a few seconds and the dashboard shouldn't wait.
  if [ -f "$MANIFEST" ]; then
    if [ -f "$STARTUP_DIR/versions.sh" ]; then
      tagged versions "$BASH" "$STARTUP_DIR/versions.sh" snapshot \
        --source "${result[0]:-local}" --detail "${result[1]:-files on the Pi (boot sync failed)}" &
    else
      warn "No $STARTUP_DIR/versions.sh; not recording this boot's files as a version"
    fi
  fi

  read -r now _ </proc/uptime
  log "Files ready after $(awk -v a="$start" -v b="$now" 'BEGIN { printf "%.1f", b - a }') s"
  if [ -n "$PRE_RUN" ] && ! pre_run; then
    code=1
    err "The pre-run command failed, so the dashboard is NOT running. Fix the code and deploy again (deploy/deploy.sh), or roll back (deploy/versions.sh)."
  else
    run_dashboard || code=$?
  fi

  # Only reached if the pre-run command failed or restart_dashboard_on_exit is false. systemd kills whatever
  # is left in the service once this script exits, so let background jobs finish.
  if [ -n "$(jobs -rp)" ]; then
    log "Waiting for the background jobs (key sync, version snapshot) to finish"
    wait || true
  fi
  exit "$code"
}

main
