#!/usr/bin/env bash
# Deploy your local copies of the files in config.json's "deploy" list to the
# dashboard Pi, then reboot it.
#
#   ./deploy.sh [-c CONFIG] [-n|--dry-run] [--reboot|--shutdown|--no-reboot]
#               [--no-pre-deploy]
#
# If config.json sets "pre_deploy_command", it runs first (with bash, in
# config.json's folder), e.g. to compile the dashboard so the build output in
# the deploy list is fresh. If it fails, nothing is deployed.
#
# The files (uncommitted changes included) are staged on the Pi in
# ~/deploy/pending, not in their final places. On its next boot the Pi installs
# them, saves them as a new version, and starts the dashboard. They stay until
# the Pi next syncs with GitHub (./reboot.sh) or rolls back (./versions.sh).
#
#   -c CONFIG      config file (default: config.json next to this script)
#   -n, --dry-run  list what would be deployed (repo path -> Pi path) and its
#                  version hash, without connecting to the Pi
#   --reboot       reboot the Pi afterwards (default; see post_deploy_action)
#   --shutdown     shut the Pi down afterwards instead
#   --no-reboot    leave the Pi running; the deploy is applied on its next boot
#   --no-pre-deploy  don't run pre_deploy_command (deploy the files as they are)
#
# PI_IP=<ip> ./deploy.sh skips looking for the Pi by its MAC address.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg= dry_run= action= pre_deploy=1
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift
      ;;
    -n | --dry-run) dry_run=1 ;;
    --no-pre-deploy) pre_deploy= ;;
    --reboot | --shutdown | --no-reboot)
      a=${1#--}
      a=${a/no-reboot/none}
      [ -z "$action" ] || [ "$action" = "$a" ] || die "use only one of --reboot, --shutdown and --no-reboot"
      action=$a
      ;;
    -h | --help) usage ;;
    *) die "unknown argument '$1' (see $(basename "$0") -h)" ;;
  esac
  shift
done

load_config "$config_arg"
action=${action:-$POST_DEPLOY_ACTION}
case $action in
  reboot | shutdown | none) ;;
  *) die "post_deploy_action in $CONFIG must be \"reboot\", \"shutdown\" or \"none\", not \"$action\"" ;;
esac

# The deploy list expanded to one entry per file, as parallel arrays: the local
# file, its path on the Pi, its path in the repo, and the repo's root folder.
LOCALS=() REMOTES=() REPO_PATHS=() REPO_ROOTS=()
declare -A LOCAL_FOR_REMOTE=()

# repo_root PATH: the root of the git repo that PATH (a file or folder) is in.
repo_root() {
  local dir=$1 top
  [ -d "$dir" ] || dir=$(dirname "$dir")
  top=$(git -C "$dir" rev-parse --show-toplevel 2>/dev/null) ||
    die "$1 is not in a git repo; the Pi can only sync files that are in the repo"
  realpath -- "$top"
}

# add_file LOCAL REMOTE REPO_ROOT: queue one file. Its repo_path is where the
# Pi downloads it from when it syncs with GitHub.
add_file() {
  local src=$1 dest=$2 root=$3 real
  real=$(realpath -- "$src")
  [[ $real == "$root"/* ]] ||
    die "$src is outside the repo ($root); the Pi can only sync files that are in the repo"
  # The Pi's version store (versions.sh) refuses these, and they'd garble the
  # manifest's tab-separated reading on both sides.
  [[ ! $src$dest$real =~ [[:cntrl:]\\] ]] ||
    die "$(printf '%q' "$src") (-> $(printf '%q' "$dest")): file names and paths can't contain backslashes or control characters"
  # Two local files for one place on the Pi is almost certainly a mistake.
  if [ -n "${LOCAL_FOR_REMOTE[$dest]+set}" ]; then
    [ "${LOCAL_FOR_REMOTE[$dest]}" = "$src" ] && return # listed twice: deploy it once
    die "both ${LOCAL_FOR_REMOTE[$dest]} and $src are mapped to $dest"
  fi
  LOCAL_FOR_REMOTE[$dest]=$src
  LOCALS+=("$src") REMOTES+=("$dest") REPO_PATHS+=("${real#"$root"/}") REPO_ROOTS+=("$root")
}

collect_files() {
  local count bad src dest root rel
  count=$(jq '.deploy | if type == "array" then length else 0 end' "$CONFIG")
  [ "$count" -gt 0 ] || die "nothing to deploy: \"deploy\" in $CONFIG is empty"
  bad=$(jq -c 'first(.deploy[]
    | select(type != "object" or (.local_path | type) != "string" or (.remote_path | type) != "string"))' "$CONFIG")
  [ -z "$bad" ] || die "each deploy entry needs a \"local_path\" and a \"remote_path\": $bad"

  # NUL-separated, so any file name works.
  while IFS= read -r -d '' src && IFS= read -r -d '' dest; do
    # ~ is expanded, and relative paths are relative to the config's folder.
    case $src in
      "~") src=$HOME ;;
      "~/"*) src=$HOME/${src#"~/"} ;;
    esac
    [[ $src == /* ]] || src=$CONFIG_DIR/$src
    src=$(realpath -ms -- "$src")
    [[ $dest == /* ]] || die "remote_path must be absolute: $dest"
    [ -e "$src" ] || die "local_path does not exist: $src"
    root=$(repo_root "$src")

    if [ -d "$src" ]; then
      # Every file under the folder, keeping its structure under remote_path.
      dest=$(realpath -ms -- "$dest")
      while IFS= read -r -d '' rel; do
        add_file "$src/$rel" "${dest%/}/$rel" "$root"
      done < <(find -H "$src" -xtype f -printf '%P\0' | LC_ALL=C sort -z)
    elif [ -f "$src" ]; then
      [[ $dest != */ ]] || die "remote_path for a file must be a file path, not a folder: $dest"
      add_file "$src" "$(realpath -ms -- "$dest")" "$root"
    else
      die "local_path is not a file or a folder: $src"
    fi
  done < <(jq -j '.deploy[] | .local_path, "\u0000", .remote_path, "\u0000"' "$CONFIG")

  [ "${#LOCALS[@]}" -gt 0 ] || die "nothing to deploy: the folders in the deploy list are empty"
}

# deploy_detail: e.g. "deploy by max@laptop: my-branch @ ba9c35a (+uncommitted)".
# (+uncommitted) means some deployed file differs from that commit.
deploy_detail() {
  local root=${REPO_ROOTS[0]} branch commit dirty= i
  local -a paths
  local -A roots=()
  branch=$(git -C "$root" symbolic-ref --short -q HEAD) || branch="detached HEAD"
  commit=$(git -C "$root" rev-parse --short HEAD 2>/dev/null) || commit="no commits"
  for root in "${REPO_ROOTS[@]}"; do roots[$root]=1; done
  for root in "${!roots[@]}"; do
    paths=()
    for i in "${!LOCALS[@]}"; do
      if [ "${REPO_ROOTS[i]}" = "$root" ]; then paths+=("${REPO_PATHS[i]}"); fi
    done
    if [ -n "$(git --literal-pathspecs -C "$root" status --porcelain -- "${paths[@]}")" ]; then
      dirty=" (+uncommitted)"
    fi
  done
  echo "deploy by ${USER:-$(id -un)}@$(hostname -s): $branch @ $commit$dirty"
}

# build_staging DIR: lay out the deploy in DIR the way the Pi expects it in
# ~/deploy/pending: deploy-manifest.json, deploy-info.json and files/.
build_staging() {
  local dir=$1 i n name stem ext
  local -a fields=()
  local -A used=()
  mkdir -p "$dir/files"
  for i in "${!LOCALS[@]}"; do
    # files/ is flat, so make repeated names unique: main.py, main_1.py, ...
    name=${LOCALS[i]##*/}
    if [ -n "${used[$name]+set}" ]; then
      stem=$name ext=
      # The extension starts at the last dot, not counting leading dots (.bashrc has none).
      if [[ ${name#"${name%%[!.]*}"} == *.* ]]; then stem=${name%.*} ext=.${name##*.}; fi
      n=1
      while [ -n "${used[${stem}_$n$ext]+set}" ]; do n=$((n + 1)); done
      name=${stem}_$n$ext
    fi
    used[$name]=1
    cp --preserve=mode,timestamps -- "${LOCALS[i]}" "$dir/files/$name"
    fields+=("$name" "${REMOTES[i]}" "${REPO_PATHS[i]}")
  done

  jq -n --indent 4 '$ARGS.positional as $f
    | [range(0; $f | length; 3) | {file_name: $f[.], file_path: $f[. + 1], repo_path: $f[. + 2]}]' \
    --args -- "${fields[@]}" >"$dir/deploy-manifest.json"
  jq -n --indent 4 --arg detail "$(deploy_detail)" --arg time "$(date -Iseconds)" \
    '{source: "deploy", detail: $detail, time: $time}' >"$dir/deploy-info.json"
}

list_files() {
  local i
  for i in "${!LOCALS[@]}"; do
    echo "  ${REPO_PATHS[i]} -> ${REMOTES[i]}"
  done
}

# run_pre_deploy: run pre_deploy_command from config.json's folder. Its output
# goes to stderr, so it can't mix with what this script prints.
run_pre_deploy() {
  local rc=0
  echo "Running the pre-deploy command: $PRE_DEPLOY_COMMAND" >&2
  (cd -- "$CONFIG_DIR" && bash -c "$PRE_DEPLOY_COMMAND") >&2 || rc=$?
  [ "$rc" -eq 0 ] || die "the pre-deploy command failed (exit $rc); nothing was deployed. Fix it, or skip it with --no-pre-deploy."
}

if [ -n "$PRE_DEPLOY_COMMAND" ]; then
  if [ -n "$pre_deploy" ]; then
    run_pre_deploy
  else
    echo "Skipping the pre-deploy command (--no-pre-deploy)." >&2
  fi
fi
collect_files
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
stage=$tmp/pending
build_staging "$stage"
hash=$(version_hash "$stage/deploy-manifest.json" "$stage/files")
detail=$(jq -r .detail "$stage/deploy-info.json")

if [ -n "$dry_run" ]; then
  echo "Would deploy ${#LOCALS[@]} file(s):"
  list_files
  echo "Version hash: ${hash:0:8} ($detail)"
  echo "Dry run, nothing sent."
  exit 0
fi

find_pi
echo "Deploying ${#LOCALS[@]} file(s) to the Pi at $PI_IP:"
list_files

# Make sure ~/deploy exists, clear out uploads that never finished, and read
# the version index while we're there (there's none until the Pi has booted
# with a deploy).
index=$(pi_ssh "mkdir -p $REMOTE_DEPLOY_DIR && rm -rf $REMOTE_DEPLOY_DIR/.incoming-* &&
  { cat $REMOTE_VERSIONS_INDEX 2>/dev/null || true; }") ||
  die "couldn't run commands on the Pi at $PI_IP (see above)"
jq -r --arg h "$hash" 'first(.versions[]? | select(.hash == $h))
  | "Note: this is identical to \(.id) on the Pi (\((.created // "?") | sub("T"; " ") | .[0:16]), \(.detail // ""))."' \
  <<<"$index" 2>/dev/null || true

# Upload next to ~/deploy/pending, then swap it into place in one step, so the
# Pi never sees a half-copied deploy. The swap and the reboot run under the
# lock startup.sh takes to claim a pending deploy (see pi_locked), so a Pi
# that's booting right now can't apply it and then reboot into a GitHub sync.
# The swap prints the info of the pending deploy it replaced, if any.
incoming=$REMOTE_DEPLOY_DIR/.incoming-$RANDOM$RANDOM
pi_scp "$stage" "$incoming" || die "couldn't copy the files to the Pi"
[ "$action" = none ] || echo "Staging it and sending the $action command..."
rc=0
replaced=$(pi_locked "test -d $incoming || { echo 'the upload disappeared from the Pi; run deploy.sh again' >&2; exit 1; }
if [ -d $REMOTE_PENDING_DIR ]; then cat $REMOTE_PENDING_DIR/deploy-info.json 2>/dev/null || echo '{}'; fi
rm -rf $REMOTE_PENDING_DIR
mv $incoming $REMOTE_PENDING_DIR" "$action") || rc=$?
# 98: staged, but the reboot/shutdown failed (pi_locked said why).
[ "$rc" -eq 0 ] || [ "$rc" -eq 98 ] || die "couldn't move the deploy into place on the Pi (see above)"
if [ -n "$replaced" ]; then
  old=$(jq -r '.detail // empty' <<<"$replaced" 2>/dev/null) || old=
  echo "Note: replaced a pending deploy that hadn't been applied yet${old:+ ($old)}."
fi
echo "Staged in ~/$REMOTE_PENDING_DIR on the Pi."
[ "$rc" -eq 0 ] || die "the deploy is staged, but the Pi didn't $action (see above). It's applied on the Pi's next boot."

if [ "$action" = none ]; then
  echo "Pi left running; the deploy is applied on its next boot (./reboot.sh)."
else
  echo "Done. The Pi applies the deploy as it boots."
fi
