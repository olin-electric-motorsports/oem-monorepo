#!/usr/bin/env bash
# List the code versions stored on the Pi, or roll back to one.
#
#   ./versions.sh [-c CONFIG] list [-n N]        the N newest (default 10); * = current
#   ./versions.sh [-c CONFIG] show VERSION       details and files of one version
#   ./versions.sh [-c CONFIG] rollback [--no-reboot] VERSION
#
# VERSION is a version id like 20261102-143005, the start of one if only one
# version starts with it (e.g. 20261102-1430), or the start of a version hash
# (at least 4 hex digits). Old-style ids (v12, or 12) still work.
#
# Every code state the Pi runs (a deploy, a GitHub sync, a rollback, or files
# edited on the Pi) is kept as a version on the Pi, up to max_versions in its
# ~/startup/startup_config.json (default 10). A version's id is the date and
# time it was made, in the Pi's time zone: for a deploy, when deploy.sh staged
# it (the laptop's clock); otherwise when the Pi stored it. The Pi has no clock
# battery, so if it boots without network a version's time can be early.
#
# A rollback is staged like a deploy and applied on the next boot; the Pi
# reboots now unless --no-reboot. Like a deploy, it lasts until a boot without
# one syncs with GitHub again (e.g. ./reboot.sh), which replaces it.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg=
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift 2
      ;;
    -h | --help) usage ;;
    --) shift && break ;;
    -*) die "unknown option '$1' (see $(basename "$0") -h)" ;;
    *) break ;;
  esac
done
[ $# -gt 0 ] || die "say what to do: list, show VERSION or rollback VERSION (see $(basename "$0") -h)"
cmd=$1
shift
for arg in "$@"; do [[ $arg != -h && $arg != --help ]] || usage; done

# Check the arguments before looking for the Pi, so mistakes fail fast.
reboot=1 version= remote=()
case $cmd in
  list)
    n=10
    while [ $# -gt 0 ]; do
      case $1 in
        -n) [[ ${2-} =~ ^[1-9][0-9]*$ ]] || die "-n needs a number (at least 1)"; n=$2; shift 2 ;;
        *) die "list: unexpected argument '$1' (see $(basename "$0") -h)" ;;
      esac
    done
    remote=(list -n "$n")
    ;;
  show)
    [ $# -eq 1 ] || die "usage: $(basename "$0") show VERSION"
    remote=(show "$1")
    ;;
  rollback)
    for arg in "$@"; do
      case $arg in
        --no-reboot) reboot= ;;
        -*) die "rollback: unknown option '$arg' (see $(basename "$0") -h)" ;;
        *) [ -z "$version" ] || die "usage: $(basename "$0") rollback [--no-reboot] VERSION"; version=$arg ;;
      esac
    done
    [ -n "$version" ] || die "usage: $(basename "$0") rollback [--no-reboot] VERSION"
    remote=(restore "$version")
    ;;
  *) die "unknown command '$cmd': use list, show or rollback (see $(basename "$0") -h)" ;;
esac

load_config "$config_arg"
find_pi

# Run ~/startup/versions.sh on the Pi. It prints its own errors; pass on its exit code.
run="[ -f $REMOTE_STARTUP_DIR/versions.sh ] || { echo 'error: ~/$REMOTE_STARTUP_DIR/versions.sh is not on the Pi (pi-setup.sh installs it)' >&2; exit 1; }
bash $REMOTE_STARTUP_DIR/versions.sh $(printf '%q ' "${remote[@]}")"
if [ "$cmd" != rollback ]; then
  rc=0
  pi_ssh "$run" || rc=$?
  [ "$rc" -ne "$SSH_CONNECTION_ERROR" ] || die "couldn't run commands on the Pi at $PI_IP (see above)"
  exit "$rc"
fi

# Stage it and reboot under the lock startup.sh takes to claim a pending
# deploy (see pi_locked), like deploy.sh does.
[ -z "$reboot" ] || echo "Staging $version and rebooting the Pi at $PI_IP to apply it..."
rc=0
pi_locked "export OEM_PENDING_LOCKED=1
$run" "${reboot:+reboot}" || rc=$?
[ "$rc" -ne "$SSH_CONNECTION_ERROR" ] || die "couldn't run commands on the Pi at $PI_IP (see above)"
# 98: staged, but the reboot failed (pi_locked said why).
[ "$rc" -ne 98 ] || die "the rollback is staged, but the Pi didn't reboot (see above). It's applied on the Pi's next boot."
[ "$rc" -eq 0 ] || exit "$rc"
if [ -n "$reboot" ]; then
  echo "Reboot command sent."
else
  echo "Pi left running; the rollback is applied on its next boot."
fi
echo "Like a deploy, the rollback lasts until a later boot syncs with GitHub again (any boot without a pending deploy, if the Pi can reach GitHub)."
