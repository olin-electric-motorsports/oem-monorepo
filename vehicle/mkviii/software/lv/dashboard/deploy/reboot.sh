#!/usr/bin/env bash
# Reboot the Pi, so it syncs its files with its GitHub branch (./set-branch.sh).
#
#   ./reboot.sh [-c CONFIG] [--discard-deploy]
#
# If a deploy or rollback is still pending on the Pi (e.g. after
# ./deploy.sh --no-reboot), this boot applies it INSTEAD of syncing with
# GitHub. --discard-deploy deletes it first, so the Pi syncs with GitHub.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg= discard=
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift
      ;;
    --discard-deploy) discard=1 ;;
    -h | --help) usage ;;
    *) die "unknown argument '$1' (see $(basename "$0") -h)" ;;
  esac
  shift
done

load_config "$config_arg"
find_pi

# Prints the pending deploy's deploy-info.json ({} if it has none), or nothing
# if no deploy is pending.
show_pending="if [ -d $REMOTE_PENDING_DIR ]; then cat $REMOTE_PENDING_DIR/deploy-info.json 2>/dev/null || echo '{}'; fi"

if [ -n "$discard" ]; then
  # Delete it and reboot under the lock startup.sh takes to claim a pending
  # deploy, so a Pi that's booting right now can't apply it in between.
  echo "Rebooting the Pi at $PI_IP, without the pending deploy..."
  rc=0
  pending=$(pi_locked "$show_pending
rm -rf $REMOTE_PENDING_DIR" reboot) || rc=$?
  # 98: discarded, but the reboot failed (pi_locked said why).
  [ "$rc" -eq 0 ] || [ "$rc" -eq 98 ] || die "couldn't discard the pending deploy (see above)"
  detail=$(jq -r '.detail // empty' <<<"$pending" 2>/dev/null) || detail=
  if [ -n "$pending" ]; then echo "Discarded the pending deploy${detail:+ ($detail)}."; else echo "No deploy was pending."; fi
  [ "$rc" -eq 0 ] || die "the Pi didn't reboot (see above)"
  echo "Reboot command sent."
  exit 0
fi

pending=$(pi_ssh "$show_pending") || die "couldn't run commands on the Pi at $PI_IP (see above)"
if [ -n "$pending" ]; then
  detail=$(jq -r '.detail // empty' <<<"$pending" 2>/dev/null) || detail=
  warn "a deploy is pending on the Pi${detail:+ ($detail)}. This boot applies it instead of syncing with GitHub (use --discard-deploy to skip it)."
fi
echo "Rebooting the Pi at $PI_IP..."
pi_power reboot
echo "Reboot command sent."
