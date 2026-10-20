#!/usr/bin/env bash
# Show or change the GitHub branch the Pi syncs its files to on boot.
#
#   ./set-branch.sh [-c CONFIG]               print the Pi's branch
#   ./set-branch.sh [-c CONFIG] [-f] BRANCH   sync to BRANCH from the next boot on
#
# Edits ~/startup/startup_config.json on the Pi. It checks that BRANCH is on
# GitHub first, so push your branch before setting it (-f/--force skips the
# check). Takes effect on the next boot (./reboot.sh).

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg= force= branch=
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift
      ;;
    -f | --force) force=1 ;;
    -h | --help) usage ;;
    -*) die "unknown option '$1' (see $(basename "$0") -h)" ;;
    *)
      [ -z "$branch" ] || die "give one branch (see $(basename "$0") -h)"
      branch=$1
      ;;
  esac
  shift
done

load_config "$config_arg"
if [ -n "$branch" ]; then
  git check-ref-format "refs/heads/$branch" || die "'$branch' isn't a valid branch name"
fi

find_pi
startup_config=$(pi_read "$REMOTE_STARTUP_CONFIG") ||
  die "couldn't read ~/$REMOTE_STARTUP_CONFIG on the Pi (has pi-setup.sh been run?)"
jq -e 'type == "object"' >/dev/null 2>&1 <<<"$startup_config" ||
  die "~/$REMOTE_STARTUP_CONFIG on the Pi is not valid JSON"
current=$(jq -r '.branch // empty' <<<"$startup_config")

if [ -z "$branch" ]; then
  echo "${current:-(no branch set)}"
  exit 0
fi
if [ "$branch" = "$current" ]; then
  echo "The Pi already syncs to '$current'."
  exit 0
fi

if [ -z "$force" ]; then
  repo_url=$(jq -r '.repo_url // empty' <<<"$startup_config")
  [ -n "$repo_url" ] || die "~/$REMOTE_STARTUP_CONFIG on the Pi has no repo_url"
  # --exit-code: 2 means GitHub answered and the branch isn't there. The full
  # ref name stops "main" from also matching e.g. "old/main".
  rc=0
  GIT_TERMINAL_PROMPT=0 timeout 10 git ls-remote --exit-code --heads "$repo_url" "refs/heads/$branch" \
    >/dev/null 2>&1 || rc=$?
  case $rc in
    0) ;;
    2) die "branch '$branch' isn't on $repo_url (push it first, or use --force)" ;;
    *) warn "couldn't reach GitHub to check that '$branch' exists; setting it anyway" ;;
  esac
fi

new_config=$(jq --indent 4 --arg b "$branch" '.branch = $b' <<<"$startup_config")
pi_write "$REMOTE_STARTUP_CONFIG" <<<"$new_config" ||
  die "couldn't write ~/$REMOTE_STARTUP_CONFIG on the Pi"
echo "Branch changed: ${current:-(none)} -> $branch"
echo "Takes effect on the next boot (./reboot.sh)."
