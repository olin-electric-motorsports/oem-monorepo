#!/usr/bin/env bash
# List, add or remove the GitHub users whose ssh keys can log into the Pi.
#
#   ./users.sh [-c CONFIG] COMMAND
#
#   list                  the users, and how many keys the Pi has for each
#   add [-f] USER...      let USER ssh in with their GitHub keys
#   remove [-f] USER...   stop letting USER in
#   sync                  fetch everyone's keys from GitHub again now
#
# The users are github_users in the Pi's ~/startup/startup_config.json. On every
# boot the Pi rewrites ~/.ssh/authorized_keys with their keys from
# github.com/<user>.keys; add, remove and sync do that straight away too.
# add checks that USER has keys on GitHub, and remove that you won't lock
# yourself out (one of this laptop's keys must belong to a remaining user).
# -f skips those checks. After any change, a fresh ssh login is tried; if this
# laptop can't get in any more, the change is undone.
#
# pi-setup.sh installs the repo's ../pi/startup_config.json instead, so put
# permanent team members in its github_users too.

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
    -*) die "unknown option '$1' (see $(basename "$0") -h)" ;;
    *) break ;;
  esac
done
[ $# -gt 0 ] || usage 2
cmd=$1
shift

force= names=()
while [ $# -gt 0 ]; do
  case $1 in
    -f | --force) force=1 ;;
    -h | --help) usage ;;
    -*) die "unknown option '$1' (see $(basename "$0") -h)" ;;
    *) names+=("$1") ;;
  esac
  shift
done

case $cmd in
  list | sync) [ ${#names[@]} -eq 0 ] && [ -z "$force" ] || die "'$cmd' takes no arguments" ;;
  add | remove) [ ${#names[@]} -gt 0 ] || die "$cmd which GitHub user(s)? (see $(basename "$0") -h)" ;;
  *) die "unknown command '$cmd' (list, add, remove or sync; see $(basename "$0") -h)" ;;
esac

REMINDER="Note: pi-setup.sh installs the repo's pi/startup_config.json, so make lasting changes to its github_users too."

# in_list NAME ITEM...: whether NAME is one of the ITEMs, ignoring case (like GitHub).
in_list() {
  local name=${1,,} item
  shift
  for item in "$@"; do
    if [ "${item,,}" = "$name" ]; then return 0; fi
  done
  return 1
}

# Read the Pi's startup_config.json into STARTUP_CONFIG, and its github_users
# into USERS. Also checks the Pi's scripts know about this version of users.sh.
read_users() {
  pi_ssh "grep -q -e --strict $REMOTE_STARTUP_DIR/sync-keys.sh 2>/dev/null" ||
    die "the scripts on the Pi are older than these (or missing). Update them with ./pi-setup.sh --update"
  STARTUP_CONFIG=$(pi_read "$REMOTE_STARTUP_CONFIG") ||
    die "couldn't read ~/$REMOTE_STARTUP_CONFIG on the Pi (has pi-setup.sh been run?)"
  jq -e 'type == "object"' >/dev/null 2>&1 <<<"$STARTUP_CONFIG" ||
    die "~/$REMOTE_STARTUP_CONFIG on the Pi is not valid JSON"
  mapfile -t USERS < <(jq -r '.github_users[]?' <<<"$STARTUP_CONFIG")
}

# write_users USER...: make USERs the Pi's github_users.
write_users() {
  local new_config
  new_config=$(jq --indent 4 '.github_users = $ARGS.positional' --args -- "$@" <<<"$STARTUP_CONFIG")
  pi_write "$REMOTE_STARTUP_CONFIG" <<<"$new_config" ||
    die "couldn't write ~/$REMOTE_STARTUP_CONFIG on the Pi"
}

BACKUP=.ssh/authorized_keys.users-sh-backup

# sync_keys: rewrite the Pi's authorized_keys from GitHub now, then make sure
# this laptop can still log in (see check_login). STARTUP_CONFIG (from
# read_users) is what the list goes back to if it can't.
sync_keys() {
  local rc=0
  pi_ssh "cp -p .ssh/authorized_keys $BACKUP 2>/dev/null || true" || die "couldn't run commands on the Pi"
  echo "Updating the Pi's authorized_keys from GitHub:"
  pi_run "$REMOTE_STARTUP_DIR/sync-keys.sh" --strict || rc=$?
  case $rc in
    0) ;;
    3) warn "the Pi couldn't get the keys from GitHub just now (see above). The list is saved, and the Pi updates its keys on its next boot (or run ./users.sh sync to try again)." ;;
    *) die "sync-keys.sh failed on the Pi (see above)" ;;
  esac
  check_login
}

# check_login: try a brand-new ssh login (not the shared connection, which was
# opened with the old keys). If this laptop can't get in any more, put the old
# list and keys back through the shared connection, which is still open.
check_login() {
  if ssh -o ControlPath=none -o BatchMode=yes "${SSH_OPTS[@]}" "$PI_TARGET" true 2>/dev/null; then
    pi_ssh "rm -f $BACKUP" || true
    return
  fi
  warn "after that, this laptop can't log into the Pi any more; undoing it"
  if pi_write "$REMOTE_STARTUP_CONFIG" <<<"$STARTUP_CONFIG" &&
    pi_ssh "if [ -f $BACKUP ]; then mv -f $BACKUP .ssh/authorized_keys; fi"; then
    die "undone: github_users and authorized_keys are back as they were. One of this laptop's keys has to belong to a GitHub user on the list (see the docs: Adding users)."
  fi
  die "couldn't undo it either. Open a shell on the Pi with Raspberry Pi Connect and fix ~/.ssh/authorized_keys and github_users in ~/$REMOTE_STARTUP_CONFIG."
}

list_users() {
  local keys user count others
  find_pi
  read_users
  if [ ${#USERS[@]} -eq 0 ]; then
    echo "The Pi has no github_users, so it leaves ~/.ssh/authorized_keys as it is."
    return
  fi
  keys=$(pi_ssh "cat .ssh/authorized_keys 2>/dev/null || true") ||
    die "couldn't read ~/.ssh/authorized_keys on the Pi"
  echo "GitHub users who can ssh into the Pi (keys in its authorized_keys):"
  for user in "${USERS[@]}"; do
    # sync-keys.sh ends each key's line with " github:<user>".
    count=$(awk -v tag=" github:$user" \
      'length($0) > length(tag) && substr($0, length($0) - length(tag) + 1) == tag {n++} END {print n + 0}' <<<"$keys")
    printf '  %-20s %s key(s)\n' "$user" "$count"
  done
  # Keys still tagged for someone who's off the list (removed while the Pi
  # couldn't reach GitHub, and they were its only keys).
  for user in $(awk '!/^#/ && $NF ~ /^github:/ { print substr($NF, 8) }' <<<"$keys" | sort -u); do
    in_list "$user" "${USERS[@]}" ||
      echo "  $user: $(grep -c " github:$user\$" <<<"$keys") key(s) still installed, but not on the list (dropped at the next sync)"
  done
  others=$(awk 'NF && !/^#/ && $NF !~ /^github:/' <<<"$keys" | wc -l)
  if [ "$others" -gt 0 ]; then
    echo "  plus $others key(s) not from GitHub (the next successful sync replaces them)"
  fi
}

add_users() {
  local name keys rc new=()
  for name in "${names[@]}"; do
    valid_github_user "$name" || die "'$name' isn't a valid GitHub username"
  done
  # Check GitHub from the laptop before touching the Pi.
  for name in "${names[@]}"; do
    rc=0
    keys=$(github_keys "$name") || rc=$?
    case $rc in
      0)
        if [ -z "$keys" ]; then
          [ -n "$force" ] || die "$name has no SSH keys on GitHub, so they couldn't log in yet. They need to add one first (see the docs: Adding users), or use -f to add them anyway."
          warn "$name has no SSH keys on GitHub yet"
        fi
        ;;
      2) die "no GitHub user '$name'" ;;
      *)
        [ -n "$force" ] || die "couldn't reach GitHub to check '$name' (-f adds them anyway)"
        warn "couldn't reach GitHub to check '$name'"
        ;;
    esac
  done

  find_pi
  read_users
  new=("${USERS[@]}")
  for name in "${names[@]}"; do
    if in_list "$name" "${new[@]}"; then
      echo "$name is already a user."
    else
      new+=("$name")
    fi
  done
  if [ ${#new[@]} -eq ${#USERS[@]} ]; then
    echo "Nothing to change."
    return
  fi
  write_users "${new[@]}"
  echo "github_users is now: ${new[*]}"
  sync_keys
  echo "$REMINDER"
}

remove_users() {
  local name user keep=()
  find_pi
  read_users
  for name in "${names[@]}"; do
    in_list "$name" "${USERS[@]}" || die "$name isn't one of the Pi's GitHub users (see ./users.sh list)"
  done
  for user in "${USERS[@]}"; do
    in_list "$user" "${names[@]}" || keep+=("$user")
  done
  # With no users, sync-keys.sh leaves authorized_keys alone, so nobody would lose access.
  [ ${#keep[@]} -gt 0 ] ||
    die "can't remove every user: with none, the Pi keeps its current keys forever. Add someone else first."
  if [ -z "$force" ] && ! laptop_key_on_github "${keep[@]}"; then
    die "none of this laptop's ssh keys belong to ${keep[*]} on GitHub, so after this you couldn't ssh into the Pi. Add your key to your GitHub account and yourself to the users first (see the docs: Adding users), or use -f if you're sure."
  fi
  write_users "${keep[@]}"
  echo "github_users is now: ${keep[*]}"
  sync_keys
  echo "$REMINDER"
}

load_config "$config_arg"
case $cmd in
  list) list_users ;;
  add) add_users ;;
  remove) remove_users ;;
  sync)
    find_pi
    read_users
    sync_keys
    ;;
esac
