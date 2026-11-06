#!/usr/bin/env bash
# Rewrite ~/.ssh/authorized_keys with the SSH keys of the GitHub users listed
# in ~/startup/startup_config.json (github_users), from github.com/<user>.keys.
#
#   sync-keys.sh [--strict]
#
# startup.sh runs it on every boot; deploy/users.sh and pi-setup.sh run it
# after editing the list. All or nothing: if GitHub can't be reached, or its
# answer isn't a list of keys, or no keys come back, nobody's keys are
# replaced, so a network blip never locks anyone out. Only the keys of users
# who were taken off the list are dropped (never the last keys). Exits 1 on a
# local error (e.g. the file can't be written), and with --strict, 3 if the
# keys weren't refreshed from GitHub (the laptop scripts use that to tell you).

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

strict=
case ${1-} in
  "") ;;
  --strict) strict=1 ;;
  -h | --help) usage ;;
  *) usage 2 ;;
esac

# unchanged MESSAGE: warn that the keys couldn't be refreshed, drop the keys of
# users no longer in github_users (taking access away needs no network), and
# stop.
unchanged() {
  warn "$*; not refreshing authorized_keys from GitHub"
  drop_removed
  exit "$((strict ? 3 : 0))"
}

# drop_removed: remove the keys tagged github:<user> (and their header line)
# for users who aren't in github_users any more, keeping everything else. Not
# if that would leave no keys at all.
drop_removed() {
  local allowed dropped left new_file
  [ -f "$AUTHORIZED_KEYS" ] || return 0
  allowed=" $(printf '%s ' "${users[@],,}")"
  dropped=$(awk -v allowed="$allowed" '
    NF >= 3 && $NF ~ /^github:/ && !index(allowed, " " tolower(substr($NF, 8)) " ") { print substr($NF, 8) }' \
    "$AUTHORIZED_KEYS" | sort | uniq -c | awk '{ printf "%s%s (%s)", sep, $2, $1; sep = ", " }')
  [ -n "$dropped" ] || return 0
  awk -v allowed="$allowed" '
    NF >= 3 && $NF ~ /^github:/ && !index(allowed, " " tolower(substr($NF, 8)) " ") { next }
    /^# github\.com\/.*\.keys$/ { u = $2; sub(/^github\.com\//, "", u); sub(/\.keys$/, "", u)
      if (!index(allowed, " " tolower(u) " ")) next }
    { print }' "$AUTHORIZED_KEYS" >"$tmp/kept"
  left=$(grep -cv -e '^#' -e '^[[:space:]]*$' "$tmp/kept" || true)
  if [ "$left" -eq 0 ]; then
    warn "Not removing the keys of $dropped: they're the only keys left"
    return 0
  fi
  new_file=$(mktemp "$ssh_dir/.authorized_keys.XXXXXX") || die "can't write in $ssh_dir"
  cp "$tmp/kept" "$new_file" && chmod 600 "$new_file" && mv -f "$new_file" "$AUTHORIZED_KEYS" ||
    { rm -f "$new_file"; die "can't write $AUTHORIZED_KEYS"; }
  log "Removed the keys of users no longer in github_users: $dropped ($left key(s) left)"
}

# OEM_AUTHORIZED_KEYS is for testing somewhere other than the Pi. Running the
# repo's copy on a laptop by mistake would replace the laptop's own file.
AUTHORIZED_KEYS=${OEM_AUTHORIZED_KEYS:-$HOME/.ssh/authorized_keys}
if [ -z "${OEM_AUTHORIZED_KEYS:-}" ] && [ "$STARTUP_DIR" != "$HOME/startup" ]; then
  die "this replaces ~/.ssh/authorized_keys, so it only runs from ~/startup on the Pi (set OEM_AUTHORIZED_KEYS to test it elsewhere)"
fi

tmp=$(mktemp -d) || die "can't create a temp folder"
trap 'rm -rf "$tmp"' EXIT

# One run at a time, from reading the list to writing the file, so a slow
# boot-time run can't undo a users.sh change made while it was fetching.
ssh_dir=$(dirname -- "$AUTHORIZED_KEYS")
mkdir -p -- "$ssh_dir" && chmod 700 -- "$ssh_dir" || die "can't create $ssh_dir"
exec 7>"$ssh_dir/.sync-keys.lock" && flock -w 60 7 || die "another sync-keys.sh has been running for 60 s"

# One jq call: the timeout, then one user per line.
out=$(jq -r '.network_timeout_s // 10, (.github_users // [] | if type == "array" then .[] | strings else empty end)' \
  "$CONFIG" 2>&1) || die "can't read $CONFIG: $out"
mapfile -t users <<<"$out"
timeout_s=${users[0]}
users=("${users[@]:1}")
# 0 would mean "no timeout" to curl.
[[ $timeout_s =~ ^[0-9]+([.][0-9]+)?$ && ! $timeout_s =~ ^0*([.]0*)?$ ]] || timeout_s=10
if [ "${#users[@]}" -eq 0 ]; then
  log "No github_users in $CONFIG; leaving authorized_keys alone"
  exit 0
fi


body="" total=0
declare -A seen=()
summary=()
for user in "${users[@]}"; do
  # GitHub's username rules (also keeps the URL below well-formed).
  if [[ ! $user =~ ^[A-Za-z0-9]([A-Za-z0-9-]{0,38})$ ]]; then
    warn "'$user' isn't a valid GitHub username; skipping it"
    continue
  fi
  [ -z "${seen[${user,,}]:-}" ] || continue
  seen[${user,,}]=1

  rc=0
  code=$(curl -sS --max-time "$timeout_s" -w '%{http_code}' -o "$tmp/keys" \
    "https://github.com/$user.keys" 2>"$tmp/err") || rc=$?
  if [ "$rc" -ne 0 ]; then
    unchanged "Couldn't fetch $user's keys from GitHub ($(tail -n 1 "$tmp/err"))"
  fi
  case $code in
    200) ;;
    404) warn "GitHub user $user not found; skipping"; continue ;;
    *) unchanged "Couldn't fetch $user's keys from GitHub (HTTP $code)" ;;
  esac

  # Keep each key's type and data; the comment becomes github:<user>, which is
  # how users.sh counts each user's keys. Only lines that look like keys, so
  # something that isn't a key list (say, an HTML page) can't replace the keys.
  keys=$(awk -v tag="github:$user" '
    $1 ~ /^(ssh-(ed25519|rsa|dss)|ecdsa-sha2-nistp(256|384|521)|sk-(ssh-ed25519|ecdsa-sha2-nistp256)@openssh\.com)$/ &&
      $2 ~ /^[A-Za-z0-9+\/]+=*$/ { print $1, $2, tag; next }
    NF { bad = 1 }
    END { exit bad }' "$tmp/keys") ||
    unchanged "GitHub's answer for $user isn't a list of SSH keys"
  n=0
  if [ -n "$keys" ]; then mapfile -t lines <<<"$keys"; n=${#lines[@]}; fi
  [ "$n" -gt 0 ] || warn "GitHub user $user has no SSH keys"
  summary+=("$user ($n)")
  if [ "$n" -gt 0 ]; then
    body+=$'\n'"# github.com/$user.keys"$'\n'"$keys"$'\n'
    total=$((total + n))
  fi
done

printf -v list '%s, ' "${summary[@]}"
list=${list%, }
if [ "$total" -eq 0 ]; then
  unchanged "No keys found for any of the github_users${list:+ ($list)}"
fi

new="# Managed by ~/startup/sync-keys.sh: rewritten on every boot from github.com/<user>.keys
# for each user in ~/startup/startup_config.json (github_users). Edit that list with deploy/users.sh.
$body"

if [ -f "$AUTHORIZED_KEYS" ] && [ "$(<"$AUTHORIZED_KEYS")"$'\n' = "$new" ]; then
  log "authorized_keys already up to date: $total key(s) from $list"
  exit 0
fi

# Atomic: a temp file in the same folder, then a rename. sshd ignores the file
# unless only its owner can write to it and to ~/.ssh.
new_file=$(mktemp "$ssh_dir/.authorized_keys.XXXXXX") || die "can't write in $ssh_dir"
trap 'rm -rf "$tmp"; rm -f "$new_file"' EXIT
printf '%s' "$new" >"$new_file" && chmod 600 "$new_file" && mv -f "$new_file" "$AUTHORIZED_KEYS" ||
  die "can't write $AUTHORIZED_KEYS"
log "authorized_keys: $total key(s) from $list"
