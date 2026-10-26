# shellcheck shell=bash
# Helpers shared by the laptop-side scripts in deploy/. Sourced, not run:
#
#   source "$(dirname "$0")/lib/common.sh"
#   load_config "$CONFIG"     # read config.json
#   find_pi                   # set PI_IP by looking up pi.mac on the local network
#   pi_ssh 'some command'     # run a command on the Pi
#
# Call find_pi once near the start, outside any $( ... ). The pi_* helpers
# call it too, but an address found inside a subshell isn't remembered.
#
# Needs bash, jq, ssh, scp, ip and ping (Linux).

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)   # deploy/
PI_FILES_DIR=$(cd "$SCRIPT_DIR/../pi" && pwd)                 # pi/, what gets installed on the Pi
DEFAULT_CONFIG=$SCRIPT_DIR/config.json

# Paths on the Pi, relative to the Pi user's home (where ssh commands start).
# They must match pi/lib/common.sh.
REMOTE_DEPLOY_DIR=deploy
REMOTE_PENDING_DIR=deploy/pending
REMOTE_PENDING_LOCK=deploy/.pending.lock
REMOTE_MANIFEST=deploy/deploy-manifest.json
REMOTE_VERSIONS_INDEX=deploy/versions/index.json
REMOTE_STARTUP_DIR=startup
REMOTE_STARTUP_CONFIG=startup/startup_config.json

# ssh exits with 255 when the connection itself fails, or is dropped because
# the Pi started shutting down before the command returned.
SSH_CONNECTION_ERROR=255

# Don't ping-sweep subnets bigger than this when looking for the Pi.
MAX_SWEEP_HOSTS=4096

log()  { echo "$*" >&2; }
warn() { echo "warning: $*" >&2; }
die()  { echo "error: $*" >&2; exit 1; }

# Print the comment block at the top of the calling script (its usage text).
usage() {
  sed -n '2,/^[^#]/{/^#/!q;s/^# \{0,1\}//p}' "$0"
  exit "${1:-0}"
}

require_cmds() {
  local cmd
  for cmd in "$@"; do
    command -v "$cmd" >/dev/null || die "'$cmd' is not installed (sudo apt install $cmd)"
  done
}

# 'B8-27-EB-5-C6-4F' -> 'b8:27:eb:05:c6:4f'. Fails if it isn't a MAC address.
normalize_mac() {
  local IFS=':-' part out=() parts
  read -r -a parts <<<"${1,,}"
  [ "${#parts[@]}" -eq 6 ] || return 1
  for part in "${parts[@]}"; do
    [[ $part =~ ^[0-9a-f]{1,2}$ ]] || return 1
    out+=("$(printf '%02x' "0x$part")")
  done
  (IFS=:; echo "${out[*]}")
}

# load_config [PATH]: read config.json into PI_* variables and build SSH_OPTS /
# SCP_OPTS. Relative local paths in the deploy list are resolved against
# CONFIG_DIR, the folder the config is in.
load_config() {
  require_cmds jq
  CONFIG=$(realpath -- "${1:-$DEFAULT_CONFIG}") || die "config file not found: ${1:-$DEFAULT_CONFIG}"
  [ -f "$CONFIG" ] || die "config file not found: $CONFIG"
  CONFIG_DIR=$(dirname "$CONFIG")
  jq empty "$CONFIG" 2>/dev/null || die "$CONFIG is not valid JSON"

  local mac
  mac=$(jq -r '.pi.mac // empty' "$CONFIG")
  [ -n "$mac" ] || die "$CONFIG is missing pi.mac"
  PI_MAC=$(normalize_mac "$mac") || die "pi.mac in $CONFIG is not a MAC address: $mac"
  PI_USER=$(jq -r '.pi.user // empty' "$CONFIG")
  [ -n "$PI_USER" ] || die "$CONFIG is missing pi.user"
  PI_PORT=$(jq -r '.pi.port // 22' "$CONFIG")
  PI_SSH_KEY=$(jq -r '.pi.ssh_key // empty' "$CONFIG")
  PI_SSH_KEY=${PI_SSH_KEY/#\~/$HOME}
  CONNECT_TIMEOUT=$(jq -r '.pi.connect_timeout_s // 10' "$CONFIG")
  DISCOVERY_TIMEOUT=$(jq -r '.pi.discovery_timeout_s // 10' "$CONFIG")
  POST_DEPLOY_ACTION=$(jq -r '.post_deploy_action // "reboot"' "$CONFIG")
  local key value
  for key in port connect_timeout_s discovery_timeout_s; do
    value=$(jq -r --arg k "$key" '.pi[$k] // empty' "$CONFIG")
    [ -z "$value" ] || [[ $value =~ ^[1-9][0-9]*$ ]] ||
      die "pi.$key in $CONFIG must be a whole number (1 or more), not $value"
  done

  # ssh stores the Pi's host key under a name based on its MAC, not its IP,
  # because the IP changes. pi-setup.sh forgets it after re-imaging.
  HOST_KEY_ALIAS="oem-dashboard-pi-${PI_MAC//:/}"
  # Each new ssh connection to the Pi Zero takes 1-1.5 s, so reuse one: the
  # first ssh/scp opens a shared connection that later ones ride on (~0.1 s
  # each) and that closes itself 30 s after the last use.
  local common=(
    -o "ConnectTimeout=$CONNECT_TIMEOUT"
    -o "HostKeyAlias=$HOST_KEY_ALIAS"
    -o StrictHostKeyChecking=accept-new
    -o ControlMaster=auto
    -o "ControlPath=${XDG_RUNTIME_DIR:-/tmp}/oem-pi-ssh-%C"
    -o ControlPersist=30
  )
  [ -n "$PI_SSH_KEY" ] && common+=(-i "$PI_SSH_KEY")
  SSH_OPTS=("${common[@]}" -p "$PI_PORT")
  SCP_OPTS=("${common[@]}" -P "$PI_PORT")
}

# The IPs the neighbor (ARP) table has for a MAC, recently confirmed ones
# first (an old lease can linger as STALE next to the Pi's current address).
neighbor_ips() {
  ip -4 neigh show 2>/dev/null |
    awk -v mac="$1" '$0 !~ /FAILED|INCOMPLETE/ {
      for (i = 1; i < NF; i++) if ($i == "lladdr" && tolower($(i + 1)) == mac) print ($NF == "REACHABLE" ? 0 : 1), $1
    }' | sort -n | cut -d' ' -f2
}

# reachable_pi_ip: the first of the MAC's IPs that answers on the ssh port.
reachable_pi_ip() {
  local ip
  for ip in $(neighbor_ips "$PI_MAC"); do
    if port_open "$ip" "$PI_PORT"; then echo "$ip"; return; fi
  done
  return 1
}

port_open() {
  timeout 3 bash -c 'exec 3<>"/dev/tcp/$1/$2"' _ "$1" "$2" 2>/dev/null
}

ip_to_int() {
  local IFS=. a b c d
  read -r a b c d <<<"$1"
  echo $(((a << 24) | (b << 16) | (c << 8) | d))
}

int_to_ip() {
  echo "$((($1 >> 24) & 255)).$((($1 >> 16) & 255)).$((($1 >> 8) & 255)).$(($1 & 255))"
}

# Every host address on the laptop's directly connected IPv4 networks.
local_hosts() {
  local cidr addr len size net i
  for cidr in $(ip -4 -o addr show scope global | awk '{print $4}'); do
    addr=${cidr%/*} len=${cidr#*/}
    size=$((1 << (32 - len)))
    if [ "$size" -gt "$MAX_SWEEP_HOSTS" ] || [ "$size" -lt 4 ]; then
      continue
    fi
    net=$(($(ip_to_int "$addr") & ~(size - 1) & 0xFFFFFFFF))
    for ((i = 1; i < size - 1; i++)); do
      int_to_ip $((net + i))
    done
  done
}

# sweep SECONDS: ping every address on the local networks so the kernel ARPs
# each one and the Pi shows up in the neighbor table. Works without root
# (arp-scan doesn't). 512 at a time, about 1 s each, so a /23 takes ~1 s and
# the largest network swept (/20) ~8 s; stops after SECONDS either way.
sweep() {
  local_hosts | timeout "$1" xargs -r -P 512 -I{} ping -c 1 -W 1 -q {} >/dev/null 2>&1 || true
}

# find_pi: set PI_IP (and PI_TARGET) to the Pi's current address by looking
# up pi.mac in the neighbor table, ping-sweeping the network if it isn't there
# or the address is stale. PI_IP=... in the environment skips the search.
find_pi() {
  if [ -n "${PI_IP:-}" ]; then
    PI_TARGET="$PI_USER@$PI_IP"
    return
  fi
  require_cmds ip ping
  local ip deadline=$((SECONDS + DISCOVERY_TIMEOUT))
  while true; do
    if ! ip=$(reachable_pi_ip); then
      sweep $((deadline - SECONDS > 1 ? deadline - SECONDS : 1))
      ip=$(reachable_pi_ip) || ip=
    fi
    if [ -n "$ip" ]; then
      PI_IP=$ip
      PI_TARGET="$PI_USER@$PI_IP"
      return
    fi
    if [ "$SECONDS" -ge "$deadline" ]; then
      die "couldn't find the Pi (MAC $PI_MAC) on the local network within $DISCOVERY_TIMEOUT s. Is it on (first boot takes minutes), and on the same network as this laptop? Networks bigger than /20 aren't searched. If you know its IP: PI_IP=<ip> $(basename "$0") ..."
    fi
    sleep 1
  done
}

# pi_ssh [SSH_FLAGS...] COMMAND: run a shell command (one string) on the Pi.
pi_ssh() {
  find_pi
  ssh "${SSH_OPTS[@]}" "$PI_TARGET" "$@"
}

# pi_run ARG...: run a command on the Pi, with each argument shell-quoted.
pi_run() {
  pi_ssh "$(printf '%q ' "$@")"
}

# pi_read PATH: print a file from the Pi. pi_write PATH: replace a file on the
# Pi with stdin, atomically (temp file, then rename).
pi_read() {
  pi_run cat "$1"
}

pi_write() {
  local q
  q=$(printf '%q' "$1")
  pi_ssh "cat > $q.tmp && mv $q.tmp $q"
}

# pi_scp LOCAL... REMOTE: copy files or folders (recursively) to the Pi.
pi_scp() {
  find_pi
  local dest=${*: -1}
  scp -q -r "${SCP_OPTS[@]}" "${@:1:$#-1}" "$PI_TARGET:$dest"
}

pi_has_pending_deploy() {
  pi_run test -d "$REMOTE_PENDING_DIR"
}

# pi_locked COMMANDS [reboot|shutdown|none]: run shell COMMANDS on the Pi
# (from its home folder) while holding the lock startup.sh takes to claim
# ~/deploy/pending, then reboot or shut down while still holding it. So the
# Pi can't apply a deploy halfway through a change to pending/, or apply it
# and then reboot into a GitHub sync that overwrites it. COMMANDS run under
# set -e. Prints their output. Returns non-zero if they fail, and 98 if they
# worked but the reboot/shutdown didn't (it has already said why).
pi_locked() {
  local marker=__oem_locked_done__ remote out rc=0
  remote="set -e
mkdir -p $REMOTE_DEPLOY_DIR
exec 8>$REMOTE_PENDING_LOCK
flock -w 30 8 || { echo 'the Pi was busy applying a deploy for 30 s; try again' >&2; exit 97; }
$1
echo $marker"
  case ${2:-none} in
    reboot) remote+=$'\nsudo -n reboot\nsleep 60' ;;
    shutdown) remote+=$'\nsudo -n shutdown -h now\nsleep 60' ;;
  esac
  out=$(pi_ssh "$remote") || rc=$?
  if [[ $out != *$marker* ]]; then
    [ -z "$out" ] || printf '%s\n' "$out"
    return "$((rc ? rc : 1))"
  fi
  out=${out%%"$marker"*}
  [ -z "$out" ] || printf '%s' "$out"
  # The lock stays held until the Pi goes down and drops the connection (255).
  if [ "$rc" -ne 0 ] && [ "$rc" -ne "$SSH_CONNECTION_ERROR" ]; then
    warn "the ${2:-} command failed on the Pi (exit $rc). Does $PI_USER have passwordless sudo? (pi-setup.sh sets that up.)"
    return 98
  fi
}

# pi_power reboot|shutdown. Tolerates the connection dropping as the Pi goes down.
pi_power() {
  local cmd rc=0
  case $1 in
    reboot) cmd="sudo -n reboot" ;;
    shutdown) cmd="sudo -n shutdown -h now" ;;
    *) die "unknown power action '$1'" ;;
  esac
  pi_ssh "$cmd" || rc=$?
  if [ "$rc" -ne 0 ] && [ "$rc" -ne "$SSH_CONNECTION_ERROR" ]; then
    die "'$cmd' failed on the Pi (exit $rc). Does $PI_USER have passwordless sudo? (pi-setup.sh sets that up.)"
  fi
}

# version_hash MANIFEST FILES_DIR: the hash that identifies a version.
# MANIFEST is a JSON list of {file_name, file_path, repo_path}; each file's
# content is FILES_DIR/<file_name>. It's the sha256 of one line per file,
# "<sha256 of content>  <file_path>  <repo_path>", sorted. Must match
# version_hash in pi/lib/common.sh.
version_hash() {
  local manifest=$1 dir=$2 name path repo sum
  jq -r '.[] | [.file_name, .file_path, (.repo_path // "")] | @tsv' "$manifest" |
    while IFS=$'\t' read -r name path repo; do
      sum=$(sha256sum <"$dir/$name") || exit 1
      printf '%s  %s  %s\n' "${sum%% *}" "$path" "$repo"
    done | LC_ALL=C sort | sha256sum | cut -c1-64
}

# This laptop's public keys ("type base64" per line), for lockout checks.
local_public_keys() {
  {
    ssh-add -L 2>/dev/null || true
    cat "$HOME"/.ssh/*.pub 2>/dev/null || true
    [ -n "${PI_SSH_KEY:-}" ] && cat "$PI_SSH_KEY.pub" 2>/dev/null || true
  } | awk '$1 ~ /^(ssh-|ecdsa-|sk-)/ && NF >= 2 {print $1, $2}' | sort -u
}

# github_keys USER: print USER's public keys from GitHub ("type base64" per
# line). Exit 2 if the user doesn't exist, 1 if GitHub couldn't be reached.
github_keys() {
  local out code
  require_cmds curl
  out=$(mktemp)
  code=$(curl -sS --max-time 10 -o "$out" -w '%{http_code}' "https://github.com/$1.keys" 2>/dev/null) || code=000
  case $code in
    200) awk 'NF >= 2 {print $1, $2}' "$out"; rm -f "$out" ;;
    404) rm -f "$out"; return 2 ;;
    *) rm -f "$out"; return 1 ;;
  esac
}

# valid_github_user NAME: GitHub's username rules (letters, digits, hyphens).
valid_github_user() {
  [[ $1 =~ ^[A-Za-z0-9]([A-Za-z0-9-]{0,38})$ ]]
}

# laptop_key_on_github USER...: whether one of this laptop's ssh keys is among
# USERs' keys on GitHub, i.e. whether you could still log in once the Pi only
# accepts their keys.
laptop_key_on_github() {
  local mine theirs user rc
  mine=$(local_public_keys)
  [ -n "$mine" ] || return 1
  for user in "$@"; do
    rc=0
    theirs=$(github_keys "$user") || rc=$?
    case $rc in
      0) if [ -n "$theirs" ] && grep -qxFf <(printf '%s\n' "$theirs") <<<"$mine"; then return 0; fi ;;
      2) warn "there's no GitHub user '$user' (a typo?)" ;;
      *) die "couldn't fetch $user's keys from GitHub to check you won't lock yourself out (-f skips the check)" ;;
    esac
  done
  return 1
}
