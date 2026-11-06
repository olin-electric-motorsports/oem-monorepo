#!/usr/bin/env bash
# ssh into the dashboard Pi, found by its MAC address like the other scripts.
#
#   ./ssh.sh [-c CONFIG]               open a shell on the Pi
#   ./ssh.sh [-c CONFIG] COMMAND...    run a command on it, e.g.
#                                      ./ssh.sh journalctl -u oem-startup -b
#   ./ssh.sh [-c CONFIG] --ip          print its IP address (for scp, VS Code, ...)

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg= ip_only=
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift 2
      ;;
    --ip) ip_only=1 && shift ;;
    -h | --help) usage ;;
    --) shift && break ;;
    -*) die "unknown option '$1' (see $(basename "$0") -h)" ;;
    *) break ;;
  esac
done

load_config "$config_arg"
find_pi
if [ -n "$ip_only" ]; then
  [ $# -eq 0 ] || die "--ip doesn't take a command"
  echo "$PI_IP"
  exit 0
fi
exec ssh "${SSH_OPTS[@]}" "$PI_TARGET" "$@"
