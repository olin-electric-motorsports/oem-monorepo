#!/usr/bin/env bash
# Set up a freshly imaged dashboard Raspberry Pi (Raspberry Pi OS trixie).
#
#   ./pi-setup.sh [-c CONFIG] [-f]
#   ./pi-setup.sh [-c CONFIG] --update
#
# --update only reinstalls ../pi's scripts (startup.sh, sync-keys.sh,
# versions.sh, lib/common.sh) in ~/startup and the systemd service, keeping
# the Pi's startup_config.json (its branch and github_users). Use it after
# changing anything in ../pi. They take effect on the next boot.
#
# Run on the laptop after the Pi's first boot has finished, with the Pi on the
# same network. The Pi is found by pi.mac in config.json; pi.user, pi.port and
# pi.ssh_key are used to log in. You'll be asked for the Pi user's password
# once, for sudo. Takes a few minutes, mostly apt on the Pi.
#
# First, on the laptop: checks that ../pi/startup_config.json has GitHub users
# (github_users) and that one of this laptop's ssh keys is among their keys on
# GitHub. Once set up, the Pi only accepts those keys, so you'd otherwise lock
# yourself out. -f skips the key check.
#
# On the Pi, as root:
#   1. Converts the netplan-backed NetworkManager profiles (the Wi-Fi from
#      Raspberry Pi Imager) into native NetworkManager keyfiles. NM otherwise
#      re-saves them through netplan on every boot, and each save costs ~18 s of
#      `systemctl daemon-reload` (~55 s per boot). Originals: /root/netfix-backup.
#   2. Wraps /usr/sbin/netplan so NM's startup `netplan generate` skips the slow
#      Python CLI + `daemon-reload` when there is no netplan YAML left (~19 s).
#   3. Uninstalls cloud-init, which otherwise runs on every boot before
#      NetworkManager (~16 s).
#   4. Caps the boot's wait for the network at 20 s (NM's default is 60 s), so
#      with no Wi-Fi (in the car) the dashboard starts at most ~20 s late.
#   5. Installs git, jq, curl and patch (the startup scripts use them) and gcc
#      (for a pre_run_command that compiles the dashboard).
#   6. Gives the Pi user passwordless sudo (deploy.sh and reboot.sh run
#      `sudo -n reboot` / `sudo -n shutdown` over ssh, where nobody can type one).
#   7. Installs a boot-time probe that records when the internet first becomes
#      reachable, and `boot-net-times` to print it (diagnostics only).
#   8. Installs ../pi/startup.sh, sync-keys.sh, versions.sh, lib/common.sh and
#      startup_config.json in ~/startup.
#   9. Installs ../pi/oem-startup.service in /etc/systemd/system and enables it.
# Then, from the laptop:
#  10. Runs ~/startup/sync-keys.sh, which replaces the Pi's authorized_keys with
#      the github_users' keys from GitHub, and checks you can still ssh in.
#  11. Runs the first deploy (./deploy.sh --reboot) and exits once the Pi starts
#      rebooting. On that boot, startup.sh applies the deploy and starts the
#      dashboard.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

config_arg= force= update=
while [ $# -gt 0 ]; do
  case $1 in
    -c | --config)
      [ $# -ge 2 ] || die "$1 needs a config file"
      config_arg=$2
      shift
      ;;
    -f | --force) force=1 ;;
    --update) update=1 ;;
    -h | --help) usage ;;
    *) die "unknown argument '$1' (see $(basename "$0") -h)" ;;
  esac
  shift
done

# Timestamped, since this takes a few minutes.
log() { echo "[$(date +%T)] $*" >&2; }

require_cmds ssh scp curl
load_config "$config_arg"
for f in startup.sh sync-keys.sh versions.sh lib/common.sh startup_config.json oem-startup.service; do
  [ -f "$PI_FILES_DIR/$f" ] || die "missing $PI_FILES_DIR/$f"
done

if [ -n "$update" ]; then
  log "looking for the Pi..."
  find_pi
  stage=$(pi_ssh 'mktemp -d /tmp/pi-update.XXXXXX') || die "couldn't run commands on the Pi at $PI_IP"
  pi_scp "$PI_FILES_DIR" "$stage/pi" || die "couldn't copy ../pi to the Pi"
  pi_ssh "bash -s -- $stage" <<'UPDATE_EOF' || die "updating ~/startup failed (see above)"
set -euo pipefail
stage=$1
trap 'rm -rf "$stage"' EXIT
[ -f startup/startup_config.json ] ||
  { echo "error: the Pi has no ~/startup/startup_config.json; run ./pi-setup.sh without --update" >&2; exit 1; }
install -d startup/lib
install -m 755 "$stage"/pi/{startup,sync-keys,versions}.sh startup/
install -m 644 "$stage/pi/lib/common.sh" startup/lib/
rm -f startup/startup.py
echo "[pi] updated startup.sh, sync-keys.sh, versions.sh and lib/common.sh in ~/startup"
sed "s/^User=.*/User=$(id -un)/" "$stage/pi/oem-startup.service" >"$stage/service"
if ! cmp -s "$stage/service" /etc/systemd/system/oem-startup.service; then
  sudo -n install -m 644 "$stage/service" /etc/systemd/system/oem-startup.service
  sudo -n systemctl daemon-reload
  echo "[pi] updated oem-startup.service"
fi
UPDATE_EOF
  log "done; the Pi's startup_config.json is unchanged. The new scripts run from its next boot (./reboot.sh)."
  exit 0
fi

# The Pi will only accept the github_users' GitHub keys (sync-keys.sh), so make
# sure that includes one of yours before going any further.
startup_config=$PI_FILES_DIR/startup_config.json
jq -e 'type == "object"' "$startup_config" >/dev/null 2>&1 || die "$startup_config is not valid JSON"
mapfile -t github_users < <(jq -r '.github_users[]?' "$startup_config")
[ ${#github_users[@]} -gt 0 ] ||
  die "$startup_config has no github_users. Add your GitHub username to it: the Pi only lets in ssh keys from those users' GitHub accounts."
for user in "${github_users[@]}"; do
  valid_github_user "$user" || die "'$user' in github_users ($startup_config) isn't a valid GitHub username"
done
if [ -z "$force" ]; then
  log "checking that one of your ssh keys is on GitHub for: ${github_users[*]}"
  laptop_key_on_github "${github_users[@]}" ||
    die "none of this laptop's ssh keys are on GitHub for ${github_users[*]} (github_users in $startup_config), so once the Pi uses their keys you couldn't ssh into it. Add your public key (e.g. ~/.ssh/id_ed25519.pub) to your GitHub account and your username to github_users (see the docs: Adding users), or use -f."
fi

# ---------------------------------------------------------------- runs on the Pi as root
# Usage: setup.sh STAGE_DIR (holds a copy of ../pi in STAGE_DIR/pi).
read -r -d '' REMOTE <<'REMOTE_EOF' || true # read -d '' "fails" at the end of its input
set -euo pipefail
say() { echo "[pi] $*"; }
trap 'say "FAILED at line $LINENO: $BASH_COMMAND"' ERR
shopt -s nullglob
STAGE=$1
PI_USER=${SUDO_USER:?run this with sudo as the Pi user}
PI_HOME=$(getent passwd "$PI_USER" | cut -d: -f6)
apt_get() { DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=600 -y -q "$@" >/dev/null; }

if command -v cloud-init >/dev/null; then
  # exit code 2 = finished with recoverable errors (normal on this image); only "done" matters
  st=$(cloud-init status 2>/dev/null | awk '/^status:/{print $2}' || true)
  [ "$st" = done ] || { say "first-boot setup (cloud-init) is '$st', not 'done'. Run again once the Pi has finished booting."; exit 1; }
fi
nmcli -t -f RUNNING general >/dev/null 2>&1 || { say "NetworkManager is not running; aborting"; exit 1; }

# --- 1. netplan-backed NM profiles -> native NM keyfiles
yamls=(/etc/netplan/*.yaml)
if [ ${#yamls[@]} -gt 0 ]; then
  profiles=(/run/NetworkManager/system-connections/netplan-*.nmconnection)
  [ ${#profiles[@]} -gt 0 ] || { say "no netplan-generated NM profiles in /run; aborting"; exit 1; }
  BK=/root/netfix-backup; mkdir -p "$BK"; chmod 700 "$BK"
  for f in "${profiles[@]}"; do
    id=$(sed -n 's/^id=//p' "$f" | head -1); new=${id#netplan-}
    dst=/etc/NetworkManager/system-connections/$new.nmconnection
    ( umask 077; sed -e "s/^id=.*/id=$new/" -e "s/^uuid=.*/uuid=$(cat /proc/sys/kernel/random/uuid)/" "$f" > "$dst" )
    cmp -s <(grep -v -e '^id=' -e '^uuid=' "$f") <(grep -v -e '^id=' -e '^uuid=' "$dst") || { say "copy of $f differs; aborting"; exit 1; }
    say "profile '$id' -> $dst"
  done
  mv "${yamls[@]}" "$BK/"
  say "moved ${#yamls[@]} netplan file(s) to $BK"
else
  say "no /etc/netplan/*.yaml; profiles already converted"
fi

# --- 2. skip NM's slow startup `netplan generate` when there is no netplan YAML
if ! dpkg-divert --list /usr/sbin/netplan | grep -q netplan.distrib; then
  dpkg-divert --local --rename --divert /usr/sbin/netplan.distrib --add /usr/sbin/netplan >/dev/null
fi
cat > /usr/sbin/netplan <<'EOF'
#!/bin/sh
# Installed by pi-setup.sh (dpkg-divert; real CLI is /usr/sbin/netplan.distrib).
# NetworkManager runs `netplan generate` on every startup; the Python CLI plus its
# `systemctl daemon-reload` take ~19 s on a Pi Zero. With no netplan YAML, run only
# the C generator. Everything else goes to the real netplan.
if [ "$*" = "generate" ] && ! ls /etc/netplan/*.yaml /run/netplan/*.yaml >/dev/null 2>&1; then
  exec /usr/libexec/netplan/generate
fi
exec /usr/sbin/netplan.distrib "$@"
EOF
chmod 755 /usr/sbin/netplan
/usr/sbin/netplan --help >/dev/null
say "netplan wrapper installed"

# --- 3. uninstall cloud-init (first-boot setup is done; it otherwise runs every boot)
if dpkg -s cloud-init >/dev/null 2>&1; then
  say "uninstalling cloud-init (a minute or two)..."
  apt_get purge cloud-init rpi-cloud-init-mods
  rm -rf /etc/cloud /var/lib/cloud
  say "cloud-init removed"
fi

# --- 4. wait at most 20 s for the network at boot (NM's default is 60 s), so with
#        no Wi-Fi (in the car) the dashboard isn't held up. Applied by the
#        daemon-reloads below.
dropin=/etc/systemd/system/NetworkManager-wait-online.service.d
mkdir -p "$dropin"
cat > "$dropin/oem-timeout.conf" <<'EOF'
# Installed by pi-setup.sh: oem-startup.service waits for network-online.target,
# so don't let a missing Wi-Fi network hold up the dashboard for the default 60 s.
[Service]
Environment=NM_ONLINE_TIMEOUT=20
EOF
unit=$(systemctl show -p FragmentPath --value NetworkManager-wait-online.service)
grep -q NM_ONLINE_TIMEOUT "$unit" 2>/dev/null ||
  say "warning: $unit doesn't use NM_ONLINE_TIMEOUT, so the 20 s cap may not apply"
say "boot waits at most 20 s for the network"

# --- 5. tools for the startup scripts (git: GitHub sync; jq: configs;
#        curl: downloads and GitHub keys; patch: the version store;
#        gcc: compiling the dashboard in pre_run_command)
missing=()
for cmd in git jq curl patch gcc; do
  command -v "$cmd" >/dev/null || missing+=("$cmd")
done
if [ ${#missing[@]} -gt 0 ]; then
  say "installing ${missing[*]} (a minute or two)..."
  apt_get update
  apt_get install "${missing[@]}"
fi
say "git, jq, curl, patch and gcc are installed"

# --- 6. passwordless sudo (deploy.sh, reboot.sh, versions.sh)
rule=/etc/sudoers.d/oem-dashboard-nopasswd
echo "$PI_USER ALL=(ALL) NOPASSWD: ALL" > "$rule.tmp"
chmod 440 "$rule.tmp"
visudo -cqf "$rule.tmp" || { say "generated sudoers rule is invalid; aborting"; rm -f "$rule.tmp"; exit 1; }
mv "$rule.tmp" "$rule"
say "$PI_USER has passwordless sudo"

# --- 7. boot-time internet probe (diagnostics)
cat > /usr/local/sbin/inetprobe.py <<'EOF'
#!/usr/bin/env python3
"""Boot-time internet probe (installed by pi-setup.sh).

Starts alongside NetworkManager. For each check, a new attempt is launched every 200 ms
(attempts overlap) and the CLOCK_MONOTONIC time of the first success is written to
/var/log/inetprobe/<boot_id>.json. CLOCK_MONOTONIC is the clock systemd and
`journalctl -o short-monotonic` use.
"""
import json, os, socket, struct, threading, time

INTERVAL, ATTEMPT_TIMEOUT, GIVE_UP = 0.2, 3.0, 900.0
mono = lambda: time.clock_gettime(time.CLOCK_MONOTONIC)

def checksum(b):
    s = sum(struct.unpack("!%dH" % (len(b) // 2), b))
    s = (s >> 16) + (s & 0xFFFF); s += s >> 16
    return ~s & 0xFFFF

def p_icmp():  # ping 1.1.1.1
    with socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP) as s:
        s.settimeout(ATTEMPT_TIMEOUT)
        ident = (os.getpid() ^ threading.get_ident()) & 0xFFFF
        body = b"probe\0"
        hdr = struct.pack("!BBHHH", 8, 0, 0, ident, 1)
        s.sendto(struct.pack("!BBHHH", 8, 0, checksum(hdr + body), ident, 1) + body, ("1.1.1.1", 0))
        deadline = mono() + ATTEMPT_TIMEOUT
        while mono() < deadline:
            data, addr = s.recvfrom(1024)
            ihl = (data[0] & 0x0F) * 4
            t, _, _, rid, _ = struct.unpack("!BBHHH", data[ihl:ihl + 8])
            if t == 0 and rid == ident and addr[0] == "1.1.1.1":
                return True
    return False

def p_tcp():  # TCP connect to 1.1.1.1:443
    with socket.create_connection(("1.1.1.1", 443), timeout=ATTEMPT_TIMEOUT):
        return True

def p_dns():  # resolve via the DHCP-provided nameservers
    socket.getaddrinfo("example.com", 80, socket.AF_INET)
    return True

def p_http():  # DNS + TCP + HTTP end to end
    with socket.create_connection(("connectivitycheck.gstatic.com", 80), timeout=ATTEMPT_TIMEOUT) as s:
        s.settimeout(ATTEMPT_TIMEOUT)
        s.sendall(b"GET /generate_204 HTTP/1.1\r\nHost: connectivitycheck.gstatic.com\r\nConnection: close\r\n\r\n")
        return s.recv(64).startswith(b"HTTP/1.1 204")

PROBES = {"icmp": p_icmp, "tcp": p_tcp, "dns": p_dns, "http": p_http}
result = {"probe_start": mono(), "attempts": {k: 0 for k in PROBES}}
lock = threading.Lock()
done = {k: threading.Event() for k in PROBES}

def attempt(name, fn):
    try:
        ok = fn()
    except Exception:
        ok = False
    t = mono()
    with lock:
        if ok and not done[name].is_set():
            result[name] = t
            done[name].set()

def driver(name, fn):
    while not done[name].is_set() and mono() - result["probe_start"] < GIVE_UP:
        with lock:
            result["attempts"][name] += 1
        threading.Thread(target=attempt, args=(name, fn), daemon=True).start()
        done[name].wait(INTERVAL)

ts = [threading.Thread(target=driver, args=kv) for kv in PROBES.items()]
for t in ts: t.start()
for t in ts: t.join()
result["boot_id"] = open("/proc/sys/kernel/random/boot_id").read().strip()
os.makedirs("/var/log/inetprobe", exist_ok=True)
with open(f"/var/log/inetprobe/{result['boot_id']}.json", "w") as f:
    json.dump(result, f, indent=1)
EOF
chmod 755 /usr/local/sbin/inetprobe.py
cat > /etc/systemd/system/inetprobe.service <<'EOF'
[Unit]
Description=Boot-time internet reachability probe (pi-setup.sh)
# No After=network-online.target: the probe must start early, before the network.

[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/local/sbin/inetprobe.py

[Install]
WantedBy=multi-user.target
EOF
cat > /usr/local/bin/boot-net-times <<'EOF'
#!/usr/bin/env python3
"""Print this boot's network timeline (seconds since kernel start). Installed by pi-setup.sh."""
import json, subprocess
def unit(u):
    v = subprocess.run(["systemctl", "show", "-p", "ActiveEnterTimestampMonotonic", "--value", u],
                       capture_output=True, text=True).stdout.strip()
    return int(v) / 1e6 if v and v != "0" else None
boot_id = open("/proc/sys/kernel/random/boot_id").read().strip()
print((subprocess.run(["systemd-analyze"], capture_output=True, text=True).stdout or "boot still in progress\n").splitlines()[0])
nm, online = unit("NetworkManager.service"), unit("network-online.target")
print(f"  NetworkManager ready     {nm:7.2f} s after kernel start" if nm else "  NetworkManager not ready")
print(f"  network-online.target    {online:7.2f} s" if online else "  network-online.target not reached")
try:
    d = json.load(open(f"/var/log/inetprobe/{boot_id}.json"))
except FileNotFoundError:
    raise SystemExit("  (internet probe has not finished yet)")
for p, desc in [("icmp", "ping 1.1.1.1"), ("tcp", "TCP 1.1.1.1:443"), ("dns", "DNS lookup"), ("http", "HTTP generate_204")]:
    if p not in d:
        print(f"  {desc:22}   never succeeded")
    elif online:
        print(f"  {desc:22}   {d[p]:7.2f} s   ({(d[p] - online) * 1000:+5.0f} ms vs network-online)")
    else:
        print(f"  {desc:22}   {d[p]:7.2f} s")
EOF
chmod 755 /usr/local/bin/boot-net-times
systemctl daemon-reload
systemctl enable --quiet inetprobe.service
say "internet probe installed (run 'boot-net-times' to see results)"

# --- 8. startup scripts -> ~/startup (owned by the Pi user, who runs them)
own=(-o "$PI_USER" -g "$PI_USER")
install -d "${own[@]}" "$PI_HOME/startup" "$PI_HOME/startup/lib"
for f in startup.sh sync-keys.sh versions.sh; do
  install -m 755 "${own[@]}" "$STAGE/pi/$f" "$PI_HOME/startup/$f"
done
install -m 644 "${own[@]}" "$STAGE/pi/lib/common.sh" "$PI_HOME/startup/lib/common.sh"
install -m 644 "${own[@]}" "$STAGE/pi/startup_config.json" "$PI_HOME/startup/startup_config.json"
# Left by the old Python version: its boot script, and its pending deploys.
rm -rf "$PI_HOME/startup/startup.py" "$PI_HOME/deploy/files" "$PI_HOME/deploy.incoming"
say "installed startup.sh, sync-keys.sh, versions.sh, lib/common.sh and startup_config.json in $PI_HOME/startup"

# --- 9. systemd service, running as the Pi user
sed "s/^User=.*/User=$PI_USER/" "$STAGE/pi/oem-startup.service" > /etc/systemd/system/oem-startup.service
systemctl daemon-reload
systemctl enable --quiet oem-startup.service
say "installed and enabled oem-startup.service (User=$PI_USER)"
REMOTE_EOF

# ---------------------------------------------------------------- main
log "looking for the Pi..."
find_pi
log "found the Pi at $PI_IP"

# A fresh image has a new ssh host key. The scripts store the Pi's key by MAC
# (HOST_KEY_ALIAS in lib/common.sh), so forget the old one.
ssh-keygen -R "$HOST_KEY_ALIAS" >/dev/null 2>&1 || true

stage=$(pi_ssh -o BatchMode=yes 'mktemp -d /tmp/pi-setup.XXXXXX' </dev/null) || die "could not ssh into the Pi at $PI_IP"
pi_scp "$PI_FILES_DIR" "$stage/pi" || die "could not copy ../pi to the Pi"
pi_ssh -o BatchMode=yes "cat > $stage/setup.sh" <<<"$REMOTE" || die "could not upload the setup script"

log "setting up the Pi (sudo will ask for the Pi user's password)"
tflag=-t
[ -t 0 ] || tflag=-tt
pi_ssh -q "$tflag" "sudo -p '[sudo] password for %u on the Pi: ' bash $stage/setup.sh $stage; rc=\$?; rm -rf $stage; exit \$rc" ||
  die "setup failed on the Pi (see above)"

log "replacing the Pi's authorized_keys with the GitHub keys of: ${github_users[*]}"
rc=0
pi_ssh -o BatchMode=yes "$REMOTE_STARTUP_DIR/sync-keys.sh" --strict || rc=$?
# 3: it left the keys as they were (the Pi couldn't get them from GitHub). The
# check below would then only prove the old keys work, and the first boot would
# swap them unchecked, so stop here.
[ "$rc" -ne 3 ] || die "the Pi couldn't get the GitHub keys (see above), so they haven't been checked. Make sure the Pi can reach github.com and run ./pi-setup.sh again."
[ "$rc" -eq 0 ] || die "sync-keys.sh failed on the Pi (see above)"
# A brand-new connection (ControlPath=none), not the shared one opened before
# the keys changed, or this wouldn't test logging in with them.
if ! ssh -o ControlPath=none -o BatchMode=yes "${SSH_OPTS[@]}" "$PI_TARGET" true; then
  log "ERROR: this laptop can't ssh into the Pi any more, now that it only accepts the"
  log "  GitHub keys of: ${github_users[*]}. To get back in, open a shell on the Pi with"
  log "  Raspberry Pi Connect (or a keyboard and screen) and add your public key to"
  log "  ~/.ssh/authorized_keys. Then add that key to your GitHub account (see the docs:"
  log "  Adding users) so the Pi keeps it after the next boot, and run ./pi-setup.sh again."
  exit 1
fi
log "ssh still works with the new keys"

log "first deploy"
PI_IP=$PI_IP "$SCRIPT_DIR/deploy.sh" -c "$CONFIG" --reboot || die "first deploy failed"
log "done. The Pi is rebooting; on this boot it applies the deploy and starts the dashboard."
log "Afterwards: ./ssh.sh journalctl -u oem-startup -b  (logs)  or  ./ssh.sh boot-net-times  (network timing)"
