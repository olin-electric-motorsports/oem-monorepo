#!/bin/bash
# Set up a freshly imaged dashboard Raspberry Pi (Raspberry Pi OS trixie).
#
#   ./pi-setup.sh [-c config.json]
#
# Run on the laptop after the Pi's first boot has finished, with the Pi on the
# same network. The Pi is found by pi.mac in config.json; pi.user, pi.port and
# pi.ssh_key are used to log in. You'll be asked for the Pi user's password
# once, for sudo. Takes a few minutes, mostly apt on the Pi.
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
#   4. Installs git (startup.py uses it to sync files from GitHub).
#   5. Gives the Pi user passwordless sudo (deploy.py and reboot_pi.py run
#      `sudo -n shutdown` / `sudo -n reboot` over ssh, where nobody can type one).
#   6. Installs a boot-time probe that records when the internet first becomes
#      reachable, and `boot-net-times` to print it (diagnostics only).
#   7. Installs ../pi/startup.py and ../pi/startup_config.json in ~/startup.
#   8. Installs ../pi/oem-startup.service in /etc/systemd/system and enables it.
# Then, from the laptop:
#   9. Runs the first deploy (./deploy.py --reboot) and exits once the Pi starts
#      rebooting. On that boot, startup.py applies the deploy and starts the
#      dashboard.

set -uo pipefail

DIR=$(cd "$(dirname "$0")" && pwd)
PI_DIR=$(cd "$DIR/../pi" && pwd)
CONFIG="$DIR/config.json"

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "$0"; exit "${1:-0}"; }
while getopts "c:h" opt; do
  case $opt in
    c) CONFIG=$OPTARG ;;
    h) usage 0 ;;
    *) usage 2 ;;
  esac
done

log() { echo "[$(date +%T)] $*" >&2; }
die() { log "ERROR: $*"; exit 1; }

for f in startup.py startup_config.json oem-startup.service; do
  [ -f "$PI_DIR/$f" ] || die "missing $PI_DIR/$f"
done

# ---------------------------------------------------------------- runs on the Pi as root
# Usage: setup.sh STAGE_DIR (holds the uploaded startup files and service).
read -r -d '' REMOTE <<'REMOTE_EOF'
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

# --- 4. git, for startup.py's GitHub sync
if ! command -v git >/dev/null; then
  say "installing git (a minute or two)..."
  apt_get update
  apt_get install git
fi
say "git: $(git --version)"

# --- 5. passwordless sudo (deploy.py, reboot_pi.py)
rule=/etc/sudoers.d/oem-dashboard-nopasswd
echo "$PI_USER ALL=(ALL) NOPASSWD: ALL" > "$rule.tmp"
chmod 440 "$rule.tmp"
visudo -cqf "$rule.tmp" || { say "generated sudoers rule is invalid; aborting"; rm -f "$rule.tmp"; exit 1; }
mv "$rule.tmp" "$rule"
say "$PI_USER has passwordless sudo"

# --- 6. boot-time internet probe (diagnostics)
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

# --- 7. startup files -> ~/startup (owned by the Pi user, who runs them)
install -d -o "$PI_USER" -g "$PI_USER" "$PI_HOME/startup"
install -m 755 -o "$PI_USER" -g "$PI_USER" "$STAGE/startup.py" "$PI_HOME/startup/startup.py"
install -m 644 -o "$PI_USER" -g "$PI_USER" "$STAGE/startup_config.json" "$PI_HOME/startup/startup_config.json"
say "installed $PI_HOME/startup/startup.py and startup_config.json"

# --- 8. systemd service, running as the Pi user
sed "s/^User=.*/User=$PI_USER/" "$STAGE/oem-startup.service" > /etc/systemd/system/oem-startup.service
systemctl daemon-reload
systemctl enable --quiet oem-startup.service
say "installed and enabled oem-startup.service (User=$PI_USER)"
REMOTE_EOF

# ---------------------------------------------------------------- main
log "looking for the Pi..."
env=$(python3 "$DIR/pi_ssh.py" shell-env -c "$CONFIG") || exit 1
eval "$env"
log "found the Pi at $PI_IP"

# A fresh image has a new ssh host key. The scripts store the Pi's key by MAC
# (see host_key_alias in pi_ssh.py), so forget the old one.
ssh-keygen -R "$HOST_KEY_ALIAS" >/dev/null 2>&1 || true

pissh() { ssh "${SSH_OPTS[@]}" -o BatchMode=yes "$PI_TARGET" "$@"; }

stage=$(pissh 'mktemp -d /tmp/pi-setup.XXXXXX' </dev/null) || die "could not ssh into the Pi at $PI_IP"
scp -q "${SCP_OPTS[@]}" -o BatchMode=yes \
  "$PI_DIR/startup.py" "$PI_DIR/startup_config.json" "$PI_DIR/oem-startup.service" \
  "$PI_TARGET:$stage/" || die "could not copy the startup files to the Pi"
pissh "cat > $stage/setup.sh" <<<"$REMOTE" || die "could not upload the setup script"

log "setting up the Pi (sudo will ask for the Pi user's password)"
tflag=-t; [ -t 0 ] || tflag=-tt
ssh $tflag "${SSH_OPTS[@]}" "$PI_TARGET" \
  "sudo -p '[sudo] password for %u on the Pi: ' bash $stage/setup.sh $stage; rc=\$?; rm -rf $stage; exit \$rc" \
  || die "setup failed on the Pi (see above)"

log "first deploy"
"$DIR/deploy.py" -c "$CONFIG" --reboot || die "first deploy failed"
log "done. The Pi is rebooting; on this boot it applies the deploy and starts the dashboard."
log "Afterwards: ssh $PI_TARGET 'journalctl -u oem-startup -b'  (logs)  or  'boot-net-times'  (network timing)"
