"""
Shared helpers for the laptop-side deploy scripts (deploy.py, set_branch.py,
reboot_pi.py, pi-setup.sh). Loads config.json, finds the Raspberry Pi on the
local network by its MAC address, and wraps ssh/scp calls to it.

Run directly, it prints shell variables for pi-setup.sh:

    eval "$(python3 pi_ssh.py shell-env [-c config.json])"
"""

import argparse
import ipaddress
import json
import os
import re
import shlex
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config.json")

# Where things live on the Pi, relative to the ssh user's home directory
# (remote commands and scp paths start there). Must match startup.py on the Pi.
REMOTE_DEPLOY_DIR = "deploy"
REMOTE_STARTUP_DIR = "startup"
REMOTE_STARTUP_CONFIG = REMOTE_STARTUP_DIR + "/startup_config.json"

# Layout of the Pi's deploy directory. Must match startup.py on the Pi.
# The manifest stays between boots (it tells the Pi which files to sync from
# GitHub); files/ only exists while a deploy is waiting to be applied.
MANIFEST_NAME = "deploy-manifest.json"
STAGED_FILES_DIR = "files"

# ssh exits with 255 when the connection itself fails (or is dropped, e.g.
# because the Pi started shutting down before the command returned).
SSH_CONNECTION_ERROR = 255

# Don't ping-sweep subnets bigger than this when looking for the Pi.
MAX_SWEEP_HOSTS = 4096

MAC_RE = re.compile(r"\b([0-9a-fA-F]{1,2}(?:[:-][0-9a-fA-F]{1,2}){5})\b")
IPV4_RE = re.compile(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b")


class DeployError(Exception):
    pass


def normalize_mac(mac):
    """'B8-27-EB-5-C6-4F' -> 'b8:27:eb:05:c6:4f'. Raises ValueError if not a MAC."""
    parts = re.split(r"[:-]", mac.strip())
    if len(parts) != 6:
        raise ValueError(mac)
    return ":".join(f"{int(part, 16):02x}" for part in parts)


def load_config(path=DEFAULT_CONFIG_PATH):
    path = os.path.abspath(path)
    try:
        with open(path) as f:
            config = json.load(f)
    except FileNotFoundError:
        raise DeployError(f"Config file not found: {path}")
    except json.JSONDecodeError as e:
        raise DeployError(f"Config file {path} is not valid JSON: {e}")

    if "pi" not in config or "mac" not in config["pi"]:
        raise DeployError(f"Config {path} is missing 'pi.mac'")
    try:
        config["pi"]["mac"] = normalize_mac(config["pi"]["mac"])
    except ValueError:
        raise DeployError(f"'pi.mac' in {path} is not a MAC address: {config['pi']['mac']}")

    # Remember where the config lives so relative local paths can be resolved
    # against it.
    config["_config_dir"] = os.path.dirname(path)
    return config


def _neighbors():
    """{mac: ip} from the laptop's ARP/neighbor table."""
    try:
        out = subprocess.run(["ip", "-4", "neigh", "show"], capture_output=True, text=True).stdout
    except FileNotFoundError:  # not Linux: fall back to `arp -an` (macOS/BSD)
        out = subprocess.run(["arp", "-an"], capture_output=True, text=True).stdout
    found = {}
    for line in out.splitlines():
        if "FAILED" in line or "incomplete" in line:
            continue
        ip, mac = IPV4_RE.search(line), MAC_RE.search(line)
        if ip and mac:
            found[normalize_mac(mac.group(1))] = ip.group(1)
    return found


def _local_subnets():
    """IPv4 networks the laptop is directly connected to (Linux only)."""
    try:
        out = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True,
            text=True,
        ).stdout
    except FileNotFoundError:
        return []
    nets = set()
    for match in re.finditer(r"\binet (\S+)", out):
        net = ipaddress.ip_interface(match.group(1)).network
        if net.num_addresses <= MAX_SWEEP_HOSTS:
            nets.add(net)
    return sorted(nets)


def _sweep(nets):
    """
    Ping every address on the given networks so the kernel ARPs each one and
    the Pi shows up in the neighbor table. Works without root (arp-scan doesn't).
    """
    hosts = [str(host) for net in nets for host in net.hosts()]

    def ping(host):
        subprocess.run(
            ["ping", "-c", "1", "-W", "1", host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    with ThreadPoolExecutor(max_workers=128) as pool:
        list(pool.map(ping, hosts))


def _port_open(ip, port, timeout):
    try:
        socket.create_connection((ip, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def pi_address(config):
    """
    The Pi's current IP address, found by looking up pi.mac in the laptop's
    neighbor table (ping-sweeping the local network if it isn't there yet).
    Cached for the rest of the run.
    """
    if "_ip" in config:
        return config["_ip"]
    pi = config["pi"]
    mac = pi["mac"]
    port = pi.get("port") or 22
    deadline = time.monotonic() + pi.get("discovery_timeout_s", 60)
    while True:
        ip = _neighbors().get(mac)
        if not (ip and _port_open(ip, port, 3)):
            # Not known yet, or the address we know is stale.
            _sweep(_local_subnets())
            ip = _neighbors().get(mac)
        if ip and _port_open(ip, port, 3):
            config["_ip"] = ip
            return ip
        if time.monotonic() > deadline:
            raise DeployError(
                f"Couldn't find the Pi (MAC {mac}) on the local network. Is it on, "
                "and on the same network as this laptop?"
            )
        time.sleep(2)


def host_key_alias(config):
    """
    Name the Pi's ssh host key is stored under in known_hosts. Keyed by MAC,
    not IP, because the Pi's IP changes. pi-setup.sh resets it after re-imaging.
    """
    return "oem-dashboard-pi-" + config["pi"]["mac"].replace(":", "")


def _target(config):
    pi = config["pi"]
    user = pi.get("user")
    ip = pi_address(config)
    return f"{user}@{ip}" if user else ip


def _ssh_options(config, port_flag):
    pi = config["pi"]
    opts = [
        "-o", f"ConnectTimeout={pi.get('connect_timeout_s', 10)}",
        "-o", f"HostKeyAlias={host_key_alias(config)}",
        "-o", "StrictHostKeyChecking=accept-new",
    ]
    if pi.get("port"):
        opts += [port_flag, str(pi["port"])]
    if pi.get("ssh_key"):
        opts += ["-i", os.path.expanduser(pi["ssh_key"])]
    return opts


def ssh(config, command, check=True, capture=False, input_text=None):
    """
    Run a shell command on the Pi. `command` is a list of arguments that gets
    shell-quoted, or a raw string that is passed through as-is.
    """
    if not isinstance(command, str):
        command = " ".join(shlex.quote(arg) for arg in command)
    args = ["ssh", *_ssh_options(config, "-p"), _target(config), command]
    result = subprocess.run(
        args,
        input=input_text,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )
    if check and result.returncode != 0:
        detail = f": {result.stderr.strip()}" if capture and result.stderr else ""
        raise DeployError(
            f"Remote command failed (exit {result.returncode}): {command}{detail}"
        )
    return result


def scp_dir(config, local_dir, remote_dir):
    """Recursively copy local_dir to remote_dir (remote_dir must not exist)."""
    args = [
        "scp",
        "-r",
        "-q",
        *_ssh_options(config, "-P"),
        local_dir,
        f"{_target(config)}:{remote_dir}",
    ]
    result = subprocess.run(args)
    if result.returncode != 0:
        raise DeployError(f"scp to {remote_dir} failed (exit {result.returncode})")


def read_remote_json(config, remote_path):
    result = ssh(config, ["cat", remote_path], capture=True)
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as e:
        raise DeployError(f"{remote_path} on the Pi is not valid JSON: {e}")


def write_remote_json(config, remote_path, data):
    """Write JSON to the Pi atomically (write to a temp file, then rename)."""
    tmp = remote_path + ".tmp"
    command = f"cat > {shlex.quote(tmp)} && mv {shlex.quote(tmp)} {shlex.quote(remote_path)}"
    ssh(config, command, input_text=json.dumps(data, indent=4) + "\n")


def remote_staged_files_dir():
    return f"{REMOTE_DEPLOY_DIR}/{STAGED_FILES_DIR}"


def remote_has_pending_deploy(config):
    return ssh(config, ["test", "-d", remote_staged_files_dir()], check=False).returncode == 0


def power_command(config, action):
    """Ask the Pi to 'shutdown' or 'reboot'. Tolerates the connection dropping."""
    commands = {
        "shutdown": "sudo -n shutdown -h now",
        "reboot": "sudo -n reboot",
    }
    if action not in commands:
        raise DeployError(f"Unknown power action '{action}'")
    result = ssh(config, commands[action], check=False)
    if result.returncode not in (0, SSH_CONNECTION_ERROR):
        raise DeployError(
            f"'{commands[action]}' failed on the Pi (exit {result.returncode}). "
            "Does the ssh user have passwordless sudo? "
            "(pi-setup.sh sets that up.)"
        )


def run_main(main):
    """Run a script's main(), turning DeployErrors into a clean error message."""
    try:
        sys.exit(main())
    except DeployError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)


def _print_shell_env(config):
    """Shell assignments for pi-setup.sh, so it shares this file's config handling."""
    values = {
        "PI_IP": pi_address(config),
        "PI_TARGET": _target(config),
        "HOST_KEY_ALIAS": host_key_alias(config),
    }
    for name, value in values.items():
        print(f"{name}={shlex.quote(value)}")
    for name, flag in (("SSH_OPTS", "-p"), ("SCP_OPTS", "-P")):
        print(f"{name}=({' '.join(shlex.quote(o) for o in _ssh_options(config, flag))})")


def main():
    parser = argparse.ArgumentParser(description="Helpers for pi-setup.sh.")
    parser.add_argument("command", choices=["shell-env"])
    parser.add_argument(
        "-c", "--config", default=DEFAULT_CONFIG_PATH, help="path to config.json"
    )
    args = parser.parse_args()
    _print_shell_env(load_config(args.config))
    return 0


if __name__ == "__main__":
    run_main(main)
