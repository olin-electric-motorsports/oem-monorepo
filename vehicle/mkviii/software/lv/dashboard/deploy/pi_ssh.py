"""
Shared helpers for the laptop-side deploy scripts (deploy.py, set_branch.py,
reboot_pi.py). Loads config.json and wraps ssh/scp calls to the Raspberry Pi.
"""

import json
import os
import shlex
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config.json")

# ssh exits with 255 when the connection itself fails (or is dropped, e.g.
# because the Pi started shutting down before the command returned).
SSH_CONNECTION_ERROR = 255


class DeployError(Exception):
    pass


def load_config(path=DEFAULT_CONFIG_PATH):
    path = os.path.abspath(path)
    try:
        with open(path) as f:
            config = json.load(f)
    except FileNotFoundError:
        raise DeployError(f"Config file not found: {path}")
    except json.JSONDecodeError as e:
        raise DeployError(f"Config file {path} is not valid JSON: {e}")

    if "pi" not in config or "host" not in config["pi"]:
        raise DeployError(f"Config {path} is missing 'pi.host'")

    # Remember where the config lives so relative local paths can be resolved
    # against it.
    config["_config_dir"] = os.path.dirname(path)
    return config


def _target(config):
    pi = config["pi"]
    user = pi.get("user")
    return f"{user}@{pi['host']}" if user else pi["host"]


def _ssh_options(config, port_flag):
    pi = config["pi"]
    opts = ["-o", f"ConnectTimeout={pi.get('connect_timeout_s', 10)}"]
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


def remote_has_pending_deploy(config):
    deploy_config = os.path.join(config["remote_deploy_dir"], "config.json")
    return ssh(config, ["test", "-f", deploy_config], check=False).returncode == 0


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
            "Does the ssh user have passwordless sudo?"
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
