#!/usr/bin/env python3
"""
Runs on the Raspberry Pi at boot (via oem-startup.service).

1. If deploy.py left files in the deploy directory, copy them to their final
   locations and empty the deploy directory.
2. Otherwise, hard-reset the local repo to the configured GitHub branch.
3. Run the dashboard.

Usage: startup.py [path/to/startup_config.json]
"""

import json
import logging
import os
import shutil
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "startup_config.json")

log = logging.getLogger("oem-startup")


def load_config(path):
    with open(path) as f:
        return json.load(f)


def has_pending_deploy(deploy_dir):
    return os.path.isfile(os.path.join(deploy_dir, "config.json"))


def clear_dir(path):
    """Delete everything inside path, but keep path itself."""
    for name in os.listdir(path):
        full = os.path.join(path, name)
        if os.path.isdir(full) and not os.path.islink(full):
            shutil.rmtree(full)
        else:
            os.remove(full)


def apply_deploy(deploy_dir):
    """Copy staged files into place. Returns True if every file was applied."""
    files_dir = os.path.join(deploy_dir, "files")
    ok = True
    try:
        with open(os.path.join(deploy_dir, "config.json")) as f:
            manifest = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        log.error("Could not read deploy config: %s", e)
        return False

    for entry in manifest:
        src = os.path.join(files_dir, entry["file_name"])
        dest = entry["file_path"]
        try:
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            # Copy then rename so a power loss never leaves a half-written file.
            tmp = dest + ".deploy-tmp"
            shutil.copy2(src, tmp)
            os.replace(tmp, dest)
            log.info("Deployed %s -> %s", entry["file_name"], dest)
        except OSError as e:
            log.error("Failed to deploy %s -> %s: %s", entry["file_name"], dest, e)
            ok = False
    return ok


def git(repo_dir, *args, timeout):
    subprocess.run(["git", "-C", repo_dir, *args], check=True, timeout=timeout)


def sync_with_github(config):
    """Make repo_dir an exact copy of the configured branch. Never raises."""
    repo_dir = config["repo_dir"]
    remote = config.get("git_remote", "origin")
    branch = config["branch"]
    timeout = config.get("git_timeout_s", 60)

    try:
        if not os.path.isdir(os.path.join(repo_dir, ".git")):
            log.info("Cloning %s (%s) into %s", config["repo_url"], branch, repo_dir)
            subprocess.run(
                ["git", "clone", "--branch", branch, config["repo_url"], repo_dir],
                check=True,
                timeout=timeout,
            )
            return

        log.info("Syncing %s with %s/%s", repo_dir, remote, branch)
        git(repo_dir, "fetch", remote, branch, timeout=timeout)
        git(repo_dir, "checkout", "--force", "-B", branch, f"{remote}/{branch}", timeout=timeout)
        git(repo_dir, "reset", "--hard", f"{remote}/{branch}", timeout=timeout)
        git(repo_dir, "clean", "-fd", timeout=timeout)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as e:
        # Most likely no network (e.g. on the car). Run whatever code we have.
        log.warning("GitHub sync failed, keeping existing code: %s", e)


def run_dashboard(config):
    dashboard = config["dashboard_path"]
    python = config.get("python_executable", sys.executable)
    restart = config.get("restart_dashboard_on_exit", True)
    delay = config.get("restart_delay_s", 3)

    while True:
        log.info("Starting dashboard: %s %s", python, dashboard)
        try:
            code = subprocess.run([python, dashboard], cwd=os.path.dirname(dashboard)).returncode
        except OSError as e:
            log.error("Could not start dashboard: %s", e)
            code = 1
        log.warning("Dashboard exited with code %s", code)
        if not restart:
            return code
        time.sleep(delay)


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    config_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG_PATH
    config = load_config(config_path)

    deploy_dir = config["deploy_dir"]
    os.makedirs(deploy_dir, exist_ok=True)

    if has_pending_deploy(deploy_dir):
        log.info("Found pending deploy in %s", deploy_dir)
        if not apply_deploy(deploy_dir):
            log.error("Some files failed to deploy (see above)")
        # Clear even on failure; otherwise a bad deploy would block GitHub
        # syncing on every future boot.
        clear_dir(deploy_dir)
    else:
        if os.listdir(deploy_dir):
            log.warning("Deploy dir has files but no config.json; ignoring them")
            clear_dir(deploy_dir)
        sync_with_github(config)

    return run_dashboard(config)


if __name__ == "__main__":
    sys.exit(main())
