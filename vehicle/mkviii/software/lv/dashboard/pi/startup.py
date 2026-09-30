#!/usr/bin/env python3
"""
Runs on the Raspberry Pi at boot (via oem-startup.service).

1. If deploy.py left files in deploy_dir/files/, copy them to their final
   locations and empty the deploy directory (except the manifest).
2. Otherwise, fetch the latest version of each file listed in the deploy
   manifest from the configured GitHub branch and copy it into place.
3. Run the dashboard.

Usage: startup.py [path/to/startup_config.json]
"""

import json
import logging
import os
import shutil
import subprocess
import sys
import tempfile
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "startup_config.json")

# Layout of the deploy directory. Must match deploy/pi_ssh.py on the laptop.
# The manifest stays between boots (it says which files to sync from GitHub);
# files/ only exists while a deploy is waiting to be applied.
MANIFEST_NAME = "deploy-manifest.json"
STAGED_FILES_DIR = "files"

log = logging.getLogger("oem-startup")


def load_config(path):
    with open(path) as f:
        return json.load(f)


def load_manifest(deploy_dir):
    """Returns the list of manifest entries, or None if there isn't a usable one."""
    path = os.path.join(deploy_dir, MANIFEST_NAME)
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        log.warning("No %s; run deploy.py from the laptop to create it", path)
    except (OSError, json.JSONDecodeError) as e:
        log.error("Could not read %s: %s", path, e)
    return None


def has_pending_deploy(deploy_dir):
    return os.path.isdir(os.path.join(deploy_dir, STAGED_FILES_DIR))


def clear_deploy_dir(deploy_dir):
    """Delete everything inside deploy_dir except the manifest."""
    for name in os.listdir(deploy_dir):
        if name == MANIFEST_NAME:
            continue
        full = os.path.join(deploy_dir, name)
        if os.path.isdir(full) and not os.path.islink(full):
            shutil.rmtree(full)
        else:
            os.remove(full)


def install_file(src, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    # Copy then rename so a power loss never leaves a half-written file.
    tmp = dest + ".deploy-tmp"
    shutil.copy2(src, tmp)
    os.replace(tmp, dest)


def apply_deploy(deploy_dir, manifest):
    """Copy staged files into place. Returns True if every file was applied."""
    files_dir = os.path.join(deploy_dir, STAGED_FILES_DIR)
    ok = True
    for entry in manifest:
        src = os.path.join(files_dir, entry["file_name"])
        dest = entry["file_path"]
        try:
            install_file(src, dest)
            log.info("Deployed %s -> %s", entry["file_name"], dest)
        except OSError as e:
            log.error("Failed to deploy %s -> %s: %s", entry["file_name"], dest, e)
            ok = False
    return ok


def git(repo_dir, *args, timeout):
    subprocess.run(["git", "-C", repo_dir, *args], check=True, timeout=timeout)


def sync_with_github(config, manifest):
    """
    Overwrite each file in the manifest with its latest version on the
    configured branch. Only those files are downloaded, not the whole repo.
    Never raises.
    """
    url = config["repo_url"]
    branch = config["branch"]
    timeout = config.get("git_timeout_s", 60)

    entries = []
    for entry in manifest:
        if entry.get("repo_path"):
            entries.append(entry)
        else:
            log.warning("No repo_path for %s in the manifest; not syncing it", entry["file_path"])
    if not entries:
        return

    try:
        with tempfile.TemporaryDirectory(prefix="oem-sync-") as tmp:
            repo = os.path.join(tmp, "repo")
            log.info("Fetching %d file(s) from %s (%s)", len(entries), url, branch)
            # Shallow, blobless clone: just the latest commit's file listing.
            # The sparse checkout then downloads only the files we need.
            subprocess.run(
                [
                    "git", "clone", "--quiet", "--depth", "1", "--filter=blob:none",
                    "--no-checkout", "--branch", branch, url, repo,
                ],
                check=True,
                timeout=timeout,
            )
            patterns = ["/" + entry["repo_path"] for entry in entries]
            git(repo, "sparse-checkout", "set", "--no-cone", *patterns, timeout=timeout)
            git(repo, "checkout", "--quiet", timeout=timeout)

            for entry in entries:
                src = os.path.join(repo, entry["repo_path"])
                dest = entry["file_path"]
                if not os.path.isfile(src):
                    log.warning(
                        "%s is not on %s; keeping the existing %s",
                        entry["repo_path"], branch, dest,
                    )
                    continue
                try:
                    install_file(src, dest)
                    log.info("Synced %s -> %s", entry["repo_path"], dest)
                except OSError as e:
                    log.error("Failed to sync %s -> %s: %s", entry["repo_path"], dest, e)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as e:
        # Most likely no network (e.g. on the car). Run whatever code we have.
        log.warning("GitHub sync failed, keeping existing files: %s", e)


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
    manifest = load_manifest(deploy_dir)

    if has_pending_deploy(deploy_dir):
        log.info("Found pending deploy in %s", deploy_dir)
        if manifest is None or not apply_deploy(deploy_dir, manifest):
            log.error("Some files failed to deploy (see above)")
        # Clear even on failure; otherwise a bad deploy would block GitHub
        # syncing on every future boot.
        clear_deploy_dir(deploy_dir)
    else:
        if set(os.listdir(deploy_dir)) - {MANIFEST_NAME}:
            log.warning("Deploy dir has unexpected files but no %s/; removing them", STAGED_FILES_DIR)
            clear_deploy_dir(deploy_dir)
        if manifest is not None:
            sync_with_github(config, manifest)

    return run_dashboard(config)


if __name__ == "__main__":
    sys.exit(main())
