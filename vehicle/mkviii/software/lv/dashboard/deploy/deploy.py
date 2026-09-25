#!/usr/bin/env python3
"""
Deploy local code to the Raspberry Pi.

Files listed in config.json are staged into the Pi's temporary deploy
directory (NOT their final locations), then the Pi is shut down. On next boot,
the Pi's startup script moves the staged files into place. See README.md.
"""

import argparse
import json
import os
import shlex
import shutil
import tempfile

from pi_ssh import (
    DEFAULT_CONFIG_PATH,
    DeployError,
    load_config,
    power_command,
    remote_has_pending_deploy,
    run_main,
    scp_dir,
    ssh,
)


def resolve_local_path(config, local_path):
    """Absolute paths are used as-is; relative paths are relative to config.json."""
    local_path = os.path.expanduser(local_path)
    if not os.path.isabs(local_path):
        local_path = os.path.join(config["_config_dir"], local_path)
    return os.path.normpath(local_path)


def collect_files(config):
    """
    Expand the deploy map into a list of (local_file, remote_file) pairs.
    Folders are walked recursively, preserving their structure under
    remote_path.
    """
    entries = config.get("deploy", [])
    if not entries:
        raise DeployError("Nothing to deploy: 'deploy' in config.json is empty")

    files = []
    for entry in entries:
        if "local_path" not in entry or "remote_path" not in entry:
            raise DeployError(
                f"Deploy entry needs 'local_path' and 'remote_path': {entry}"
            )
        local = resolve_local_path(config, entry["local_path"])
        remote = entry["remote_path"]
        if not os.path.isabs(remote):
            raise DeployError(f"remote_path must be absolute: {remote}")

        if os.path.isfile(local):
            files.append((local, remote))
        elif os.path.isdir(local):
            for root, dirs, names in os.walk(local):
                dirs.sort()
                for name in sorted(names):
                    local_file = os.path.join(root, name)
                    rel = os.path.relpath(local_file, local)
                    files.append((local_file, os.path.join(remote, rel)))
        else:
            raise DeployError(f"local_path does not exist: {local}")

    # Two entries targeting the same remote file is almost certainly a mistake.
    seen = {}
    for local, remote in files:
        if remote in seen and seen[remote] != local:
            raise DeployError(
                f"Both {seen[remote]} and {local} are mapped to {remote}"
            )
        seen[remote] = local
    return list(dict.fromkeys(files))


def unique_name(name, used):
    """files/ is flat, so disambiguate duplicate basenames (main.py, main_1.py)."""
    if name not in used:
        return name
    stem, ext = os.path.splitext(name)
    i = 1
    while f"{stem}_{i}{ext}" in used:
        i += 1
    return f"{stem}_{i}{ext}"


def build_staging_dir(files, staging_dir):
    """Create the deploy/ layout (config.json + files/) locally."""
    files_dir = os.path.join(staging_dir, "files")
    os.makedirs(files_dir)

    manifest = []
    used = set()
    for local, remote in files:
        name = unique_name(os.path.basename(local), used)
        used.add(name)
        shutil.copy2(local, os.path.join(files_dir, name))
        manifest.append({"file_name": name, "file_path": remote})

    with open(os.path.join(staging_dir, "config.json"), "w") as f:
        json.dump(manifest, f, indent=4)
        f.write("\n")
    return manifest


def upload(config, staging_dir):
    """
    Upload to <deploy_dir>.incoming, then swap it into place, so the Pi never
    sees a half-copied deploy if the transfer is interrupted.
    """
    deploy_dir = config["remote_deploy_dir"].rstrip("/")
    incoming = deploy_dir + ".incoming"
    ssh(config, ["rm", "-rf", incoming])
    ssh(config, ["mkdir", "-p", os.path.dirname(incoming)])
    scp_dir(config, staging_dir, incoming)
    ssh(
        config,
        f"rm -rf {shlex.quote(deploy_dir)} && mv {shlex.quote(incoming)} {shlex.quote(deploy_dir)}",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "-c", "--config", default=DEFAULT_CONFIG_PATH, help="path to config.json"
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="show what would be deployed without connecting to the Pi",
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--reboot",
        dest="action",
        action="store_const",
        const="reboot",
        help="reboot the Pi after deploying instead of shutting it down",
    )
    action.add_argument(
        "--no-shutdown",
        dest="action",
        action="store_const",
        const="none",
        help="leave the Pi running (changes apply on its next boot)",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    if "remote_deploy_dir" not in config:
        raise DeployError("config.json is missing 'remote_deploy_dir'")
    post_action = args.action or config.get("post_deploy_action", "shutdown")

    files = collect_files(config)
    print(f"Deploying {len(files)} file(s) to {config['pi']['host']}:")
    for local, remote in files:
        print(f"  {os.path.relpath(local)} -> {remote}")

    if args.dry_run:
        print("Dry run, nothing sent.")
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        staging_dir = os.path.join(tmp, "deploy")
        build_staging_dir(files, staging_dir)

        if remote_has_pending_deploy(config):
            print("Note: replacing a previous deploy that was never applied.")
        upload(config, staging_dir)

    print(f"Staged in {config['remote_deploy_dir']} on the Pi.")
    if post_action == "none":
        print("Pi left running; files will be applied on its next boot.")
    else:
        print(f"Sending {post_action} command...")
        power_command(config, post_action)
        print("Done. Files will be applied when the Pi boots.")
    return 0


if __name__ == "__main__":
    run_main(main)
