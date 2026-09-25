#!/usr/bin/env python3
"""
Show or change which git branch the Pi syncs to on boot.

    ./set_branch.py            # print the currently configured branch
    ./set_branch.py my-branch  # make the Pi pull my-branch on next boot
"""

import argparse
import subprocess

from pi_ssh import (
    DEFAULT_CONFIG_PATH,
    DeployError,
    load_config,
    read_remote_json,
    run_main,
    write_remote_json,
)


def branch_exists(repo_url, branch):
    """Returns True/False, or None if the remote couldn't be reached."""
    result = subprocess.run(
        ["git", "ls-remote", "--exit-code", "--heads", repo_url, branch],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if result.returncode == 0:
        return True
    if result.returncode == 2:  # --exit-code: reachable, but no matching ref
        return False
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("branch", nargs="?", help="branch for the Pi to pull on boot")
    parser.add_argument(
        "-c", "--config", default=DEFAULT_CONFIG_PATH, help="path to config.json"
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="set the branch even if it can't be found on GitHub",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    startup_config_path = config.get("remote_startup_config")
    if not startup_config_path:
        raise DeployError("config.json is missing 'remote_startup_config'")

    startup_config = read_remote_json(config, startup_config_path)
    current = startup_config.get("branch")

    if args.branch is None:
        print(current)
        return 0

    if args.branch == current:
        print(f"Pi is already configured to use '{current}'.")
        return 0

    if not args.force:
        exists = branch_exists(startup_config["repo_url"], args.branch)
        if exists is False:
            raise DeployError(
                f"Branch '{args.branch}' not found on {startup_config['repo_url']} "
                "(push it first, or use --force)"
            )
        if exists is None:
            print("Warning: couldn't reach GitHub to check that the branch exists.")

    startup_config["branch"] = args.branch
    write_remote_json(config, startup_config_path, startup_config)
    print(f"Branch changed: {current} -> {args.branch}")
    print("Takes effect on next boot (see ./reboot_pi.py).")
    return 0


if __name__ == "__main__":
    run_main(main)
