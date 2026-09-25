#!/usr/bin/env python3
"""
Reboot the Pi so it syncs its code with the configured GitHub branch.

If a deploy (from deploy.py) is still waiting on the Pi, the next boot applies
that deploy INSTEAD of syncing with GitHub. Use --discard-deploy to throw the
pending deploy away and force a GitHub sync.
"""

import argparse

from pi_ssh import (
    DEFAULT_CONFIG_PATH,
    load_config,
    power_command,
    remote_has_pending_deploy,
    run_main,
    ssh,
)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0],
    )
    parser.add_argument(
        "-c", "--config", default=DEFAULT_CONFIG_PATH, help="path to config.json"
    )
    parser.add_argument(
        "--discard-deploy",
        action="store_true",
        help="delete any pending deploy so the Pi syncs with GitHub",
    )
    args = parser.parse_args()

    config = load_config(args.config)

    if remote_has_pending_deploy(config):
        if args.discard_deploy:
            deploy_dir = config["remote_deploy_dir"]
            ssh(config, ["find", deploy_dir, "-mindepth", "1", "-delete"])
            print("Discarded pending deploy.")
        else:
            print(
                "Warning: a deploy is pending on the Pi. It will be applied on this "
                "boot instead of syncing with GitHub (use --discard-deploy to skip it)."
            )

    print(f"Rebooting {config['pi']['host']}...")
    power_command(config, "reboot")
    print("Reboot command sent.")
    return 0


if __name__ == "__main__":
    run_main(main)
