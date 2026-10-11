# Dashboard Deploy (laptop side)

Scripts that run on the programming laptop (Ubuntu) to push code to the
dashboard Raspberry Pi. For what happens on the Pi, see [`../pi/README.md`](../pi/README.md).

| Script | What it does |
| --- | --- |
| `deploy.py` | Stage local files on the Pi, then shut the Pi down. Files are applied on next boot. |
| `set_branch.py` | Show or change which GitHub branch the Pi syncs to on boot. |
| `reboot_pi.py` | Reboot the Pi so it syncs with GitHub. |
| `pi-setup.sh` | One-time setup of a freshly imaged Pi, ending with the first deploy. |
| `pi_ssh.py` | Shared ssh/config helpers (not run directly). |
| `config.json` | The Pi's MAC address and login, and the list of files that live on the Pi. |

All scripts only need Python 3 and the `ssh`/`scp`/`ping`/`ip` commands (Linux).
Each takes `-c/--config` to use a config other than `config.json`, and `-h` for help.

## Finding the Pi

The Pi gets its IP address from DHCP, so it changes, and `.local` hostnames
often don't resolve on the school network. Instead, `config.json` has the Pi's
**MAC address** (`pi.mac`), and every script looks it up in the laptop's
neighbor (ARP) table. If it isn't there, the script pings every address on the
laptop's subnet so the table fills in, then looks again. This doesn't need root.
It retries for `pi.discovery_timeout_s` seconds (default 60) before giving up.

To get the MAC address, run this on the Pi:

```shell
cat /sys/class/net/wlan0/address
```

Because the IP changes, ssh stores the Pi's host key under a name based on its
MAC (`oem-dashboard-pi-<mac>`) instead of its IP. A new key is accepted
automatically the first time. After re-imaging the Pi has a new key, which
`pi-setup.sh` handles by forgetting the old one.

## Setting up a new Pi: `pi-setup.sh`

1. Flash Raspberry Pi OS (trixie) with Raspberry Pi Imager. In OS
   customisation set the username (must match `pi.user`), the Wi-Fi network,
   and enable SSH with your public key.
2. Boot the Pi and wait for its first boot to finish. That can take up to
   ~10 minutes, and it's done when you can ssh in.
3. Put its MAC address in `config.json` (`pi.mac`, see above), then run:

   ```shell
   ./pi-setup.sh
   ```

   It asks for the Pi user's password once, for `sudo`. It takes a few
   minutes, mostly `apt` on the Pi, and ends by rebooting the Pi. It doesn't
   wait for the Pi to come back.

What it does:

- **Speeds up boot to network.** These changes take the Pi Zero W from ~161 s
  to ~72 s between power-on and `network-online.target`:
  - converts the Imager's Wi-Fi config from netplan to a plain NetworkManager
    profile (the original is kept in `/root/netfix-backup`);
  - makes NetworkManager's startup `netplan generate` skip a slow
    `systemctl daemon-reload`;
  - uninstalls cloud-init, which ran on every boot.
- Installs `git` (the Pi uses it for GitHub syncs).
- Gives the Pi user passwordless `sudo`. `deploy.py` and `reboot_pi.py` need
  it to shut down and reboot the Pi over ssh.
- Installs a boot-time internet probe (diagnostics). On the Pi,
  `boot-net-times` prints when this boot reached the network.
- Installs `../pi/startup.py` and `../pi/startup_config.json` in `~/startup`,
  and `../pi/oem-startup.service` in `/etc/systemd/system` (enabled, with
  `User=` set to `pi.user`).
- Runs the first deploy (`./deploy.py --reboot`). On that boot the Pi applies
  the deploy, which also creates the manifest the GitHub sync uses, then starts
  the dashboard.

It refuses to run while the Pi's first boot is still going. Running it again on
an already set-up Pi is safe, but it resets `startup_config.json` (e.g. the
branch back to `main`).

## How deploying works

The Pi does **not** have a copy of the repo. It only has the files listed in
the `deploy` list of `config.json`, each at its `remote_path`. There are two
ways those files get updated:

1. **Deploy (for quick testing):** `./deploy.py` copies your local, possibly
   uncommitted files to the Pi. The Pi applies them on next boot and runs them.
2. **GitHub sync (the normal state):** every boot that has *no* pending deploy,
   the Pi downloads the latest version of each of those files from the
   configured branch (and only those files) and overwrites its copies. This
   **throws away** anything a previous deploy changed.

So a deploy lasts for exactly one boot. To go back to what's on GitHub, run
`./reboot_pi.py`.

The list of files the Pi syncs comes from the last deploy: each deploy leaves a
manifest (`deploy-manifest.json`) on the Pi recording where each file lives in
the repo and where it goes on the Pi. So **if you change the `deploy` list,
run `./deploy.py` once** so the Pi picks up the new list.

```
laptop                                   Raspberry Pi
------                                   ------------
./deploy.py ── scp ──> ~/deploy/  (staged, not in place yet)
            ── ssh ──> shutdown
                                         ...power on...
                                         startup.py: files/ -> final paths,
                                                     delete files/ (keep manifest),
                                                     run dashboard
./reboot_pi.py ── ssh ──> reboot
                                         startup.py: no deploy pending,
                                                     download manifest's files
                                                     from GitHub branch,
                                                     run dashboard
```

### What `deploy.py` does

1. Reads the `deploy` list in `config.json` and expands any folders into every
   file under them (recursively). For each file it works out its path inside
   the repo (`repo_path`), which the Pi uses to download it from GitHub later.
2. Builds this layout in a local temp folder:
   ```
   deploy/
   ├─ deploy-manifest.json
   ├─ files/
   │  ├─ dashboard.py
   │  ├─ mkvi.dbc
   ```
   `deploy-manifest.json` lists each file:
   ```json
   [
       {
           "file_name": "dashboard.py",
           "file_path": "/home/oem/dashboard.py",
           "repo_path": "vehicle/mkviii/software/lv/dashboard/dashboard.py"
       },
       {
           "file_name": "mkvi.dbc",
           "file_path": "/home/oem/mkvi.dbc",
           "repo_path": "vehicle/mkviii/software/lv/dashboard/mkvi.dbc"
       }
   ]
   ```
   `file_name` is the file's name in `files/`, `file_path` its final location
   on the Pi (`remote_path`), and `repo_path` its path in the repo. `files/`
   is flat. If two files share a name, later ones get a suffix (`main.py`,
   `main_1.py`, ...).
3. Uploads it to `~/deploy.incoming` on the Pi, then renames it to
   `~/deploy`. The rename means the Pi never sees a half-uploaded
   deploy. Any earlier deploy that was never applied is replaced, along with
   the old manifest.
4. Shuts the Pi down (configurable, see below).

A deploy is pending while `~/deploy/files/` exists. After applying it,
the Pi deletes `files/` but keeps `deploy-manifest.json`.

```shell
./deploy.py              # deploy, then shut the Pi down
./deploy.py --dry-run    # just list what would be deployed (repo path -> Pi path)
./deploy.py --reboot     # deploy, then reboot instead of shutting down
./deploy.py --no-shutdown  # deploy and leave the Pi running
```

## Helper scripts

### `set_branch.py`

```shell
./set_branch.py             # print the branch the Pi currently syncs to
./set_branch.py my-branch   # sync to my-branch from now on
```

This edits the Pi's startup config (`~/startup/startup_config.json`) over ssh. It
checks that the branch exists on GitHub first, so **push your branch before
setting it**. Use `--force` to skip the check. The change takes effect on the
next boot.

### `reboot_pi.py`

```shell
./reboot_pi.py                   # reboot (syncs with GitHub on boot)
./reboot_pi.py --discard-deploy  # delete any pending deploy first
```

If a deploy is still pending (e.g. you used `deploy.py --no-shutdown`), the
next boot applies it *instead of* syncing with GitHub. The script warns you;
pass `--discard-deploy` to delete the staged files and force a GitHub sync.
The pending deploy's manifest is kept, so the Pi syncs the files listed in it.

## `config.json`

```json
{
    "pi": {
        "mac": "b8:27:eb:25:c6:4f",
        "user": "oem",
        "port": 22,
        "ssh_key": null,
        "connect_timeout_s": 10,
        "discovery_timeout_s": 60
    },
    "post_deploy_action": "shutdown",
    "deploy": [
        {
            "local_path": "../dashboard.py",
            "remote_path": "/home/oem/dashboard.py"
        },
        {
            "local_path": "../mkvi.dbc",
            "remote_path": "/home/oem/mkvi.dbc"
        }
    ]
}
```

| Key | Description |
| --- | --- |
| `pi.mac` | MAC address of the Pi's Wi-Fi (see [Finding the Pi](#finding-the-pi)). |
| `pi.user` | ssh username on the Pi (the user created in Raspberry Pi Imager). |
| `pi.port` | ssh port. Optional, defaults to 22. |
| `pi.ssh_key` | Path to a private key. `null` to use your ssh defaults/agent. |
| `pi.connect_timeout_s` | Seconds ssh waits to connect before giving up. Optional, defaults to 10. |
| `pi.discovery_timeout_s` | Seconds to keep looking for the Pi's MAC on the network. Optional, defaults to 60. |
| `post_deploy_action` | What `deploy.py` does after uploading: `"shutdown"` (default), `"reboot"`, or `"none"`. |
| `deploy` | List of `local_path` → `remote_path` mappings: the files that live on the Pi (see below). |

### The `deploy` list

- `local_path` can be absolute, or relative to **the folder `config.json` is in**
  (not the folder you run the script from). `~` is expanded.
- `local_path` must be inside the git repo. Its path in the repo is where the
  Pi downloads it from when syncing with GitHub, so a file that isn't on the
  branch yet (e.g. new and not pushed) still deploys, but the next GitHub sync
  logs a warning and leaves the Pi's copy alone.
- `remote_path` must be absolute. It's the file's final location on the Pi.
  The Pi runs `startup.py` as `pi.user`, so it must be somewhere that user can
  write to (e.g. under `/home/oem`).
- If `local_path` is a **folder**, every file and subfolder under it is
  deployed, with the same structure under `remote_path`. For example,
  `"local_path": "../gui"`, `"remote_path": "/home/oem/gui"` sends
  `../gui/widgets/gauge.py` to `/home/oem/gui/widgets/gauge.py`.
- Mapping two different local files to the same remote path is an error.
- Removing an entry doesn't delete the file from the Pi; the Pi just stops
  syncing it (after your next deploy).

## Pi requirements

All of these are set up by [`pi-setup.sh`](#setting-up-a-new-pi-pi-setupsh):

- The Pi is on the same network as the laptop, and you can ssh into it with a
  key (set in Raspberry Pi Imager, or `ssh-copy-id`), so you aren't asked for a
  password several times per run.
- The ssh user has passwordless `sudo` (check by running `sudo -n true` on
  the Pi). The scripts use it for shutdown/reboot.
- The Pi side is installed as described in [`../pi/README.md`](../pi/README.md).
