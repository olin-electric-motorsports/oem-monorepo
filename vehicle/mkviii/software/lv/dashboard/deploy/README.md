# Dashboard Deploy (laptop side)

Scripts that run on the programming laptop (Ubuntu) to push code to the
dashboard Raspberry Pi. For what happens on the Pi, see [`../pi/README.md`](../pi/README.md).

| Script | What it does |
| --- | --- |
| `deploy.py` | Stage local files on the Pi, then shut the Pi down. Files are applied on next boot. |
| `set_branch.py` | Show or change which GitHub branch the Pi syncs to on boot. |
| `reboot_pi.py` | Reboot the Pi so it syncs with GitHub. |
| `pi_ssh.py` | Shared ssh/config helpers (not run directly). |
| `config.json` | Pi address and the list of files to deploy. |

All scripts only need Python 3 and the `ssh`/`scp` commands. Each takes
`-c/--config` to use a config other than `config.json`, and `-h` for help.

## How deploying works

There are two ways code gets onto the Pi:

1. **Deploy (for quick testing):** `./deploy.py` copies your local, possibly
   uncommitted files to the Pi. The Pi applies them on next boot and runs them.
2. **GitHub sync (the normal state):** every boot that has *no* pending deploy,
   the Pi hard-resets its copy of the repo to the configured branch. This
   **throws away** anything a previous deploy changed inside the repo.

So a deploy lasts for exactly one boot. To go back to what's on GitHub, run
`./reboot_pi.py`.

```
laptop                                   Raspberry Pi
------                                   ------------
./deploy.py ── scp ──> /home/oemdashboard/deploy/  (staged, not in place yet)
            ── ssh ──> shutdown
                                         ...power on...
                                         startup.py: files -> final paths,
                                                     empty /home/oemdashboard/deploy/,
                                                     run dashboard
./reboot_pi.py ── ssh ──> reboot
                                         startup.py: no deploy pending,
                                                     reset repo to GitHub branch,
                                                     run dashboard
```

### What `deploy.py` does

1. Reads the `deploy` list in `config.json` and expands any folders into every
   file under them (recursively).
2. Builds this layout in a local temp folder:
   ```
   deploy/
   ├─ config.json      <- [{"file_name": "dashboard.py", "file_path": "/home/oemdashboard/.../dashboard.py"}, ...]
   ├─ files/
   │  ├─ dashboard.py
   │  ├─ myscript.py
   ```
   `files/` is flat. If two files share a name, later ones get a suffix
   (`main.py`, `main_1.py`, ...); `config.json` records the real destination.
3. Uploads it to `<remote_deploy_dir>.incoming` on the Pi, then renames it to
   `remote_deploy_dir`. The rename means the Pi never sees a half-uploaded
   deploy. Any earlier deploy that was never applied is replaced.
4. Shuts the Pi down (configurable, see below).

```shell
./deploy.py              # deploy, then shut the Pi down
./deploy.py --dry-run    # just list what would be deployed
./deploy.py --reboot     # deploy, then reboot instead of shutting down
./deploy.py --no-shutdown  # deploy and leave the Pi running
```

## Helper scripts

### `set_branch.py`

```shell
./set_branch.py             # print the branch the Pi currently syncs to
./set_branch.py my-branch   # sync to my-branch from now on
```

This edits the Pi's startup config (`remote_startup_config`) over ssh. It
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
pass `--discard-deploy` to clear it and force a GitHub sync.

## `config.json`

```json
{
    "pi": {
        "host": "raspberrypi.local",
        "user": "oemdashboard",
        "port": 22,
        "ssh_key": null,
        "connect_timeout_s": 10
    },
    "remote_deploy_dir": "/home/oemdashboard/deploy",
    "remote_startup_config": "/home/oemdashboard/oem_startup/startup_config.json",
    "post_deploy_action": "shutdown",
    "deploy": [
        {
            "local_path": "../dashboard.py",
            "remote_path": "/home/oemdashboard/oem-monorepo/vehicle/mkviii/software/lv/dashboard/dashboard.py"
        },
        {
            "local_path": "../mkvi.dbc",
            "remote_path": "/home/oemdashboard/mkvi.dbc"
        }
    ]
}
```

| Key | Description |
| --- | --- |
| `pi.host` | Hostname or IP address of the Pi. |
| `pi.user` | ssh username. Leave out to use your ssh default. |
| `pi.port` | ssh port. Optional, defaults to 22. |
| `pi.ssh_key` | Path to a private key. `null` to use your ssh defaults/agent. |
| `pi.connect_timeout_s` | Seconds to wait for the Pi before giving up. Optional, defaults to 10. |
| `remote_deploy_dir` | Temporary deploy directory on the Pi. **Must match `deploy_dir` in the Pi's `startup_config.json`.** |
| `remote_startup_config` | Path to the Pi's `startup_config.json` (used by `set_branch.py`). |
| `post_deploy_action` | What `deploy.py` does after uploading: `"shutdown"` (default), `"reboot"`, or `"none"`. |
| `deploy` | List of `local_path` → `remote_path` mappings (see below). |

### The `deploy` list

- `local_path` can be absolute, or relative to **the folder `config.json` is in**
  (not the folder you run the script from). `~` is expanded.
- `remote_path` must be absolute. It's the file's final location on the Pi.
- If `local_path` is a **folder**, every file and subfolder under it is
  deployed, with the same structure under `remote_path`. For example,
  `"local_path": "../gui"`, `"remote_path": "/home/oemdashboard/gui"` sends
  `../gui/widgets/gauge.py` to `/home/oemdashboard/gui/widgets/gauge.py`.
- Mapping two different local files to the same remote path is an error.

## Pi requirements

- You can ssh into the Pi from the laptop. Set up an ssh key
  (`ssh-copy-id oemdashboard@raspberrypi.local`) so you aren't asked for a password
  several times per run.
- The ssh user has passwordless `sudo` (check by running
  `sudo -n true` on the Pi). The scripts use it for shutdown/reboot.
- The Pi side is installed as described in [`../pi/README.md`](../pi/README.md).
