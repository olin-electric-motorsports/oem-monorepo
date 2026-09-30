# Dashboard Startup (Raspberry Pi side)

These files run on the dashboard Raspberry Pi. The Pi doesn't keep a copy of
the repo, only the files it needs (e.g. `/home/oemdashboard/dashboard.py` and
`/home/oemdashboard/mkvi.dbc`). Which files those are, and where each one comes
from in the repo, is recorded in `deploy_dir/deploy-manifest.json`, written by
[`../deploy/deploy.py`](../deploy/README.md).

On every boot, a systemd service runs `startup.py`, which:

1. **If a deploy is pending**, meaning `deploy_dir/files/` exists: copies each
   file in `deploy_dir/files/` to the `file_path` listed for it in
   `deploy_dir/deploy-manifest.json`, then deletes everything in `deploy_dir`
   *except* `deploy-manifest.json`.
2. **Otherwise**, downloads the latest version of every file in the manifest
   from the configured GitHub branch (its `repo_path`) and overwrites the copy
   at its `file_path`. Only those files are downloaded, using a shallow,
   sparse clone into a temp folder that is deleted afterwards. Files that
   aren't on the branch are left alone (with a warning). If GitHub can't be
   reached (e.g. no Wi-Fi on the car), it logs a warning and keeps the
   existing files.
3. **Runs the dashboard** with `python_executable dashboard_path`. If the
   dashboard exits, it is restarted after `restart_delay_s` seconds.

A deploy only lasts one boot. The next boot without a pending deploy syncs with
GitHub again, which overwrites the deployed files.

| File | Purpose |
| --- | --- |
| `startup.py` | The boot script described above. |
| `startup_config.json` | Its configuration. |
| `oem-startup.service` | systemd unit that runs `startup.py` at boot. |

## Installation

The Pi needs `git` (`sudo apt install git`) to download files from GitHub.
The repo itself is **not** cloned onto the Pi.

From the laptop, in this folder:

```shell
ssh oemdashboard@raspberrypi.local mkdir -p oem_startup
scp startup.py startup_config.json oemdashboard@raspberrypi.local:oem_startup/
scp oem-startup.service oemdashboard@raspberrypi.local:
```

Then on the Pi:

```shell
sudo mv ~/oem-startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable oem-startup.service
```

Finally, run [`../deploy/deploy.py`](../deploy/README.md) from the laptop once.
That puts the dashboard files on the Pi and writes `deploy-manifest.json`.
Until then the Pi has no manifest, so it doesn't know which files to sync
(it logs a warning and just tries to run the dashboard).

The service runs as user `oemdashboard` and expects the files in `/home/oemdashboard/oem_startup/`.
If you use a different user or folder, edit `User=`, `WorkingDirectory=` and
`ExecStart=` in the `.service` file.

Because the service runs as `oemdashboard`, deploys and GitHub syncs can only
write to places `oemdashboard` can write to (e.g. anywhere under `/home/oemdashboard`).

## Useful commands

```shell
journalctl -u oem-startup -f          # follow the startup/dashboard logs
journalctl -u oem-startup -b          # logs from this boot
sudo systemctl restart oem-startup    # re-run startup.py without rebooting
sudo systemctl stop oem-startup       # stop the dashboard
```

## `startup_config.json`

```json
{
    "deploy_dir": "/home/oemdashboard/deploy",
    "repo_url": "https://github.com/olin-electric-motorsports/oem-monorepo.git",
    "branch": "main",
    "git_timeout_s": 60,
    "dashboard_path": "/home/oemdashboard/dashboard.py",
    "python_executable": "/usr/bin/python3",
    "restart_dashboard_on_exit": true,
    "restart_delay_s": 3
}
```

| Key | Description |
| --- | --- |
| `deploy_dir` | Deploy directory (staged deploys and the manifest). **Must match `remote_deploy_dir` in the laptop's `deploy/config.json`.** |
| `repo_url` | Repo to download files from. Also used by `set_branch.py` to check branches exist. |
| `branch` | Branch to sync to on boot. Change it from the laptop with `deploy/set_branch.py`. |
| `git_timeout_s` | Seconds before a git command is abandoned (e.g. no network). Default 60. |
| `dashboard_path` | Python file to run as the dashboard (a `file_path` from the manifest). |
| `python_executable` | Python interpreter to run it with (e.g. a virtualenv's `bin/python`). |
| `restart_dashboard_on_exit` | Restart the dashboard if it exits or crashes. Default `true`. |
| `restart_delay_s` | Seconds to wait before restarting. Default 3. |

### Deploy directory format

Written by `deploy.py`; documented here for reference:

```
deploy/
├─ deploy-manifest.json   <- kept between boots
├─ files/                 <- only while a deploy is pending
│  ├─ dashboard.py
│  ├─ mkvi.dbc
```

```json
[
    {
        "file_name": "dashboard.py",
        "file_path": "/home/oemdashboard/dashboard.py",
        "repo_path": "vehicle/mkviii/software/lv/dashboard/dashboard.py"
    },
    {
        "file_name": "mkvi.dbc",
        "file_path": "/home/oemdashboard/mkvi.dbc",
        "repo_path": "vehicle/mkviii/software/lv/dashboard/mkvi.dbc"
    }
]
```

| Key | Description |
| --- | --- |
| `file_name` | Name of the staged file in `files/` (used when applying a deploy). |
| `file_path` | Where the file goes on the Pi. |
| `repo_path` | Where the file is in the repo (used when syncing with GitHub). |

Missing parent folders are created. Each file is copied to a temp name and then
renamed, so losing power mid-copy never leaves a half-written file. If some
files fail to copy, the errors are logged and `files/` is still deleted, so one
bad deploy can't block GitHub syncing on later boots.
