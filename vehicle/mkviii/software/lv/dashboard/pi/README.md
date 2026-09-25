# Dashboard Startup (Raspberry Pi side)

These files run on the dashboard Raspberry Pi. On every boot, a systemd service
runs `startup.py`, which:

1. **If a deploy is pending**, meaning `deploy_dir/config.json` exists (put
   there by [`../deploy/deploy.py`](../deploy/README.md)): copies each file in
   `deploy_dir/files/` to the path listed for it in `deploy_dir/config.json`,
   then deletes everything in `deploy_dir`.
2. **Otherwise**, completely overwrites the local repo with the configured
   GitHub branch (`git fetch`, `checkout -B`, `reset --hard`, `clean -fd`). Any
   local changes and untracked files in the repo are lost. If the repo doesn't
   exist yet it is cloned. If GitHub can't be reached (e.g. no Wi-Fi on the
   car), it logs a warning and keeps the existing code.
3. **Runs the dashboard** with `python_executable dashboard_path`. If the
   dashboard exits, it is restarted after `restart_delay_s` seconds.

A deploy only lasts one boot. The next boot without a pending deploy syncs with
GitHub again, which undoes any deployed changes inside the repo. Deployed files
*outside* the repo stay where they are.

| File | Purpose |
| --- | --- |
| `startup.py` | The boot script described above. |
| `startup_config.json` | Its configuration. |
| `oem-startup.service` | systemd unit that runs `startup.py` at boot. |

## Installation

Install the startup files **outside** the repo. The GitHub sync overwrites the
repo, so if `startup.py` ran from inside it, checking out a branch without
these files would break booting.

On the Pi:

```shell
git clone https://github.com/olin-electric-motorsports/oem-monorepo.git ~/oem-monorepo
mkdir -p ~/oem_startup
cp ~/oem-monorepo/vehicle/mkviii/software/lv/dashboard/pi/{startup.py,startup_config.json} ~/oem_startup/
sudo cp ~/oem-monorepo/vehicle/mkviii/software/lv/dashboard/pi/oem-startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable oem-startup.service
```

The service runs as user `oemdashboard` and expects the files in `/home/oemdashboard/oem_startup/`.
If you use a different user or folder, edit `User=`, `WorkingDirectory=` and
`ExecStart=` in the `.service` file.

Because the service runs as `oemdashboard`, deploys can only write to places `oemdashboard` can
write to (e.g. anywhere under `/home/oemdashboard`).

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
    "repo_dir": "/home/oemdashboard/oem-monorepo",
    "repo_url": "https://github.com/olin-electric-motorsports/oem-monorepo.git",
    "git_remote": "origin",
    "branch": "main",
    "git_timeout_s": 60,
    "dashboard_path": "/home/oemdashboard/oem-monorepo/vehicle/mkviii/software/lv/dashboard/dashboard.py",
    "python_executable": "/usr/bin/python3",
    "restart_dashboard_on_exit": true,
    "restart_delay_s": 3
}
```

| Key | Description |
| --- | --- |
| `deploy_dir` | Temporary deploy directory. **Must match `remote_deploy_dir` in the laptop's `deploy/config.json`.** |
| `repo_dir` | Where the repo lives on the Pi. |
| `repo_url` | Repo to clone if `repo_dir` doesn't exist yet. Also used by `set_branch.py` to check branches exist. |
| `git_remote` | Remote name to sync from. Default `origin`. |
| `branch` | Branch to sync to on boot. Change it from the laptop with `deploy/set_branch.py`. |
| `git_timeout_s` | Seconds before a git command is abandoned (e.g. no network). Default 60. |
| `dashboard_path` | Python file to run as the dashboard. |
| `python_executable` | Python interpreter to run it with (e.g. a virtualenv's `bin/python`). |
| `restart_dashboard_on_exit` | Restart the dashboard if it exits or crashes. Default `true`. |
| `restart_delay_s` | Seconds to wait before restarting. Default 3. |

### Deploy directory format

Written by `deploy.py`; documented here for reference:

```
deploy/
├─ config.json
├─ files/
│  ├─ dashboard.py
│  ├─ myscript.py
```

```json
[
    {"file_name": "dashboard.py", "file_path": "/home/oemdashboard/dashboard.py"},
    {"file_name": "myscript.py", "file_path": "/home/oemdashboard/mystuff/myscript.py"}
]
```

Missing parent folders are created. Each file is copied to a temp name and then
renamed, so losing power mid-copy never leaves a half-written file. If some
files fail to copy, the errors are logged and the deploy directory is still
cleared, so one bad deploy can't block GitHub syncing on later boots.
