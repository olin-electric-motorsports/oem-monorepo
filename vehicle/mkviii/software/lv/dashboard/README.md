# Dashboard

Driver dashboard code for the MK VIII, which runs on a Raspberry Pi Zero W in the car,
plus the scripts that put code on that Pi over Wi-Fi: deploy local files to it, have it
pull a GitHub branch on every boot, roll back to earlier versions, and manage who can ssh in.

**Docs:** [Dashboard OTA Updates](https://docs.superhuman.com/d/MK-VIII-Documentation_dbOBIYXOHlq/Dashboard-OTA-Updates_suMvKQZK)
(setting up the Pi, deploying code, and how it all works).

- `dashboard.py`, `mkvi.dbc`, `dashboard.yml`, `can_logger`: the dashboard and its CAN files.
- `deploy/`: scripts you run on your laptop (`./deploy.sh`, `./ssh.sh`, `./versions.sh`, ... each takes `-h`).
- `pi/`: what gets installed on the Pi (the boot script, key sync, version store, systemd service).
- `tests/`: `bash tests/test_versions.sh` tests the version store locally.
