# Dashboard

Driver dashboard code, which runs on a Raspberry Pi.

- `dashboard.py`: the dashboard. The Pi runs it on boot.
- `mkvi.dbc`: CAN database the dashboard loads (from `/home/oemdashboard/mkvi.dbc` on the Pi).
- `dashboard.yml`: CAN API config for the dashboard's messages.
- `can_logger`: script that logs CAN traffic on the Pi.
- [`deploy/`](deploy/README.md): laptop-side scripts to deploy code to the Pi,
  change which branch it syncs to, and reboot it.
- [`pi/`](pi/README.md): the boot script and systemd service installed on the Pi.

Quick reference, from `deploy/`:

```shell
./deploy.py               # push local files to the Pi (applied on next boot)
./set_branch.py           # show the branch the Pi syncs to
./set_branch.py my-branch # change it
./reboot_pi.py            # reboot, which syncs the Pi with GitHub
```
