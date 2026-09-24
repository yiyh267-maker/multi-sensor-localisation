# Release copy setup

This directory is a source distribution of the Raspberry Pi ROS2 localisation project. Local Python environments, ROS2 build/install directories, logs, editor caches and nested Git history are excluded. Driver source, robot resources, existing maps and upstream licence files are retained.

The intended runtime is the Raspberry Pi with Ubuntu and ROS2 Jazzy. This copy has not yet been rebuilt or tested with the physical sensors. The existing launch scripts expect the project at `~/rtls_project` and depend on ROS2 packages and workspace installation directories recreated on that machine.

## Configure the map key

Both `display_bridge/bridge_server.py` and the separate `Display/app.py` read `AMAP_API_KEY` from the process environment. No real key is included in this distribution. The main demo uses `display_bridge/bridge_server.py`.

On the Raspberry Pi, in Bash:

```bash
cd ~/rtls_project
cp .env.example .env
nano .env
```

Set `AMAP_API_KEY` to your AMap JavaScript API key, keeping it quoted. Then export the configuration in the same shell used to start the demo:

```bash
set -a
source .env
set +a
```

After installing the project dependencies and building its ROS2 workspaces, use the existing launch script:

```bash
bash scripts/start_demo1.sh
```

The `.env` file is not loaded automatically by Python. Repeat the export step in each new launch shell. If a service manager starts the dashboard, configure the environment there instead.

An unset or blank key returns HTTP 503 with a configuration message when opening the dashboard. With a configured key, the template supplies it to the same AMap script URL used by the original project. Position calculations, ROS2 callbacks, motor commands and localisation parameters are unchanged.

The key is visible to the browser when the map loads. Environment configuration keeps it out of repository source; it does not hide a browser API key from website visitors. Configure the appropriate usage restrictions in your AMap account.

## Before publishing

- Keep `.env` local. Only `.env.example` belongs in the repository.
- Preserve upstream author notices and licence files. No new project-wide licence is assigned by this preparation step.
- Follow the source-derived build procedure in `docs/INSTALL.md`; its target-device verification is pending.
- Rebuild and test the source distribution on the target Raspberry Pi. Generated `install/` trees and `display_bridge/venv` are intentionally absent.
- Review upstream packages before publication. Nested `.git` metadata was excluded from this copy so the source can be included as ordinary files; the original project is unchanged.

The Word report and cleanup backups are not included. Selected evidence images have been extracted from the report into `docs/images`, alongside owner-supplied hardware photographs; their sources are recorded in `docs/image_sources.json`.
