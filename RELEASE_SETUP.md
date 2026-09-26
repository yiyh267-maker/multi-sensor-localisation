# Dashboard configuration

The repository contains the source code, configuration and resources for the Raspberry Pi ROS2 localisation project. Create the local Python environment and ROS2 build outputs using the [installation instructions](docs/INSTALL.md).

The system runs on a Raspberry Pi with Ubuntu and ROS2 Jazzy. The launch scripts expect the project at `~/rtls_project` and use the installed ROS2 workspace overlays.

## Configure the map key

Both `display_bridge/bridge_server.py` and the separate `Display/app.py` read `AMAP_API_KEY` from the process environment. No real key is included in the repository. The main demo uses `display_bridge/bridge_server.py`.

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

An unset or blank key returns HTTP 503 with a configuration message when opening the dashboard. With a configured key, the dashboard loads the AMap JavaScript API through its page template.

The key is visible to the browser when the map loads. Environment configuration keeps it out of repository source; it does not hide a browser API key from website visitors. Configure the appropriate usage restrictions in your AMap account.

## Local files and configuration

- Keep `.env` local. Only `.env.example` belongs in the repository.
- Build the ROS2 workspaces and create `display_bridge/venv` locally; generated environments and build directories are excluded from Git.
- Preserve upstream author notices and licence files. See [THIRD_PARTY.md](THIRD_PARTY.md) for the component inventory.

Test screenshots from my project report and hardware photographs are included in `docs/images`. Their sources are recorded in [image_sources.json](docs/image_sources.json).
