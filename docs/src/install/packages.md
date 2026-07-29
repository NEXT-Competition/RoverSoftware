# What each package puts where

Two packages, both `Architecture: all` because the code is pure Python. Knowing
what lands where is what lets you debug a rover you did not personally set up.

## `roversoftware-robot`

Everything that moves. Install it on the Pi bolted to the vehicle.

| Path | What it is |
|---|---|
| `/opt/roversoftware/` | The application: `robot/`, `tools/`, `run_robot.py` |
| `/etc/roversoftware/robot.env` | **conffile** — this rover's id, serial port, start mode |
| `/lib/systemd/system/roversoftware-robot.service` | The unit, enabled and started on install |
| `/var/lib/roversoftware/network.rpk` | The IMX500 detection network, if one was bundled |
| `/var/lib/roversoftware/labels.txt` | Its class names |

Depends on `python3 (>= 3.7)` and `python3-serial`. The SunFounder `fusion_hat`
library is deliberately **not** a dependency — it is not in any archive, and the
motor layer mocks it when absent so the service still starts and reports.

```bash
systemctl status roversoftware-robot
journalctl -u roversoftware-robot -f
sudo systemctl restart roversoftware-robot
```

The vision network is **not** a conffile, on purpose: dpkg replaces it on
upgrade, because the repository copy is the model of record. A locally-installed
experiment in `/var/lib` is therefore overwritten by the next upgrade.

## `roversoftware-basestation`

The dashboard. Install it on a laptop, or on a Pi with a screen to get the
kiosk.

| Path | What it is |
|---|---|
| `/opt/roversoftware-basestation/` | `basestation/`, `robot/`, `run_basestation.py`, `kiosk.sh` |
| `/opt/roversoftware-basestation/ui/` | The built touch UI and its Deno server |
| `/etc/roversoftware/basestation.env` | **conffile** — radio port, ports, sim flag |
| `/lib/systemd/system/roversoftware-basestation.service` | The Python bridge |
| `/lib/systemd/system/roversoftware-ui.service` | The Deno front door |
| `/etc/xdg/autostart/roversoftware-kiosk.desktop` | Launches Chromium full-screen on login |

**Two services, not one.** The bridge listens on `RS_WEB_PORT` (default 8001)
and owns the radio, the gamepad and the tile cache. The Deno UI listens on
`RS_UI_PORT` (default 8000) — the port the kiosk and any tablet connect to — and
proxies `/ws` and `/tiles` back to the bridge.

```bash
systemctl status roversoftware-basestation roversoftware-ui
sudo nano /etc/roversoftware/basestation.env
sudo systemctl restart roversoftware-basestation roversoftware-ui
```

Useful settings in `basestation.env`:

| Variable | Effect |
|---|---|
| `RS_XBEE_PORT` / `RS_XBEE_BAUD` | Match your radios |
| `RS_SIM=1` | Fake robots — test the kiosk with no radio attached |
| `RS_NO_CONTROLLER=1` | A pure touch base station, no gamepad reader |
| `RS_WEB_PORT` / `RS_UI_PORT` | Move either service off its default port |

Requires Debian **Bookworm** or newer, for the packaged FastAPI and uvicorn.

## Removing them

```bash
sudo apt-get remove roversoftware-robot        # keeps /etc/roversoftware
sudo apt-get purge  roversoftware-robot        # takes the config too
```

`remove` stops and disables the services and deletes `/opt`, leaving your
config. `purge` is what you want when you are handing the Pi to someone else.

## The other ways to install

The `.deb` is the right answer for a machine that stays built. Two others exist:

- **`just deploy`** — builds the package from your working tree and installs it
  over SSH. This is the development loop, not a distribution channel. `just
  sync` is faster still: it rsyncs changed code straight into `/opt` and
  restarts, with no packaging round-trip. See
  [step 9](../bringup/deploy.md).
- **From a clone** — `pip install -r requirements.txt` and run
  `python run_robot.py` directly. Right for a laptop you are developing on,
  wrong for a rover that has to come up on its own after a power cut.
