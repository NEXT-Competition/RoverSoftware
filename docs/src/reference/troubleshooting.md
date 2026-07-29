# When it misbehaves

| What you see | What it usually is |
|---|---|
| The dashboard exits at start-up with a hint | Neither `--port` nor `--sim` was given. It refuses to start empty rather than showing a fleet that is not there. |
| No robots in the list, but the bridge is running | Nothing is reporting in. Run `tools/xbee_monitor.py` at both ends — raw bytes instead of JSON means a baud or wiring mismatch. |
| `LINK` climbing on the board | Telemetry is not arriving. Range, antenna, or a robot that has stopped. Teleop's own timeout will have parked it already. |
| Motors do not move, everything else works | E-stop latched (the fleet row says so), or the ESC never armed — check the motor's *Type* is **ESC** and give it its *arm hold* at neutral. |
| One side of the robot runs backwards | *Inverted* is unticked on the mirrored side. It is per-motor, on the motor's card. |
| Both motors do the same thing | They are on the same PWM channel — except the Hardware tab refuses that, so check they are not both assigned to the same *side*. |
| A hardware change did nothing | Layouts take effect on the next start. `just restart`, or `sudo systemctl restart roversoftware-robot`. |
| A tuning value came back different | It was clamped, not refused. The field shows what the robot is actually using. |
| Waypoint mode steers but does not move | A steered chassis with *pivot creep* at zero. It cannot turn on the spot. |
| The map is blank | No route to the tile provider. Point `--tiles` at a local server, or build a cache first with `tools/fetch_tiles.py`. |
| The camera panel says "waiting for video" | FPV needs WiFi, not the radio. Start the robot with `--fpv --fpv-host <base-ip>`. If it is streaming at the wrong machine, fix **Base station host** under Tuning → FPV video: that setting goes over the radio and the feed re-aims without a restart. |
| Tuning, Hardware or Routines never fills in | Those are fetched from the robot over the radio, in fragments. The tab retries a few times and then offers **Ask again** — if that keeps failing, the robot is off, out of range, or on a build old enough not to answer. |
| Voice is dead but typing works | faster-whisper is not installed, or there is no microphone. The badges at the top of the Command screen say which. |

## Install and packaging

| What you see | What it usually is |
|---|---|
| `apt` cannot find `roversoftware-robot` | The repository is not added, or `apt-get update` has not run since. Re-run the three lines in [Install from apt](../install/apt.md). |
| `NO_PUBKEY` or "not signed" from apt | The keyring file is missing or truncated. Re-fetch it into `/etc/apt/keyrings/roversoftware.asc` and check it starts with `-----BEGIN PGP PUBLIC KEY BLOCK-----`. |
| `dpkg -i` complains about dependencies | Use `sudo apt-get install ./file.deb` instead — apt resolves them, dpkg does not. |
| The service starts but nothing moves | `fusion_hat` is not installed, so the motor layer is mocked. `just bootstrap`. |
| The kiosk shows nothing but the bridge is up | `deno` is not on PATH, so `roversoftware-ui` cannot start. `just bs_host=... bootstrap-deno`. |
| An upgrade asked about `robot.env` | That is the conffile prompt. Keep your version unless the release notes say a new variable is required. |
| A release workflow run failed at "publish apt" | Usually the deploy token expired or the signing key secret is missing. See [Cutting a release](releasing.md). |

## Getting more out of the logs

```bash
journalctl -u roversoftware-robot -f              # the robot, live
journalctl -u roversoftware-basestation -f        # the bridge
journalctl -u roversoftware-ui -f                 # the Deno front door
journalctl -u roversoftware-robot --since "10 min ago" --no-pager
```

On the base station, the terminal it was started from prints the same, plus the
tile cache's decisions about what it fetched and what it served from disk.
