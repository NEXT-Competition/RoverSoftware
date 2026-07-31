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
| The rover curves when told to go straight | Two motors never match on the same throttle. Fit [wheel encoders](wiring.md#wheel-encoders-optional) and set Tuning → Wheel speed matching → Mode to `match`. Without encoders, trim it by hand with the mirrored motor's *Forward cap*. |
| No `wheels` row on the driving view | This build has no encoder pins set, or `fusion_hat` is missing on the robot — the journal says which at start-up, and if it is the library then the motors are mocked too. Pins are claimed at start-up, so a newly saved layout needs a restart. |
| The `wheels` row says "not turning — matching off" | A wheel was commanded and its encoder never moved: the encoder is unplugged, its pins are wrong, or the wheel is genuinely stalled. The loop opened itself deliberately; it clears when the drivetrain next stops. |
| The count stays at exactly 0 while the wheel turns | Watch the `states n/4` field. Stuck at 1/4 means neither channel is moving — almost always an unpowered encoder (it needs the supply wire, not just A/B/ground). 2/4 means one dead channel. `encoder_monitor.py` names the fault when you Ctrl-C. |
| `encoder_monitor.py` says "Failed to add edge detection" | Something else holds the pins — almost always the robot service, which claims them at start-up whenever a layout or `robot.env` names them. `sudo systemctl stop roversoftware-robot`, then run the tool. |
| The `wheels` row keeps going blank while driving | Edges are being lost faster than 10% of a window, so the speed is no longer trustworthy and the loop opens on purpose — the journal says "losing edges". Move the decoding into the kernel: `just encoder-overlay A B` per encoder, then `just reboot`. |
| `just encoder-devices` lists nothing after a reboot | The overlay line never took. Check it is in `/boot/firmware/config.txt` (not `/boot/config.txt` on Bookworm and later) and that the firmware actually rebooted. |
| The RPM readout is wildly wrong but steady | *Counts per rev* is wrong. Measure it rather than deriving it: `tools/encoder_monitor.py`, turn the wheel one turn, read the count. `match` mode still works with a wrong value; `velocity` mode does not. |
| A tuning value came back different | It was clamped, not refused. The field shows what the robot is actually using. |
| Waypoint mode steers but does not move | A steered chassis with *pivot creep* at zero. It cannot turn on the spot. |
| The map is blank | No route to the tile provider. Point `--tiles` at a local server, or build a cache first with `tools/fetch_tiles.py`. |
| The camera panel says "waiting for video" | FPV needs WiFi, not the radio — the rover has to be on the same network. Everything else is under Tuning → FPV video, which goes over the radio: switch **Streaming enabled** on, and check **Base station host** is this machine. Neither needs a restart. |
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
