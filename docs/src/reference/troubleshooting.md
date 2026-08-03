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
| `[IMU] read error: 123` — just a number | That number is an SHTP *report id* the BNO085 driver does not recognise (123 = 0x7B, which is not a report the chip emits at all), and the bare digits are what a Python `KeyError` prints. So the sensor is there and answering; the byte stream reaching the driver is corrupted or has lost frame alignment. In order: something else on the I²C bus (a monitor tool running while the service is up — stop the service first), wiring or motor noise, a stale `dtparam=i2c_arm_baudrate=10000` left over from the BNO055, or a sagging supply. The line is throttled to one every 5 s, with a running count. Measure it rather than guessing: `python tools/imu_monitor.py --seconds 60` counts every packet, diagnoses the bad ones, and tells you whether each was a single flipped bit (a physical-layer fault — cable, clock, pull-ups) or something else. Run it for the same duration before and after a change, or the two rates are not comparable. |
| The heading row switched to the GPS course on its own | The IMU stopped producing valid samples for longer than **Reading timeout** (Settings → IMU, 2 s by default), so it stopped being believed — the journal says so on the way out and again on the way back. Deliberate: the reader survives bus errors by retrying, which means a dead sensor otherwise looks exactly like one reporting a heading that is not changing. |
| The calibration pips vanished but the rover drives fine | Same thing. The pips are only shown while the IMU is currently answering, because three pips beside a bearing that stopped updating is the dashboard being reassuring about a sensor that is not there. |
| The rover flaps between IMU heading and GPS course | A marginal I²C bus dropping samples in bursts. Fix the bus, and meanwhile raise **Reading timeout** so short gaps ride through — it is live, and 0 disables the check entirely. |
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
