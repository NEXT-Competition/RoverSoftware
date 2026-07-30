# The radio protocol

Newline-delimited JSON over the shared XBee channel. `to` addresses a robot (or
`"all"`); robots stamp telemetry with `from`.

```jsonc
// base station → robot
{"type": "drive", "throttle": 0.5, "steer": -0.2, "to": "rover1"}   // arcade
{"type": "drive", "left": 0.4, "right": 0.6, "to": "rover1"}        // direct tank
{"type": "mode", "mode": "teleop", "to": "rover1"}                  // or object_align / waypoint / routine
{"type": "route", "waypoints": [[lat, lon], "..."], "to": "rover1"}
{"type": "estop", "to": "rover1"}                                   // latch motors off
{"type": "clear_estop", "to": "rover1"}
{"type": "get_config", "to": "rover1"}                              // every tunable parameter
{"type": "set_config", "config": {"align.pid.kp": 0.6}, "to": "rover1"}

// documents: structure rather than scalars, sent as numbered fragments
{"type": "get_layout", "to": "rover1"}                              // what this build HAS
{"type": "get_routines", "to": "rover1"}                            // its state machines
{"type": "put_layout", "txid": "B1", "seq": 0, "n": 3, "part": "{\"vers…", "to": "rover1"}
{"type": "select_routine", "id": "collect", "to": "rover1"}
{"type": "routine_cmd", "cmd": "start", "to": "rover1"}              // start | stop | restart
{"type": "routine_event", "name": "go", "to": "rover1"}              // advance a "when I press" transition
{"type": "jog", "mech": "intake", "power": 0.3, "to": "rover1"}      // bench test, teleop only

// WiFi: the one config path that must work with NO WiFi, so it may ride the radio
{"type": "get_wifi", "to": "rover1"}                                 // what network am I on
{"type": "scan_wifi", "to": "rover1"}                                // what can I see
{"type": "set_wifi", "ssid": "Venue-Guest", "psk": "…", "country": "GB", "to": "rover1"}
{"type": "forget_wifi", "ssid": "Venue-Guest", "to": "rover1"}

// robot → base station (telemetry, ~5 Hz)
{"type": "telemetry", "from": "rover1", "mode": "teleop", "estop": false,
 "left": 0.4, "right": 0.6, "battery": 87.0,                          // what it was TOLD
 "lat": 37.77, "lon": -122.41, "heading": 30.0,
 // what the wheels DID, and what the speed loop did about it. Absent entirely
 // on a build with no encoders wired, which is the default.
 "enc": {"rpm": {"left": 118.2, "right": 117.9}, "mode": "match",
         "tl": -0.01, "tr": 0.01}}                                    // "fault":"left" if one stalled
{"type": "config", "from": "rover1", "config": {"align.pid.kp": 0.6},
 "rejected": {}, "restart": [], "save_error": null}
{"type": "layout_result", "from": "rover1", "ok": true, "errors": [], "restart_required": true}
{"type": "routines_result", "from": "rover1", "ok": false,
 "errors": ["state 'shoot': unknown mechanism 'intak'"]}
{"type": "wifi", "from": "rover1", "ok": true, "ssid": "Venue-Guest",
 "ip": "10.0.0.9", "signal": 82}                                     // never carries a psk
```

## WiFi is the exception that proves the rule

Everything bulk goes over WiFi and never the radio — a config snapshot is half a
second of airtime on a channel shared with every rover. `set_wifi` is the one
command that inverts that, because a rover cannot be told about a network over
that network. The base station sends it over WiFi when there is a link and falls
back to the radio when there is not, and the reply comes back the same way (a
rover that just failed to join has nothing else to answer over).

Two consequences worth stating plainly:

**The password crosses the radio in the clear** when the rover is not already on
WiFi, because the XBee runs transparent mode with no encryption unless you have
configured its AES key (`ATEE`/`ATKY`). The dashboard says so at the point of
typing. A rover that is already on some network is moved to another one over
WiFi, and nothing goes on air.

**It is not a tunable path.** Config is snapshotted, echoed to every connected
browser and saved to `tuning.json`; a credential belongs in none of those. So
WiFi travels as its own message type, is handed straight to NetworkManager, and
is stripped from the reply — the robot's `wifi` frame carries only what a scanner
standing next to the rover would see anyway.

## Config is merged; documents are not

A `config` payload is independent scalars, so half a snapshot is a valid smaller
snapshot. The payload is a *partial* set the base station merges: everything in
reply to `get_config`, only the applied fields after a `set_config`. A full one
is about 0.4 s of airtime at 57600 baud, which is why it is requested explicitly
and never polled.

A layout is a **tree**, and half a tree is a robot with one drive motor. Layouts
and routines are sliced into numbered fragments and nothing is applied until
every fragment arrives. The robot replies with a verdict and echoes the *stored*
copy back, since the validator clamps and what was saved is not always what was
sent.

`config` frames carry flat dotted paths into `RobotConfig`; `robot/tuning.py`
decides which exist and clamps every value, so a browser cannot reach an
arbitrary attribute.

## Video does not ride the radio

57600 baud cannot carry a camera. FPV goes over WiFi as JPEG-over-UDP, keeping
the *freshest* frame rather than every frame — a corrupted or incomplete frame
is discarded and the next one shown, which is what keeps glass-to-glass latency
low. The XBee stays the long-range control link.

Framing, shared by both ends
(`robot/comms/video_udp.py`):

```text
magic 'UCV1' | robot_id[16] | frame_seq(u32) | chunk_index(u16) | chunk_count(u16)
```

## Safety, built in

Teleop stops if commands stop arriving (`command_timeout`), and e-stop overrides
every mode until cleared. Position fields appear in telemetry once a
`pose_provider` — i.e. GPS — is attached on the robot.
