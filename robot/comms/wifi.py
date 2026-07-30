"""Joining the Pi to a WiFi network, from the base station.

Why this exists: everything bulk — config, layouts, routines, FPV video — rides
WiFi (ip_link.py, video_udp.py), and WiFi is the one thing that changes every
time the rover goes somewhere new. Without this, arriving at a venue means an
HDMI cable, a keyboard, or an SD card in a laptop, for each rover, before the
dashboard can do anything but drive.

The radio is what makes it possible: it is always there, it needs no network,
and it is how the rover can be told about a network it is not yet on. Same
chicken-and-egg the base_host bootstrap solves (tuning.BOOTSTRAP_PATHS), one
layer down.

--- this is NOT configuration ---
Deliberately not a tunable path. Config is snapshotted, echoed to every
connected browser and saved to tuning.json; a WiFi password must be in none of
those places. So credentials travel as their own message, are handed straight to
NetworkManager, and are never stored, echoed or logged by this code. What comes
back is only ever what a network scan would tell you anyway: SSID, signal, an IP.

NetworkManager owns the profile it creates, which is the point — it is stored
under /etc/NetworkManager/system-connections (root-only, as it should be) and
the rover rejoins that network by itself on the next power-up.

--- what runs, and what happens when it isn't there ---
`nmcli`, the NetworkManager CLI, present by default on Raspberry Pi OS Bookworm
and later. A Pi running the older dhcpcd/wpa_supplicant stack has no nmcli, and
this says so plainly rather than half-working: an operator who is told "this
image manages WiFi with wpa_supplicant" goes and edits the right file, while one
whose Connect button silently does nothing learns nothing.

Everything here shells out with a timeout and returns a dict. Nothing raises:
this is called from a robot that is holding a drivetrain at neutral.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from typing import Any, Dict, List, Optional

# Long enough for a DHCP lease on a slow venue network, short enough that a
# stuck association reports back inside the operator's patience.
CONNECT_TIMEOUT = 35.0
SCAN_TIMEOUT = 15.0
QUERY_TIMEOUT = 5.0

# What the radio will carry back. A busy venue shows fifty access points and
# nobody scrolls a list of fifty on a 7" panel — the strongest dozen is the
# useful part, and the rest is airtime.
MAX_NETWORKS = 12


def available() -> bool:
    """Is there an nmcli to drive? Checked live, not at import: an image can
    gain NetworkManager between one boot and the next."""
    return shutil.which("nmcli") is not None


def _run(args: List[str], timeout: float, secret: str = "") -> subprocess.CompletedProcess:
    """Run a command, never raise, and never let `secret` reach a log.

    The scrub is belt-and-braces — nmcli does not echo the password back in its
    own errors — but "the password ended up in the journal" is the kind of
    mistake that is discovered much later by somebody else, so it is not left to
    nmcli's discretion.
    """
    try:
        done = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        return subprocess.CompletedProcess(args, 127, "", "nmcli is not installed")
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(args, 124, "", "timed out")
    except Exception as e:  # a robot must not fall over because a shell-out did
        return subprocess.CompletedProcess(args, 1, "", str(e))
    if secret:
        done.stdout = done.stdout.replace(secret, "***")
        done.stderr = done.stderr.replace(secret, "***")
    return done


def _terse(line: str) -> List[str]:
    r"""Split one `nmcli -t` line into fields.

    nmcli's terse output is colon-separated with `\:` for a literal colon — and
    an SSID may contain one. Splitting on a bare `:` mangles exactly the network
    somebody needs, so the escape is honoured.
    """
    out: List[str] = []
    field = []
    escaped = False
    for ch in line:
        if escaped:
            field.append(ch)
            escaped = False
        elif ch == "\\":
            escaped = True
        elif ch == ":":
            out.append("".join(field))
            field = []
        else:
            field.append(ch)
    out.append("".join(field))
    return out


def _device() -> Optional[str]:
    """The name of the WiFi interface, or None.

    Asked rather than assumed: `wlan0` is usual on a Pi but a USB adapter shows
    up as something else, and a hard-coded name is a Connect button that fails
    on hardware nobody tested it on.
    """
    done = _run(["nmcli", "-t", "-f", "DEVICE,TYPE", "device"], QUERY_TIMEOUT)
    if done.returncode != 0:
        return None
    for line in done.stdout.splitlines():
        parts = _terse(line)
        if len(parts) >= 2 and parts[1] == "wifi":
            return parts[0]
    return None


def status() -> Dict[str, Any]:
    """What network this Pi is on, if any.

    Small on purpose — this crosses the radio. Everything in it is information a
    scanner standing next to the rover would have anyway.
    """
    if not available():
        return {"ok": False, "managed": False,
                "error": "NetworkManager (nmcli) is not installed on this Pi"}
    device = _device()
    if device is None:
        return {"ok": False, "managed": True, "error": "no WiFi interface found"}

    out: Dict[str, Any] = {"ok": True, "managed": True, "device": device,
                              "ssid": None, "ip": None, "signal": None}
    done = _run(["nmcli", "-t", "-f", "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS",
                 "device", "show", device], QUERY_TIMEOUT)
    for line in done.stdout.splitlines():
        parts = _terse(line)
        if len(parts) < 2:
            continue
        key, value = parts[0], parts[1]
        if key == "GENERAL.CONNECTION" and value not in ("", "--"):
            out["ssid"] = value
        elif key.startswith("IP4.ADDRESS") and value not in ("", "--"):
            out["ip"] = value.split("/")[0]
        elif key == "GENERAL.STATE":
            out["state"] = value

    # Signal comes from the scan list rather than the device, because that is
    # where nmcli keeps it. Absent is fine: it is a nicety, not the answer.
    if out["ssid"]:
        done = _run(["nmcli", "-t", "-f", "ACTIVE,SIGNAL", "device", "wifi"], QUERY_TIMEOUT)
        for line in done.stdout.splitlines():
            parts = _terse(line)
            if len(parts) >= 2 and parts[0] == "yes" and parts[1].isdigit():
                out["signal"] = int(parts[1])
                break
    return out


def scan() -> Dict[str, Any]:
    """The networks in range, strongest first.

    A list to pick from rather than a field to type into: an SSID typed from
    memory at a competition is how you spend ten minutes discovering it was
    "Venue-Guest" and not "Venue Guest".
    """
    if not available():
        return {"ok": False, "error": "NetworkManager (nmcli) is not installed on this Pi",
                "networks": []}
    # --rescan yes: the cached list can be minutes old, and "the network I am
    # standing next to isn't listed" is the one failure that makes an operator
    # stop trusting the button.
    done = _run(["nmcli", "--rescan", "yes", "-t", "-f", "SSID,SIGNAL,SECURITY",
                 "device", "wifi", "list"], SCAN_TIMEOUT)
    if done.returncode != 0:
        return {"ok": False, "error": (done.stderr or "scan failed").strip(),
                "networks": []}

    seen: Dict[str, Dict[str, Any]] = {}
    for line in done.stdout.splitlines():
        parts = _terse(line)
        if len(parts) < 3 or not parts[0]:
            continue  # a hidden network reports an empty SSID; nothing to offer
        ssid = parts[0]
        signal = int(parts[1]) if parts[1].isdigit() else 0
        secure = parts[2] not in ("", "--", "none")
        # One entry per name, keeping the strongest: a venue with three access
        # points on one SSID is one network as far as anybody choosing is
        # concerned.
        if ssid not in seen or signal > int(seen[ssid]["signal"] or 0):
            seen[ssid] = {"ssid": ssid, "signal": signal, "secure": secure}
    networks = sorted(seen.values(), key=lambda n: -int(n["signal"] or 0))
    return {"ok": True, "networks": networks[:MAX_NETWORKS]}


_COUNTRY = re.compile(r"^[A-Z]{2}$")


def set_country(code: str) -> Optional[str]:
    """Set the WiFi regulatory domain, or return why not.

    Worth having because of a specific, baffling failure: a Pi with no country
    set has 5 GHz soft-blocked by rfkill, so the venue's 5 GHz network is simply
    absent from the scan and everything looks broken for a reason nothing on
    screen mentions.
    """
    code = (code or "").strip().upper()
    if not _COUNTRY.match(code):
        return f"{code!r} is not a two-letter country code"
    if shutil.which("raspi-config") is None:
        return "raspi-config is not available to set the WiFi country"
    done = _run(["raspi-config", "nonint", "do_wifi_country", code], QUERY_TIMEOUT)
    if done.returncode != 0:
        return (done.stderr or "could not set the WiFi country").strip()
    return None


def connect(ssid: str, psk: str = "", hidden: bool = False) -> Dict[str, Any]:
    """Join `ssid`, and say what happened.

    Blocking for as long as an association plus a DHCP lease takes, which is why
    the caller runs it off the control loop.

    On failure NetworkManager is asked to bring the previous connection back up.
    A rover that dropped its old network to fail onto a mistyped one — and so
    lost FPV and configuration until somebody walks out to it — is a worse
    outcome than the one the operator was trying to fix.
    """
    if not available():
        return {"ok": False, "error": "NetworkManager (nmcli) is not installed on this Pi"}
    ssid = (ssid or "").strip()
    if not ssid:
        return {"ok": False, "error": "no network name given"}
    device = _device()
    if device is None:
        return {"ok": False, "error": "no WiFi interface found"}

    was = str(status().get("ssid") or "")
    args = ["nmcli", "--wait", str(int(CONNECT_TIMEOUT)), "device", "wifi",
            "connect", ssid, "ifname", device]
    if psk:
        args += ["password", psk]
    if hidden:
        args += ["hidden", "yes"]
    print(f"[wifi] joining {ssid!r}…")  # the name, never the password
    done = _run(args, CONNECT_TIMEOUT + 5.0, secret=psk)

    if done.returncode != 0:
        error = (done.stderr or done.stdout or "could not join").strip().splitlines()
        why = error[-1] if error else "could not join"
        print(f"[wifi] {ssid!r} refused: {why}")
        if was and was != ssid:
            # Best effort, and unreported: the operator's problem is the network
            # they failed to join, not the tidy-up.
            _run(["nmcli", "connection", "up", was], QUERY_TIMEOUT)
        return {"ok": False, "error": why, **{k: v for k, v in status().items()
                                              if k in ("ssid", "ip", "signal")}}

    now = status()
    print(f"[wifi] on {now.get('ssid')!r} at {now.get('ip')}")
    return {"ok": True, "ssid": now.get("ssid"), "ip": now.get("ip"),
            "signal": now.get("signal")}


def forget(ssid: str) -> Dict[str, Any]:
    """Delete a stored profile.

    The counterpart to connect, and not just tidiness: a rover that keeps
    rejoining last venue's network is a rover that will not be on the one in
    front of it.
    """
    if not available():
        return {"ok": False, "error": "NetworkManager (nmcli) is not installed on this Pi"}
    ssid = (ssid or "").strip()
    if not ssid:
        return {"ok": False, "error": "no network name given"}
    done = _run(["nmcli", "connection", "delete", ssid], QUERY_TIMEOUT)
    if done.returncode != 0:
        return {"ok": False, "error": (done.stderr or "nothing to forget").strip()}
    return {"ok": True, "forgot": ssid}
