# megamind remote desktop over Sunshine/Moonlight

Remote GPU-accelerated desktop on megamind, streamed to the laptop. megamind has no
monitor attached and no dummy HDMI plug. The NVIDIA driver fakes one.

Set up 2026-08-30.

## Final state

| Piece | Value |
|---|---|
| X server | A6000 at `PCI:1:0:0`, virtual output `DP-0` |
| Resolution | 1920x1080@60, set by a synthetic EDID |
| Session | GDM autologin as `ben`, X11, `Class=user` |
| Host | Sunshine 2026.516.143833, `encoder = nvenc`, `capture = x11` |
| Client | Moonlight on the laptop, host `100.114.188.57` |
| Transport | Tailscale, direct (not DERP), 27 ms RTT, ~64 Mbps measured |

megamind runs Ubuntu 22.04.5, GNOME 42.9, driver 580.126.20, 3x RTX A6000 plus an
ASPEED BMC at `e4:00.0`.

## Why a fake monitor is needed

Two separate problems, both invisible until you look:

- Bug: GDM binds X to the ASPEED BMC, not to a GPU. The log line is
  `modeset(0): using drv /dev/dri/card0`. Everything renders in software and NVENC
  never comes into play.
- Bug: an NVIDIA output with `ConnectedMonitor` but no EDID reports 1024x768 as its
  native mode. X sets the requested mode, then mutter reads the native mode and snaps
  the desktop down to 1024x768 about 14 seconds later.

Fix for both: bind X to the A6000 by BusID and feed that output a synthetic EDID whose
preferred timing is the resolution you want.

## 1. Generate the EDID

The resolution of the virtual display comes from the EDID's first detailed timing.
This script writes a 128-byte EDID 1.3 block for 1920x1080@60 using CEA-861 timing:

```python
"""Build a 128-byte EDID 1.3 block advertising 1920x1080@60 (CEA-861) as preferred."""
import struct

# CEA-861 1080p60: Modeline 148.50 1920 2008 2052 2200  1080 1084 1089 1125 +hsync +vsync
PCLK_KHZ = 148500
HACT, HFP, HSW, HTOT = 1920, 88, 44, 2200
VACT, VFP, VSW, VTOT = 1080, 4, 5, 1125
HBLANK, VBLANK = HTOT - HACT, VTOT - VACT
HMM, VMM = 600, 340  # physical size in mm


def mfg_id(letters: str) -> bytes:
    v = 0
    for ch in letters:
        v = (v << 5) | (ord(ch) - ord("A") + 1)
    return struct.pack(">H", v)


def detailed_timing() -> bytes:
    d = bytearray(18)
    d[0:2] = struct.pack("<H", PCLK_KHZ // 10)
    d[2] = HACT & 0xFF
    d[3] = HBLANK & 0xFF
    d[4] = ((HACT >> 8) << 4) | (HBLANK >> 8)
    d[5] = VACT & 0xFF
    d[6] = VBLANK & 0xFF
    d[7] = ((VACT >> 8) << 4) | (VBLANK >> 8)
    d[8] = HFP & 0xFF
    d[9] = HSW & 0xFF
    d[10] = ((VFP & 0x0F) << 4) | (VSW & 0x0F)
    d[11] = ((HFP >> 8) << 6) | ((HSW >> 8) << 4) | ((VFP >> 4) << 2) | (VSW >> 4)
    d[12] = HMM & 0xFF
    d[13] = VMM & 0xFF
    d[14] = ((HMM >> 8) << 4) | (VMM >> 8)
    d[15] = 0
    d[16] = 0
    d[17] = 0x1E  # digital separate sync, +hsync +vsync
    return bytes(d)


def descriptor(tag: int, text: str) -> bytes:
    return bytes([0, 0, 0, tag, 0]) + text.ljust(13)[:13].encode("ascii")


def range_limits() -> bytes:
    # 5-byte header, vrate 50-75 Hz, hrate 30-140 kHz, max pclk 300 MHz, no 2nd timing
    return bytes([0, 0, 0, 0xFD, 0, 50, 75, 30, 140, 30, 0x00, 0x0A] + [0x20] * 6)


e = bytearray()
e += bytes([0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00])  # header
e += mfg_id("LNX")                       # manufacturer
e += struct.pack("<H", 0x1081)           # product code
e += struct.pack("<I", 0x00000001)       # serial
e += bytes([1, 34])                      # week 1, year 2024
e += bytes([1, 3])                       # EDID 1.3
e += bytes([0x80, HMM // 10, VMM // 10, 0x78])  # digital, size cm, gamma 2.2
e += bytes([0x0A, 0x80, 0xB4, 0xA0, 0x59, 0x4A, 0x98, 0x25, 0x20, 0x50, 0x54])
e += bytes([0x00, 0x00, 0x00])           # no established timings
e += bytes([0x01, 0x01] * 8)             # no standard timings
e += detailed_timing()                   # preferred timing
e += range_limits()
e += descriptor(0xFC, "Virtual-1920")    # monitor name
e += descriptor(0x10, "")                # dummy
e += bytes([0])                          # no extensions
e += bytes([(256 - sum(e) % 256) % 256])  # checksum

assert len(e) == 128, len(e)
assert sum(e) % 256 == 0, "checksum"
open("edid-1920x1080.bin", "wb").write(bytes(e))
```

Two easy mistakes when hand-building an EDID: the range-limits descriptor must be
exactly 18 bytes (5-byte header, 5 rate/clock bytes, a `0x00` flag, `0x0A`, then six
`0x20` pad bytes), and the max pixel clock field is in units of 10 MHz, so 300 MHz is
`30`, not `3`.

Verify the detailed timing block against a known-good 1080p60 EDID:

```bash
xxd -s 54 -l 18 edid-1920x1080.bin
# 023a 8018 7138 2d40 582c 4500 5854 2100 001e
```

Install it:

```bash
sudo install -m 0644 -o root -g root edid-1920x1080.bin /etc/X11/edid-1920x1080.bin
```

## 2. Bind X to the A6000

`/etc/X11/xorg.conf`:

```
Section "ServerLayout"
    Identifier "Layout0"
    Screen 0 "Screen0"
EndSection

Section "Monitor"
    Identifier  "Monitor0"
    HorizSync   5.0 - 1000.0
    VertRefresh 5.0 - 1000.0
    Modeline "3840x1080R" 266.50 3840 3888 3920 4000 1080 1083 1093 1111 +hsync -vsync
    Modeline "2560x1440R" 241.50 2560 2608 2640 2720 1440 1443 1448 1481 +hsync -vsync
    Option "DPMS" "false"
EndSection

Section "Device"
    Identifier "A6000"
    Driver     "nvidia"
    BusID      "PCI:1:0:0"
    Option "ConnectedMonitor" "DP-0"
    Option "CustomEDID" "DP-0:/etc/X11/edid-1920x1080.bin"
    Option "IgnoreEDIDChecksum" "DP-0"
    Option "AllowEmptyInitialConfiguration" "true"
    Option "ModeValidation" "DP-0: AllowNonEdidModes, NoEdidMaxPClkCheck, NoMaxPClkCheck, NoHorizSyncCheck, NoVertRefreshCheck, NoVirtualSizeCheck, NoDFPNativeResolutionCheck, NoMaxSizeCheck, NoTotalSizeCheck"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device     "A6000"
    Monitor    "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080" "2560x1440R" "3840x1080R"
    EndSubSection
EndSection

Section "ServerFlags"
    Option "AutoAddGPU" "false"
EndSection
```

`BusID "PCI:1:0:0"` is the A6000 at `01:00.0`. `AutoAddGPU false` keeps the ASPEED out
of the layout. Do not add a `MetaModes` option: a mode name that does not match what the
driver derives from the EDID stops X from starting, and the EDID already picks the mode.

## 3. GDM autologin

Sunshine runs as your user and captures a real session, so one has to exist without a
keyboard present. `/etc/gdm3/custom.conf`:

```
[daemon]
WaylandEnable=false
AutomaticLoginEnable=true
AutomaticLogin=ben

[security]

[xdmcp]

[chooser]

[debug]
```

Wayland is off because GNOME 42 has no usable headless virtual output and Sunshine's
X11 capture path is the reliable one here.

- Bug: restarting `gdm3` several times in quick succession makes GDM fall back to the
  greeter. `loginctl show-session` then reports `Class=greeter` and uid 128, no user
  session exists, and Sunshine crash-loops on "Unable to open display".
- Fix: reboot instead of restarting `gdm3` again. On a clean boot autologin fires and
  the session comes up as `Class=user`.

## 4. Install Sunshine

```bash
cd /tmp
curl -fLO https://github.com/LizardByte/Sunshine/releases/download/v2026.516.143833/sunshine-ubuntu-22.04-amd64.deb
sudo apt install -y ./sunshine-ubuntu-22.04-amd64.deb
sudo usermod -aG input,render ben
XDG_RUNTIME_DIR=/run/user/1000 systemctl --user enable app-dev.lizardbyte.app.Sunshine.service
```

The deb sets `cap_sys_admin,cap_sys_nice` on the binary, installs `60-sunshine.rules`,
and wires the unit into `graphical-session.target.wants`, so it starts with the desktop.
Group changes need a fresh login, which the reboot in the next step provides.

`~/.config/sunshine/sunshine.conf`:

```
csrf_allowed_origins = https://100.114.188.57:47990,https://megamind:47990
encoder = nvenc
capture = x11
```

Three problems that config solves:

- Bug: left to autodetect, Sunshine picks KMS capture with the Vulkan encoder and
  segfaults while creating `h264_vulkan`. The service then restarts in a loop and
  Moonlight reports "connection refused (Error 1)" because nothing is listening.
- Fix: force `encoder = nvenc` and `capture = x11`. The log should then read
  `Found H.264 encoder: h264_nvenc [nvenc]` and `Found HEVC encoder: hevc_nvenc`.

- Bug: `Couldn't open render node: /dev/dri/renderD128: Permission denied`. Those nodes
  are `root:render 660` and `ben` was not in `render`.
- Fix: `usermod -aG render ben`, then log in again.

- Bug: the web UI rejects logins and saves when reached at the Tailscale IP or hostname
  rather than localhost, because the request origin is not on Sunshine's allow list.
- Fix: set `csrf_allowed_origins` to every origin you will actually browse to, comma
  separated, including the port.

Two log lines during encoder probing are expected and harmless: `av1_nvenc` fails
because Ampere has no AV1 encode, and the `AV_PIX_FMT_NV12` error is the 10-bit HDR probe.

## 5. Client

```bash
flatpak install -y flathub com.moonlight_stream.Moonlight
```

1. Open `https://100.114.188.57:47990` and set the username and password on first load.
   The certificate is self-signed, so expect a browser warning.
2. In Moonlight, add the host `100.114.188.57` manually. mDNS does not cross Tailscale.
3. Enter Moonlight's PIN in the web UI.

Settings for a 27 ms, 64 Mbps link at 1080p: 60 fps, 25 to 30 Mbps, HEVC, V-Sync off.
Sunshine listens on TCP 47984, 47989, 47990, 48010 and UDP 47998-48010. `ufw` is
inactive on megamind, and the traffic rides the Tailscale tunnel.

## Changing the resolution

Build a new EDID with the target timing, point `CustomEDID` at it, and apply the change
live so no restart is needed:

```bash
sudo sed -i 's|edid-1920x1080.bin|edid-3840x1080.bin|' /etc/X11/xorg.conf
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority xrandr --output DP-0 --mode 3840x1080
```

The EDID makes it stick across reboots; the `xrandr` call only covers the current
session. `/etc/X11/edid-3840x1080.bin` already holds a 3840x1080@60 CVT-RB timing, which
matches the ultrawide client panel's aspect ratio.

## Verifying

```bash
ssh megamind '
  for s in $(loginctl list-sessions --no-legend | awk "\$4==\"seat0\"{print \$1}"); do
    loginctl show-session $s -p Class -p Type -p Active
  done
  grep "Setting mode" ~/.local/share/xorg/Xorg.0.log | tail -1
  XDG_RUNTIME_DIR=/run/user/1000 systemctl --user is-active sunshine
  ss -lnt | grep -E "47984|47989|47990|48010"
'
```

Healthy output is `Class=user`, a `Setting mode` line ending in the resolution you set,
`active`, and four listening ports.

- Bug: checking the display over SSH shows the laptop's panel instead of megamind's, so
  a working remote config looks broken.
- Fix: set `DISPLAY=:0` explicitly. SSH X11 forwarding sets `DISPLAY=localhost:10.0`,
  and a bare `xrandr` then queries your own machine. Reading the `Setting mode` line out
  of the Xorg log avoids the trap entirely, since it needs no X connection.

## Recovery

X config is reversible over SSH, and there is no physical monitor to lose:

```bash
sudo rm /etc/X11/xorg.conf && sudo systemctl restart gdm3
```

That drops the desktop back onto the ASPEED BMC at 1600x1200 with software rendering,
which is enough to get in and fix things.

## Notes

- A reboot takes about 13 minutes before the kernel starts, spent in POST and memory
  training on the Threadripper PRO with 256 GB. It is not hung. Nothing in this setup
  runs before `multi-user.target`, so if the machine never rejoins the network, the
  cause is firmware or hardware, not the display config.
- `~/.config/monitors.xml` still holds a March 2024 entry for a Samsung U28D590 on
  connector `DP-6.3`. It never matches the virtual `DP-0`, so mutter falls through to
  the EDID preferred mode. Harmless.
- For heavy 3D work such as Isaac Sim, its own WebRTC streaming beats Sunshine over this
  link, because it streams the viewport instead of the whole desktop.
