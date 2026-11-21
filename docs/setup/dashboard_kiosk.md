# Dashboard kiosk

`service/dashboard_kiosk.py` shows the web dashboard full screen on the robot computer's own display. It is one GTK window holding one WebKit view, with no tabs, address bar, or browser profile.

- WebKitGTK and its GObject bindings ship with the Ubuntu desktop, so it runs on the system python, not the venv. `python3-gi` is built for `/usr/bin/python3` only.
- `dashboard-kiosk.service` runs it through `with_display.sh`. With no display it exits 0, so the unit stays stopped on a headless box instead of restarting forever.
- `service/build_and_install_service.sh` installs each unit from `service/<name>.service` when it is missing or differs, expanding `__USER__` and `__UID__` to the invoking user. `dashboard-kiosk` is installed enabled only when the box boots a desktop (`display-manager` enabled, graphical default target). Otherwise it goes in with `--no-enable`. The installer never disables a unit, so one you enabled by hand stays enabled.
- The dashboard page reconnects when the app or the relay restarts. The kiosk script covers what the page cannot: the relay not being up yet at boot, a failed page load, and a crashed web process.

## enable_kiosk.sh

`service/enable_kiosk.sh` does nothing without a display. Both `gsettings` and `xset` fail noisily on a headless deploy, and `with_display.sh` leaves `DISPLAY` unset on those boxes, so the script returns early when `DISPLAY` is empty.

`gsettings` needs the session dbus. GLib finds it at `$XDG_RUNTIME_DIR/bus` without `DBUS_SESSION_BUS_ADDRESS` being set. The script uses it to disable the GNOME on-screen keyboard so it does not cover the UI on the touchscreen.

## with_display.sh

`service/with_display.sh` runs a command with `DISPLAY` pointing at the X server of the logged-in session on boxes that have one. Headless boxes run the command straight away. `dashboard-kiosk.service` runs through it for two reasons:

1. The display number moves. GDM hands the session whichever number is free: `:1` on the handheld on 2026-09-22 and `:0` on 2026-10-13. A hardcoded `DISPLAY` aims at a server that is not there.
2. At boot the service starts before GDM's autologin session exists. Measured on the handheld: the service started at t=11.8 s and the session's Xorg at t=13.6 s. The kiosk treats no display as a headless box and exits for good, so it has to wait rather than start and hope.

The wait is gated on the machine booting a desktop at all, so a headless deploy never waits for a session that is not coming. Both conditions must hold: a display manager enabled (that creates the X session) and `graphical.target` as the boot target (that runs it). Anything else counts as headless, so an unclear answer costs no startup time. No `XAUTHORITY` handling is needed: GDM grants the session owner by uid (`xhost` shows `SI:localuser:<user>`), so a process running as that user connects without a cookie.

