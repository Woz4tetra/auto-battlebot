# Dashboard kiosk

`service/dashboard_kiosk.py` shows the web dashboard full screen on the robot computer's own display. It is one GTK window holding one WebKit view, with no tabs, address bar, or browser profile.

- WebKitGTK and its GObject bindings ship with the Ubuntu desktop, so it runs on the system python, not the venv. `python3-gi` is built for `/usr/bin/python3` only.
- `dashboard-kiosk.service` runs it through `with_display.sh`. With no display it exits 0, so the unit stays stopped on a headless box instead of restarting forever.
- The dashboard page reconnects when the app or the relay restarts. The kiosk script covers what the page cannot: the relay not being up yet at boot, a failed page load, and a crashed web process.

## enable_kiosk.sh

`service/enable_kiosk.sh` does nothing without a display. Both `gsettings` and `xset` fail noisily on a headless deploy, and `with_display.sh` leaves `DISPLAY` unset on those boxes, so the script returns early when `DISPLAY` is empty.

`gsettings` needs the session dbus. GLib finds it at `$XDG_RUNTIME_DIR/bus` without `DBUS_SESSION_BUS_ADDRESS` being set. The script uses it to disable the GNOME on-screen keyboard so it does not cover the UI on the touchscreen.
