#!/usr/bin/python3 -I
"""Show the web dashboard full screen on the robot computer's own display.

One GTK window holding one WebKit view: no tabs, address bar, or browser profile. WebKitGTK and
its GObject bindings ship with the Ubuntu desktop, so this runs on the system python, not the
venv (python3-gi is built for /usr/bin/python3 only).

Run by dashboard-kiosk.service through with_display.sh. With no display it exits 0, so the unit
stays stopped on a headless box instead of restarting forever.

The dashboard's own page reconnects when the app or the relay restarts. This script only covers
what the page cannot: the relay not being up yet at boot, a failed page load, and a crashed web
process.
"""

from __future__ import annotations

import argparse
import sys
import time
import urllib.error
import urllib.request

# service/viz-relay.service serves the dashboard on port 80.
DASHBOARD_URL = "http://127.0.0.1/"
RETRY_SECONDS = 2.0


def wait_for_dashboard(url: str) -> None:
    """Blocks until the relay answers /healthz."""
    announced = False
    while True:
        try:
            with urllib.request.urlopen(url + "healthz", timeout=1.0) as response:
                if response.status == 200:
                    return
        except (urllib.error.URLError, OSError):
            pass
        if not announced:
            print(f"dashboard_kiosk: waiting for the relay at {url}", flush=True)
            announced = True
        time.sleep(RETRY_SECONDS)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default=DASHBOARD_URL, help=f"Default: {DASHBOARD_URL}")
    parser.add_argument("--windowed", action="store_true", help="Do not go full screen")
    args = parser.parse_args()

    import gi

    gi.require_version("Gdk", "3.0")
    gi.require_version("Gtk", "3.0")
    gi.require_version("WebKit2", "4.1")
    from gi.repository import Gdk, GLib, Gtk, WebKit2

    ok, _ = Gtk.init_check(sys.argv)
    if not ok:
        print("dashboard_kiosk: no display; nothing to show.", flush=True)
        return 0

    url = args.url if args.url.endswith("/") else args.url + "/"
    wait_for_dashboard(url)
    print(f"dashboard_kiosk: showing {url}", flush=True)

    window = Gtk.Window(title="BWBots dashboard")
    window.set_default_size(1024, 600)
    view = WebKit2.WebView()
    view.get_settings().set_enable_write_console_messages_to_stdout(True)
    window.add(view)
    window.connect("destroy", lambda _w: Gtk.main_quit())

    def reload_later() -> None:
        def reload() -> bool:
            view.load_uri(url)
            return False

        GLib.timeout_add(int(RETRY_SECONDS * 1000), reload)

    def on_load_failed(_view: object, _event: object, uri: str, error: GLib.Error) -> bool:
        print(f"dashboard_kiosk: {uri} failed to load ({error.message}); retrying", flush=True)
        reload_later()
        return False

    def on_web_process_terminated(_view: object, reason: object) -> None:
        print(f"dashboard_kiosk: web process ended ({reason}); reloading", flush=True)
        reload_later()

    view.connect("load-failed", on_load_failed)
    view.connect("web-process-terminated", on_web_process_terminated)
    # A long press on the touchscreen would otherwise open WebKit's Back/Reload menu.
    view.connect("context-menu", lambda *_: True)

    def hide_cursor(widget: Gtk.Widget) -> None:
        gdk_window = widget.get_window()
        if gdk_window is not None:
            gdk_window.set_cursor(Gdk.Cursor.new_from_name(widget.get_display(), "none"))

    window.connect("realize", hide_cursor)
    if not args.windowed:
        window.fullscreen()
    view.load_uri(url)
    window.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    sys.exit(main())
