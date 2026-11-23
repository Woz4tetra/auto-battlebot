"""Serve the feature map UI on the Tailscale address and push an SSE event on input changes."""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import shutil
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from auto_battlebot.features.feature_map import build_map
from auto_battlebot.features.registry import FEATURES_FILE
from auto_battlebot.paths import REPO_ROOT
from auto_battlebot.verify.state import changed_since

STATIC_DIR = Path(__file__).with_name("static")
TOOL_DIR = Path("tools/feature_map")
LOOPBACK = "127.0.0.1"
DEFAULT_PORT = 8780
POLL_S = 1.0
KEEPALIVE_S = 15.0
WATCHED = (FEATURES_FILE, "data/verify/runs", "data/verify/coverage", ".git/HEAD", ".git/refs")
SOURCE_PARTS = ("src", "index.html", "package.json", "vite.config.ts")


def newest_mtime(path: Path) -> float:
    if not path.exists():
        return 0.0
    if path.is_file():
        return path.stat().st_mtime
    newest = path.stat().st_mtime
    for child in path.rglob("*"):
        try:
            newest = max(newest, child.stat().st_mtime)
        except OSError:
            continue
    return newest


def bundle_is_stale(root: Path, static: Path = STATIC_DIR) -> bool:
    index = static / "index.html"
    if not index.exists():
        return True
    built = index.stat().st_mtime
    return any(newest_mtime(root / TOOL_DIR / part) > built for part in SOURCE_PARTS)


def ensure_bundle(root: Path) -> str | None:
    """Build the UI when the bundle is missing or older than its source; returns a problem."""
    if not bundle_is_stale(root):
        return None
    tool = root / TOOL_DIR
    if not (tool / "node_modules").is_dir():
        return f"{TOOL_DIR}/node_modules is missing; run `npm ci` in {TOOL_DIR}"
    if shutil.which("npm") is None:
        return "npm is not installed"
    result = subprocess.run(["npm", "run", "build"], cwd=tool, capture_output=True, text=True)
    if result.returncode != 0:
        return f"UI build failed:\n{result.stdout[-2000:]}{result.stderr[-2000:]}"
    return None


def signature(root: Path) -> str:
    """Hash of every watched mtime plus the working tree's changed paths and their mtimes."""
    digest = hashlib.sha256()
    for rel in WATCHED:
        digest.update(f"{rel}:{newest_mtime(root / rel)}\n".encode())
    for rel in changed_since(root, "HEAD"):
        path = root / rel
        mtime = path.stat().st_mtime if path.exists() else -1.0
        digest.update(f"{rel}:{mtime}\n".encode())
    return digest.hexdigest()


class MapState:
    """The current map, rebuilt when the signature moves, and a version counter for SSE."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.lock = threading.Lock()
        self.version = 0
        self.signature = ""
        self.payload = b"{}"
        self.changed = threading.Condition(self.lock)

    def poll(self) -> bool:
        current = signature(self.root)
        if current == self.signature:
            return False
        payload = json.dumps(build_map(self.root)).encode()
        with self.changed:
            self.signature = current
            self.payload = payload
            self.version += 1
            self.changed.notify_all()
        return True

    def watch(self, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                self.poll()
            except Exception as error:
                print(f"feature map rebuild failed: {error}", file=sys.stderr, flush=True)
            stop.wait(POLL_S)

    def wait_for(self, version: int, timeout: float) -> int:
        with self.changed:
            self.changed.wait_for(lambda: self.version != version, timeout)
            return self.version


class Handler(BaseHTTPRequestHandler):
    """GET handlers for the map JSON, the diff selection, the event stream, and the bundle."""

    server_version = "feature-map"
    state: MapState
    static_dir: Path

    def log_message(self, format: str, *args: Any) -> None:
        del format, args

    def send_bytes(self, body: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/api/map":
            with self.state.lock:
                body = self.state.payload
            self.send_bytes(body, "application/json")
        elif path == "/api/diff":
            data = build_map(self.state.root, with_git=True, with_selection=True)["diff"]
            self.send_bytes(json.dumps(data).encode(), "application/json")
        elif path == "/api/events":
            self.events()
        else:
            self.static(path)

    def events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        version = self.state.version
        try:
            while True:
                latest = self.state.wait_for(version, KEEPALIVE_S)
                if latest != version:
                    version = latest
                    self.wfile.write(f"event: change\ndata: {version}\n\n".encode())
                else:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def static(self, path: str) -> None:
        rel = path.lstrip("/") or "index.html"
        target = (self.static_dir / rel).resolve()
        if not target.is_relative_to(self.static_dir.resolve()) or not target.is_file():
            target = self.static_dir / "index.html"
        if not target.is_file():
            self.send_bytes(b"UI bundle missing", "text/plain", 503)
            return
        kind = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_bytes(target.read_bytes(), kind)


def make_handler(state: MapState, static: Path) -> type[BaseHTTPRequestHandler]:
    return type("BoundHandler", (Handler,), {"state": state, "static_dir": static})


def tailscale_ip() -> str | None:
    """Return this machine's Tailscale IPv4 address, or None without Tailscale."""
    try:
        result = subprocess.run(
            ["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=5, check=False
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    lines = result.stdout.split()
    return lines[0] if result.returncode == 0 and lines else None


def default_host() -> str:
    address = tailscale_ip()
    if address is None:
        print(f"tailscale is not up, serving on {LOOPBACK}", file=sys.stderr)
        return LOOPBACK
    return address


def serve(port: int, host: str, root: Path = REPO_ROOT, static: Path = STATIC_DIR) -> None:
    problem = ensure_bundle(root)
    if problem:
        print(problem, file=sys.stderr)
    state = MapState(root)
    state.poll()
    stop = threading.Event()
    threading.Thread(target=state.watch, args=(stop,), daemon=True).start()
    server = ThreadingHTTPServer((host, port), make_handler(state, static))
    server.daemon_threads = True
    print(f"feature map at http://{host}:{port}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m auto_battlebot.features.serve")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--host", help="bind address (default: this machine's Tailscale IPv4)")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    serve(args.port, args.host or default_host())
    return 0


if __name__ == "__main__":
    sys.exit(main())
