"""Print a commit's combined Forgejo CI status; exit 0 success, 1 failure, 2 pending, 3 unknown."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from auto_battlebot.compat import tomllib

CONFIG_DIR = Path.home() / ".config" / "auto-battlebot"
CONFIG_FILE = CONFIG_DIR / "forgejo.toml"
TOKEN_FILE = CONFIG_DIR / "forgejo_token"
SUCCESS, FAILURE, PENDING, UNKNOWN = 0, 1, 2, 3
WAIT_POLL_S = 30.0
WAIT_LIMIT_S = 45 * 60.0
STATE_CODES = {"success": SUCCESS, "warning": SUCCESS, "failure": FAILURE, "error": FAILURE}


@dataclass
class Forge:
    """Where the Forgejo API is and how to authenticate to it."""

    url: str
    repo: str
    token: str | None


@dataclass
class Result:
    code: int
    lines: list[str]


def load_forge(config: Path = CONFIG_FILE, token_file: Path = TOKEN_FILE) -> Forge:
    """The forge from `forgejo.toml` (`url`, `repo`) and the token file beside it."""
    data = tomllib.loads(config.read_text())
    token = token_file.read_text().strip() if token_file.exists() else None
    return Forge(str(data["url"]).rstrip("/"), str(data["repo"]), token)


def git(root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True, text=True
    ).stdout.strip()


def fetch_combined(forge: Forge, sha: str, timeout: float) -> dict[str, Any] | None:
    """The combined status, or None when Forgejo has no such commit."""
    request = urllib.request.Request(f"{forge.url}/api/v1/repos/{forge.repo}/commits/{sha}/status")
    if forge.token:
        request.add_header("Authorization", f"token {forge.token}")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            payload: dict[str, Any] = json.load(response)
            return payload
    except urllib.error.HTTPError as error:
        if error.code in (404, 422):
            return None
        raise


def classify(sha: str, payload: dict[str, Any] | None) -> Result:
    short = sha[:10]
    if payload is None:
        return Result(PENDING, [f"{short}: not on Forgejo (pushed?)"])
    statuses = payload.get("statuses") or []
    if not statuses:
        return Result(PENDING, [f"{short}: no CI status yet"])
    state = str(payload.get("state", "pending"))
    lines = [f"{short}: {state}"]
    for status in statuses:
        url = status.get("target_url") or ""
        lines.append(f"  {status.get('context')}: {status.get('status')} {url}".rstrip())
    return Result(STATE_CODES.get(state, PENDING), lines)


def check(forge: Forge, sha: str, timeout: float) -> Result:
    try:
        return classify(sha, fetch_combined(forge, sha, timeout))
    except (OSError, ValueError) as error:
        return Result(UNKNOWN, [f"Forgejo unreachable: {error}"])


def wait(
    probe: Callable[[], Result],
    poll_s: float = WAIT_POLL_S,
    limit_s: float = WAIT_LIMIT_S,
    sleep: Callable[[float], None] = time.sleep,
) -> Result:
    """Poll until the status settles or the limit passes."""
    waited = 0.0
    result = probe()
    while result.code == PENDING and waited < limit_s:
        sleep(poll_s)
        waited += poll_s
        result = probe()
    return result


def run(ref: str, root: Path, forge: Forge, timeout: float, wait_for_result: bool) -> Result:
    if git(root, "status", "--porcelain", "--untracked-files=no"):
        return Result(UNKNOWN, ["dirty tree: CI never saw these changes"])
    sha = git(root, "rev-parse", ref)
    if wait_for_result:
        return wait(lambda: check(forge, sha, timeout))
    return check(forge, sha, timeout)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ref", nargs="?", default="HEAD")
    parser.add_argument("--wait", action="store_true", help="Poll every 30 s, up to 45 min")
    parser.add_argument("--timeout", type=float, default=10.0, help="HTTP timeout in seconds")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    try:
        forge = load_forge()
    except (OSError, KeyError, ValueError) as error:
        print(f"Forgejo not configured ({CONFIG_FILE}): {error}")
        return UNKNOWN
    try:
        result = run(args.ref, args.root, forge, args.timeout, args.wait)
    except subprocess.CalledProcessError as error:
        print(f"git failed: {error.stderr.strip() if error.stderr else error}")
        return UNKNOWN
    print("\n".join(result.lines))
    return result.code


if __name__ == "__main__":
    sys.exit(main())
