"""Copy Forgejo commit statuses on branch heads to GitHub, one-way, with no target URL."""

from __future__ import annotations

import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

GITHUB_API = "https://api.github.com"
GITHUB_STATES = {
    "pending": "pending",
    "success": "success",
    "warning": "success",
    "failure": "failure",
    "error": "error",
}
EVENT_SUFFIX = re.compile(r"\s*\([^)]*\)\s*$")


@dataclass
class Settings:
    forgejo_url: str
    forgejo_repo: str
    forgejo_token: str
    github_repo: str
    github_token: str
    state_file: Path
    interval_s: float
    github_api: str = GITHUB_API

    @classmethod
    def from_env(cls, env: dict[str, str]) -> Settings:
        return cls(
            forgejo_url=env["FORGEJO_URL"].rstrip("/"),
            forgejo_repo=env["FORGEJO_REPO"],
            forgejo_token=env["FORGEJO_TOKEN"],
            github_repo=env["GITHUB_REPO"],
            github_token=env["GITHUB_TOKEN"],
            state_file=Path(env.get("STATE_FILE", "/data/relay_state.json")),
            interval_s=float(env.get("INTERVAL_S", "30")),
            github_api=env.get("GITHUB_API", GITHUB_API).rstrip("/"),
        )


def github_context(forgejo_context: str) -> str:
    """`push / verify jetson (push)` becomes `forgejo/push/verify-jetson`."""
    parts = [part.strip() for part in EVENT_SUFFIX.sub("", forgejo_context).split("/")]
    return "forgejo/" + "/".join(part.replace(" ", "-") for part in parts if part)


def request_json(url: str, token_header: str, body: dict[str, Any] | None = None) -> Any:
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method="GET" if body is None else "POST")
    request.add_header("Authorization", token_header)
    request.add_header("Accept", "application/json")
    if body is not None:
        request.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.load(response)


def branch_heads(settings: Settings) -> list[str]:
    url = f"{settings.forgejo_url}/api/v1/repos/{settings.forgejo_repo}/branches?limit=50"
    branches = request_json(url, f"token {settings.forgejo_token}")
    return sorted({branch["commit"]["id"] for branch in branches})


def forgejo_statuses(settings: Settings, sha: str) -> dict[str, str]:
    """Latest Forgejo state per GitHub context for one commit."""
    url = f"{settings.forgejo_url}/api/v1/repos/{settings.forgejo_repo}/commits/{sha}/status"
    combined = request_json(url, f"token {settings.forgejo_token}")
    states = {}
    for status in combined.get("statuses") or []:
        state = GITHUB_STATES.get(str(status.get("status")))
        if state:
            states[github_context(str(status.get("context")))] = state
    return states


def post(settings: Settings, sha: str, context: str, state: str) -> bool:
    url = f"{settings.github_api}/repos/{settings.github_repo}/statuses/{sha}"
    body = {"state": state, "context": context, "description": f"Forgejo CI: {state}"}
    try:
        request_json(url, f"Bearer {settings.github_token}", body)
    except urllib.error.HTTPError as error:
        if error.code == 422:
            print(f"{sha[:10]} not on GitHub yet; retrying next poll", flush=True)
            return False
        raise
    print(f"{sha[:10]} {context} -> {state}", flush=True)
    return True


def load_state(path: Path) -> dict[str, str]:
    try:
        return dict(json.loads(path.read_text()))
    except (OSError, ValueError):
        return {}


def poll_once(settings: Settings) -> None:
    """Post every state that changed since the last poll; forget commits no branch points at."""
    posted = load_state(settings.state_file)
    current = {}
    for sha in branch_heads(settings):
        for context, state in forgejo_statuses(settings, sha).items():
            key = f"{sha} {context}"
            if posted.get(key) == state or post(settings, sha, context, state):
                current[key] = state
    settings.state_file.parent.mkdir(parents=True, exist_ok=True)
    settings.state_file.write_text(json.dumps(current, indent=1, sort_keys=True))


def main() -> int:
    settings = Settings.from_env(dict(os.environ))
    print(f"relaying {settings.forgejo_repo} to {settings.github_repo}", flush=True)
    while True:
        try:
            poll_once(settings)
        except (OSError, ValueError, KeyError) as error:
            print(f"poll failed: {error}", flush=True)
        time.sleep(settings.interval_s)


if __name__ == "__main__":
    sys.exit(main())
