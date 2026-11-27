from __future__ import annotations

import json
import subprocess
import threading
import time
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import pytest

from auto_battlebot.ci import relay, status

SHA = "a" * 40


class Stub:
    """A tiny HTTP server: GET routes answer from `routes`, POSTs are recorded."""

    def __init__(self) -> None:
        self.routes: dict[str, Any] = {}
        self.posts: list[tuple[str, dict[str, Any], str]] = []
        self.delay_s = 0.0
        self.post_code = 201
        stub = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                time.sleep(stub.delay_s)
                path = self.path.split("?")[0]
                if path not in stub.routes:
                    self.send_response(404)
                    self.end_headers()
                    return
                self.reply(200, stub.routes[path])

            def do_POST(self) -> None:
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                stub.posts.append((self.path, body, self.headers["Authorization"]))
                self.reply(stub.post_code, {})

            def reply(self, code: int, body: Any) -> None:
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args: object) -> None:
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()


@pytest.fixture
def stub() -> Iterator[Stub]:
    server = Stub()
    yield server
    server.server.shutdown()


def combined(state: str, *statuses: tuple[str, str]) -> dict[str, Any]:
    return {
        "state": state,
        "statuses": [
            {"context": context, "status": s, "target_url": f"http://forge/run/{i}"}
            for i, (context, s) in enumerate(statuses)
        ],
    }


def forge(stub: Stub) -> status.Forge:
    return status.Forge(stub.url, "beb/auto-battlebot", "secret")


@pytest.mark.parametrize(
    ("state", "code"),
    [("success", 0), ("warning", 0), ("failure", 1), ("error", 1), ("pending", 2)],
)
def test_each_state_maps_to_its_exit_code(stub: Stub, state: str, code: int) -> None:
    stub.routes[f"/api/v1/repos/beb/auto-battlebot/commits/{SHA}/status"] = combined(
        state, ("push / jetson (push)", state)
    )
    result = status.check(forge(stub), SHA, timeout=5)
    assert result.code == code
    assert "push / jetson (push)" in result.lines[1] and "http://forge/run/0" in result.lines[1]


def test_a_commit_with_no_statuses_is_pending(stub: Stub) -> None:
    stub.routes[f"/api/v1/repos/beb/auto-battlebot/commits/{SHA}/status"] = combined("pending")
    assert status.check(forge(stub), SHA, timeout=5).code == status.PENDING


def test_an_unknown_sha_is_pending_with_a_push_hint(stub: Stub) -> None:
    result = status.check(forge(stub), SHA, timeout=5)
    assert result.code == status.PENDING and "pushed?" in result.lines[0]


def test_a_slow_or_absent_forge_is_unknown(stub: Stub) -> None:
    stub.delay_s = 1.0
    stub.routes[f"/api/v1/repos/beb/auto-battlebot/commits/{SHA}/status"] = combined("success")
    assert status.check(forge(stub), SHA, timeout=0.2).code == status.UNKNOWN
    gone = status.Forge("http://127.0.0.1:9", "beb/auto-battlebot", None)
    assert status.check(gone, SHA, timeout=1).code == status.UNKNOWN


def test_wait_polls_until_the_state_settles() -> None:
    results = iter([status.Result(2, []), status.Result(2, []), status.Result(1, ["red"])])
    sleeps: list[float] = []
    final = status.wait(lambda: next(results), poll_s=30, limit_s=600, sleep=sleeps.append)
    assert final.code == 1 and sleeps == [30, 30]


def test_wait_gives_up_at_the_limit() -> None:
    sleeps: list[float] = []
    final = status.wait(lambda: status.Result(2, []), poll_s=30, limit_s=60, sleep=sleeps.append)
    assert final.code == 2 and sleeps == [30, 30]


def git_repo(tmp_path: Path) -> Path:
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    (tmp_path / "f").write_text("1")
    for args in (["add", "f"], ["-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "c"]):
        subprocess.run(["git", *args], cwd=tmp_path, check=True)
    return tmp_path


def test_a_dirty_tree_is_unknown_before_any_request(tmp_path: Path) -> None:
    root = git_repo(tmp_path)
    (root / "f").write_text("2")
    gone = status.Forge("http://127.0.0.1:9", "beb/auto-battlebot", None)
    result = status.run("HEAD", root, gone, timeout=1, wait_for_result=False)
    assert result.code == status.UNKNOWN and "dirty" in result.lines[0]


def test_forge_config_comes_from_toml_and_token_file(tmp_path: Path) -> None:
    (tmp_path / "forgejo.toml").write_text('url = "https://forge/forgejo/"\nrepo = "beb/r"\n')
    (tmp_path / "token").write_text("abc\n")
    loaded = status.load_forge(tmp_path / "forgejo.toml", tmp_path / "token")
    assert loaded == status.Forge("https://forge/forgejo", "beb/r", "abc")


@pytest.mark.parametrize(
    ("forgejo", "github"),
    [
        ("push / jetson (push)", "forgejo/push/jetson"),
        ("push / x86 / verify (push)", "forgejo/push/x86/verify"),
        ("nightly / record", "forgejo/nightly/record"),
    ],
)
def test_github_context_names(forgejo: str, github: str) -> None:
    assert relay.github_context(forgejo) == github


def relay_settings(stub: Stub, tmp_path: Path) -> relay.Settings:
    stub.routes["/api/v1/repos/beb/auto-battlebot/branches"] = [{"commit": {"id": SHA}}]
    return relay.Settings(
        forgejo_url=stub.url,
        forgejo_repo="beb/auto-battlebot",
        forgejo_token="ftoken",
        github_repo="owner/auto-battlebot",
        github_token="gtoken",
        state_file=tmp_path / "state.json",
        interval_s=0,
        github_api=stub.url,
    )


def test_relay_posts_each_state_change_once_without_a_target_url(
    stub: Stub, tmp_path: Path
) -> None:
    settings = relay_settings(stub, tmp_path)
    route = f"/api/v1/repos/beb/auto-battlebot/commits/{SHA}/status"
    stub.routes[route] = combined("pending", ("push / jetson (push)", "pending"))
    relay.poll_once(settings)
    relay.poll_once(settings)
    stub.routes[route] = combined("failure", ("push / jetson (push)", "failure"))
    relay.poll_once(settings)
    assert [(p[1]["context"], p[1]["state"]) for p in stub.posts] == [
        ("forgejo/push/jetson", "pending"),
        ("forgejo/push/jetson", "failure"),
    ]
    path, body, auth = stub.posts[0]
    assert path == f"/repos/owner/auto-battlebot/statuses/{SHA}"
    assert "target_url" not in body and auth == "Bearer gtoken"


def test_relay_retries_a_commit_github_does_not_have_yet(stub: Stub, tmp_path: Path) -> None:
    settings = relay_settings(stub, tmp_path)
    stub.routes[f"/api/v1/repos/beb/auto-battlebot/commits/{SHA}/status"] = combined(
        "success", ("push / jetson (push)", "success")
    )
    stub.post_code = 422
    relay.poll_once(settings)
    stub.post_code = 201
    relay.poll_once(settings)
    assert len(stub.posts) == 2
    assert json.loads(settings.state_file.read_text()) == {f"{SHA} forgejo/push/jetson": "success"}
