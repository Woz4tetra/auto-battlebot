"""Generate 3D robot meshes from NHRL thumbnails via the Meshy image-to-3D API.

Step 2 of the NHRL robot distractor pipeline.  Reads the state file written by
``download_nhrl_bots.py``, submits Meshy image-to-3D tasks for bots that have a
thumbnail, polls them, downloads the resulting GLBs, and appends VRAM audit
rows.  Fully resumable: submitted tasks are never resubmitted, and progress is
persisted after every state transition.

Requires the ``MESHY_API_KEY`` environment variable.

Usage:
    MESHY_API_KEY=... python generate_nhrl_meshes.py ../data/distractor_models/robots --limit 10
    python generate_nhrl_meshes.py ../data/distractor_models/robots --dry-run
"""

from __future__ import annotations

import argparse
import base64
import os
import time
from pathlib import Path
from typing import Any

import nhrl_common as nc
import requests
from nhrl_audit import estimate_model_gpu_row, load_audit_table, write_audit_table

_REQUEST_TIMEOUT = 60
_DEFAULT_AUDIT_CSV = "training/data/distractor_models/distractor_gpu_audit.csv"


class OutOfCreditsError(RuntimeError):
    """Raised when the Meshy account has insufficient credits (HTTP 402)."""


class MeshyClient:
    """Thin wrapper over the Meshy image-to-3D REST endpoints."""

    def __init__(self, api_key: str) -> None:
        self._session = requests.Session()
        self._session.headers.update({"Authorization": f"Bearer {api_key}"})

    def submit(self, image_data_uri: str, params: dict[str, Any]) -> str:
        """Create an image-to-3D task and return its id (retries on 429)."""
        body = {"image_url": image_data_uri, **params}
        for attempt in range(5):
            resp = self._session.post(
                f"{nc.MESHY_API_BASE}/image-to-3d", json=body, timeout=_REQUEST_TIMEOUT
            )
            if resp.status_code == 402:
                raise OutOfCreditsError("Meshy returned 402: out of credits")
            if resp.status_code == 429:
                backoff = 2**attempt
                print(f"  Rate limited on submit, backing off {backoff}s")
                time.sleep(backoff)
                continue
            resp.raise_for_status()
            return str(resp.json()["result"])
        raise RuntimeError("Meshy submit failed after repeated rate limiting")

    def get_task(self, task_id: str) -> dict[str, Any]:
        """Fetch task status (retries on 429). Returns the task JSON."""
        for attempt in range(5):
            resp = self._session.get(
                f"{nc.MESHY_API_BASE}/image-to-3d/{task_id}", timeout=_REQUEST_TIMEOUT
            )
            if resp.status_code == 429:
                backoff = 2**attempt
                time.sleep(backoff)
                continue
            resp.raise_for_status()
            result: dict[str, Any] = resp.json()
            return result
        raise RuntimeError("Meshy get_task failed after repeated rate limiting")


def thumbnail_data_uri(thumb_path: Path) -> str:
    """Encode a PNG thumbnail as a base64 data URI for Meshy."""
    encoded = base64.b64encode(thumb_path.read_bytes()).decode("ascii")
    return f"data:image/png;base64,{encoded}"


def download_glb(url: str, dest: Path) -> None:
    """Stream a GLB to *dest*."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    with requests.get(url, stream=True, timeout=_REQUEST_TIMEOUT) as resp:
        resp.raise_for_status()
        with dest.open("wb") as f:
            for chunk in resp.iter_content(chunk_size=1 << 20):
                if chunk:
                    f.write(chunk)


def _usable(entry: dict[str, Any]) -> bool:
    """True if the bot has a downloaded thumbnail and is eligible for meshing."""
    return (
        not nc.is_rejected(entry)
        and entry.get("status") != nc.STATUS_SKIPPED_NO_THUMBNAIL
        and bool(entry.get("thumbnail_file"))
    )


def _meshy_params(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "ai_model": args.ai_model,
        "target_polycount": args.target_polycount,
        "should_texture": True,
        "enable_pbr": bool(args.enable_pbr),
        "target_formats": ["glb"],
    }


def _update_audit(dest: Path, audit_csv: Path) -> None:
    try:
        rows = load_audit_table(audit_csv)
        rows[str(dest.resolve())] = estimate_model_gpu_row(dest, source=dest.parent.name)
        write_audit_table(audit_csv, rows)
    except Exception as e:  # noqa: BLE001 - audit is best-effort
        print(f"  Warning: failed to update audit for {dest.name}: {e}")


class _Generator:
    """Drives the submit/poll/download loop over the shared state file."""

    def __init__(
        self,
        client: MeshyClient,
        params: dict[str, Any],
        state: dict[str, Any],
        state_path: Path,
        thumb_dir: Path,
        output_dir: Path,
        audit_csv: Path,
        args: argparse.Namespace,
    ) -> None:
        self.client = client
        self.params = params
        self.state = state
        self.state_path = state_path
        self.thumb_dir = thumb_dir
        self.output_dir = output_dir
        self.audit_csv = audit_csv
        self.args = args
        self.inflight: dict[str, str] = {}

    def _save(self) -> None:
        nc.save_state(self.state_path, self.state)

    def _submit_one(self, clean_name: str) -> None:
        entry = self.state[clean_name]
        exhausted = int(entry.get("retries", 0)) >= self.args.max_retries
        if entry.get("status") == nc.STATUS_FAILED and exhausted:
            return
        thumb = self.thumb_dir / str(entry["thumbnail_file"])
        if not thumb.exists():
            entry["status"] = nc.STATUS_SKIPPED_NO_THUMBNAIL
            self._save()
            return
        task_id = self.client.submit(thumbnail_data_uri(thumb), self.params)
        entry["task_id"] = task_id
        entry["status"] = nc.STATUS_SUBMITTED
        entry["meshy"] = self.params
        entry["submitted_at"] = time.time()
        self.inflight[clean_name] = task_id
        self._save()
        print(f"  Submitted {entry.get('name', clean_name)} -> {task_id}")

    def _on_succeeded(self, clean_name: str, info: dict[str, Any]) -> bool:
        """Download the GLB. Returns True when a mesh was completed."""
        glb_url = info.get("model_urls", {}).get("glb")
        if not glb_url:
            print(f"  {clean_name}: SUCCEEDED but no GLB url; will re-poll")
            return False
        entry = self.state[clean_name]
        dest = self.output_dir / f"{nc.model_stem(clean_name)}.glb"
        download_glb(glb_url, dest)
        entry["glb_file"] = dest.name
        entry["sidecar_file"] = nc.sidecar_path_for(dest).name
        entry["status"] = nc.STATUS_DONE
        entry["downloaded_at"] = time.time()
        del self.inflight[clean_name]
        self._save()
        if not self.args.skip_audit_update:
            _update_audit(dest, self.audit_csv)
        return True

    def _on_failed(self, clean_name: str, info: dict[str, Any], pending: list[str]) -> None:
        entry = self.state[clean_name]
        retries = int(entry.get("retries", 0)) + 1
        status = str(info.get("status", ""))
        entry["retries"] = retries
        entry["error"] = str(info.get("task_error", status))
        del self.inflight[clean_name]
        if retries <= self.args.max_retries:
            print(f"  {clean_name}: {status}, retry {retries}/{self.args.max_retries}")
            pending.append(clean_name)
            entry["status"] = "selected"
        else:
            entry["status"] = nc.STATUS_FAILED
            print(f"  {clean_name}: {status}, giving up after {retries} tries")
        self._save()

    def run(self, worklist: list[str], completed: int) -> int:
        """Run to completion (or out of credits). Returns the completed count."""
        self.inflight = {
            cn: str(self.state[cn]["task_id"])
            for cn in worklist
            if self.state[cn].get("status") == nc.STATUS_SUBMITTED and self.state[cn].get("task_id")
        }
        pending = [
            cn
            for cn in worklist
            if self.state[cn].get("status") not in (nc.STATUS_DONE, nc.STATUS_SUBMITTED)
        ]
        limit = self.args.limit
        try:
            while completed < limit and (pending or self.inflight):
                while (
                    len(self.inflight) < self.args.max_in_flight
                    and pending
                    and (completed + len(self.inflight)) < limit
                ):
                    self._submit_one(pending.pop(0))

                for clean_name, task_id in list(self.inflight.items()):
                    info = self.client.get_task(task_id)
                    status = str(info.get("status", ""))
                    if status == "SUCCEEDED":
                        if self._on_succeeded(clean_name, info):
                            completed += 1
                            print(f"  Downloaded ({completed}/{limit})")
                    elif status in ("FAILED", "CANCELED", "EXPIRED"):
                        self._on_failed(clean_name, info, pending)

                if self.inflight:
                    time.sleep(self.args.poll_interval)
        except OutOfCreditsError as e:
            print(f"\nStopping: {e}")
        return completed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="Robot distractor directory")
    parser.add_argument("--limit", type=int, default=300, help="Target completed meshes")
    parser.add_argument("--max-in-flight", type=int, default=3, help="Concurrent Meshy tasks")
    parser.add_argument("--poll-interval", type=float, default=15.0, help="Seconds between polls")
    parser.add_argument("--target-polycount", type=int, default=15000, help="Meshy polycount")
    parser.add_argument("--ai-model", default="meshy-6", help="Meshy model (default: meshy-6)")
    parser.add_argument("--enable-pbr", action="store_true", help="Request PBR texture maps")
    parser.add_argument("--max-retries", type=int, default=2, help="Resubmits per failed task")
    parser.add_argument("--dry-run", action="store_true", help="List submissions and exit")
    parser.add_argument(
        "--audit-csv", type=Path, default=Path(_DEFAULT_AUDIT_CSV), help="VRAM audit CSV path"
    )
    parser.add_argument("--skip-audit-update", action="store_true", help="Skip audit CSV update")
    parser.add_argument(
        "--only",
        nargs="*",
        default=None,
        metavar="CLEAN_NAME",
        help="Restrict generation to these clean_names (e.g. sphinx ironwarrior wreckcreation). "
        "Guards against spending credits on the rest of the not-done worklist.",
    )
    args = parser.parse_args()

    output_dir = nc.resolve_cli_path(args.output_dir)
    thumb_dir = output_dir / "thumbnails"
    state_path = output_dir / nc.STATE_FILENAME
    audit_csv = nc.resolve_cli_path(args.audit_csv)

    state = nc.load_state(state_path)
    if not state:
        raise SystemExit(f"No state file at {state_path}. Run download_nhrl_bots.py first.")

    worklist = [cn for cn, e in state.items() if _usable(e)]
    if args.only is not None:
        only = set(args.only)
        found = [cn for cn in worklist if cn in only]
        missing = only - set(found)
        if missing:
            print(f"WARNING: --only names not usable/in-state, skipped: {sorted(missing)}")
        worklist = found
    done = [cn for cn in worklist if state[cn].get("status") == nc.STATUS_DONE]
    print(f"State: {len(worklist)} usable bots, {len(done)} already done, target {args.limit}")

    if args.dry_run:
        todo = [cn for cn in worklist if state[cn].get("status") != nc.STATUS_DONE][
            : max(0, args.limit - len(done))
        ]
        print(f"Would submit {len(todo)} tasks (params: {_meshy_params(args)}):")
        for cn in todo:
            print(f"  {state[cn].get('name', cn)}  ({cn})")
        return

    api_key = os.environ.get("MESHY_API_KEY")
    if not api_key:
        raise SystemExit("MESHY_API_KEY is not set")

    runner = _Generator(
        MeshyClient(api_key),
        _meshy_params(args),
        state,
        state_path,
        thumb_dir,
        output_dir,
        audit_csv,
        args,
    )
    completed = runner.run(worklist, len(done))
    print(f"\nDone. {completed}/{args.limit} meshes complete. State at {state_path}")
    print(f"Next: python compute_nhrl_keypoints.py {args.output_dir}")


if __name__ == "__main__":
    main()
