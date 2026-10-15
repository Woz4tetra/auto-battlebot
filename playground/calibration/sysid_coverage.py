"""Live drivetrain sysid coverage monitor (docs/plans/sysid_coverage_monitor_plan.md).

Drive Mr Stabs Mk2 by hand under the sysid profile; this shows how much of the grey-box plant
the driving so far has pinned, labels each motion, and suggests what would help next. It only
listens: it subscribes to the relay and never sends anything back.

    # on the ZED Box
    ./scripts/build_and_run.sh -c config/mr_stabs_mk2_sysid_zed_box.toml
    # on the laptop, then open http://localhost:8790
    venv/bin/python playground/calibration/sysid_coverage.py --live ws://<zed-box>:8765

    # a recording, at 8x real time
    venv/bin/python playground/calibration/sysid_coverage.py --replay rec.mcap --speed 8

    # a simulated session to replay, until a real one exists
    venv/bin/python playground/calibration/sysid_coverage.py --synthetic /tmp/sysid_sim.mcap

On exit the live estimate, the last snapshot and the event log go to
``data/sysid_sessions/<stamp>/``. The final calibration stays the offline fit on the MCAP.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator

from auto_battlebot.calibration.sysid import model, synthetic
from auto_battlebot.calibration.sysid.coverage import CoverageSession
from auto_battlebot.calibration.sysid.profile import DEFAULT_PROFILE, load_profile
from auto_battlebot.calibration.sysid.server import PageServer
from auto_battlebot.calibration.sysid.sources import Dispatcher, live, replay
from auto_battlebot.calibration.sysid.worker import InlineEngine, ProcessEngine, RefitEngine
from auto_battlebot.recording.mcap_io import read_active_profile

REPO_ROOT = Path(__file__).resolve().parents[2]
ADVANCE_S = 0.1
PUBLISH_S = 0.25


def clean(value: Any) -> Any:
    """NaN and inf to None, so the snapshot is strict JSON."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def hello(profile: str, source: str) -> dict[str, Any]:
    prior = model.PRIOR_VECTOR
    return {
        "profile": profile,
        "source": source,
        "params": [
            {
                "key": p.key,
                "short": p.short,
                "label": p.label,
                "group": p.group,
                "unit": p.unit,
                "dp": p.dp,
                "target": p.target_sd,
                "prior": float(prior[i]),
            }
            for i, p in enumerate(model.PARAMS)
        ],
    }


def write_session(session: CoverageSession, out_root: Path) -> Path:
    out = out_root / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out.mkdir(parents=True, exist_ok=True)
    (out / "plant.toml").write_text(model.params_of(session.theta).to_toml())
    snap = clean(session.snapshot())
    (out / "coverage.json").write_text(json.dumps(snap, indent=1))
    (out / "events.json").write_text(json.dumps(session.events, indent=1))
    return out


def run(
    messages: Iterator[Any],
    session: CoverageSession,
    server: PageServer | None,
    paced: bool,
) -> None:
    dispatch = Dispatcher(session)
    last_advance = last_publish = time.monotonic()
    last_log: int | None = None
    for topic, log_time_ns, data in messages:
        dispatch(topic, log_time_ns, data)
        now = time.monotonic()
        # Unpaced replays advance on recording time, so a fast replay closes the same windows
        # as a live run would.
        due = (
            now - last_advance >= ADVANCE_S
            if paced
            else last_log is None or log_time_ns - last_log >= ADVANCE_S * 1e9
        )
        if due:
            session.advance()
            last_advance, last_log = now, log_time_ns
        if server is not None and now - last_publish >= PUBLISH_S:
            server.publish(clean(session.snapshot()))
            last_publish = now
    session.advance()
    if server is not None:
        server.publish(clean(session.snapshot()))


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", metavar="WS_URL", help="viz_relay to subscribe to, ws://host:8765")
    src.add_argument("--replay", type=Path, metavar="MCAP", help="recording to replay")
    src.add_argument("--synthetic", type=Path, metavar="MCAP", help="write a simulated session")
    ap.add_argument(
        "--profile",
        default=None,
        help=f"app profile (default: the recording's, else {DEFAULT_PROFILE})",
    )
    ap.add_argument(
        "--speed", type=float, default=1.0, help="replay speed; 0 runs as fast as it can"
    )
    ap.add_argument("--port", type=int, default=8790, help="page port on this machine")
    ap.add_argument("--host", default="127.0.0.1", help="page bind address")
    ap.add_argument("--no-serve", action="store_true", help="replay without the page")
    ap.add_argument(
        "--out", type=Path, default=REPO_ROOT / "data/sysid_sessions", help="session output root"
    )
    ap.add_argument("--seconds", type=float, default=300.0, help="--synthetic: session length")
    ap.add_argument(
        "--loop-on-from",
        type=float,
        default=None,
        help="--synthetic: flip switch down from this many seconds",
    )
    ap.add_argument("--seed", type=int, default=1, help="--synthetic: random seed")
    args = ap.parse_args()

    if args.synthetic:
        prof = load_profile(args.profile or DEFAULT_PROFILE)
        sim = synthetic.simulate_session(prof, args.seconds, args.seed, args.loop_on_from)
        path = synthetic.write_mcap(sim, args.synthetic, prof.name)
        print(
            f"wrote {path}: {len(sim.telemetry)} telemetry frames, "
            f"{len(sim.frames)} camera frames, {sim.contacts} wall contacts"
        )
        return

    profile_name = args.profile
    if profile_name is None and args.replay:
        profile_name = read_active_profile(args.replay)
    prof = load_profile(profile_name or DEFAULT_PROFILE)

    paced = bool(args.live) or args.speed > 0
    engine: RefitEngine = ProcessEngine() if paced else InlineEngine()
    session = CoverageSession(prof, engine)
    source = f"live: {args.live}" if args.live else f"replay: {args.replay.name} at {args.speed:g}x"
    server = None
    if args.live or not args.no_serve:
        server = PageServer(args.host, args.port, hello(prof.name, source))
        print(f"page: http://{args.host}:{args.port}")
    try:
        if args.live:
            _sub, messages = live(args.live)
        else:
            messages = replay(args.replay, args.speed)
        run(messages, session, server, paced)
        print(f"replay done: {session.snapshot()['windows']}")
        if server is not None:
            print("page still up; Ctrl-C to write the session and exit")
            while True:
                session.advance()  # applies the refits still in flight
                server.publish(clean(session.snapshot()))
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        out = write_session(session, args.out)
        print(f"session written to {out}")
        engine.close()
        if server is not None:
            server.close()


if __name__ == "__main__":
    main()
