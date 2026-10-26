"""Score field RL sessions with the sim's episode metrics (`auto_battlebot/rl/metrics.py`).

Reads both robots' field poses from `/status/tracks` and scores only the time autonomy was
engaged (`runner/autonomy_on_s` above zero). Contact is inferred from the tracked poses, not
measured, so the counts are approximate: the robots touch when their centers are under
`--contact-m` apart.

- Strike: contact, the opponent inside our wedge cone, and we are outside the opponent's arc.
- Hit taken: contact while we are inside the opponent's arc.
- Danger: within `--contact-m` plus `WarpNavConfig.danger_margin_m` and inside the arc.
- Contacts closer than `WarpNavConfig.engagement_gap_s` apart merge into one event.

The contact and wedge thresholds are unvalidated against video. Check strike and hit timestamps
in Foxglove before trusting the strikes-per-hit ratio.

    source scripts/activate_python.sh
    python playground/rl/score_field_session.py data/recordings/<session>.mcap [...]
    python playground/rl/score_field_session.py data/recordings/<session>.mcap --clips hit
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader

from auto_battlebot.recording.diag_io import iter_diagnostic_statuses
from auto_battlebot.recording.mcap_io import iter_messages, iter_video_frames

REPO = Path(__file__).resolve().parents[2]
TRACKS_TOPIC = "/status/tracks"
OPPONENT_LABELS = ("mrs_buff_mk3", "opponent")
# Mirrors of WarpNavConfig defaults, so field and sim numbers share definitions.
ARC_HALF_RAD = math.radians(60.0)
DANGER_MARGIN_M = 0.10
ENGAGEMENT_GAP_S = 0.5
STILL_SPEED_MPS = 0.2
MAX_DT_S = 0.2  # cap on one track sample's weight, so a recording gap adds no time
SOURCE_FPS = 60.0  # /camera/video rate on the ZED Box profiles
CLIP_SPEED = 0.5
CLIP_PRE_S = 1.5
CLIP_POST_S = 1.5
CLIP_WIDTH = 960
CLIP_CROP = 0.5  # crop width and height as a fraction of the frame, centered on the contact
FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class Event:
    kind: str  # "strike" or "hit"
    t0: float  # first and last contact sample, wall clock seconds
    t1: float
    elapsed_s: float  # t0 from the recording's first message
    dist_m: float  # at closest approach, as are both bearings
    our_bearing_deg: float
    their_bearing_deg: float


def wrap(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2 * np.pi) - np.pi


def event_spans(t: np.ndarray, mask: np.ndarray) -> list[tuple[int, int]]:
    """(first, last) indices of runs of `mask`, merging runs closer than ENGAGEMENT_GAP_S."""
    spans: list[tuple[int, int]] = []
    for i in np.flatnonzero(mask):
        if spans and t[i] - t[spans[-1][1]] <= ENGAGEMENT_GAP_S:
            spans[-1] = (spans[-1][0], int(i))
        else:
            spans.append((int(i), int(i)))
    return spans


def recording_start_s(path: Path) -> float:
    with open(path, "rb") as f:
        summary = make_reader(f).get_summary()
    assert summary is not None and summary.statistics is not None, f"{path} has no summary"
    return summary.statistics.message_start_time / 1e9


def load_autonomy(path: Path) -> tuple[np.ndarray, np.ndarray]:
    stamps, engaged = [], []
    for log_time_ns, status in iter_diagnostic_statuses(path):
        if status["hardware_id"] == "runner" and status["name"] == "runner":
            stamps.append(log_time_ns / 1e9)
            engaged.append(status["values"].get("autonomy_on_s", 0) > 0)
    return np.array(stamps), np.array(engaged, dtype=bool)


def load_tracks(path: Path) -> np.ndarray:
    """Rows of (t, our x, y, yaw, opponent x, y, yaw, our u, v, opponent u, v).

    Poses are NaN when either robot is untracked. u and v are normalized image coordinates.
    """
    rows = []
    for _, log_time_ns, data in iter_messages(path, [TRACKS_TOPIC]):
        tracks = json.loads(bytes(data))
        live = [r for r in tracks["robots"] if not r["stale"]]
        ours = [r for r in live if r["ours"]]
        theirs = [r for r in live if not r["ours"] and r["label"] in OPPONENT_LABELS]
        pose: tuple[float, ...] = (math.nan,) * 6
        if ours and theirs:
            o, p = ours[0], theirs[0]
            pose = (o["x"], o["y"], o["yaw"], p["x"], p["y"], p["yaw"])
        pixels: list[float] = []
        for robots in (ours, theirs):
            image = robots[0].get("image") if robots else None
            pixels += [image["u"], image["v"]] if image else [math.nan, math.nan]
        rows.append((log_time_ns / 1e9, *pose, *pixels))
    return np.array(rows)


def score(path: Path, a: np.ndarray, contact_m: float, wedge_half_rad: float) -> list[Event]:
    """Print the session summary and its event list, and return the events."""
    auto_t, auto_on = load_autonomy(path)
    if not auto_on.any():
        print(f"\n{path.name}: autonomy never engaged")
        return []
    t = a[:, 0]
    on = auto_on[np.clip(np.searchsorted(auto_t, t) - 1, 0, len(auto_on) - 1)]
    both = ~np.isnan(a[:, 1])

    dx, dy = a[:, 4] - a[:, 1], a[:, 5] - a[:, 2]
    dist = np.hypot(dx, dy)
    bearing_us = np.abs(wrap(np.arctan2(dy, dx) - a[:, 3]))  # opponent off our heading
    bearing_opp = np.abs(wrap(np.arctan2(-dy, -dx) - a[:, 6]))  # us off the opponent's heading
    in_arc = bearing_opp < ARC_HALF_RAD
    contact = dist < contact_m
    dt = np.minimum(np.diff(t, append=t[-1]), MAX_DT_S)
    speed = np.hypot(np.gradient(a[:, 1], t), np.gradient(a[:, 2], t))

    m = on & both
    auto_s = float(np.sum(dt[on]))
    tm = t[m]
    strikes = event_spans(tm, (contact & (bearing_us < wedge_half_rad) & ~in_arc)[m])
    hits = event_spans(tm, (contact & in_arc)[m])
    danger = m & (dist < contact_m + DANGER_MARGIN_M) & in_arc
    still = m & ~contact & (speed < STILL_SPEED_MPS)
    first_engage = t[on][0]
    first_strike = tm[strikes[0][0]] - first_engage if strikes else math.nan

    def pct(mask: np.ndarray) -> float:
        return 100 * float(np.sum(dt[mask])) / auto_s

    print(f"\n{path.name}")
    print(f"  autonomy time       {auto_s:6.1f} s, both tracked {100 * m.sum() / on.sum():.0f}%")
    print(f"  strikes             {len(strikes)}  ({len(strikes) / auto_s * 60:.1f}/min)")
    print(f"  hits taken          {len(hits)}  ({len(hits) / auto_s * 60:.1f}/min)")
    print(f"  strikes/hit taken   {len(strikes) / max(len(hits), 1):.2f}")
    print(f"  danger time         {pct(danger):.0f}%")
    print(f"  still (no contact)  {pct(still):.0f}%")
    print(f"  median distance     {np.median(dist[m]):.2f} m")
    print(f"  pointed at opp      {100 * np.mean(bearing_us[m] < wedge_half_rad):.0f}%")
    print(f"  behind opp (>120d)  {100 * np.mean(bearing_opp[m] > 2 * math.pi / 3):.0f}%")
    print(f"  first strike        {first_strike:.1f} s after first engage")

    # Elapsed is from the recording's first message, which Foxglove's elapsed time also uses.
    start = recording_start_s(path)
    dist_m, us_deg, opp_deg = dist[m], np.degrees(bearing_us[m]), np.degrees(bearing_opp[m])
    events = []
    for kind, spans in (("strike", strikes), ("hit", hits)):
        for i0, i1 in spans:
            closest = i0 + int(np.argmin(dist_m[i0 : i1 + 1]))
            events.append(
                Event(
                    kind,
                    float(tm[i0]),
                    float(tm[i1]),
                    float(tm[i0] - start),
                    float(dist_m[closest]),
                    float(us_deg[closest]),
                    float(opp_deg[closest]),
                )
            )
    events.sort(key=lambda e: e.t0)
    print("\n  event   elapsed s  wall clock     dur s  min dist m  our bearing  their bearing")
    for e in events:
        wall = datetime.fromtimestamp(e.t0).strftime("%H:%M:%S.%f")[:-3]
        print(
            f"  {e.kind:<6}  {e.elapsed_s:9.2f}  {wall}  {e.t1 - e.t0:5.2f}"
            f"  {e.dist_m:10.2f}  {e.our_bearing_deg:10.0f}d  {e.their_bearing_deg:12.0f}d"
        )
    return events


def render_clips(path: Path, a: np.ndarray, events: list[Event], out_dir: Path) -> None:
    """One clip per event, slowed to CLIP_SPEED, plus review.csv to mark each one.

    Each clip is cropped to half the frame around the contact. Our robot is circled red and the
    opponent blue, at their latest tracked image position.
    A red border marks the frames inside the contact. The recording decodes once, in order.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    clips = [(e, out_dir / f"{e.kind}_{e.elapsed_s:07.2f}s.mp4") for e in events]
    last_t = max(e.t1 for e in events) + CLIP_POST_S
    encoders: dict[int, subprocess.Popen] = {}
    crops: dict[int, tuple[int, int]] = {}
    track_t = a[:, 0]
    for stamp_ns, frame in iter_video_frames(path):
        t = stamp_ns / 1e9
        if t > last_t:
            break
        for i, (e, clip_path) in enumerate(clips):
            if not e.t0 - CLIP_PRE_S <= t <= e.t1 + CLIP_POST_S:
                if i in encoders and t > e.t1 + CLIP_POST_S:
                    close(encoders.pop(i))
                continue
            fh, fw = frame.shape[:2]
            if i not in crops:
                crops[i] = crop_origin(a, e.t0, fw, fh)
            x0, y0 = crops[i]
            crop = frame[y0 : y0 + int(fh * CLIP_CROP), x0 : x0 + int(fw * CLIP_CROP)]
            scale = CLIP_WIDTH / crop.shape[1]
            img = cv2.resize(crop, (CLIP_WIDTH, int(crop.shape[0] * scale) // 2 * 2))
            h, w = img.shape[:2]
            row = a[max(int(np.searchsorted(track_t, t, side="right")) - 1, 0)]
            for (u, v), color, name in (
                ((row[7], row[8]), (40, 40, 230), "ours"),
                ((row[9], row[10]), (230, 160, 20), "opp"),
            ):
                if not math.isnan(u):
                    center = (int((u * fw - x0) * scale), int((v * fh - y0) * scale))
                    cv2.circle(img, center, 50, color, 2, cv2.LINE_AA)
                    offset = (center[0] - 20, center[1] + (-58 if name == "ours" else 72))
                    cv2.putText(img, name, offset, FONT, 0.6, color, 2, cv2.LINE_AA)
            if e.t0 <= t <= e.t1:
                cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 8)
            label = (
                f"{e.kind.upper()}  elapsed {e.elapsed_s:.2f}s  t {t - e.t0:+.2f}s  "
                f"dist {e.dist_m:.2f}m  their bearing {e.their_bearing_deg:.0f}d  "
                f"{CLIP_SPEED:g}x"
            )
            cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
            cv2.putText(img, label, (8, 21), FONT, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            if i not in encoders:
                encoders[i] = open_encoder(clip_path, w, h)
            assert encoders[i].stdin is not None
            encoders[i].stdin.write(img.tobytes())
    for encoder in encoders.values():
        close(encoder)

    with open(out_dir / "review.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["clip", "kind", "elapsed_s", "dist_m", "their_bearing_deg", "confirmed", "notes"]
        )
        for e, clip_path in clips:
            writer.writerow(
                [
                    clip_path.name,
                    e.kind,
                    f"{e.elapsed_s:.2f}",
                    f"{e.dist_m:.2f}",
                    f"{e.their_bearing_deg:.0f}",
                    "",
                    "",
                ]
            )
    print(f"\n  wrote {len(clips)} clips and review.csv to {out_dir}")


def crop_origin(a: np.ndarray, t: float, fw: int, fh: int) -> tuple[int, int]:
    """Top-left pixel of a CLIP_CROP window centered between the robots at time t."""
    row = a[max(int(np.searchsorted(a[:, 0], t, side="right")) - 1, 0)]
    u, v = np.nanmean(row[[7, 9]]), np.nanmean(row[[8, 10]])
    if math.isnan(u):
        u = v = 0.5
    cw, ch = int(fw * CLIP_CROP), int(fh * CLIP_CROP)
    x0 = int(np.clip(u * fw - cw / 2, 0, fw - cw))
    y0 = int(np.clip(v * fh - ch / 2, 0, fh - ch))
    return x0, y0


def open_encoder(path: Path, width: int, height: int) -> subprocess.Popen:
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo"]
    cmd += ["-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", f"{SOURCE_FPS * CLIP_SPEED:g}"]
    cmd += ["-i", "pipe:0", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20"]
    cmd += ["-pix_fmt", "yuv420p", str(path)]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def close(encoder: subprocess.Popen) -> None:
    assert encoder.stdin is not None
    encoder.stdin.close()
    encoder.wait()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument(
        "--contact-m",
        type=float,
        default=0.20,
        help="center distance counted as contact: the two half-widths plus pose noise",
    )
    parser.add_argument("--wedge-half-deg", type=float, default=35.0)
    parser.add_argument(
        "--clips",
        nargs="+",
        choices=("hit", "strike"),
        help="render a review clip per event of these kinds into --clips-dir/<recording>/",
    )
    parser.add_argument("--clips-dir", type=Path, default=REPO / "playground/rl/out/field_clips")
    args = parser.parse_args()
    for path in args.recordings:
        tracks = load_tracks(path)
        events = score(path, tracks, args.contact_m, math.radians(args.wedge_half_deg))
        selected = [e for e in events if args.clips and e.kind in args.clips]
        if selected:
            render_clips(path, tracks, selected, args.clips_dir / path.stem)


if __name__ == "__main__":
    main()
