"""Review NHRL bot thumbnails and reject the ones unfit for Meshy image-to-3D.

Step 1.5 of the NHRL robot distractor pipeline, run on the HOST after
``download_nhrl_bots.py`` and BEFORE ``generate_nhrl_meshes.py`` so no Meshy
credits are spent on bad source images (busy backgrounds, multiple bots, tiny/
side-on shots, logos).  Decisions are recorded in the shared state file
(``.nhrl_generation_state.json``) as a ``rejected`` flag on each bot entry.  A
rejected bot is skipped by both ``generate_nhrl_meshes.py`` (never meshed) and
``download_nhrl_bots.py`` (never re-downloaded, even after its thumbnail is
deleted), so the veto is sticky across re-runs.

Two ways to use it:

1. Interactive review (needs a GUI display + ``opencv-python``):

       ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots

   Shows each downloaded thumbnail; ``a``/space accepts, ``r`` rejects (and moves
   the PNG to ``rejected_thumbnails/``), ``n``/``p`` navigate, ``u`` clears the
   decision, ``q`` quits.  Resumes on the first un-reviewed thumbnail.

2. Bulk reject from a list (no GUI), for names you have already triaged
   elsewhere.  Tokens may be bare clean names, ``<name>.png``, or ``trash:///``
   URIs; pass them as arguments, from a file, or on stdin:

       ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots \
           --reject yeschef mako apex
       ../../venv/bin/python review_nhrl_thumbnails.py ../data/distractor_models/robots \
           --reject-file rejected.txt
       cat rejected.txt | review_nhrl_thumbnails.py ../data/distractor_models/robots --reject -

Controls (interactive):
    a / space  accept and go to next
    r          reject (move PNG out of the pool) and go to next
    n / .      next        p / ,      previous
    u          clear the decision (un-review, un-reject)
    q / esc    quit
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import nhrl_common as nc

_THUMB_REVIEWED_KEY = "thumbnail_reviewed"
_REJECTED_DIRNAME = "rejected_thumbnails"


def clean_name_from_token(token: str) -> str:
    """Normalize a pasted token to a bot clean name.

    Accepts bare names (``yeschef``), filenames (``yeschef.png``), and URIs
    (``trash:///yeschef.png``, ``file:///path/yeschef.png``); returns the stem.
    """
    token = token.strip()
    if not token:
        return ""
    token = token.rsplit("/", 1)[-1]  # drop any scheme/directory prefix
    return Path(token).stem  # drop the .png extension


def _thumbnail_path(thumb_dir: Path, entry: dict[str, Any], clean_name: str) -> Path:
    """On-disk thumbnail path for a state entry."""
    return thumb_dir / str(entry.get("thumbnail_file") or f"{clean_name}.png")


def set_rejected(
    entry: dict[str, Any],
    rejected: bool,
    *,
    thumb_dir: Path,
    rejected_dir: Path,
    clean_name: str,
) -> None:
    """Flag *entry* rejected/accepted and move its PNG in or out of the pool."""
    entry[_THUMB_REVIEWED_KEY] = True
    entry[nc.STATE_REJECTED_KEY] = bool(rejected)
    src = _thumbnail_path(thumb_dir, entry, clean_name)
    if rejected:
        # Move the PNG out of the pool so a stray glob never picks it up.
        if src.exists():
            rejected_dir.mkdir(parents=True, exist_ok=True)
            src.replace(rejected_dir / src.name)
    else:
        # Restore a previously-rejected PNG if it is sitting in the archive.
        archived = rejected_dir / src.name
        if archived.exists() and not src.exists():
            src.parent.mkdir(parents=True, exist_ok=True)
            archived.replace(src)


def clear_decision(
    entry: dict[str, Any], *, thumb_dir: Path, rejected_dir: Path, clean_name: str
) -> None:
    """Undo a prior review: un-reject (restoring the PNG) and un-mark reviewed."""
    set_rejected(
        entry, False, thumb_dir=thumb_dir, rejected_dir=rejected_dir, clean_name=clean_name
    )
    entry[_THUMB_REVIEWED_KEY] = False


# ---------------------------------------------------------------------------
# Bulk (non-interactive) reject
# ---------------------------------------------------------------------------


def _gather_tokens(args: argparse.Namespace) -> list[str]:
    tokens: list[str] = []
    for tok in args.reject or []:
        if tok == "-":
            tokens.extend(sys.stdin.read().split())
        else:
            tokens.append(tok)
    if args.reject_file:
        path = nc.resolve_cli_path(Path(args.reject_file))
        tokens.extend(path.read_text().split())
    return tokens


def _bulk_reject(
    state: dict[str, Any], state_path: Path, thumb_dir: Path, rejected_dir: Path, tokens: list[str]
) -> None:
    names = [n for n in (clean_name_from_token(t) for t in tokens) if n]
    seen: set[str] = set()
    rejected = 0
    unknown: list[str] = []
    for clean_name in names:
        if clean_name in seen:
            continue
        seen.add(clean_name)
        entry = state.get(clean_name)
        if entry is None:
            unknown.append(clean_name)
            continue
        set_rejected(
            entry, True, thumb_dir=thumb_dir, rejected_dir=rejected_dir, clean_name=clean_name
        )
        rejected += 1
        print(f"  rejected {entry.get('name') or clean_name}")

    nc.save_state(state_path, state)
    print(f"\nFlagged {rejected} bot(s) as rejected. State written to {state_path}.")
    if unknown:
        print(f"{len(unknown)} name(s) not found in state (ignored): {', '.join(unknown)}")


# ---------------------------------------------------------------------------
# Interactive review
# ---------------------------------------------------------------------------


def _reviewable_bots(state: dict[str, Any], thumb_dir: Path) -> list[str]:
    """Clean names with a thumbnail on disk (or archived), sorted deterministically."""
    names: list[str] = []
    for clean_name, entry in state.items():
        if entry.get("status") == nc.STATUS_SKIPPED_NO_THUMBNAIL:
            continue
        src = _thumbnail_path(thumb_dir, entry, clean_name)
        archived = thumb_dir.parent / _REJECTED_DIRNAME / src.name
        if src.exists() or archived.exists():
            names.append(clean_name)
    names.sort(key=lambda n: (-int(state[n].get("total_fights", 0)), n))
    return names


def _load_thumb(path: Path, min_width: int = 480) -> Any:
    import cv2

    img = cv2.imread(str(path))
    if img is None:
        return None
    h, w = img.shape[:2]
    if w < min_width and w > 0:
        scale = min_width / w
        img = cv2.resize(img, (min_width, int(round(h * scale))), interpolation=cv2.INTER_NEAREST)
    return img


def _status_label(entry: dict[str, Any]) -> str:
    if nc.is_rejected(entry):
        return "REJECTED"
    return "accepted" if entry.get(_THUMB_REVIEWED_KEY) else "unreviewed"


def _compose_frame(img: Any, entry: dict[str, Any], clean_name: str, index: int, total: int) -> Any:
    """Stack an info HUD above the thumbnail, padded to a minimum width."""
    import cv2
    import numpy as np

    def _text(dst: Any, text: str, x: int, y: int, scale: float = 0.6) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(dst, text, (x, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(dst, text, (x, y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)

    h, w = img.shape[:2]
    panel_w = max(w, 360)
    hud = np.full((88, panel_w, 3), 18, dtype=np.uint8)
    wc = "/".join(str(c) for c in (entry.get("weight_classes") or [])) or "?"
    fights = int(entry.get("total_fights", 0))
    _text(hud, f"[{index + 1}/{total}] {entry.get('name') or clean_name}", 10, 26)
    _text(hud, f"{wc}   {fights} fights   {_status_label(entry)}", 10, 52)
    _text(hud, "a accept  r reject  n/p nav  u clear  q quit", 10, 78, 0.52)
    if w < panel_w:
        img = cv2.copyMakeBorder(img, 0, 0, 0, panel_w - w, cv2.BORDER_CONSTANT, value=(18,) * 3)
    return np.vstack([hud, img])


def _apply_thumb_key(
    key: int,
    entry: dict[str, Any],
    clean_name: str,
    index: int,
    total: int,
    *,
    thumb_dir: Path,
    rejected_dir: Path,
    state: dict[str, Any],
    state_path: Path,
) -> tuple[int, bool]:
    """Apply a keypress; persist any change. Returns (new_index, quit)."""
    if key in (27, ord("q")):
        return index, True
    if key in (ord("a"), ord(" "), 13, 10):
        set_rejected(
            entry, False, thumb_dir=thumb_dir, rejected_dir=rejected_dir, clean_name=clean_name
        )
        nc.save_state(state_path, state)
        return index + 1, False
    if key == ord("r"):
        set_rejected(
            entry, True, thumb_dir=thumb_dir, rejected_dir=rejected_dir, clean_name=clean_name
        )
        nc.save_state(state_path, state)
        return index + 1, False
    if key == ord("u"):
        clear_decision(entry, thumb_dir=thumb_dir, rejected_dir=rejected_dir, clean_name=clean_name)
        nc.save_state(state_path, state)
    elif key in (81, 2, ord("p"), ord(",")):
        return max(0, index - 1), False
    elif key in (83, 3, ord("n"), ord(".")):
        return min(total - 1, index + 1), False
    return index, False


def _run_interactive(
    state: dict[str, Any], state_path: Path, thumb_dir: Path, rejected_dir: Path
) -> None:
    import cv2

    names = _reviewable_bots(state, thumb_dir)
    if not names:
        raise SystemExit(f"No thumbnails found under {thumb_dir} (run download_nhrl_bots.py first)")

    start = next((i for i, n in enumerate(names) if not state[n].get(_THUMB_REVIEWED_KEY)), 0)
    print(
        f"Reviewing {len(names)} thumbnails (starting at #{start + 1}). "
        "a=accept r=reject n/p=nav u=clear q=quit"
    )

    window = "NHRL thumbnail review"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    index = start
    while 0 <= index < len(names):
        clean_name = names[index]
        entry = state[clean_name]
        src = _thumbnail_path(thumb_dir, entry, clean_name)
        if not src.exists():  # rejected PNGs live in the archive dir
            src = rejected_dir / src.name
        img = _load_thumb(src)
        if img is None:
            print(f"  Skipping {clean_name}: thumbnail unreadable ({src})")
            index += 1
            continue

        cv2.imshow(window, _compose_frame(img, entry, clean_name, index, len(names)))
        key = cv2.waitKey(20) & 0xFF
        if key == 255:  # no key this tick
            continue
        index, quit_now = _apply_thumb_key(
            key,
            entry,
            clean_name,
            index,
            len(names),
            thumb_dir=thumb_dir,
            rejected_dir=rejected_dir,
            state=state,
            state_path=state_path,
        )
        if quit_now:
            break

    cv2.destroyAllWindows()
    reviewed = sum(1 for n in names if state[n].get(_THUMB_REVIEWED_KEY))
    rejected = sum(1 for n in names if nc.is_rejected(state[n]))
    print(f"\n{reviewed}/{len(names)} thumbnails reviewed, {rejected} rejected.")
    print(f"State written to {state_path}.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robots_dir", type=Path, help="Robot distractor directory")
    parser.add_argument(
        "--reject",
        nargs="+",
        metavar="TOKEN",
        help="Bulk-reject these names/filenames/URIs (no GUI). Use '-' to read stdin.",
    )
    parser.add_argument(
        "--reject-file", help="Bulk-reject names/filenames/URIs listed in this file (no GUI)"
    )
    args = parser.parse_args()

    robots_dir = nc.resolve_cli_path(args.robots_dir)
    state_path = robots_dir / nc.STATE_FILENAME
    thumb_dir = robots_dir / "thumbnails"
    rejected_dir = robots_dir / _REJECTED_DIRNAME
    state = nc.load_state(state_path)
    if not state:
        raise SystemExit(f"No state at {state_path}; run download_nhrl_bots.py first.")

    if args.reject or args.reject_file:
        tokens = _gather_tokens(args)
        if not tokens:
            raise SystemExit("No reject tokens provided.")
        _bulk_reject(state, state_path, thumb_dir, rejected_dir, tokens)
    else:
        _run_interactive(state, state_path, thumb_dir, rejected_dir)


if __name__ == "__main__":
    main()
