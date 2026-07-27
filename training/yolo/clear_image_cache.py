"""Delete ultralytics ``cache="disk"`` image caches, keeping recently-used ones.

Training with ``cache="disk"`` writes one ``.npy`` per image next to it. These are pure
derived data -- ultralytics regenerates them on the next run -- but they dwarf the images
themselves: on megamind 297 GB of ``.npy`` sat next to 24 GB of actual unique data, and
filled the NVMe to 97 %.

Deleting them is safe but not free: the next training run on that dataset re-caches, which
costs a few minutes of IO. So this tool skips caches that were *used* recently.

**Age is measured from access time by default, not modification time.** A cache is written
once and then read by every subsequent run, so mtime says "when was this dataset first
trained on" -- a cache generated months ago and used daily looks ancient by mtime. atime
answers the question that matters: when was this last actually read. The root filesystem is
mounted ``relatime``, which updates atime on read once a day, so day-granularity ages are
accurate. On a ``noatime`` mount atime is frozen and this tool says so rather than silently
deleting a hot cache.

Datasets referenced by a running process are never touched, whatever their age.

Usage:
  # what would go, using the default 7-day floor
  python training/yolo/clear_image_cache.py --dry-run

  # actually delete caches unused for 14+ days
  python training/yolo/clear_image_cache.py --older-than 14

  # aggressive: everything not currently in use
  python training/yolo/clear_image_cache.py --older-than 0
"""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

# Datasets live here. `data/` at the repo root holds MCAP/SVO/engines and is off limits.
DEFAULT_ROOT = Path("training/data")
PROTECTED = ("data/recordings", "data/models")


@dataclass
class CacheGroup:
    """One dataset's ``.npy`` cache: its files, size, and last-use time."""

    dataset: Path
    files: list[Path] = field(default_factory=list)
    nbytes: int = 0
    last_used: float = 0.0

    @property
    def age_days(self) -> float:
        return (time.time() - self.last_used) / 86400.0


def relatime_enabled() -> bool:
    """True when the root filesystem updates atime (relatime/strictatime, not noatime)."""
    try:
        opts = subprocess.run(
            ["findmnt", "-no", "OPTIONS", "/"], capture_output=True, text=True, check=True
        ).stdout
    except (OSError, subprocess.CalledProcessError):
        return False
    return "noatime" not in opts


def dataset_root_for(npy: Path, scan_root: Path) -> Path:
    """The dataset directory a cache file belongs to.

    Walks up from the ``.npy`` to the child of ``scan_root`` -- i.e. groups
    ``<root>/<dataset>/<split>/images/x.npy`` under ``<root>/<dataset>``.
    """
    rel = npy.relative_to(scan_root)
    return scan_root / rel.parts[0]


def own_process_chain() -> set[int]:
    """This process and its ancestors.

    They must be excluded from the in-use scan: the shell that invoked this tool very often
    has the dataset name in its own command line (``rm -rf .../old_dataset``, a prior
    ``find``), which would otherwise mark every dataset you just typed about as busy.
    """
    chain: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in chain:
        chain.add(pid)
        try:
            # /proc/<pid>/stat field 4 is PPid; comm (field 2) may contain spaces or parens,
            # so split after the trailing ')'.
            stat = Path(f"/proc/{pid}/stat").read_text()
            pid = int(stat[stat.rindex(")") + 1 :].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return chain


def busy_datasets(candidates: list[Path]) -> set[Path]:
    """Which candidate datasets are being used by a live process.

    Two signals:

    * an **open file descriptor** resolving inside the dataset -- exact, but only catches a
      run at the instant it holds a file open, and ultralytics closes each ``.npy`` right
      after reading it;
    * a **python** process whose command line mentions the dataset, by absolute path *or* by
      directory name. The name is needed because training is launched from ``training/yolo``
      with a relative dataset path (``../data/scenesplit_2026-07-25/old+new.yml``), which an
      absolute-path match silently misses -- the exact way an earlier version of this tool
      offered to delete the cache out from under a live 3-hour run.

    Restricting the command-line signal to *python* processes, and skipping this process and
    its ancestors, is what keeps the loose name match from firing on the shell you are typing
    in. Erring toward "busy" is deliberate: a false positive costs disk, a false negative
    costs a training run.
    """
    resolved = {c: (str(c.resolve()), c.name) for c in candidates}
    skip = own_process_chain()
    busy: set[Path] = set()

    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in skip:
            continue
        try:
            exe = os.path.basename(os.readlink(proc / "exe"))
        except OSError:
            exe = ""
        if exe.startswith("python"):
            try:
                cmdline = (proc / "cmdline").read_bytes().decode("utf-8", "replace")
                busy.update(
                    c for c, (path, name) in resolved.items() if path in cmdline or name in cmdline
                )
            except OSError:
                pass
        try:
            for fd in (proc / "fd").iterdir():
                target = str(fd.resolve())
                busy.update(
                    c for c, (path, _) in resolved.items() if target.startswith(path + os.sep)
                )
        except OSError:
            continue
    return busy


def collect(scan_root: Path, use_atime: bool) -> list[CacheGroup]:
    """Group every ``.npy`` under ``scan_root`` by dataset, summing size and last use."""
    groups: dict[Path, CacheGroup] = {}
    for dirpath, _, filenames in os.walk(scan_root):
        here = Path(dirpath)
        for name in filenames:
            if not name.endswith(".npy"):
                continue
            npy = here / name
            try:
                st = npy.stat()
            except OSError:
                continue
            key = dataset_root_for(npy, scan_root)
            group = groups.setdefault(key, CacheGroup(dataset=key))
            group.files.append(npy)
            group.nbytes += st.st_size
            group.last_used = max(group.last_used, st.st_atime if use_atime else st.st_mtime)
    return sorted(groups.values(), key=lambda g: -g.nbytes)


def classify(
    groups: list[CacheGroup], older_than: float, busy: set[Path]
) -> tuple[list[CacheGroup], list[tuple[CacheGroup, str]]]:
    """Split groups into (deletable, [(kept, reason)])."""
    delete: list[CacheGroup] = []
    keep: list[tuple[CacheGroup, str]] = []
    for group in groups:
        if group.dataset in busy:
            keep.append((group, "in use by a running process"))
        elif group.age_days < older_than:
            keep.append((group, f"used {group.age_days:.1f}d ago (< {older_than:g}d)"))
        else:
            delete.append(group)
    return delete, keep


def gib(nbytes: int) -> str:
    """Human-readable size."""
    return f"{nbytes / 1e9:.1f} GB"


def build_arg_parser() -> argparse.ArgumentParser:
    """Construct the command line argument parser."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument(
        "root",
        nargs="?",
        type=Path,
        default=DEFAULT_ROOT,
        help=f"scan root (default: {DEFAULT_ROOT})",
    )
    parser.add_argument(
        "--older-than",
        type=float,
        default=7.0,
        metavar="DAYS",
        help="keep caches used within this many days (default: 7; 0 = only spare in-use ones)",
    )
    parser.add_argument(
        "--time-source",
        choices=("atime", "mtime"),
        default="atime",
        help="atime = last read (default, the useful one); mtime = when the cache was written",
    )
    parser.add_argument(
        "--labels-cache",
        action="store_true",
        help="also delete labels.cache files (small, but forces a label rescan)",
    )
    parser.add_argument("--dry-run", action="store_true", help="report only, delete nothing")
    return parser


def report(delete: list[CacheGroup], keep: list[tuple[CacheGroup, str]], source: str) -> None:
    """Print the plan."""
    print(f"age measured from {source}\n")
    if keep:
        print("KEEPING:")
        for group, reason in keep:
            print(f"  {gib(group.nbytes):>10s}  {group.dataset.name:34s} {reason}")
        print()
    if delete:
        print("DELETING:")
        for group in delete:
            print(
                f"  {gib(group.nbytes):>10s}  {group.dataset.name:34s} "
                f"{len(group.files)} files, last used {group.age_days:.1f}d ago"
            )
    else:
        print("nothing to delete")


def validate(args: argparse.Namespace) -> bool:
    """Check the scan root and time source; return whether to use atime."""
    root: Path = args.root
    if not root.is_dir():
        raise SystemExit(f"not a directory: {root}")
    if any(p in str(root.resolve()) for p in PROTECTED):
        raise SystemExit(f"refusing to touch protected path: {root}")

    use_atime = bool(args.time_source == "atime")
    if use_atime and not relatime_enabled():
        raise SystemExit(
            "root filesystem is mounted noatime, so atime is frozen and every cache would "
            "look stale. Re-run with --time-source mtime (and note mtime records when the "
            "cache was written, not when it was last used)."
        )
    return use_atime


def sweep(
    root: Path, older_than: float, *, use_atime: bool = True, protect: set[Path] | None = None
) -> tuple[int, int]:
    """Delete stale caches under ``root``. Returns (bytes freed, files removed).

    The importable entry point, used by ``train.py`` to reclaim disk before a run. ``protect`` names
    datasets to spare regardless of age: ``busy_datasets`` skips this process and its ancestors, so
    a caller about to train on a dataset must name it explicitly or risk deleting its own cache
    (harmless -- it regenerates -- but a slow way to start a run).
    """
    groups = collect(root, use_atime)
    if not groups:
        return 0, 0
    protect = protect or set()
    busy = busy_datasets([g.dataset for g in groups]) | protect
    delete, _ = classify(groups, older_than, busy)

    bytes_freed = files_removed = 0
    for group in delete:
        for npy in group.files:
            try:
                npy.unlink()
                files_removed += 1
            except OSError:
                continue
        bytes_freed += group.nbytes
    return bytes_freed, files_removed


def main() -> None:
    """Delete stale ultralytics image caches."""
    args = build_arg_parser().parse_args()
    root: Path = args.root
    use_atime = validate(args)

    groups = collect(root, use_atime)
    if not groups:
        print(f"no .npy caches under {root}")
        return

    delete, keep = classify(groups, args.older_than, busy_datasets([g.dataset for g in groups]))
    report(delete, keep, args.time_source)

    freed = sum(g.nbytes for g in delete)
    if args.dry_run:
        print(f"\ndry run: would free {gib(freed)}")
        return

    removed = 0
    for group in delete:
        for npy in group.files:
            try:
                npy.unlink()
                removed += 1
            except OSError as exc:
                print(f"  could not remove {npy}: {exc}")
        if args.labels_cache:
            for cache in group.dataset.rglob("labels.cache"):
                cache.unlink(missing_ok=True)
    print(f"\nfreed {gib(freed)} ({removed} files)")


if __name__ == "__main__":
    main()
