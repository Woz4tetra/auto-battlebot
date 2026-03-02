"""Download robot/mechanical 3D models from Objaverse for use as distractors.

Usage:
    python download_objaverse.py ../data/distractor_models/objaverse --max-models 100
    python download_objaverse.py ../data/distractor_models/objaverse --tags robot vehicle drone
"""

import argparse
import json
import shutil
from pathlib import Path

import objaverse


DEFAULT_TAGS = [
    "robot",
    "mech",
    "mechanical",
    "rc car",
    "vehicle",
    "drone",
    "tank",
    "machine",
    "automaton",
]


def search_by_tags(annotations: dict, tags: list[str], max_models: int) -> list[str]:
    """Filter Objaverse annotations by tag keywords, return UIDs."""
    matched_uids: list[str] = []
    tags_lower = [t.lower() for t in tags]

    for uid, ann in annotations.items():
        if len(matched_uids) >= max_models:
            break
        ann_tags = [t.get("name", "").lower() for t in ann.get("tags", [])]
        ann_name = ann.get("name", "").lower()
        ann_categories = [c.get("name", "").lower() for c in ann.get("categories", [])]
        searchable = ann_tags + [ann_name] + ann_categories

        for tag in tags_lower:
            if any(tag in s for s in searchable):
                matched_uids.append(uid)
                break

    return matched_uids


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, help="Directory to save models")
    parser.add_argument(
        "--max-models",
        type=int,
        default=100,
        help="Maximum number of models to download (default: 100)",
    )
    parser.add_argument(
        "--tags",
        nargs="+",
        default=DEFAULT_TAGS,
        help=f"Tags to search for (default: {DEFAULT_TAGS})",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Only list matching models, don't download",
    )
    args = parser.parse_args()

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Loading Objaverse annotations (this may take a minute on first run)...")
    annotations = objaverse.load_annotations()
    print(f"Loaded metadata for {len(annotations)} objects")

    print(f"Searching for tags: {args.tags}")
    matched_uids = search_by_tags(annotations, args.tags, args.max_models)
    print(f"Found {len(matched_uids)} matching models")

    if args.list_only:
        for uid in matched_uids:
            ann = annotations[uid]
            name = ann.get("name", "unnamed")
            tags = [t.get("name", "") for t in ann.get("tags", [])]
            print(f"  {uid}: {name}  tags={tags}")
        return

    if not matched_uids:
        print("No matching models found. Try different tags with --tags.")
        return

    print(f"Downloading {len(matched_uids)} models...")
    downloaded = objaverse.load_objects(uids=matched_uids)

    manifest = {}
    copied = 0
    for uid, local_path in downloaded.items():
        local_path = Path(local_path)
        dest = output_dir / f"{uid}{local_path.suffix}"
        shutil.copy2(local_path, dest)
        manifest[uid] = {
            "name": annotations[uid].get("name", "unnamed"),
            "file": dest.name,
            "tags": [t.get("name", "") for t in annotations[uid].get("tags", [])],
        }
        copied += 1

    manifest_path = output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"Saved {copied} models to {output_dir}")
    print(f"Manifest written to {manifest_path}")


if __name__ == "__main__":
    main()
