"""Download robot/mechanical 3D models from Objaverse for use as distractors.

Usage:
    python download_objaverse.py ../data/distractor_models/objaverse --max-models 100
    python download_objaverse.py ../data/distractor_models/objaverse --tags robot vehicle drone
"""

import argparse
import csv
import json
import shutil
from pathlib import Path

import objaverse
import trimesh


DEFAULT_TAGS = [
    # plausible objects
    "robot",
    "mech",
    "mechanical",
    "rc car",
    "vehicle",
    "drone",
    "tank",
    "machine",
    "automaton",
    "debris",
    # random
    "rancho",
    "gauntlet",
    "rebozuelo",
    "recylebin",
    "vrdrawing",
    "nusatenggara",
    "sponge",
    "seaofthievesmegalodon",
    "lakalono",
    "greencriminalff",
    "ianes",
    "rubypistol",
    "ritualistic",
    "avarisclari",
    "book",
    "flub",
    "mtle-2100",
    "drummer",
    "samurai",
    "hanamaru",
    "trusses",
    "slavakondrashevdaun",
    "elodierakoto",
    "kaijin",
    "jorts",
    "screwconveyor",
    "thesis",
    "tinyhome",
    "stolln",
    "dusky",
    "borage",
    "gmtk",
    "shadman",
    "urbanfurnture",
    "croissant",
    "choroon",
    "fresques",
    "flexible",
    "lions-head",
    "rock",
    "house",
    "vistoriabrasil",
    "warblades",
    "espada-del-rey-arruinado",
    "buzzard",
    "battleaxe",
    "commissions",
    "glockenturm",
    "fotogrametria-modelo3d",
    "canadianarmy",
    "madden",
    "hossein",
    "display",
    "endevour",
    "kopf",
    "penicillatus",
    "stg45",
    "witch",
    "clancer",
    "candyshop",
    "agisoftphotoscan",
    "wildhunt",
    "medievalwheelbarrow",
    "aurum",
    "shoe",
    "perusic",
    "deck",
    "supersabre",
    "exotic",
    "chocolate_bar",
    "uchun",
    "bibendum",
    "poly",
    "eggstorage",
    "blade",
    "goal",
    "free",
    "museum",
    "building",
    "place2",
    "quema",
    "sunstrider",
    "pentagonal",
    "cavebear",
    "kitchener",
    "casas-grandes",
    "journeyman",
    "wesselstoop",
    "paleontolgy",
    "kandrakar",
    "whiteplotplant",
    "waniery",
    "trigonal_bipyramidal_intermediate",
    "antonine",
    "simbolo",
    "crowd",
    "helsingingolfklubi",
]


AUDIT_COLUMNS = [
    "source",
    "name",
    "file",
    "disk_mb",
    "mesh_count",
    "verts",
    "faces",
    "textures",
    "max_tex_dim",
    "geom_mb_est",
    "tex_mb_est",
    "total_gpu_mb_est",
]


_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def resolve_cli_path(path: Path) -> Path:
    """Resolve CLI paths independent of caller CWD."""
    if path.is_absolute():
        return path.resolve()

    project_candidate = (_PROJECT_ROOT / path).resolve()
    cwd_candidate = (Path.cwd() / path).resolve()
    script_candidate = (_SCRIPT_DIR / path).resolve()

    for candidate in (project_candidate, cwd_candidate, script_candidate):
        if candidate.exists():
            return candidate
    return project_candidate


def get_available_tags(annotations: dict) -> list[str]:
    all_tags = set()
    for uid, ann in annotations.items():
        ann_tags = [t.get("name", "").lower() for t in ann.get("tags", [])]
        for ann_tag in ann_tags:
            all_tags.add(ann_tag.lower())
    return list(all_tags)


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


def estimate_model_gpu_row(model_path: Path, source: str) -> dict:
    """Estimate model GPU memory usage and return one audit CSV row."""
    scene = trimesh.load(str(model_path), force="scene")
    if scene is None:
        raise RuntimeError(f"Failed to load model: {model_path}")

    geom_bytes = 0
    tex_bytes = 0
    tex_count = 0
    max_tex_dim = 0
    seen_images = set()
    mesh_count = 0
    vert_total = 0
    face_total = 0

    def image_key(img):
        fn = getattr(img, "filename", None)
        if fn:
            return ("fn", fn)
        return ("id", id(img))

    for _, g in scene.geometry.items():
        mesh_count += 1
        v = len(g.vertices) if getattr(g, "vertices", None) is not None else 0
        fcount = len(g.faces) if getattr(g, "faces", None) is not None else 0
        vert_total += v
        face_total += fcount

        # Approximate per-vertex attrs + index buffer.
        geom_bytes += v * (12 + 12 + 8 + 4) + fcount * 12

        visual = getattr(g, "visual", None)
        mat = getattr(visual, "material", None)
        if mat is None:
            continue

        for attr in (
            "image",
            "baseColorTexture",
            "metallicRoughnessTexture",
            "normalTexture",
            "emissiveTexture",
            "occlusionTexture",
            "roughnessTexture",
            "metallicTexture",
        ):
            img = getattr(mat, attr, None)
            if img is None:
                continue
            try:
                key = image_key(img)
                if key in seen_images:
                    continue
                seen_images.add(key)
                w, h = img.size
                max_tex_dim = max(max_tex_dim, w, h)
                tex_count += 1
                # RGBA8 with full mip chain approximation.
                tex_bytes += int(w * h * 4 * (4.0 / 3.0))
            except Exception:
                continue

    total_bytes = geom_bytes + tex_bytes
    return {
        "source": source,
        "name": model_path.name,
        "file": str(model_path.resolve()),
        "disk_mb": round(model_path.stat().st_size / (1024**2), 3),
        "mesh_count": mesh_count,
        "verts": vert_total,
        "faces": face_total,
        "textures": tex_count,
        "max_tex_dim": max_tex_dim,
        "geom_mb_est": round(geom_bytes / (1024**2), 3),
        "tex_mb_est": round(tex_bytes / (1024**2), 3),
        "total_gpu_mb_est": round(total_bytes / (1024**2), 3),
    }


def load_audit_table(audit_csv: Path) -> dict[str, dict]:
    """Load existing audit rows keyed by absolute file path."""
    rows_by_file: dict[str, dict] = {}
    if not audit_csv.exists():
        return rows_by_file

    with audit_csv.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            file_value = row.get("file")
            if not file_value:
                continue
            rows_by_file[str(Path(file_value).resolve())] = row
    return rows_by_file


def write_audit_table(audit_csv: Path, rows_by_file: dict[str, dict]) -> None:
    """Write audit rows sorted by descending total estimated GPU MB."""
    rows = list(rows_by_file.values())
    rows.sort(key=lambda r: float(r.get("total_gpu_mb_est", 0.0)), reverse=True)

    audit_csv.parent.mkdir(parents=True, exist_ok=True)
    with audit_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=AUDIT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


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
    parser.add_argument(
        "--audit-csv",
        type=Path,
        default=_PROJECT_ROOT / "training/data/distractor_models/distractor_gpu_audit.csv",
        help=(
            "Path to distractor GPU audit CSV to update as files are downloaded "
            "(default: training/data/distractor_models/distractor_gpu_audit.csv)"
        ),
    )
    parser.add_argument(
        "--skip-audit-update",
        action="store_true",
        help="Skip updating distractor_gpu_audit.csv after each download",
    )
    args = parser.parse_args()

    output_dir: Path = resolve_cli_path(args.output_dir)
    audit_csv: Path = resolve_cli_path(args.audit_csv)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Loading Objaverse annotations (this may take a minute on first run)...")
    annotations = objaverse.load_annotations()
    print(f"Loaded metadata for {len(annotations)} objects")

    print(f"Searching for tags: {args.tags}")
    matched_uids = search_by_tags(annotations, args.tags, args.max_models)
    print(f"Found {len(matched_uids)} matching models")

    if args.list_only:
        print(f"Available tags {get_available_tags(annotations)} objects")
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
    audit_rows = None
    if not args.skip_audit_update:
        audit_rows = load_audit_table(audit_csv)
        print(f"Loaded {len(audit_rows)} existing audit rows from {audit_csv}")

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

        if audit_rows is not None:
            try:
                row = estimate_model_gpu_row(dest, source=output_dir.name)
                audit_rows[str(dest.resolve())] = row
                write_audit_table(audit_csv, audit_rows)
                print(
                    f"  Audit updated: {dest.name} "
                    f"({row['total_gpu_mb_est']:.1f} MB est)"
                )
            except Exception as e:
                print(f"  Warning: failed to update audit for {dest.name}: {e}")

    manifest_path = output_dir / "manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"Saved {copied} models to {output_dir}")
    print(f"Manifest written to {manifest_path}")
    if not args.skip_audit_update:
        print(f"Audit CSV updated at {audit_csv}")


if __name__ == "__main__":
    main()
