"""Context-vs-appearance interpretability for the synthetic+bbox opponent detectors.

Tests the report's central claim: the opponent detector keys on *context* (arena / "not us,
not house bot, on the floor") rather than the opponent's own *appearance*. Runs four probes on
two models (real_bbox baseline, mix_all) over the independent eval, and prints metrics + saves
figures:

  1. RISE occlusion-saliency  - where does the evidence for an opponent box live? Inside the box
     (appearance) or in the surrounding context? Metric: saliency concentration inside vs a
     context ring vs background, normalized by area.
  2. Cut-paste context swap   - opponent crop on a neutral gray canvas (appearance only) vs on a
     different frame's real arena (appearance + swapped context) vs original. If the score
     collapses without arena context, the model needs context, not the robot's pixels.
  3. Feature-space check      - are real-opponent and synthetic-opponent crops separable in
     backbone-embedding space (domain gap)? Linear-probe accuracy + centroid cosine distance,
     per model. Also whether mix_all pulls the two closer.
  4. Grad-CAM                 - gradient saliency at the P3 neck layer for the opponent score at
     each GT box; corroborates RISE with the inside-box mass fraction.

All probes define the target consistently as the max opponent class score (channels object+robot)
among head anchors whose center falls inside the target box.

Usage:
  PYTHONPATH=. venv/bin/python training/model_eval/interpret_context_vs_appearance.py \
      --eval training/data/nhrl_keypoints_eval_test \
      --real data/models/yolo26n_nhrl_robots_bbox_2026-07-16.pt \
      --mix  data/models/yolo26n_mix_all_2026-07-22.pt \
      --synth training/data/synth_bbox_from_keypoints \
      --out training/data/nhrl_keypoints_eval_test/scores_synth_plus_bbox/interpretability
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import cast

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import yaml
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import StandardScaler
from ultralytics import YOLO

IMGSZ = 640
OPP_CHANNELS = (0, 1)  # engine classes object, robot -> opponent bucket
STRIDES = (8, 16, 32)
GEN = torch.Generator(device="cpu").manual_seed(0)
RNG = np.random.default_rng(0)


# ----------------------------- data -----------------------------


def anchor_centers() -> np.ndarray:
    """(8400, 2) anchor-center pixel coords across the 3 detection scales, matching head order."""
    pts = []
    for s in STRIDES:
        n = IMGSZ // s
        ys, xs = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
        cx = (xs.reshape(-1) + 0.5) * s
        cy = (ys.reshape(-1) + 0.5) * s
        pts.append(np.stack([cx, cy], 1))
    return np.concatenate(pts, 0).astype(np.float32)


def letterbox(img: np.ndarray) -> tuple[np.ndarray, float, float, float]:
    """Resize (aspect-preserving) into IMGSZ square, pad 114. Returns img, scale, pad_x, pad_y."""
    h, w = img.shape[:2]
    r = min(IMGSZ / h, IMGSZ / w)
    nh, nw = round(h * r), round(w * r)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
    px, py = (IMGSZ - nw) // 2, (IMGSZ - nh) // 2
    out[py : py + nh, px : px + nw] = resized
    return out, r, float(px), float(py)


def _frame_from_label(lbl: Path, images_dir: Path, opp_idx: int) -> dict | None:
    """Load one labeled frame's letterboxed image + opponent boxes (640 space), or None."""
    img_path = next(images_dir.glob(lbl.stem + ".*"), None)
    if img_path is None:
        return None
    opp = [
        r.split()
        for r in lbl.read_text().splitlines()
        if r.strip() and int(float(r.split()[0])) == opp_idx
    ]
    if not opp:
        return None
    img: np.ndarray | None = cv2.imread(str(img_path))
    if img is None:
        return None
    h, w = img.shape[:2]
    img640, r, px, py = letterbox(img)
    boxes = []
    for row in opp:
        cx, cy, bw, bh = (float(v) for v in row[1:5])
        x0, y0 = (cx - bw / 2) * w * r + px, (cy - bh / 2) * h * r + py
        x1, y1 = (cx + bw / 2) * w * r + px, (cy + bh / 2) * h * r + py
        if (x1 - x0) > 4 and (y1 - y0) > 4:
            boxes.append([x0, y0, x1, y1])
    if not boxes:
        return None
    return {"img": cv2.cvtColor(img640, cv2.COLOR_BGR2RGB), "boxes": boxes}


def load_eval(root: Path, max_frames: int) -> list[dict]:
    """Reviewed eval frames that contain an opponent box. Returns list of {img640, boxes640}."""
    reviewed = None
    state = root / ".edit_state.json"
    if state.exists():
        reviewed = {Path(p).stem for p in json.loads(state.read_text()).get("reviewed", [])}
    frames: list[dict] = []
    for sub in sorted(d for d in root.iterdir() if (d / "data.yaml").exists()):
        names = list(yaml.safe_load((sub / "data.yaml").read_text())["names"])
        if "opponent" not in names:
            continue
        opp_idx = names.index("opponent")
        for lbl in sorted((sub / "labels").glob("*.txt")):
            if reviewed is not None and lbl.stem not in reviewed:
                continue
            frame = _frame_from_label(lbl, sub / "images", opp_idx)
            if frame is not None:
                frames.append(frame)
            if len(frames) >= max_frames:
                return frames
    return frames


# ----------------------------- model -----------------------------


class Model:
    """Wraps a YOLO .pt: raw opponent-score readout + backbone embedding + Grad-CAM at layer 16."""

    CAM_LAYER = 16

    def __init__(self, pt: str):
        self.yolo = YOLO(pt)
        self.net = cast(torch.nn.Module, self.yolo.model).eval().cuda()
        self.anchors = anchor_centers()

    def _pre(self, imgs_rgb: list[np.ndarray]) -> torch.Tensor:
        arr = np.stack(imgs_rgb).astype(np.float32) / 255.0
        return torch.from_numpy(arr).permute(0, 3, 1, 2).contiguous().cuda()

    def opp_scores(self, imgs_rgb: list[np.ndarray]) -> np.ndarray:
        """(B, 8400) max opponent class probability per anchor for a batch of RGB 640 images."""
        with torch.no_grad():
            _, extra = self.net(self._pre(imgs_rgb))
        s = extra["one2one"]["scores"]  # (B, 5, 8400) raw logits
        opp = torch.sigmoid(s[:, OPP_CHANNELS, :]).amax(1)
        return opp.float().cpu().numpy()

    def box_score(self, imgs_rgb: list[np.ndarray], box: list[float]) -> np.ndarray:
        """Per-image target-box opponent score = max anchor score inside `box`."""
        cx, cy = self.anchors[:, 0], self.anchors[:, 1]
        inside = (cx >= box[0]) & (cx <= box[2]) & (cy >= box[1]) & (cy <= box[3])
        if not inside.any():
            inside = np.ones(len(cx), bool)
        return np.asarray(self.opp_scores(imgs_rgb)[:, inside].max(1))

    def embed(self, crops_rgb: list[np.ndarray]) -> np.ndarray:
        out = []
        for i in range(0, len(crops_rgb), 64):
            batch = [cv2.resize(c, (IMGSZ, IMGSZ)) for c in crops_rgb[i : i + 64]]
            with torch.no_grad():
                _, extra = self.net(self._pre(batch))
            feat = extra["one2one"]["feats"][2]  # (B,256,20,20) deepest neck feature
            out.append(feat.mean((2, 3)).float().cpu().numpy())
        return np.concatenate(out, 0)

    def gradcam(self, img_rgb: np.ndarray, box: list[float]) -> np.ndarray:
        """Grad-CAM (640,640) for the opponent score at `box`, at the P3 neck layer.

        The end2end head detaches its postprocessed dict, so the target is rebuilt from the
        grad-carrying class-branch conv outputs (Detect.cv3), flattened+concatenated into the
        8400-anchor order to match anchor_centers, then the max opponent prob inside the box.
        """
        acts: dict[str, torch.Tensor] = {}
        grads: dict[str, torch.Tensor] = {}
        cls_maps: dict[int, torch.Tensor] = {}
        seq = cast(torch.nn.ModuleList, self.net.model)
        layer = seq[self.CAM_LAYER]
        det = seq[-1]
        handles = [
            layer.register_forward_hook(lambda m, i, o: acts.__setitem__("a", o)),
            layer.register_full_backward_hook(lambda m, gi, go: grads.__setitem__("g", go[0])),
        ]
        for j, branch in enumerate(cast(torch.nn.ModuleList, det.cv3)):
            handles.append(
                branch.register_forward_hook(lambda m, i, o, j=j: cls_maps.__setitem__(j, o))
            )
        try:
            self.net.zero_grad(set_to_none=True)
            x = self._pre([img_rgb]).requires_grad_(True)
            with torch.enable_grad():
                self.net(x)
                opp = torch.cat(
                    [
                        torch.sigmoid(cls_maps[j][0, OPP_CHANNELS]).amax(0).reshape(-1)
                        for j in range(3)
                    ]
                )  # (8400,) opponent prob per anchor, matches anchor_centers order
                cx, cy = self.anchors[:, 0], self.anchors[:, 1]
                inside = (cx >= box[0]) & (cx <= box[2]) & (cy >= box[1]) & (cy <= box[3])
                idx = np.where(inside)[0]
                target = opp[torch.as_tensor(idx).cuda()].max() if len(idx) else opp.max()
                target.backward()
            a, g = acts["a"][0], grads["g"][0]
            w = g.mean((1, 2), keepdim=True)
            cam = torch.relu((w * a).sum(0))
            cam = cam / (cam.max() + 1e-8)
            cam_np = cam.detach().float().cpu().numpy()
        finally:
            for h in handles:
                h.remove()
        return cv2.resize(cam_np, (IMGSZ, IMGSZ))


# ----------------------------- metrics -----------------------------


def mass_fractions(sal: np.ndarray, box: list[float]) -> dict[str, float]:
    """Fraction of saliency mass inside box / in a 2x context ring / in background, and each
    fraction normalized by that region's area fraction (concentration; >1 = focused there)."""
    h, w = sal.shape
    total = sal.sum() + 1e-8
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    bw, bh = x1 - x0, y1 - y0
    rx0, ry0 = max(0, x0 - bw // 2), max(0, y0 - bh // 2)
    rx1, ry1 = min(w, x1 + bw // 2), min(h, y1 + bh // 2)
    inside_mask = np.zeros_like(sal, bool)
    inside_mask[max(0, y0) : y1, max(0, x0) : x1] = True
    ring_mask = np.zeros_like(sal, bool)
    ring_mask[ry0:ry1, rx0:rx1] = True
    ring_mask &= ~inside_mask
    bg_mask = ~inside_mask & ~ring_mask
    px = h * w
    out = {}
    for name, m in (("inside", inside_mask), ("ring", ring_mask), ("background", bg_mask)):
        area_frac = m.sum() / px + 1e-8
        frac = sal[m].sum() / total
        out[f"{name}_frac"] = float(frac)
        out[f"{name}_concentration"] = float(frac / area_frac)
    return out


# ----------------------------- probes -----------------------------


def rise(model: Model, frames: list[dict], n_masks: int, grid: int) -> dict:
    """RISE saliency; returns mean mass fractions/concentrations over sampled opponent boxes."""
    accum: list[dict] = []
    examples: list[tuple] = []
    for fi, fr in enumerate(frames):
        box = fr["boxes"][0]
        base = model.box_score([fr["img"]], box)[0]
        if base < 0.15:  # only explain boxes the model actually detects
            continue
        masks = (RNG.random((n_masks, grid, grid)) < 0.5).astype(np.float32)
        sal = np.zeros((IMGSZ, IMGSZ), np.float32)
        weights = []
        up = []
        for m in masks:
            big = cv2.resize(m, (IMGSZ + grid, IMGSZ + grid), interpolation=cv2.INTER_LINEAR)
            sx, sy = RNG.integers(0, grid), RNG.integers(0, grid)
            up.append(big[sy : sy + IMGSZ, sx : sx + IMGSZ])
        for i in range(0, n_masks, 100):
            chunk = up[i : i + 100]
            imgs = [(fr["img"] * mm[..., None]).astype(np.uint8) for mm in chunk]
            scores = _batch_box_score(model, imgs, box)
            for mm, sc in zip(chunk, scores):
                sal += mm * sc
                weights.append(sc)
        sal /= n_masks * 0.5
        accum.append(mass_fractions(sal, box))
        if len(examples) < 4:
            examples.append((fr["img"], box, sal / (sal.max() + 1e-8)))
    agg = {k: float(np.mean([a[k] for a in accum])) for k in accum[0]} if accum else {}
    agg["n"] = len(accum)
    return {"agg": agg, "examples": examples}


def _batch_box_score(model: Model, imgs: list[np.ndarray], box: list[float]) -> np.ndarray:
    cx, cy = model.anchors[:, 0], model.anchors[:, 1]
    inside = (cx >= box[0]) & (cx <= box[2]) & (cy >= box[1]) & (cy <= box[3])
    if not inside.any():
        inside = np.ones(len(cx), bool)
    return np.asarray(model.opp_scores(imgs)[:, inside].max(1))


def donor_index(frames: list[dict], fi: int, rect: list[float]) -> int:
    """Index of the arena `fi`'s crop gets pasted onto: another frame, clear at `rect`.

    Must not be `fi` itself - pasting a crop back at its own coordinates on its own
    frame rebuilds the original image, which measures nothing. Donors whose own labeled
    robots sit under the paste rect are skipped too, so the swap does not bury a second
    robot. The search starts half the list away because frames are ordered by recording,
    and a donor from a different fight is a stronger context swap than the next frame of
    the same one.
    """
    n = len(frames)
    for offset in range(n // 2, n // 2 + n):
        cand = (fi + offset) % n
        if cand != fi and all(_iou(rect, b) < 0.05 for b in frames[cand]["boxes"]):
            return cand
    return (fi + 1) % n


def cut_paste(model: Model, frames: list[dict]) -> dict:
    """opponent score at the box for: original / crop-on-gray / crop-on-arena / robot-removed."""
    orig, gray_bg, arena_bg, removed = [], [], [], []
    for fi, fr in enumerate(frames):
        img, box = fr["img"], fr["boxes"][0]
        x0, y0, x1, y1 = (int(round(v)) for v in box)
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(IMGSZ, x1), min(IMGSZ, y1)
        if x1 - x0 < 6 or y1 - y0 < 6:
            continue
        crop = img[y0:y1, x0:x1]
        orig.append(model.box_score([img], box)[0])
        g = np.full((IMGSZ, IMGSZ, 3), 114, np.uint8)
        g[y0:y1, x0:x1] = crop
        gray_bg.append(model.box_score([g], box)[0])
        rect = [float(x0), float(y0), float(x1), float(y1)]
        bg = frames[donor_index(frames, fi, rect)]["img"].copy()
        bg[y0:y1, x0:x1] = crop
        arena_bg.append(model.box_score([bg], box)[0])
        rem = img.copy()
        patch = cv2.GaussianBlur(img, (0, 0), 15)[y0:y1, x0:x1]
        rem[y0:y1, x0:x1] = patch
        removed.append(model.box_score([rem], box)[0])
    return {
        "n": len(orig),
        "original": float(np.mean(orig)),
        "crop_on_gray": float(np.mean(gray_bg)),
        "crop_on_arena": float(np.mean(arena_bg)),
        "robot_removed": float(np.mean(removed)),
    }


def feature_space(
    model: Model, real: list[np.ndarray], synth: list[np.ndarray], bg: list[np.ndarray]
) -> dict:
    er, es, eb = model.embed(real), model.embed(synth), model.embed(bg)
    x = np.concatenate([er, es])
    y = np.array([0] * len(er) + [1] * len(es))
    xs = StandardScaler().fit_transform(x)
    acc = float(cross_val_score(LogisticRegression(max_iter=2000), xs, y, cv=5).mean())

    def cos(a: np.ndarray, b: np.ndarray) -> float:
        a, b = a.mean(0), b.mean(0)
        return float(1 - a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-8))

    return {
        "real_vs_synth_probe_acc": acc,
        "real_vs_synth_centroid_cosdist": cos(er, es),
        "real_vs_bg_centroid_cosdist": cos(er, eb),
        "synth_vs_bg_centroid_cosdist": cos(es, eb),
        "embeddings": (er, es, eb),
    }


def gradcam_probe(model: Model, frames: list[dict], k: int) -> dict:
    accum: list[dict] = []
    examples: list[tuple] = []
    for fr in frames[:k]:
        box = fr["boxes"][0]
        if model.box_score([fr["img"]], box)[0] < 0.15:
            continue
        cam = model.gradcam(fr["img"], box)
        accum.append(mass_fractions(cam, box))
        if len(examples) < 4:
            examples.append((fr["img"], box, cam))
    agg = {kk: float(np.mean([a[kk] for a in accum])) for kk in accum[0]} if accum else {}
    agg["n"] = len(accum)
    return {"agg": agg, "examples": examples}


# ----------------------------- crops for feature space -----------------------------


def opponent_crops(frames: list[dict], k: int) -> list[np.ndarray]:
    crops = []
    for fr in frames:
        for box in fr["boxes"]:
            x0, y0, x1, y1 = (int(round(v)) for v in box)
            c = fr["img"][max(0, y0) : y1, max(0, x0) : x1]
            if c.size and c.shape[0] > 8 and c.shape[1] > 8:
                crops.append(c)
            if len(crops) >= k:
                return crops
    return crops


def synth_crops(root: Path, k: int) -> list[np.ndarray]:
    crops: list[np.ndarray] = []
    lbls = sorted((root / "train" / "labels").glob("*.txt"))
    RNG.shuffle(lbls)
    for lbl in lbls:
        img_path = root / "train" / "images" / (lbl.stem + ".jpg")
        if not img_path.exists():
            continue
        rows = [r.split() for r in lbl.read_text().splitlines() if r.strip()]
        opp = [r for r in rows if int(float(r[0])) == 1]  # remapped nhrl_robot -> robot(1)
        if not opp:
            continue
        img = cv2.cvtColor(cv2.imread(str(img_path)), cv2.COLOR_BGR2RGB)
        h, w = img.shape[:2]
        for row in opp:
            cx, cy, bw, bh = (float(v) for v in row[1:5])
            x0, y0 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
            x1, y1 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
            c = img[max(0, y0) : y1, max(0, x0) : x1]
            if c.size and c.shape[0] > 8 and c.shape[1] > 8:
                crops.append(c)
        if len(crops) >= k:
            break
    return crops[:k]


def background_crops(frames: list[dict], k: int) -> list[np.ndarray]:
    crops = []
    for fr in frames:
        boxes = fr["boxes"]
        for _ in range(3):
            s = int(RNG.integers(60, 160))
            x0 = int(RNG.integers(0, IMGSZ - s))
            y0 = int(RNG.integers(0, IMGSZ - s))
            cand = [float(x0), float(y0), float(x0 + s), float(y0 + s)]
            if all(_iou(cand, b) < 0.05 for b in boxes):
                crops.append(fr["img"][y0 : y0 + s, x0 : x0 + s])
            if len(crops) >= k:
                return crops
    return crops


def _iou(a: list[float], b: list[float]) -> float:
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix1 - ix0), max(0, iy1 - iy0)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / (ua + 1e-8)


# ----------------------------- figures -----------------------------


def save_overlays(examples: list, path: Path, title: str) -> None:
    if not examples:
        return
    fig, axes = plt.subplots(1, len(examples), figsize=(4 * len(examples), 4))
    if len(examples) == 1:
        axes = [axes]
    for ax, (img, box, sal) in zip(axes, examples):
        ax.imshow(img)
        ax.imshow(sal, cmap="jet", alpha=0.5)
        x0, y0, x1, y1 = box
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, ec="w", fc="none", lw=2))
        ax.axis("off")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def save_embedding_scatter(feats: dict, path: Path, title: str) -> None:
    er, es, eb = feats["embeddings"]
    x = np.concatenate([er, es, eb])
    x = StandardScaler().fit_transform(x)
    x = x - x.mean(0)
    u, s, vt = np.linalg.svd(x, full_matrices=False)
    proj = x @ vt[:2].T
    n1, n2 = len(er), len(es)
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(proj[:n1, 0], proj[:n1, 1], s=12, label="real opponent", alpha=0.6)
    ax.scatter(
        proj[n1 : n1 + n2, 0], proj[n1 : n1 + n2, 1], s=12, label="synthetic opponent", alpha=0.6
    )
    ax.scatter(proj[n1 + n2 :, 0], proj[n1 + n2 :, 1], s=12, label="background", alpha=0.6)
    ax.legend()
    ax.set_title(title)
    ax.set_xlabel("PC1")
    ax.set_ylabel("PC2")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ----------------------------- main -----------------------------


def main() -> None:
    ap = argparse.ArgumentParser(description="Context-vs-appearance interpretability probes")
    ap.add_argument("--eval", type=Path, required=True)
    ap.add_argument("--real", required=True)
    ap.add_argument("--mix", required=True)
    ap.add_argument("--synth", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--rise-masks", type=int, default=400)
    ap.add_argument("--rise-grid", type=int, default=8)
    ap.add_argument("--gradcam-k", type=int, default=20)
    ap.add_argument("--crops", type=int, default=150)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    frames = load_eval(args.eval, args.frames)
    print(f"loaded {len(frames)} eval frames with opponent boxes")
    real_crops = opponent_crops(frames, args.crops)
    synth = synth_crops(args.synth, args.crops)
    bg = background_crops(frames, args.crops)
    print(f"crops: real_opp={len(real_crops)} synth_opp={len(synth)} background={len(bg)}")

    results: dict = {}
    for tag, pt in (("real_bbox", args.real), ("mix_all", args.mix)):
        print(f"\n===== {tag} =====")
        model = Model(pt)
        r = rise(model, frames, args.rise_masks, args.rise_grid)
        print("RISE:", {k: round(v, 3) for k, v in r["agg"].items()})
        save_overlays(r["examples"], args.out / f"rise_{tag}.png", f"RISE saliency - {tag}")
        cp = cut_paste(model, frames)
        print("cut-paste:", {k: round(v, 3) if isinstance(v, float) else v for k, v in cp.items()})
        fs = feature_space(model, real_crops, synth, bg)
        print("feature-space:", {k: round(v, 3) for k, v in fs.items() if k != "embeddings"})
        save_embedding_scatter(fs, args.out / f"embed_{tag}.png", f"backbone embeddings - {tag}")
        gc = gradcam_probe(model, frames, args.gradcam_k)
        print("grad-cam:", {k: round(v, 3) for k, v in gc["agg"].items()})
        save_overlays(gc["examples"], args.out / f"gradcam_{tag}.png", f"Grad-CAM - {tag}")
        results[tag] = {
            "rise": r["agg"],
            "cut_paste": cp,
            "feature_space": {k: v for k, v in fs.items() if k != "embeddings"},
            "gradcam": gc["agg"],
        }
        del model
        torch.cuda.empty_cache()

    (args.out / "metrics.json").write_text(json.dumps(results, indent=2))
    print(f"\nwrote {args.out / 'metrics.json'} and figures")


if __name__ == "__main__":
    main()
