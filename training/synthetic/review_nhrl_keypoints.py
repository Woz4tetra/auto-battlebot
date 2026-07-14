"""Interactively confirm front/back keypoint direction for NHRL robot models.

Step 3.5 of the NHRL robot distractor pipeline, run on the HOST after
``compute_nhrl_keypoints.py`` (which renders each robot's full-material
orthographic top-down and records the model->pixel affine in the sidecar).
Which end is *front* (thrust/weapon direction), and even which axis, cannot be
recovered from a static mesh, so this shows the render and lets an operator
**draw the centerline**: drag across the robot to set the forward axis at any
angle (front = the end you drag toward).  Front/back snap to the footprint's
extremes along that direction, centered on the body.  Confirmed sidecars are
marked ``reviewed: true`` so recompute never clobbers them.  Bad meshes can be
marked ``rejected: true``; ``archive_rejected_nhrl.py`` then moves them out of
the pool so they never render.

The review starts on the first model that is neither reviewed nor rejected, so
re-running the tool resumes where the last pass left off.

This tool loads only the sidecar PNG + JSON (no mesh, no trimesh, no rendering),
so it is light and host-only, but it needs a GUI display.  Run in a root project
venv (needs ``opencv-python`` + ``numpy``):

    venv/bin/python training/synthetic/review_nhrl_keypoints.py \
        training/data/distractor_models/robots

Controls:
    drag       draw the centerline direction (front = drag end)
    f          flip front/back
    a / space  accept (mark reviewed) and go to next
    r          reject the model (mark rejected) and go to next
    n / .      next        p / ,      previous
    u          reset to geometry default (clears reviewed/rejected)
    q / esc    quit
"""

from __future__ import annotations

import argparse
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, cast

import cv2
import nhrl_common as nc
import numpy as np

_HUD_H = 96
_MIN_DRAG_PX = 8
_FRONT_BGR = (60, 60, 235)  # red   (matches preview red = front)
_BACK_BGR = (235, 60, 60)  # blue  (matches preview blue = back)
_AXIS_BGR = (0, 220, 220)  # yellow centerline
_DRAW_BGR = (0, 230, 0)  # green rubber-band while drawing
_WINDOW = "NHRL keypoint direction review"


class Model:
    """One reviewable robot: the top-down render + footprint geometry + axis state."""

    def __init__(self, glb_path: Path) -> None:
        self.glb_path = glb_path
        self.sidecar = nc.sidecar_path_for(glb_path)
        self.existing: dict[str, Any] = nc.load_json(self.sidecar) or {}
        self.reviewed = bool(self.existing.get("reviewed"))
        self.rejected = bool(self.existing.get("rejected"))
        td = self.existing.get("topdown") or {}
        self.image: np.ndarray | None = None
        self.affine: np.ndarray | None = None

        img_path = td.get("image")
        hull = td.get("hull")
        affine = td.get("affine")
        if img_path and hull and affine:
            image = cv2.imread(str(self.sidecar.parent / img_path))
            if image is not None and len(hull) >= 3:
                self.image = image
                self.h, self.w = image.shape[:2]
                self.affine = np.asarray(affine, dtype=float)  # 2x3: [x,z,1]->[px,py]
                self.lin_inv = np.linalg.inv(self.affine[:, :2])
                self.hull = np.asarray(hull, dtype=float)  # (K,2) model x,z
                self.y_ground = float(td.get("y_ground", 0.0))
                self.hull3 = np.column_stack(
                    [self.hull[:, 0], np.full(len(self.hull), self.y_ground), self.hull[:, 1]]
                )
                self.base_xz = 0.5 * (self.hull.min(axis=0) + self.hull.max(axis=0))
                self.default_axis = self._unit(np.asarray(td.get("default_axis", [1.0, 0.0])))
                self.axis_u = self._initial_axis()

    @property
    def reviewable(self) -> bool:
        return self.image is not None and self.affine is not None

    @staticmethod
    def _unit(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n > 1e-9:
            return cast(np.ndarray, np.asarray(v, dtype=float) / n)
        return np.array([1.0, 0.0])

    def _initial_axis(self) -> np.ndarray:
        """Restore a reviewed axis from stored keypoints, else the PCA default."""
        kp = self.existing.get("keypoints") or {}
        front, back = kp.get("front"), kp.get("back")
        if self.reviewed and front is not None and back is not None:
            direction = np.array([front[0] - back[0], front[2] - back[2]], dtype=float)
            if float(np.linalg.norm(direction)) > 1e-9:
                return self._unit(direction)
        return np.array(self.default_axis, dtype=float)

    def reset_axis(self) -> None:
        self.axis_u = np.array(self.default_axis, dtype=float)

    def set_axis_from_pixels(self, d_px: float, d_py: float) -> bool:
        """Set the forward axis from a pixel-space drag delta. False if too small."""
        model_dir = self.lin_inv @ np.array([d_px, d_py])
        if float(np.linalg.norm(model_dir)) < 1e-9:
            return False
        self.axis_u = self._unit(model_dir)
        return True

    def keypoints(self) -> tuple[np.ndarray, np.ndarray]:
        return nc.keypoints_along_axis(self.base_xz, self.axis_u, self.hull3, self.y_ground)

    def to_pixel(self, model_xz: np.ndarray) -> tuple[int, int]:
        px = self.affine @ np.array([model_xz[0], model_xz[1], 1.0])
        return int(round(px[0])), int(round(px[1]))

    def label(self) -> str:
        return str(self.existing.get("name") or self.glb_path.stem)

    def weight_classes(self) -> str:
        wc = self.existing.get("weight_classes") or []
        return ", ".join(str(w) for w in wc) if wc else "?"

    def save(self, reviewed: bool) -> None:
        """Patch the sidecar's keypoints + reviewed flag; keep topdown/metadata.

        Writing keypoints always clears any prior ``rejected`` mark: keeping a
        model and rejecting it are mutually exclusive decisions.
        """
        front, back = self.keypoints()
        data = dict(self.existing)
        data["reviewed"] = bool(reviewed)
        data["rejected"] = False
        data["generated_at"] = datetime.now(timezone.utc).isoformat()
        data["keypoints"] = {
            "frame": nc.KEYPOINT_FRAME,
            "method": nc.KEYPOINT_METHOD,
            "front": [round(float(v), 6) for v in front],
            "back": [round(float(v), 6) for v in back],
        }
        nc.atomic_write_json(self.sidecar, data)
        self.existing = data
        self.reviewed = bool(reviewed)
        self.rejected = False

    def reject(self) -> None:
        """Mark the sidecar ``rejected`` so archive_rejected_nhrl.py can pull it."""
        data = dict(self.existing)
        data["reviewed"] = False
        data["rejected"] = True
        data["generated_at"] = datetime.now(timezone.utc).isoformat()
        nc.atomic_write_json(self.sidecar, data)
        self.existing = data
        self.reviewed = False
        self.rejected = True


def _text(img: np.ndarray, text: str, x: int, y: int, scale: float = 0.6) -> None:
    """Draw readable HUD text (white fill with black outline)."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, text, (x, y), font, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), font, scale, (255, 255, 255), 1, cv2.LINE_AA)


def _compose(model: Model, index: int, total: int) -> np.ndarray:
    assert model.image is not None
    panel = model.image.copy()
    front, back = model.keypoints()
    fpx = model.to_pixel(front[[0, 2]])
    bpx = model.to_pixel(back[[0, 2]])
    cv2.arrowedLine(panel, bpx, fpx, _AXIS_BGR, 2, cv2.LINE_AA, tipLength=0.16)
    cv2.circle(panel, bpx, 9, _BACK_BGR, -1, cv2.LINE_AA)
    cv2.circle(panel, fpx, 9, _FRONT_BGR, -1, cv2.LINE_AA)
    _text(panel, "drag to draw centerline (front = red)", 10, model.h - 14, 0.5)

    hud = np.full((_HUD_H, model.w, 3), 18, dtype=np.uint8)
    if model.rejected:
        status = "REJECTED"
    elif model.reviewed:
        status = "REVIEWED"
    else:
        status = "unreviewed"
    angle = math.degrees(math.atan2(float(model.axis_u[1]), float(model.axis_u[0])))
    span = float(np.linalg.norm(front - back))
    _text(hud, f"[{index + 1}/{total}] {model.label()}  ({model.weight_classes()})", 10, 26, 0.6)
    _text(hud, f"axis: {angle:+.0f} deg   span: {span:.3f} m   {status}", 10, 52, 0.6)
    _text(hud, "drag draw  f flip  a accept  r reject  n/p nav  u reset  q quit", 10, 80, 0.55)
    return np.vstack([hud, panel])


def _in_panel(x: int, y: int, model: Model) -> bool:
    return bool(0 <= x < model.w and _HUD_H <= y < _HUD_H + model.h)


def _collect_models(models_dir: Path, glob: str, limit: int) -> list[Model]:
    models: list[Model] = []
    for glb_path in sorted(models_dir.glob(glob)):
        model = Model(glb_path)
        if not model.reviewable:
            print(f"  Skipping {glb_path.name}: no top-down render in sidecar (run compute first)")
            continue
        models.append(model)
        if limit > 0 and len(models) >= limit:
            break
    return models


def _apply_drag(ui: dict[str, Any], commit: bool) -> None:
    """Update the current model's axis from the active pixel drag."""
    if ui["start"] is None or ui["cur"] is None:
        return
    (sx, sy), (cx, cy) = ui["start"], ui["cur"]
    if math.hypot(cx - sx, cy - sy) < _MIN_DRAG_PX:
        return
    model: Model = ui["model"]
    if model.set_axis_from_pixels(cx - sx, cy - sy) and commit:
        model.save(reviewed=True)


def _on_mouse(event: int, x: int, y: int, flags: int, ui: Any) -> None:
    if event == cv2.EVENT_LBUTTONDOWN and _in_panel(x, y, ui["model"]):
        ui.update(dragging=True, start=(x, y), cur=(x, y), dirty=True)
    elif event == cv2.EVENT_MOUSEMOVE and ui["dragging"]:
        ui["cur"] = (x, y)
        _apply_drag(ui, commit=False)
        ui["dirty"] = True
    elif event == cv2.EVENT_LBUTTONUP and ui["dragging"]:
        _apply_drag(ui, commit=True)
        ui.update(dragging=False, start=None, cur=None, dirty=True)


def _handle_key(models: list[Model], index: int, key: int, ui: dict[str, Any]) -> tuple[int, bool]:
    """Apply a keypress; return (new_index, quit)."""
    model = models[index]
    ui["dirty"] = True
    if key in (27, ord("q")):
        return index, True
    if key == ord("f"):
        model.axis_u = -model.axis_u
        model.save(reviewed=True)
    elif key in (ord("a"), ord(" "), 13, 10):
        model.save(reviewed=True)
        index += 1  # advancing past the last model ends the review loop
    elif key == ord("r"):
        model.reject()
        index += 1  # advancing past the last model ends the review loop
    elif key == ord("u"):
        model.reset_axis()
        model.save(reviewed=False)
    elif key in (81, 2, ord("p"), ord(",")):
        index = max(0, index - 1)
    elif key in (83, 3, ord("n"), ord(".")):
        index = min(len(models) - 1, index + 1)
    return index, False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models_dir", type=Path, help="Directory of robot .glb models")
    parser.add_argument("--models", default="*.glb", help="Glob for models (default: *.glb)")
    parser.add_argument("--limit", type=int, default=0, help="Max models to review (0 = all)")
    args = parser.parse_args()

    models_dir = nc.resolve_cli_path(args.models_dir)
    models = _collect_models(models_dir, args.models, args.limit)
    if not models:
        raise SystemExit(f"No reviewable models matched {args.models} in {models_dir}")

    # Resume on the first model that has not been reviewed or rejected yet.
    start_index = next((i for i, m in enumerate(models) if not m.reviewed and not m.rejected), 0)

    print(
        f"Reviewing {len(models)} models (starting at #{start_index + 1}). "
        "drag=draw f=flip a=accept r=reject n/p=nav u=reset q=quit"
    )
    ui: dict[str, Any] = {
        "model": models[start_index],
        "dragging": False,
        "start": None,
        "cur": None,
        "dirty": True,
    }
    cv2.namedWindow(_WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(_WINDOW, _on_mouse, ui)

    index = start_index
    while 0 <= index < len(models):
        model = models[index]
        ui["model"] = model
        if ui["dirty"]:
            img = _compose(model, index, len(models))
            if ui["dragging"] and ui["start"] and ui["cur"]:
                cv2.arrowedLine(
                    img, ui["start"], ui["cur"], _DRAW_BGR, 2, cv2.LINE_AA, tipLength=0.15
                )
            cv2.imshow(_WINDOW, img)
            ui["dirty"] = False

        key = cv2.waitKey(20) & 0xFF
        if key != 255:  # 255 = no key this tick
            index, quit_now = _handle_key(models, index, key, ui)
            if quit_now:
                break

    cv2.destroyAllWindows()
    reviewed_count = sum(1 for m in models if m.reviewed)
    rejected_count = sum(1 for m in models if m.rejected)
    print(
        f"\n{reviewed_count}/{len(models)} models reviewed, {rejected_count} rejected. "
        f"Sidecars written to {models_dir}."
    )
    if rejected_count:
        print("Run archive_rejected_nhrl.py to move rejected models out of the pool.")


if __name__ == "__main__":
    main()
