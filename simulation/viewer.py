"""In-loop OpenCV viewer for the kinematic sim.

Off by default (``[viewer] enable = false``) so sweeps stay headless. Rendering costs wall-clock
time but cannot corrupt a run: the sim owns logical time and ships ``sim_time`` in the response
header, which the C++ side adopts through ManualClock. A slow render makes the sim slower in real
time and changes nothing the controller sees. Keep that property when extending this file.

Layers, cheapest first: pre-scaled arena background, hazard discs (raw geometry plus the inflated
keep-out ring the controller actually steers on), robot sprites rotated by yaw, then overlays.

Mouse: press inside an opponent to drag it, release to resume its configured behaviour from the
drop point. Our own robot is deliberately not draggable -- its pose is the plant's integrated
state, and moving it out from under the plant desynchronises the EKF in ways that read as control
bugs.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

HOLE_BGR = (40, 40, 45)
HOLE_RIM_BGR = (90, 90, 100)
BLOCK_BGR = (70, 90, 140)
KEEPOUT_BGR = (60, 170, 240)
OUR_BGR = (120, 220, 120)
OPP_BGR = (90, 90, 235)
TEXT_BGR = (240, 240, 240)


def _resolve(path: str) -> Path:
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


class SpriteSet:
    """Sprites plus the two conventions the runtime depends on, both read from sprites.json:
    pixels-per-metre (so blits scale to any window) and front-points-+X (so rotation is a plain
    rotation by yaw with no per-asset offset). Missing sprites degrade to drawn discs."""

    def __init__(self, sprites_dir: str) -> None:
        self._sprites: dict[str, tuple[np.ndarray, float]] = {}
        directory = _resolve(sprites_dir)
        manifest_path = directory / "sprites.json"
        if not manifest_path.exists():
            return
        manifest = json.loads(manifest_path.read_text())
        for name, entry in manifest.get("sprites", {}).items():
            image_path = directory / entry["file"]
            image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            if image is None:
                continue
            if image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            self._sprites[name] = (image, float(entry["pixels_per_meter"]))

    def get(self, name: str) -> tuple[np.ndarray, float] | None:
        return self._sprites.get(name)


class Viewer:
    def __init__(self, cfg: Any, obstacles: list[Any]) -> None:
        self._cfg = cfg
        self._obstacles = obstacles
        self._size = int(cfg.viewer.window_px)
        self._arena_w = cfg.arena.width
        self._arena_h = cfg.arena.height
        self._px_per_m = self._size / max(self._arena_w, self._arena_h)
        self._sprites = SpriteSet(cfg.viewer.sprites_dir)
        self._background = self._load_background(cfg.viewer.background)
        self._window = "auto-battlebot sim"
        self._drag_index: int | None = None
        self._opponents: list[Any] = []
        # The window is opened on the first render, not here, so the frame composition can be
        # exercised without a display.
        self._window_open = False

    # -- coordinates -------------------------------------------------------

    def _to_px(self, x: float, y: float) -> tuple[int, int]:
        """Field frame (x right, y up, origin at centre) to image pixels (y down)."""
        return (
            int(round(self._size / 2 + x * self._px_per_m)),
            int(round(self._size / 2 - y * self._px_per_m)),
        )

    def _to_world(self, px: int, py: int) -> tuple[float, float]:
        return (
            (px - self._size / 2) / self._px_per_m,
            (self._size / 2 - py) / self._px_per_m,
        )

    # -- layers ------------------------------------------------------------

    def _load_background(self, path: str) -> np.ndarray:
        """Scale the arena texture once at startup and keep the result; blitting a pre-scaled
        background per frame is free. The texture maps the whole arena, so it scales with it."""
        image = cv2.imread(str(_resolve(path)), cv2.IMREAD_COLOR)
        if image is None:
            return np.full((self._size, self._size, 3), 30, dtype=np.uint8)
        scaled: np.ndarray = cv2.resize(
            image, (self._size, self._size), interpolation=cv2.INTER_AREA
        )
        return scaled

    def _draw_obstacles(self, canvas: np.ndarray, inflate: float) -> None:
        for obstacle in self._obstacles:
            cx, cy = obstacle.center
            center = self._to_px(cx, cy)
            radius_px = int(round(obstacle.radius * self._px_per_m))
            if obstacle.kind == "hole":
                cv2.circle(canvas, center, radius_px, HOLE_BGR, -1, cv2.LINE_AA)
                cv2.circle(canvas, center, radius_px, HOLE_RIM_BGR, 2, cv2.LINE_AA)
            else:
                cv2.circle(canvas, center, radius_px, BLOCK_BGR, -1, cv2.LINE_AA)
            # The keep-out ring is the geometry the controller actually steers on. Showing it
            # next to the real hole is the single most useful thing this window does.
            keepout_px = int(round((obstacle.radius + inflate) * self._px_per_m))
            cv2.circle(canvas, center, keepout_px, KEEPOUT_BGR, 1, cv2.LINE_AA)

    def _draw_robot(
        self,
        canvas: np.ndarray,
        x: float,
        y: float,
        yaw: float,
        sprite: str,
        color: tuple,
        r: float,
    ) -> None:
        entry = self._sprites.get(sprite)
        if entry is None:
            center = self._to_px(x, y)
            cv2.circle(canvas, center, int(round(r * self._px_per_m)), color, -1, cv2.LINE_AA)
            nose = self._to_px(x + r * math.cos(yaw), y + r * math.sin(yaw))
            cv2.line(canvas, center, nose, (20, 20, 20), 2, cv2.LINE_AA)
            return
        image, px_per_m = entry
        scale = self._px_per_m / px_per_m
        # Sprites are rendered with the robot front along +X, so this is a plain rotation by yaw.
        # Image y runs down, hence the sign flip.
        matrix = cv2.getRotationMatrix2D(
            (image.shape[1] / 2, image.shape[0] / 2), math.degrees(yaw), scale
        )
        side = int(round(max(image.shape[:2]) * max(scale, 1.0) * 1.5))
        matrix[0, 2] += side / 2 - image.shape[1] / 2
        matrix[1, 2] += side / 2 - image.shape[0] / 2
        rotated = cv2.warpAffine(
            image, matrix, (side, side), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0, 0)
        )
        self._blit(canvas, rotated, self._to_px(x, y))

    @staticmethod
    def _blit(canvas: np.ndarray, rgba: np.ndarray, center: tuple[int, int]) -> None:
        h, w = rgba.shape[:2]
        x0, y0 = center[0] - w // 2, center[1] - h // 2
        x1, y1 = x0 + w, y0 + h
        cx0, cy0 = max(0, x0), max(0, y0)
        cx1, cy1 = min(canvas.shape[1], x1), min(canvas.shape[0], y1)
        if cx0 >= cx1 or cy0 >= cy1:
            return
        patch = rgba[cy0 - y0 : cy1 - y0, cx0 - x0 : cx1 - x0]
        alpha = patch[:, :, 3:4].astype(np.float32) / 255.0
        region = canvas[cy0:cy1, cx0:cx1].astype(np.float32)
        canvas[cy0:cy1, cx0:cx1] = (
            patch[:, :, :3].astype(np.float32) * alpha + region * (1.0 - alpha)
        ).astype(np.uint8)

    def _draw_overlays(
        self, canvas: np.ndarray, plant: Any, tick: int, sim_time: float, command: tuple
    ) -> None:
        x, y, yaw = plant.pose()
        origin = self._to_px(x, y)
        heading = self._to_px(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw))
        cv2.arrowedLine(canvas, origin, heading, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)

        linear, angular = command
        cmd_end = self._to_px(
            x + 0.3 * linear * math.cos(yaw) - 0.3 * angular * math.sin(yaw),
            y + 0.3 * linear * math.sin(yaw) + 0.3 * angular * math.cos(yaw),
        )
        cv2.arrowedLine(canvas, origin, cmd_end, (0, 200, 255), 2, cv2.LINE_AA, tipLength=0.25)

        clearance = plant.min_hazard_clearance
        lines = [
            f"t={sim_time:6.2f}s  tick={tick}",
            f"cmd lin={linear:+.2f} ang={angular:+.2f}",
            f"v={plant.v:+.2f} m/s  w={plant.w:+.2f} rad/s",
        ]
        if self._obstacles:
            gap = "n/a" if clearance == float("inf") else f"{clearance:+.3f} m"
            lines.append(f"min hazard clearance {gap}")
        for i, text in enumerate(lines):
            cv2.putText(
                canvas,
                text,
                (12, 26 + 22 * i),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                TEXT_BGR,
                1,
                cv2.LINE_AA,
            )
        if plant.fell_in:
            cv2.putText(
                canvas,
                "FELL IN",
                (self._size // 2 - 130, self._size // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.8,
                (60, 60, 255),
                4,
                cv2.LINE_AA,
            )

    # -- mouse -------------------------------------------------------------

    def _on_mouse(self, event: int, px: int, py: int, _flags: int, _param: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            wx, wy = self._to_world(px, py)
            for i, opponent in enumerate(self._opponents):
                ox, oy, _ = opponent.pose()
                if math.hypot(wx - ox, wy - oy) < 0.15:
                    self._drag_index = i
                    opponent.dragged = True
                    return
        elif event == cv2.EVENT_MOUSEMOVE and self._drag_index is not None:
            wx, wy = self._to_world(px, py)
            self._opponents[self._drag_index].place(wx, wy)
        elif event == cv2.EVENT_LBUTTONUP and self._drag_index is not None:
            self._opponents[self._drag_index].dragged = False
            self._drag_index = None

    def _opponent_sprite(self, index: int) -> str:
        """Sprite for opponent *index*, from its own config entry.

        Opponents are drawn in config order, so the index maps straight onto ``[[opponents]]``. A
        run that adds opponents beyond the configured list (none do today) falls back to the first
        entry rather than raising mid-frame.
        """
        opponents = self._cfg.opponents
        if not opponents:
            return "mrs_buff_mk2"
        return str(opponents[min(index, len(opponents) - 1)].sprite)

    # -- entry point -------------------------------------------------------

    def compose(
        self, plant: Any, opponents: list[Any], tick: int, sim_time: float, command: tuple
    ) -> np.ndarray:
        """Build one frame. Pure: no window, no waiting, so it can be unit-tested headless."""
        canvas: np.ndarray = self._background.copy()
        self._draw_obstacles(canvas, inflate=self._cfg.our_robot.radius)
        for index, opponent in enumerate(opponents):
            ox, oy, oyaw = opponent.pose()
            self._draw_robot(canvas, ox, oy, oyaw, self._opponent_sprite(index), OPP_BGR, 0.11)
        x, y, yaw = plant.pose()
        self._draw_robot(
            canvas, x, y, yaw, self._cfg.our_robot.sprite, OUR_BGR, self._cfg.our_robot.radius
        )
        self._draw_overlays(canvas, plant, tick, sim_time, command)
        return canvas

    def render(
        self, plant: Any, opponents: list[Any], tick: int, sim_time: float, command: tuple
    ) -> None:
        self._opponents = opponents
        if tick % max(1, self._cfg.viewer.render_every) != 0:
            return

        canvas = self.compose(plant, opponents, tick, sim_time, command)
        if not self._window_open:
            cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self._window, self._on_mouse)
            self._window_open = True
        cv2.imshow(self._window, canvas)
        # Free-running at sim speed makes the window unusable for dragging things by hand, so
        # realtime paces the wait to sim.dt. Headless sweeps never get here.
        wait_ms = 1
        if self._cfg.viewer.realtime:
            wait_ms = max(1, int(self._cfg.sim.dt * self._cfg.viewer.render_every * 1000))
        cv2.waitKey(wait_ms)
