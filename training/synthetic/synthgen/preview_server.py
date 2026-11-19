"""The HTTP side of `pose_camera_server.py`: a mailbox and a stdlib request handler."""

from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, TypedDict
from urllib.parse import parse_qs, urlparse

import numpy as np

from auto_battlebot.perception.cage_calibration import OPENCV_TO_BLENDER_CAMERA
from synthgen.cage_mount import (
    MOUNT_FIELD_BY_RANGE_NAME,
    CageMount,
    CageMountRanges,
    mount_by_range_name,
)
from synthgen.cage_spec import CageSceneSpec, panels_outside_camera
from synthgen.constants import ALPHAS, VIEWS
from synthgen.freefly import (
    FreeflyPose,
    clamp_mount_to_ranges,
    clamp_pose,
    freefly_cam2world,
    freefly_from_cam2world,
    mount_cam2world,
    mount_from_cam2world,
    mount_residual,
)
from synthgen.logsetup import get_logger

logger = get_logger(__name__)

FRAME_POLL_TIMEOUT_S = 2.0
_MAX_BODY_BYTES = 64 * 1024


@dataclass(frozen=True)
class RenderRequest:
    """What the render loop should put on screen next."""

    pose: FreeflyPose
    view: str = "pinhole"
    alpha: float = 1.0
    show_robots: bool = False


@dataclass(frozen=True)
class PublishedFrame:
    """One rendered preview, ready to hand to whichever client is waiting."""

    seq: int
    jpeg: bytes
    view: str
    render_ms: float
    width: int
    height: int


@dataclass(frozen=True)
class Command:
    """Work the render loop has to do on the main thread, in the order it was asked for."""

    kind: str
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class SceneInfo:
    """Static facts about the built scene, gathered once on the main thread."""

    spec_path: str
    calibration_id: str
    calibration_path: str
    out_dir: str
    wall_half_m: float
    mat_size_m: float
    render_width: int
    render_height: int
    preview_width: int
    preview_height: int
    k_rect_full: list[list[float]]
    k_rect_cropped: list[list[float]]
    k_calibrated: list[list[float]]
    distortion: list[float]
    rectified_fov_full_deg: tuple[float, float]
    rectified_fov_cropped_deg: tuple[float, float]
    calibrated_fov_deg: tuple[float, float]
    robot_width_m: float
    robot_length_m: float
    robot_height_m: float


class Mailbox:
    """One pending pose, one published frame, and a command queue, all under one lock."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame_ready = threading.Condition(self._lock)
        self._pending: RenderRequest | None = None
        self._latest: RenderRequest | None = None
        self._frame: PublishedFrame | None = None
        self._commands: list[Command] = []
        self._marks: list[CageMount] = []
        self._status = "starting"
        self._frame_times: list[float] = []
        self._seq = 0

    def submit_pose(self, request: RenderRequest) -> None:
        """Replace whatever the render loop had not started yet."""
        with self._lock:
            self._pending = request
            self._latest = request

    def take_pose(self) -> RenderRequest | None:
        with self._lock:
            pending, self._pending = self._pending, None
            return pending

    @property
    def latest_request(self) -> RenderRequest | None:
        """The newest pose submitted, whether or not it has been rendered."""
        with self._lock:
            return self._latest

    def set_pose(self, request: RenderRequest) -> None:
        """Force the current pose, for a snap or a reset. Same slot, so it still coalesces."""
        self.submit_pose(request)

    def publish(self, jpeg: bytes, view: str, render_ms: float, size: tuple[int, int]) -> int:
        with self._frame_ready:
            self._seq += 1
            self._frame = PublishedFrame(
                seq=self._seq,
                jpeg=jpeg,
                view=view,
                render_ms=render_ms,
                width=size[0],
                height=size[1],
            )
            now = time.monotonic()
            self._frame_times.append(now)
            del self._frame_times[:-30]
            self._frame_ready.notify_all()
            return self._seq

    def wait_for_frame(self, since: int, timeout_s: float) -> PublishedFrame | None:
        deadline = time.monotonic() + timeout_s
        with self._frame_ready:
            while self._frame is None or self._frame.seq <= since:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._frame_ready.wait(remaining)
            return self._frame

    @property
    def fps(self) -> float:
        with self._lock:
            if len(self._frame_times) < 2:
                return 0.0
            span = self._frame_times[-1] - self._frame_times[0]
            return (len(self._frame_times) - 1) / span if span > 0 else 0.0

    def push_command(self, command: Command) -> None:
        with self._lock:
            self._commands.append(command)

    def drain_commands(self) -> list[Command]:
        with self._lock:
            commands, self._commands = self._commands, []
            return commands

    def add_mark(self, mount: CageMount) -> int:
        with self._lock:
            self._marks.append(mount)
            return len(self._marks)

    def clear_marks(self) -> None:
        with self._lock:
            self._marks = []

    @property
    def marks(self) -> list[CageMount]:
        with self._lock:
            return list(self._marks)

    def set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    @property
    def status(self) -> str:
        with self._lock:
            return self._status


def pose_from_json(data: dict[str, Any]) -> FreeflyPose:
    """Read a pose the page sent, clamped. The server owns the clamp so the TOML cannot disagree."""
    return clamp_pose(
        FreeflyPose(
            x_m=float(data["x_m"]),
            y_m=float(data["y_m"]),
            z_m=float(data["z_m"]),
            yaw_deg=float(data["yaw_deg"]),
            pitch_deg=float(data["pitch_deg"]),
            roll_deg=float(data.get("roll_deg", 0.0)),
        )
    )


class OverlayRobot(TypedDict):
    name: str
    center_m: list[float]
    pixels: list[list[float]]
    visible: bool
    width_px: float
    height_px: float


class OverlayCorner(TypedDict):
    pixel: list[float] | None
    in_front: bool
    in_frame: bool


class OverlayMat(TypedDict):
    outline: list[list[float]]
    corners: list[OverlayCorner]


class Overlay(TypedDict):
    mat: OverlayMat
    robots: list[OverlayRobot]


class MountReadout(TypedDict):
    mount: dict[str, Any]
    clamped: dict[str, Any]
    in_ranges: dict[str, bool]
    residual_m: float
    residual_deg: float
    overlay: Overlay
    glass_hidden: list[str]


def mount_readout(
    pose: FreeflyPose,
    scene: SceneInfo,
    ranges: CageMountRanges,
    spec: CageSceneSpec | None,
    alpha: float,
) -> MountReadout:
    """The live `CageMount` the page shows, plus how far outside the sampling ranges it sits."""
    cam2world = freefly_cam2world(pose)
    exact = mount_from_cam2world(cam2world, scene.wall_half_m)
    allowed = mount_from_cam2world(cam2world, scene.wall_half_m, walls=ranges.walls)
    clamped = clamp_mount_to_ranges(allowed, ranges)
    residual = mount_residual(cam2world, clamped, scene.wall_half_m)
    return {
        "mount": mount_by_range_name(exact),
        "clamped": mount_by_range_name(clamped),
        "in_ranges": _within(exact, ranges),
        "residual_m": residual.position_m,
        "residual_deg": residual.angle_deg,
        "overlay": overlay_geometry(pose, scene, alpha),
        "glass_hidden": list(panels_outside_camera(spec, (pose.x_m, pose.y_m))) if spec else [],
    }


COMPASS: tuple[tuple[str, int, int], ...] = (
    ("center", 0, 0),
    ("N", 0, 1),
    ("NE", 1, 1),
    ("E", 1, 0),
    ("SE", 1, -1),
    ("S", 0, -1),
    ("SW", -1, -1),
    ("W", -1, 0),
    ("NW", -1, 1),
)
MAT_MARGIN_M = 0.20
NEAR_PLANE_M = 0.02


def scene_k_rect(scene: SceneInfo, alpha: float) -> np.ndarray:
    """The rectified matrix *alpha* produces: 1.0 keeps every source pixel, 0.0 crops to valid."""
    return np.asarray(scene.k_rect_full if alpha == 1.0 else scene.k_rect_cropped)


def mat_sample_points(mat_size_m: float, margin_m: float = MAT_MARGIN_M) -> dict[str, np.ndarray]:
    """The nine mat points worth measuring a robot at, keyed by compass label."""
    offset = mat_size_m / 2 - margin_m
    return {name: np.array([x * offset, y * offset, 0.0]) for name, x, y in COMPASS}


def world_to_camera(pose: FreeflyPose, points_w: np.ndarray) -> np.ndarray:
    """World points in *pose*'s OpenCV camera frame: z forward, y down."""
    cam2world = freefly_cam2world(pose)
    world_from_camera_cv = cam2world @ np.linalg.inv(OPENCV_TO_BLENDER_CAMERA)
    camera_from_world = np.linalg.inv(world_from_camera_cv)
    points = np.asarray(points_w, dtype=np.float64).reshape(-1, 3)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    return np.asarray((camera_from_world @ homogeneous.T).T[:, :3])


def project_camera_points(k: np.ndarray, points_cam: np.ndarray) -> np.ndarray:
    """Pixels for camera-frame points that are already known to be in front of the lens."""
    points = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if len(points) == 0:
        return np.zeros((0, 2))
    projected = (np.asarray(k) @ (points / points[:, 2:3]).T).T
    return np.asarray(projected[:, :2])


def project_world_points(
    pose: FreeflyPose, k: np.ndarray, points_w: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """(pixels, in_front) for world points seen by *pose* through pinhole matrix *k*."""
    in_camera = world_to_camera(pose, points_w)
    in_front = in_camera[:, 2] > NEAR_PLANE_M
    pixels = np.full((len(in_camera), 2), np.nan)
    if in_front.any():
        pixels[in_front] = project_camera_points(k, in_camera[in_front])
    return pixels, in_front


def clip_to_near_plane(points_cam: np.ndarray, near_m: float = NEAR_PLANE_M) -> np.ndarray:
    """Sutherland-Hodgman clip of a closed polygon against the camera's near plane."""
    points = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    clipped: list[np.ndarray] = []
    for index, current in enumerate(points):
        previous = points[index - 1]
        current_in = bool(current[2] >= near_m)
        previous_in = bool(previous[2] >= near_m)
        if current_in != previous_in:
            span = current[2] - previous[2]
            if abs(span) > 1e-12:
                clipped.append(previous + (near_m - previous[2]) / span * (current - previous))
        if current_in:
            clipped.append(current)
    return np.asarray(clipped) if clipped else np.zeros((0, 3))


def overlay_geometry(pose: FreeflyPose, scene: SceneInfo, alpha: float) -> Overlay:
    """Mat outline and robot-sized boxes, projected for the page to draw over the frame."""
    half = scene.mat_size_m / 2
    corners = np.array(
        [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]]
    )
    k_rect = scene_k_rect(scene, alpha)
    mat_px, mat_front = project_world_points(pose, k_rect, corners)
    outline = project_camera_points(k_rect, clip_to_near_plane(world_to_camera(pose, corners)))

    robots: list[OverlayRobot] = []
    for name, center in mat_sample_points(scene.mat_size_m).items():
        box = _robot_box(center, scene)
        pixels, in_front = project_world_points(pose, k_rect, box)
        visible = bool(np.all(in_front))
        width_px = float(np.ptp(pixels[:, 0])) if visible else 0.0
        height_px = float(np.ptp(pixels[:, 1])) if visible else 0.0
        robots.append(
            {
                "name": name,
                "center_m": [float(v) for v in center],
                "pixels": pixels.tolist() if visible else [],
                "visible": visible,
                "width_px": width_px,
                "height_px": height_px,
            }
        )
    return {
        "mat": {
            "outline": outline.tolist(),
            "corners": [
                {
                    "pixel": [float(x), float(y)] if front else None,
                    "in_front": bool(front),
                    "in_frame": bool(
                        front and 0 <= x < scene.render_width and 0 <= y < scene.render_height
                    ),
                }
                for front, (x, y) in zip(mat_front, mat_px)
            ],
        },
        "robots": robots,
    }


def _robot_box(center: np.ndarray, scene: SceneInfo) -> np.ndarray:
    """The eight corners of a robot-sized box standing on the mat at *center*."""
    half_length, half_width = scene.robot_length_m / 2, scene.robot_width_m / 2
    height = scene.robot_height_m
    return np.array(
        [
            center + np.array([sx * half_length, sy * half_width, sz * height])
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (0, 1)
        ]
    )


def snapped_pose(pose: FreeflyPose, scene: SceneInfo, ranges: CageMountRanges) -> FreeflyPose:
    """The nearest pose the batch sampler could have drawn from *ranges*."""
    cam2world = freefly_cam2world(pose)
    allowed = mount_from_cam2world(cam2world, scene.wall_half_m, walls=ranges.walls)
    clamped = clamp_mount_to_ranges(allowed, ranges)
    return freefly_from_cam2world(mount_cam2world(clamped, scene.wall_half_m))


@dataclass(frozen=True)
class Response:
    """One answer, independent of the HTTP plumbing, so routing can be tested without a socket."""

    code: int
    body: bytes = b""
    content_type: str = "text/plain"
    headers: tuple[tuple[str, str], ...] = ()

    @staticmethod
    def json(payload: Mapping[str, Any], code: int = 200) -> Response:
        return Response(code, json.dumps(payload).encode(), "application/json")


class Router:
    """Every endpoint except the long-poll frame fetch, which needs to block on the condition."""

    def __init__(
        self,
        mailbox: Mailbox,
        scene: SceneInfo,
        ranges: CageMountRanges,
        page_source: Callable[[], bytes],
        spec: CageSceneSpec | None = None,
        out_dir: Path | None = None,
    ) -> None:
        self._mailbox = mailbox
        self._scene = scene
        self._ranges = ranges
        self._page_source = page_source
        self._spec = spec
        self._out_dir = out_dir

    def outputs(self) -> list[dict[str, Any]]:
        """Files written under `--out`, newest first, so the page can link them for download."""
        if self._out_dir is None or not self._out_dir.is_dir():
            return []
        files = [path for path in self._out_dir.iterdir() if path.is_file()]
        files.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        return [
            {
                "name": path.name,
                "size_kb": round(path.stat().st_size / 1024, 1),
                "modified": path.stat().st_mtime,
            }
            for path in files
        ]

    def download(self, name: str) -> Response:
        """One output file as an attachment. Flat directory only, so a name cannot escape it."""
        if self._out_dir is None:
            return Response(404, b"no output directory\n")
        target = self._out_dir / Path(name).name
        if not target.is_file():
            return Response(404, f"no output named {name!r}\n".encode())
        media_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        return Response(
            200,
            target.read_bytes(),
            media_type,
            (("Content-Disposition", f'attachment; filename="{target.name}"'),),
        )

    def get(self, path: str) -> Response:
        if path == "/":
            return Response(200, self._page_source(), "text/html; charset=utf-8")
        if path == "/state":
            return Response.json(self.state())
        if path == "/outputs":
            return Response.json({"files": self.outputs()})
        return Response(404, b"no such endpoint\n")

    def post(self, path: str, body: dict[str, Any]) -> Response:
        if path == "/pose":
            return self._pose(body)
        if path == "/snap":
            return self._snap(body)
        if path in ("/mark", "/save", "/render_full", "/clear_marks"):
            self._mailbox.push_command(Command(path.lstrip("/"), body))
            return Response(204)
        return Response(404, b"no such endpoint\n")

    def state(self) -> dict[str, Any]:
        request = self._mailbox.latest_request
        state: dict[str, Any] = {
            "scene": asdict(self._scene),
            "ranges": asdict(self._ranges),
            "views": list(VIEWS),
            "alphas": list(ALPHAS),
            "fps": self._mailbox.fps,
            "status": self._mailbox.status,
            "marks": [mount_by_range_name(mount) for mount in self._mailbox.marks],
        }
        if request is not None:
            state["pose"] = asdict(request.pose)
            state["view"] = request.view
            state["alpha"] = request.alpha
            state["show_robots"] = request.show_robots
            state.update(
                mount_readout(request.pose, self._scene, self._ranges, self._spec, request.alpha)
            )
        return state

    def _pose(self, body: dict[str, Any]) -> Response:
        try:
            request = RenderRequest(
                pose=pose_from_json(body),
                view=_valid_view(body.get("view", "pinhole")),
                alpha=_valid_alpha(body.get("alpha", 1.0)),
                show_robots=bool(body.get("show_robots", False)),
            )
        except (KeyError, TypeError, ValueError) as error:
            return Response(400, f"bad pose: {error}\n".encode())
        self._mailbox.submit_pose(request)
        return Response.json(
            mount_readout(request.pose, self._scene, self._ranges, self._spec, request.alpha)
        )

    def _snap(self, body: dict[str, Any]) -> Response:
        try:
            pose = pose_from_json(body)
        except (KeyError, TypeError, ValueError) as error:
            return Response(400, f"bad pose: {error}\n".encode())
        return Response.json({"pose": asdict(snapped_pose(pose, self._scene, self._ranges))})


def frame_response(mailbox: Mailbox, since: int) -> Response:
    """Long-poll for a frame newer than *since*. `204` means the render loop is still working."""
    frame = mailbox.wait_for_frame(since, FRAME_POLL_TIMEOUT_S)
    if frame is None:
        return Response(204)
    return Response(
        200,
        frame.jpeg,
        "image/jpeg",
        (
            ("Cache-Control", "no-store"),
            ("X-Pose-Seq", str(frame.seq)),
            ("X-Render-Ms", f"{frame.render_ms:.1f}"),
            ("X-View", frame.view),
        ),
    )


def read_json_body(headers: Any, stream: Any) -> dict[str, Any]:
    """The POST body as a dict, with a cap so a stray request cannot exhaust memory."""
    length = int(headers.get("Content-Length", "0"))
    if length > _MAX_BODY_BYTES:
        raise ValueError(f"request body of {length} bytes is too large")
    return dict(json.loads(stream.read(length))) if length > 0 else {}


def make_handler(mailbox: Mailbox, router: Router) -> type[BaseHTTPRequestHandler]:
    """Wire *router* to the stdlib server. Only the frame long-poll is handled here."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True

        def log_message(self, format: str, *args: Any) -> None:
            logger.debug("%s %s", self.address_string(), format % args)

        def _reply(self, response: Response) -> None:
            self.send_response(response.code)
            for name, value in (*response.headers, *_body_headers(response)):
                self.send_header(name, value)
            self.end_headers()
            if response.body:
                self.wfile.write(response.body)

        def do_GET(self) -> None:
            route = urlparse(self.path)
            if route.path == "/frame":
                since = parse_qs(route.query).get("since", ["0"])[0]
                self._reply(frame_response(mailbox, int(since or 0)))
                return
            if route.path == "/download":
                name = parse_qs(route.query).get("name", [""])[0]
                self._reply(router.download(name))
                return
            self._reply(router.get(route.path))

        def do_POST(self) -> None:
            try:
                body = read_json_body(self.headers, self.rfile)
            except ValueError as error:
                self._reply(Response(400, f"{error}\n".encode()))
                return
            self._reply(router.post(urlparse(self.path).path, body))

    return Handler


def _body_headers(response: Response) -> tuple[tuple[str, str], ...]:
    if not response.body:
        return (("Content-Length", "0"),)
    return (
        ("Content-Type", response.content_type),
        ("Content-Length", str(len(response.body))),
    )


def serve(
    mailbox: Mailbox,
    scene: SceneInfo,
    ranges: CageMountRanges,
    spec: CageSceneSpec,
    out_dir: Path,
    page_source: Callable[[], bytes],
    port: int,
    host: str = "0.0.0.0",
) -> ThreadingHTTPServer:
    """Bind the port and serve on a daemon thread. The caller keeps the main thread for `bpy`."""
    handler = make_handler(mailbox, Router(mailbox, scene, ranges, page_source, spec, out_dir))
    server = ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name="preview-http", daemon=True).start()
    return server


def fov_degrees(k: np.ndarray, width: int, height: int) -> tuple[float, float]:
    """(horizontal, vertical) field of view of a pinhole matrix, in degrees."""
    return (
        math.degrees(2 * math.atan(width / 2 / float(k[0, 0]))),
        math.degrees(2 * math.atan(height / 2 / float(k[1, 1]))),
    )


def _valid_view(view: Any) -> str:
    name = str(view)
    if name not in VIEWS:
        raise ValueError(f"unknown view {name!r}; valid views are {list(VIEWS)}")
    return name


def _valid_alpha(alpha: Any) -> float:
    value = float(alpha)
    if value not in ALPHAS:
        raise ValueError(f"unknown alpha {value!r}; valid alphas are {list(ALPHAS)}")
    return value


def _within(mount: CageMount, ranges: CageMountRanges) -> dict[str, bool]:
    inside = {"wall": mount.wall in ranges.walls}
    for name, mount_field in MOUNT_FIELD_BY_RANGE_NAME.items():
        low, high = getattr(ranges, name)
        inside[name] = min(low, high) <= getattr(mount, mount_field) <= max(low, high)
    return inside
