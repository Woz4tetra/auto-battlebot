"""The mailbox coalesces, and the router answers without ever touching Blender.

The render loop runs on the main thread and HTTP handlers run on their own, so the mailbox is the
only shared state. Two things have to hold: a pose that arrives mid-render replaces the pending one
rather than queueing behind it (otherwise releasing W plays back a backlog of stale frames), and
commands do queue, because dropping a mark or a save would lose work.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from synthgen.cage_mount import CageMountRanges
from synthgen.cage_spec import load_cage_spec
from synthgen.freefly import FreeflyPose
from synthgen.preview_server import (
    Command,
    Mailbox,
    RenderRequest,
    Router,
    SceneInfo,
    fov_degrees,
    mat_sample_points,
    overlay_geometry,
    pose_from_json,
)

SCENE = SceneInfo(
    spec_path="cage/cage2_overhead_high.toml",
    calibration_id="ecam25_h01r1_estimated",
    calibration_path="config/cameras/ecam25_h01r1_estimated.toml",
    out_dir="runs/cage_pose",
    wall_half_m=2.4384 / 2,
    mat_size_m=2.35,
    render_width=1280,
    render_height=720,
    preview_width=640,
    preview_height=360,
    k_rect_full=[[384.4, 0.0, 640.8], [0.0, 384.2, 360.2], [0.0, 0.0, 1.0]],
    k_rect_cropped=[[514.3, 0.0, 640.8], [0.0, 514.0, 360.2], [0.0, 0.0, 1.0]],
    k_calibrated=[[731.3, 0.0, 640.0], [0.0, 731.3, 360.0], [0.0, 0.0, 1.0]],
    distortion=[-0.24271, 0.02949, 0.0, 0.0, 0.0],
    rectified_fov_full_deg=(118.0, 86.3),
    rectified_fov_cropped_deg=(102.4, 55.5),
    calibrated_fov_deg=(82.4, 52.4),
    robot_length_m=0.21,
    robot_width_m=0.18,
    robot_height_m=0.09,
)
FITTED = {"x_m": 0.02, "y_m": -1.1102, "z_m": 1.16, "yaw_deg": 0.0, "pitch_deg": -58.1}
SPEC = Path(__file__).resolve().parents[1] / "cage" / "cage2_overhead_high.toml"


@pytest.fixture
def router() -> Router:
    return Router(Mailbox(), SCENE, CageMountRanges(), lambda: b"<html></html>")


def request_for(**overrides: float) -> RenderRequest:
    return RenderRequest(pose=pose_from_json(dict(FITTED, **overrides)))


def test_poses_coalesce_to_the_newest() -> None:
    mailbox = Mailbox()
    for height in (1.1, 1.2, 1.3):
        mailbox.submit_pose(request_for(z_m=height))

    taken = mailbox.take_pose()
    assert taken is not None
    assert taken.pose.z_m == pytest.approx(1.3)
    assert mailbox.take_pose() is None  # the two older poses were dropped, not queued


def test_commands_queue_rather_than_coalesce() -> None:
    mailbox = Mailbox()
    mailbox.push_command(Command("mark"))
    mailbox.push_command(Command("mark"))
    mailbox.push_command(Command("save", {"name": "x"}))

    drained = mailbox.drain_commands()
    assert [command.kind for command in drained] == ["mark", "mark", "save"]
    assert mailbox.drain_commands() == []


def test_a_published_frame_is_only_handed_out_once_it_is_newer() -> None:
    mailbox = Mailbox()
    assert mailbox.wait_for_frame(since=0, timeout_s=0.01) is None

    seq = mailbox.publish(b"jpeg", "pinhole", 42.0, (640, 360))
    frame = mailbox.wait_for_frame(since=0, timeout_s=0.01)
    assert frame is not None
    assert (frame.seq, frame.view, frame.render_ms) == (seq, "pinhole", 42.0)
    assert mailbox.wait_for_frame(since=seq, timeout_s=0.01) is None


def test_posting_a_pose_answers_with_the_readout(router: Router) -> None:
    response = router.post("/pose", dict(FITTED, view="rectified"))
    assert response.code == 200
    payload = json.loads(response.body)
    assert payload["mount"]["wall"] == "near"
    assert payload["mount"]["height_m"] == pytest.approx(1.16)
    assert payload["residual_m"] == pytest.approx(0.0, abs=1e-9)
    assert [robot["name"] for robot in payload["overlay"]["robots"]] == [
        "center",
        "N",
        "NE",
        "E",
        "SE",
        "S",
        "SW",
        "W",
        "NW",
    ]
    assert [corner["in_frame"] for corner in payload["overlay"]["mat"]["corners"]] == [
        True,
        True,
        True,
        True,
    ]


def test_an_unknown_view_is_rejected_rather_than_rendered(router: Router) -> None:
    assert router.post("/pose", dict(FITTED, view="orthographic")).code == 400
    assert router.post("/pose", {"x_m": 0.0}).code == 400


def test_marks_and_saves_become_commands_not_answers(router: Router) -> None:
    assert router.post("/mark", {}).code == 204
    assert router.post("/save", {"name": "cage_pose"}).code == 204
    assert router.post("/nowhere", {}).code == 404


def test_snap_pulls_a_pose_back_into_the_ranges() -> None:
    ranges = CageMountRanges(walls=("near",), height_m=(1.00, 1.20))
    router = Router(Mailbox(), SCENE, ranges, lambda: b"")
    payload = json.loads(router.post("/snap", dict(FITTED, z_m=1.9)).body)
    assert payload["pose"]["z_m"] == pytest.approx(1.20)


def test_state_reports_the_scene_before_any_pose_arrives(router: Router) -> None:
    state = router.state()
    assert state["scene"]["calibration_id"] == "ecam25_h01r1_estimated"
    assert state["views"] == ["pinhole", "distorted", "rectified"]
    assert "pose" not in state

    router.post("/pose", FITTED)
    assert router.state()["pose"]["z_m"] == pytest.approx(1.16)


def test_the_page_is_re_read_per_request_so_edits_need_no_restart() -> None:
    pages = iter([b"first", b"second"])
    router = Router(Mailbox(), SCENE, CageMountRanges(), lambda: next(pages))
    assert router.get("/").body == b"first"
    assert router.get("/").body == b"second"


def test_sample_points_cover_the_center_and_eight_compass_directions() -> None:
    points = mat_sample_points(2.35, margin_m=0.20)
    assert list(points) == ["center", "N", "NE", "E", "SE", "S", "SW", "W", "NW"]

    offset = 2.35 / 2 - 0.20
    assert np.allclose(points["center"], [0.0, 0.0, 0.0])
    assert np.allclose(points["N"], [0.0, offset, 0.0])  # north is +y, away from the near wall
    assert np.allclose(points["E"], [offset, 0.0, 0.0])
    assert np.allclose(points["SW"], [-offset, -offset, 0.0])
    # Every point stands on the mat, inside the margin the batch pipeline keeps clear.
    for point in points.values():
        assert point[2] == 0.0
        assert max(abs(point[0]), abs(point[1])) <= 2.35 / 2 - 0.20 + 1e-12


def test_sample_points_do_not_move_with_the_camera() -> None:
    """Fixed in the world, so the readout stays stable while flying."""
    assert mat_sample_points(2.35).keys() == mat_sample_points(2.35).keys()
    assert np.allclose(mat_sample_points(2.35)["N"], mat_sample_points(2.35)["N"])


def test_fov_degrees_matches_the_rectified_numbers() -> None:
    horizontal, vertical = fov_degrees(np.asarray(SCENE.k_rect_full), 1280, 720)
    assert horizontal == pytest.approx(118.0, abs=0.1)
    assert vertical == pytest.approx(86.3, abs=0.1)


def test_the_readout_flags_a_mount_outside_the_ranges() -> None:
    ranges = CageMountRanges(walls=("near",), height_m=(1.00, 1.10))
    router = Router(Mailbox(), SCENE, ranges, lambda: b"")
    payload = json.loads(router.post("/pose", FITTED).body)
    assert payload["in_ranges"]["height_m"] is False
    assert payload["clamped"]["height_m"] == pytest.approx(1.10)
    assert payload["residual_m"] == pytest.approx(0.06, abs=1e-6)


def test_the_readout_reports_which_pane_the_render_will_hide() -> None:
    """A mount outside the wall plane loses that pane, because segmentation stops at the glass."""
    spec = load_cage_spec(SPEC)
    router = Router(Mailbox(), SCENE, CageMountRanges(), lambda: b"", spec)
    wall_half = spec.cage.interior / 2

    inside = json.loads(router.post("/pose", dict(FITTED, y_m=-(wall_half - 0.109))).body)
    assert inside["glass_hidden"] == []

    outside = json.loads(router.post("/pose", dict(FITTED, y_m=-(wall_half + 0.35))).body)
    assert outside["glass_hidden"] == ["near"]
    assert outside["mount"]["inset_m"] == pytest.approx(-0.35)


def test_without_a_spec_the_readout_claims_nothing_about_the_glass() -> None:
    router = Router(Mailbox(), SCENE, CageMountRanges(), lambda: b"")
    assert json.loads(router.post("/pose", dict(FITTED, y_m=-9.0)).body)["glass_hidden"] == []


def test_a_corner_behind_the_lens_has_no_pixel_instead_of_a_wrong_one() -> None:
    """A pinhole cannot project a point behind it; substituting a depth puts it somewhere wrong."""
    # Low, level, aimed north: the two southern mat corners fall behind the lens.
    looking_north = FreeflyPose(0.0, 0.0, 0.15, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
    overlay = overlay_geometry(looking_north, SCENE, alpha=1.0)
    corners = overlay["mat"]["corners"]

    behind = [corner for corner in corners if not corner["in_front"]]
    assert len(behind) == 2
    for corner in behind:
        assert corner["pixel"] is None
        assert corner["in_frame"] is False
    for corner in corners:
        if corner["in_front"]:
            assert all(math.isfinite(v) for v in corner["pixel"])


def test_the_outline_is_clipped_to_the_near_plane_not_drawn_through_bad_points() -> None:
    looking_north = FreeflyPose(0.0, 0.0, 0.15, yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
    outline = overlay_geometry(looking_north, SCENE, alpha=1.0)["mat"]["outline"]

    # Two corners in front plus two near-plane crossings: a quad, not a quad with flung corners.
    assert len(outline) == 4
    assert all(math.isfinite(value) for point in outline for value in point)


def test_a_mat_entirely_behind_the_lens_draws_nothing() -> None:
    looking_away = FreeflyPose(0.0, -4.0, 0.5, yaw_deg=180.0, pitch_deg=0.0, roll_deg=0.0)
    overlay = overlay_geometry(looking_away, SCENE, alpha=1.0)
    assert overlay["mat"]["outline"] == []
    assert all(corner["pixel"] is None for corner in overlay["mat"]["corners"])
    assert all(not robot["visible"] for robot in overlay["robots"])


def test_the_whole_mat_in_view_still_gives_a_plain_quad() -> None:
    overhead = FreeflyPose(0.0, -1.11, 1.16, yaw_deg=0.0, pitch_deg=-58.1, roll_deg=0.0)
    overlay = overlay_geometry(overhead, SCENE, alpha=1.0)
    assert len(overlay["mat"]["outline"]) == 4
    assert all(corner["in_front"] for corner in overlay["mat"]["corners"])


def test_alpha_zero_crops_the_field_and_alpha_one_keeps_it() -> None:
    """The two rectifications are different cameras, so the overlay has to follow the toggle."""
    pose = FreeflyPose(**FITTED, roll_deg=0.0)
    full = overlay_geometry(pose, SCENE, alpha=1.0)
    cropped = overlay_geometry(pose, SCENE, alpha=0.0)

    widths = {view["robots"][0]["width_px"] for view in (full, cropped)}
    assert len(widths) == 2  # alpha 0.0 has the longer focal length, so the robot reads larger
    assert cropped["robots"][0]["width_px"] > full["robots"][0]["width_px"]


def test_the_router_rejects_an_alpha_it_cannot_render(router: Router) -> None:
    assert router.post("/pose", dict(FITTED, alpha=0.5)).code == 400
    assert router.post("/pose", dict(FITTED, alpha=1.0)).code == 200
    assert router.post("/pose", dict(FITTED, alpha=0.0)).code == 200


def test_the_state_reports_the_alpha_that_was_rendered(router: Router) -> None:
    router.post("/pose", dict(FITTED, alpha=0.0))
    state = router.state()
    assert state["alpha"] == 0.0
    assert state["alphas"] == [1.0, 0.0]


def test_the_pose_reply_projects_through_the_alpha_it_was_given(router: Router) -> None:
    """The live readout must follow the alpha toggle, not quietly stay on 1.0."""
    full = json.loads(router.post("/pose", dict(FITTED, alpha=1.0)).body)
    cropped = json.loads(router.post("/pose", dict(FITTED, alpha=0.0)).body)
    assert cropped["overlay"]["robots"][0]["width_px"] > full["overlay"]["robots"][0]["width_px"]
