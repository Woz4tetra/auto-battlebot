"""The cage spec expands to geometry that sits where the frame says it should."""

from __future__ import annotations

from pathlib import Path

import pytest
from synthgen.cage_spec import (
    CageSceneSpec,
    MatSpec,
    PitSpec,
    all_tubes,
    bolt_positions,
    frame_rails,
    house_bot_box,
    load_cage_spec,
    mat_boxes,
    panels,
    panels_outside_camera,
    pit_boxes,
    posts,
    spec_to_dict,
    stage_riser_boxes,
    subtract_rects,
)

SPEC = Path(__file__).resolve().parents[1] / "cage" / "cage2_overhead_high.toml"
MASSD_SPEC = Path(__file__).resolve().parents[1] / "cage" / "massd_resurgence6.toml"


def test_spec_loads_and_overrides_apply() -> None:
    spec = load_cage_spec(SPEC, ["mat.size=2.30", "lights.grid.rows=2", "cage.side_posts=[0.0]"])
    assert spec.mat.size == 2.30
    assert spec.lights.grid.rows == 2 and spec.lights.grid.cols == 2
    assert len(all_tubes(spec)) == 4 and len(spec.lights.wash) == 2
    assert spec.cage.side_posts == (0.0,)
    # The grid is centred: opposite corners of the rig are mirror images.
    first, last = all_tubes(spec)[0].position, all_tubes(spec)[-1].position
    assert first[0] == pytest.approx(-last[0]) and first[1] == pytest.approx(-last[1])
    assert spec_to_dict(spec)["mat"]["size"] == 2.30


def test_unknown_key_is_rejected() -> None:
    with pytest.raises(KeyError):
        load_cage_spec(SPEC, ["mat.sizee=2.3"])


def test_rails_fill_the_gap_between_mat_and_wall() -> None:
    spec = CageSceneSpec()
    rails = {box.name: box for box in frame_rails(spec)}
    near = rails["rail_near"]
    inner_edge = near.center[1] + near.size[1] / 2
    outer_edge = near.center[1] - near.size[1] / 2
    assert inner_edge == pytest.approx(-spec.mat.size / 2)
    assert outer_edge == pytest.approx(-spec.cage.interior / 2)
    top = near.center[2] + near.size[2] / 2
    assert top == pytest.approx(spec.frame.height_above_mat)


def test_bolts_sit_on_the_rail_tops() -> None:
    spec = CageSceneSpec()
    bolts = bolt_positions(spec)
    assert len(bolts) % 4 == 0 and len(bolts) > 0
    for bolt in bolts:
        assert bolt.center[2] == pytest.approx(
            spec.frame.height_above_mat + spec.frame.bolt_head_height / 2
        )
        assert max(abs(bolt.center[0]), abs(bolt.center[1])) > spec.mat.size / 2


def test_panels_and_posts_stand_on_the_wall_plane() -> None:
    spec = CageSceneSpec()
    for panel in panels(spec):
        assert panel.center[2] + panel.size[2] / 2 == pytest.approx(spec.cage.wall_height)
        assert max(abs(panel.center[0]), abs(panel.center[1])) == pytest.approx(
            spec.cage.interior / 2 + spec.panel.thickness / 2
        )
    corners = [box for box in posts(spec) if box.name.startswith("post_corner")]
    assert len(corners) == 4
    for corner in corners:
        assert abs(corner.center[0]) == pytest.approx(
            spec.cage.interior / 2 + spec.cage.post_size / 2
        )
        assert abs(corner.center[1]) == pytest.approx(
            spec.cage.interior / 2 + spec.cage.post_size / 2
        )


def test_house_bot_box_can_be_disabled() -> None:
    spec = load_cage_spec(SPEC, ["house_bot_box.enabled=false"])
    assert house_bot_box(spec) is None
    box = house_bot_box(CageSceneSpec())
    assert box is not None and box.center[2] == pytest.approx(0.15)


def test_mat_image_uv_matches_the_hfield_texture_layout() -> None:
    from synthgen.cage_spec import mat_image_uv

    size = 2.35
    # hfield (+1.0, 0) is W (0, -1.0): column fraction (1 + 1.175) / 2.35, middle row.
    u, v = mat_image_uv(0.0, -1.0, size)
    assert u == pytest.approx((1.0 + 1.175) / 2.35)
    assert v == pytest.approx(0.5)
    # hfield (0, +1.0) is W (+1.0, 0): middle column, row fraction 0.9255 -> v near the bottom.
    u, v = mat_image_uv(1.0, 0.0, size)
    assert u == pytest.approx(0.5)
    assert v == pytest.approx(1.0 - (1.0 + 1.175) / 2.35)


def test_mat_without_pits_is_one_box() -> None:
    boxes = mat_boxes(CageSceneSpec())
    assert [box.name for box in boxes] == ["mat"]
    assert boxes[0].size[:2] == (2.35, 2.35)


def test_pits_cut_the_mat_and_the_riser_without_losing_area() -> None:
    spec = CageSceneSpec(
        mat=MatSpec(size=2.26),
        pits=(PitSpec(center=(0.88, 0.0)), PitSpec(center=(-0.88, 0.1), size=(0.3, 0.5))),
    )
    boxes = mat_boxes(spec)
    kept = sum(box.size[0] * box.size[1] for box in boxes)
    assert kept == pytest.approx(2.26**2 - 0.43 * 0.43 - 0.3 * 0.5)
    # No piece overlaps a pit opening.
    for box in boxes:
        for pit in spec.pits:
            gap_x = abs(box.center[0] - pit.center[0]) - (box.size[0] + pit.size[0]) / 2
            gap_y = abs(box.center[1] - pit.center[1]) - (box.size[1] + pit.size[1]) / 2
            assert gap_x > -1e-9 or gap_y > -1e-9
    assert len(stage_riser_boxes(spec)) > 1


def test_subtract_rects_passes_through_a_miss() -> None:
    rects = [(-1.0, 1.0, -1.0, 1.0)]
    assert subtract_rects(rects, (2.0, 3.0, 2.0, 3.0)) == rects


def test_pit_walls_close_the_hole_below_the_mat() -> None:
    spec = CageSceneSpec(pits=(PitSpec(center=(0.5, -0.2), size=(0.4, 0.4), depth=0.13),))
    boxes = {box.name: box for box in pit_boxes(spec)}
    assert set(boxes) == {
        "pit0_wall_near",
        "pit0_wall_far",
        "pit0_wall_left",
        "pit0_wall_right",
        "pit0_floor",
    }
    thickness = CageSceneSpec().mat.thickness
    for name in ("pit0_wall_near", "pit0_wall_left"):
        # Hangs from the mat underside so no face is coplanar with the mat's top.
        top = boxes[name].center[2] + boxes[name].size[2] / 2
        bottom = boxes[name].center[2] - boxes[name].size[2] / 2
        assert top == pytest.approx(-thickness)
        assert bottom < -0.13
    assert boxes["pit0_floor"].center[2] < -0.13


def test_bolts_can_be_driven_through_the_inner_rail_face() -> None:
    spec = load_cage_spec(SPEC, ['frame.bolt_face="inner"', "frame.bolt_height_above_mat=0.052"])
    bolts = bolt_positions(spec)
    far = [bolt for bolt in bolts if bolt.name.startswith("bolt_far")]
    assert far, "expected bolts on the far rail"
    for bolt in far:
        assert bolt.axis == "y"
        assert bolt.center[2] == pytest.approx(0.052)
        assert bolt.center[1] == pytest.approx(spec.mat.size / 2 + spec.frame.bolt_head_height / 2)


def test_panel_walls_can_be_left_out() -> None:
    spec = load_cage_spec(SPEC, ['panel.walls=["far", "right"]'])
    assert [box.name for box in panels(spec)] == ["panel_far", "panel_right"]
    with pytest.raises(ValueError, match="unknown walls"):
        panels(load_cage_spec(SPEC, ['panel.walls=["top"]']))


def test_one_way_glass_hides_only_the_walls_the_camera_is_behind() -> None:
    spec = load_cage_spec(SPEC, [])
    half = spec.cage.interior / 2
    # Inside the cage nothing is hidden: panes seen across the arena are what the real
    # picture shows on the far side.
    assert panels_outside_camera(spec, (0.0, 0.0)) == ()
    assert panels_outside_camera(spec, (half - 0.01, half - 0.01)) == ()
    # Past a wall plane, that wall's pane sits between the camera and the mat.
    assert panels_outside_camera(spec, (0.0, -half - 0.5)) == ("near",)
    assert panels_outside_camera(spec, (0.0, half + 0.5)) == ("far",)
    assert panels_outside_camera(spec, (-half - 0.5, 0.0)) == ("left",)
    assert panels_outside_camera(spec, (half + 0.5, 0.0)) == ("right",)
    # A corner mount is outside two of them at once, always in wall order.
    assert panels_outside_camera(spec, (half + 0.5, -half - 0.5)) == ("near", "right")


def test_one_way_glass_covers_the_broadcast_camera_and_the_picked_mounts() -> None:
    spec = load_cage_spec(MASSD_SPEC, [])
    # The fitted MassD broadcast camera sits past the near wall but inside the left one.
    assert panels_outside_camera(spec, (-1.132, -1.621)) == ("near",)
    # The mount distances picked off the sweep are all outside the near wall.
    for distance in (1.49, 1.71, 1.93):
        assert panels_outside_camera(spec, (0.0, -distance)) == ("near",)
