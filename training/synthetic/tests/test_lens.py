"""Lens views: the wide render covers the sensor, and pixels and points follow the same model."""

from pathlib import Path

import cv2
import numpy as np
import pytest
from synthgen.lens import (
    build_lens_view,
    render_normalized_to_output,
    render_points_to_output,
    warp_image,
    warp_label,
)

from auto_battlebot.perception.camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    rectify_maps,
)

ECAM25 = Path(__file__).resolve().parents[3] / "config" / "cameras" / "ecam25_h01r1_estimated.toml"
SIZE = (320, 180)


@pytest.fixture(scope="module")
def ecam25() -> CameraCalibration:
    return load_camera_calibration(ECAM25)


@pytest.fixture(scope="module")
def distorted(ecam25):
    return build_lens_view(ecam25, SIZE, "distorted")


@pytest.fixture(scope="module")
def rectified(ecam25):
    return build_lens_view(ecam25, SIZE, "rectified", alpha=1.0)


def test_pinhole_renders_at_the_rectified_matrix_without_a_warp(ecam25):
    lens = build_lens_view(ecam25, SIZE, "pinhole")
    assert not lens.warps
    assert lens.render_size == SIZE
    assert np.allclose(lens.render_k, rectify_maps(ecam25, SIZE, 1.0)[2])
    image = np.arange(SIZE[0] * SIZE[1], dtype=np.float32).reshape(SIZE[1], SIZE[0])
    assert warp_image(lens, image) is image


def test_unknown_view_is_an_error(ecam25):
    with pytest.raises(ValueError, match="valid views"):
        build_lens_view(ecam25, SIZE, "fisheye")


def test_wide_render_holds_every_sensor_ray(distorted):
    map_x, map_y = distorted.distortion_map
    render_w, render_h = distorted.render_size
    assert render_w > SIZE[0] and render_h > SIZE[1]
    assert map_x.min() >= 0 and map_x.max() <= render_w - 1
    assert map_y.min() >= 0 and map_y.max() <= render_h - 1


def test_render_centre_keeps_sensor_resolution(ecam25, distorted):
    scaled = ecam25.scaled(*SIZE)
    assert distorted.render_k[0, 0] == pytest.approx(scaled.fx)


def test_pixels_and_points_follow_one_model(distorted):
    # A render pixel the map samples for sensor pixel p must project back onto p.
    map_x, map_y = distorted.distortion_map
    ys, xs = np.mgrid[0 : SIZE[1] : 7, 0 : SIZE[0] : 11]
    render_uv = np.column_stack((map_x[ys, xs].ravel(), map_y[ys, xs].ravel()))
    out, in_view = render_points_to_output(distorted, render_uv)
    assert in_view.all()
    assert np.abs(out - np.column_stack((xs.ravel(), ys.ravel()))).max() < 0.01


def test_rectified_points_land_on_the_rectified_matrix(rectified):
    # The centre of the render is the optical axis, which the rectified matrix puts at its centre.
    k = rectified.render_k
    out, in_view = render_points_to_output(rectified, np.array([[k[0, 2], k[1, 2]]]))
    assert in_view[0]
    assert out[0] == pytest.approx((rectified.rect_k[0, 2], rectified.rect_k[1, 2]), abs=1e-6)


def test_rectified_border_matches_the_rectifier(ecam25, rectified):
    # 36 percent of an alpha 1.0 frame from this lens is black; alpha 0.0 has no border.
    assert 1.0 - rectified.valid.mean() == pytest.approx(0.363, abs=0.02)
    cropped = build_lens_view(ecam25, SIZE, "rectified", alpha=0.0)
    assert cropped.valid.mean() > 0.99


def test_points_in_the_black_border_are_unseen(rectified):
    # A barrel lens at alpha 1.0 keeps its corners and loses the middle of each edge, so take a
    # border pixel from the mask rather than assuming where one is.
    py, px = np.argwhere(~rectified.valid)[0]
    border = np.array([[float(px), float(py)]])
    k, r = rectified.render_k, rectified.rect_k
    norm = (border - (r[0, 2], r[1, 2])) / (r[0, 0], r[1, 1])
    render_uv = norm * (k[0, 0], k[1, 1]) + (k[0, 2], k[1, 2])
    _, in_view = render_points_to_output(rectified, render_uv)
    assert not in_view[0]


def test_points_beyond_the_sensor_are_unseen(distorted):
    k = distorted.render_k
    far = np.array([[k[0, 2] + 50 * k[0, 0], k[1, 2]]])
    _, in_view = render_points_to_output(distorted, far)
    assert not in_view[0]


def test_normalized_round_trip_matches_pixels(distorted):
    map_x, map_y = distorted.distortion_map
    u, v = 100, 60
    render_w, render_h = distorted.render_size
    x_norm = (map_x[v, u] + 0.5) / render_w
    y_norm = (map_y[v, u] + 0.5) / render_h
    out = render_normalized_to_output(distorted, x_norm, y_norm)
    assert out is not None
    assert out == pytest.approx(((u + 0.5) / SIZE[0], (v + 0.5) / SIZE[1]), abs=1e-4)


def test_labels_warp_nearest_and_keep_their_dtype(rectified):
    render_w, render_h = rectified.render_size
    ids = np.zeros((render_h, render_w), dtype=np.int32)
    ids[: render_h // 2] = 7
    ids[render_h // 2 :] = 300_000
    warped = warp_label(rectified, ids)
    assert warped.dtype == np.int32 and warped.shape == (SIZE[1], SIZE[0])
    assert set(np.unique(warped).tolist()) <= {0, 7, 300_000}
    assert (warped[~rectified.valid] == 0).all()


def test_single_channel_maps_keep_their_shape(distorted):
    render_w, render_h = distorted.render_size
    depth = np.full((render_h, render_w, 1), 2.5, dtype=np.float32)
    warped = warp_label(distorted, depth)
    assert warped.shape == (SIZE[1], SIZE[0], 1)
    assert np.allclose(warped, 2.5)


def test_colour_warp_is_the_distortion_remap(distorted):
    render_w, render_h = distorted.render_size
    gradient = np.tile(np.linspace(0, 255, render_w, dtype=np.float32), (render_h, 1))
    image = cv2.merge([gradient, gradient, gradient]).astype(np.uint8)
    warped = warp_image(distorted, image)
    assert warped.shape == (SIZE[1], SIZE[0], 3) and warped.dtype == np.uint8
