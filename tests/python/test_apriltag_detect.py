from __future__ import annotations

import cv2
import numpy as np

from auto_battlebot.calibration.apriltag import apriltag_detect as ad

COLS, ROWS, MARKER_M, SEP_M, FIRST_ID = 3, 5, 0.065, 0.015, 160
K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)


def _render(board: cv2.aruco.GridBoard, r_fc: np.ndarray, t_fc: np.ndarray) -> np.ndarray:
    px_per_m = 4000
    w = int((COLS * MARKER_M + (COLS - 1) * SEP_M) * px_per_m)
    h = int((ROWS * MARKER_M + (ROWS - 1) * SEP_M) * px_per_m)
    margin = 40
    board_img = board.generateImage((w + 2 * margin, h + 2 * margin), marginSize=margin)
    src = np.array(
        [[margin, margin], [margin + w, margin], [margin + w, margin + h], [margin, margin + h]],
        dtype=np.float32,
    )
    corners_f = np.array(
        [[0, 0, 0], [w / px_per_m, 0, 0], [w / px_per_m, h / px_per_m, 0], [0, h / px_per_m, 0]]
    )
    rvec, _ = cv2.Rodrigues(r_fc)
    dst, _ = cv2.projectPoints(corners_f, rvec, t_fc, K, D)
    homography = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(board_img, homography, (1280, 720), borderValue=255)


def test_floor_board_ids_match_physical_layout() -> None:
    board = ad.make_floor_board(COLS, ROWS, MARKER_M, SEP_M, FIRST_ID)
    ids = np.asarray(board.getIds()).reshape(ROWS, COLS)
    assert ids[0, 0] == FIRST_ID + COLS - 1, "top-left tag"
    assert ids[0, -1] == FIRST_ID, "top-right tag"
    assert ids[-1, 0] == FIRST_ID + COLS * ROWS - 1, "bottom-left tag"


def test_floor_extrinsic_recovers_pose_of_rendered_board() -> None:
    board = ad.make_floor_board(COLS, ROWS, MARKER_M, SEP_M, FIRST_ID)
    rvec_true = np.array([0.15, 0.1, 0.05])
    r_true, _ = cv2.Rodrigues(rvec_true)
    t_true = np.array([-0.1, -0.15, 0.6])
    frame = cv2.cvtColor(_render(board, r_true, t_true), cv2.COLOR_GRAY2BGR)
    pose = ad.solve_floor_extrinsic([frame], ad.make_detector(), board, K, D)
    assert np.allclose(pose.r_fc, r_true, atol=0.05)
    assert np.allclose(pose.t_fc, t_true, atol=0.03)
