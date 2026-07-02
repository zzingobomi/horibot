"""Capture 이미지 처리 — ChArUco detect + solvePnP + tilt/reproj (순수 함수).

module 밖 순수 로직 (camera/DB 무관) — 렌더된 sim board 로 단위 검증 가능.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from . import board as calib_board
from .se3 import make_T


@dataclass
class CaptureDetection:
    board_in_cam: list[list[float]]  # 4x4
    corners_2d: list[list[float]]  # (N,2)
    corner_ids: list[int]  # (N,)
    reproj_rms_px: float
    tilt_deg: float


def detect_and_pnp(
    frame_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> CaptureDetection | None:
    """ChArUco 검출 + PnP. 미검출/실패면 None.

    tilt = board normal(cam frame) vs 카메라 광축 각 (0°=정면, 90°=edge-on).
    reproj_rms = solvePnP 재투영 RMS (품질 gate 입력).
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    ok, ch_corners, ch_ids = calib_board.detect(gray)
    if not ok or ch_corners is None or ch_ids is None:
        return None

    obj_pts, img_pts = calib_board.match_object_points(ch_corners, ch_ids)
    if obj_pts is None or img_pts is None or len(obj_pts) < 4:
        return None

    ok2, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok2:
        return None

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
    err = proj.reshape(-1, 2) - img_pts.reshape(-1, 2)
    reproj_rms = float(np.sqrt(np.mean(np.sum(err**2, axis=1))))

    R, _ = cv2.Rodrigues(rvec)
    board_in_cam = make_T(R, tvec)
    # board z-axis(normal) 의 cam-z 성분 = R[2,2]. 정면이면 |R[2,2]|=1 → tilt 0.
    tilt_deg = float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), 0.0, 1.0))))

    return CaptureDetection(
        board_in_cam=board_in_cam.tolist(),
        corners_2d=ch_corners.reshape(-1, 2).astype(float).tolist(),
        corner_ids=ch_ids.reshape(-1).astype(int).tolist(),
        reproj_rms_px=reproj_rms,
        tilt_deg=tilt_deg,
    )
