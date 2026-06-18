"""ChArUco eye-in-hand 시뮬레이션 — headless 캘리브레이션 e2e 용.

mock 카메라가 *로봇 joint 상태로부터* board_in_cam 포즈를 계산해 ChArUco 보드를
렌더 → FrameCache → board.detect → solvePnP → BA 전체 파이프라인을 브라우저/실
하드웨어 없이 e2e 검증 (docs/handeye_ux_solver_v3_plan.md §8 테스트 전략).

eye-in-hand 모델:
  T_cam_base   = FK(joints) @ X_cam2gripper        (카메라 = EE 에 장착)
  board_in_cam = inv(T_cam_base) @ T_board_base    (보드 = base 에 고정)
로봇이 움직이면 board_in_cam 이 일관되게 변함 → BA 가 X 를 복원 가능.

본 모듈은 *시뮬레이션 전용* — 실 캘 코드 경로(board.py / hand_eye.py)는 안 건드림.
mock 카메라만 본 모듈을 import (CALIB_SIM_BOARD env 켜졌을 때).
"""

from __future__ import annotations

import cv2
import numpy as np

from . import board as calib_board
from .se3 import make_T

# 시뮬 ground-truth — 실 SO-101 캘 데이터(2026-06-18)의 BA 결과에서 도출한 값.
# 이 X + board 를 쓰면 FK-sim 이 실제 캡처 자세에서 보드를 화면에 재현 → e2e 가
# self-contained (npz 의존 X). 실 setup 과 일치하는 X/board 라 BA 가 잘 수렴.
SIM_X_ROD: tuple[float, float, float] = (0.82991, 0.79699, 1.43307)
SIM_X_T: tuple[float, float, float] = (-0.04465, -0.00629, -0.06977)
SIM_BOARD_ROD: tuple[float, float, float] = (-1.18429, 1.40124, -1.24038)
SIM_BOARD_T: tuple[float, float, float] = (0.40298, 0.03074, 0.25535)

# 실 SO-101 캘 자세 (joint degree) — FK-sim 에서 보드가 시야에 들어오는 검증된
# 자세들. e2e 가 MoveJ target 으로 사용 (self-contained, npz 불필요).
SIM_CALIB_POSES_DEG: list[list[float]] = [
    [4.04, 41.76, -78.95, 46.42, -6.95, 95.91],
    [22.95, 41.76, -78.95, 46.42, -6.95, 95.91],
    [29.71, 47.12, -82.73, 52.75, -26.81, 68.31],
    [16.53, 35.69, -64.18, 35.87, -2.64, 120.97],
    [16.53, 66.02, -117.89, 70.15, -2.64, 120.97],
    [16.53, 44.4, -64.0, 21.54, -4.22, 114.11],
    [35.43, 55.12, -64.0, 19.43, -46.51, 119.21],
    [35.43, 53.98, -50.29, -23.82, -46.68, 52.57],
    [35.43, 60.92, -82.11, 32.53, -46.59, 52.57],
]


def sim_X_cam2gripper() -> np.ndarray:
    """시뮬 ground-truth hand-eye (4x4)."""
    return make_T(cv2.Rodrigues(np.array(SIM_X_ROD))[0], np.array(SIM_X_T))


def sim_T_board_base() -> np.ndarray:
    """시뮬 ground-truth 보드 base 포즈 (4x4)."""
    return make_T(cv2.Rodrigues(np.array(SIM_BOARD_ROD))[0], np.array(SIM_BOARD_T))


def board_in_cam_from_fk(T_gripper_base: np.ndarray) -> np.ndarray:
    """FK(joints)=T_gripper_base → board_in_cam (4x4). eye-in-hand 모델."""
    T_cam_base = T_gripper_base @ sim_X_cam2gripper()
    return np.linalg.inv(T_cam_base) @ sim_T_board_base()


# 프론탈 보드 이미지 캐시 (재생성 비쌈).
_FRONTAL: np.ndarray | None = None
_FRONTAL_PX = (700, 500)  # 7:5 비율 (SQUARES_X:SQUARES_Y)


def _frontal_board() -> np.ndarray:
    global _FRONTAL
    if _FRONTAL is None:
        img = calib_board.board().generateImage(_FRONTAL_PX, marginSize=0, borderBits=1)
        _FRONTAL = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return _FRONTAL


def render_charuco_at_pose(
    *,
    width: int,
    height: int,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    board_in_cam: np.ndarray,
    background: tuple[int, int, int] = (60, 55, 50),
) -> np.ndarray:
    """board_in_cam 포즈의 ChArUco 보드를 카메라 이미지에 렌더 (BGR).

    프론탈 보드 이미지를 board 3D 코너의 투영 위치로 perspective warp.
    보드가 카메라 뒤 / 화면 밖이면 background 만 반환.
    """
    out = np.empty((height, width, 3), dtype=np.uint8)
    out[:] = background

    corners3d = calib_board.board_corner_points_3d()  # (4,3) TL,TR,BR,BL (board frame)
    rvec, _ = cv2.Rodrigues(board_in_cam[:3, :3])
    tvec = board_in_cam[:3, 3].reshape(3, 1)

    # 카메라 앞 (z>0) 인지 — 모든 코너가 카메라 뒤면 안 보임.
    cam_pts = (board_in_cam[:3, :3] @ corners3d.T).T + board_in_cam[:3, 3]
    if np.all(cam_pts[:, 2] <= 0.02):
        return out

    proj, _ = cv2.projectPoints(corners3d, rvec, tvec, camera_matrix, dist_coeffs)
    dst = proj.reshape(-1, 2).astype(np.float32)
    if not np.all(np.isfinite(dst)):
        return out

    fw, fh = _FRONTAL_PX
    src = np.array(
        [[0, 0], [fw - 1, 0], [fw - 1, fh - 1], [0, fh - 1]], dtype=np.float32
    )
    H = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(_frontal_board(), H, (width, height))
    mask = cv2.warpPerspective(
        np.full((fh, fw), 255, dtype=np.uint8), H, (width, height)
    )
    out[mask > 0] = warped[mask > 0]
    return out
