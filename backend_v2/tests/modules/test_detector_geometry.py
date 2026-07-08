"""geometry.obb_from_base_points 단위테스트 (결정적, cv2 만).

의미 있는 검증: 알려진 yaw/크기로 만든 회전 사각형 base 점군에서 OBB(yaw, footprint)를
복원. minAreaRect angle 필드가 아니라 boxPoints 긴 변으로 yaw 를 뽑는 규약이 회전을
정확히 되살리는지 — 규약을 되돌리면(예: 짧은 변 사용) 즉시 깨짐.
"""

from __future__ import annotations

import math

import numpy as np

from modules.detector.geometry import (
    Obb,
    mask_contour,
    obb_corners,
    obb_from_base_points,
)


def _rect_points(
    cx: float, cy: float, long: float, short: float, yaw: float, n: int = 40
) -> np.ndarray:
    """(cx,cy) 중심, 긴 변=long(local X)/짧은 변=short(local Y), yaw 회전한 격자 점군."""
    xs = np.linspace(-long / 2, long / 2, n)
    ys = np.linspace(-short / 2, short / 2, n)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
    rot = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    return pts @ rot.T + np.array([cx, cy])


def test_obb_recovers_axis_aligned():
    obb = obb_from_base_points(_rect_points(0.5, -0.2, 0.10, 0.04, 0.0))
    assert obb is not None
    assert abs(obb.center_xy[0] - 0.5) < 2e-3, obb.center_xy
    assert abs(obb.center_xy[1] + 0.2) < 2e-3, obb.center_xy
    assert abs(obb.footprint[0] - 0.10) < 3e-3, obb.footprint
    assert abs(obb.footprint[1] - 0.04) < 3e-3, obb.footprint
    assert abs(obb.yaw_rad) < math.radians(2), obb.yaw_rad


def test_obb_recovers_rotation():
    yaw = math.radians(30)
    obb = obb_from_base_points(_rect_points(0.0, 0.0, 0.12, 0.05, yaw))
    assert obb is not None
    # footprint 는 (긴 변, 짧은 변) — 회전 무관하게 실제 변 길이 복원
    assert abs(obb.footprint[0] - 0.12) < 3e-3, obb.footprint
    assert abs(obb.footprint[1] - 0.05) < 3e-3, obb.footprint
    assert abs(obb.yaw_rad - yaw) < math.radians(2), obb.yaw_rad


def test_obb_yaw_wraps_into_symmetric_range():
    # 80° 사각형 → 사각형 180° 대칭이라 [-π/2, π/2) 로 wrap.
    obb = obb_from_base_points(_rect_points(0.0, 0.0, 0.12, 0.05, math.radians(80)))
    assert obb is not None
    assert -math.pi / 2 <= obb.yaw_rad < math.pi / 2, obb.yaw_rad
    assert abs(obb.yaw_rad - math.radians(80)) < math.radians(2), obb.yaw_rad


def test_obb_too_few_points_returns_none():
    assert obb_from_base_points(None) is None
    assert obb_from_base_points(np.zeros((2, 2))) is None


def test_obb_corners_axis_aligned():
    # yaw=0, 중심 (1,2), long=0.2(x)/short=0.1(y), 평면 z=0.5.
    obb = Obb(center_xy=(1.0, 2.0), footprint=(0.2, 0.1), yaw_rad=0.0)
    c = obb_corners(obb, z=0.5)
    assert c.shape == (4, 3)
    assert np.allclose(c[:, 2], 0.5)  # 전부 평면 z
    # x 범위 = 1±0.1, y 범위 = 2±0.05
    assert abs(c[:, 0].min() - 0.9) < 1e-9 and abs(c[:, 0].max() - 1.1) < 1e-9
    assert abs(c[:, 1].min() - 1.95) < 1e-9 and abs(c[:, 1].max() - 2.05) < 1e-9


def test_obb_corners_rotated_90_swaps_extent():
    # yaw=90° → 긴 변(x)이 y 로. x 범위=±short/2, y 범위=±long/2.
    obb = Obb(center_xy=(0.0, 0.0), footprint=(0.2, 0.1), yaw_rad=math.pi / 2)
    c = obb_corners(obb, z=0.0)
    assert abs(c[:, 0].max() - 0.05) < 1e-9, c  # short/2
    assert abs(c[:, 1].max() - 0.1) < 1e-9, c  # long/2


def test_mask_contour_rectangle():
    mask = np.zeros((100, 100), dtype=bool)
    mask[20:60, 30:80] = True  # 40×50 사각형
    poly = mask_contour(mask)
    assert poly is not None and len(poly) == 4, poly  # 사각형 → 코너 4
    xs, ys = poly[:, 0], poly[:, 1]
    assert 29 <= xs.min() <= 31 and 78 <= xs.max() <= 80, poly
    assert 19 <= ys.min() <= 21 and 58 <= ys.max() <= 60, poly


def test_mask_contour_empty_none():
    assert mask_contour(np.zeros((10, 10), dtype=bool)) is None
