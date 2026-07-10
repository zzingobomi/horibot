"""base-frame 형상 계산 — base 점군 → OBB(footprint + grasp yaw). 순수 numpy/cv2.

projection.base_points_from_mask 가 만든 물체 base 점군을 base XY 평면에 투영해
cv2.minAreaRect 로 회전 사각형을 구한다. **base frame 에서 직접 계산** — 픽셀
minAreaRect 의 원근 왜곡이 없다 (depth 로 이미 base 3D 를 알기 때문). 책임 분리:
detector=모델(mask), projection=좌표변환(base 점군), geometry=형상(OBB) — 여기.

yaw 규약: 긴 변 벡터의 base X 기준 각도 [-π/2, π/2). cv2 버전마다 다른 minAreaRect
angle 필드 대신 boxPoints 의 실제 코너로 긴 변을 뽑아 모호성 제거 (사각형 180° 대칭
→ wrap). footprint = (긴 변, 짧은 변) m. 결정적 — 회사 단위테스트 검증 가능.

overlay 보조(obb_corners / mask_contour)도 여기 — 형상의 2D/3D 표현. mask_contour 는
image-space (SAM mask 윤곽, 카메라 패널 오버레이 전용, base 아님).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


# 윗면 band 선택 규약 — projection.object_top_center_base 와 동일 (base_z 상위
# percentile 기준 band). OBB 는 파지 접근면(윗면) 단면이 의미: mask 픽셀 전부를 쓰면
# 비스듬한 시점에서 옆면 + mask 경계의 배경(테이블) bleed + boundary depth 노이즈가
# base XY 에 깔려 footprint 를 부풀리고 yaw 를 비튼다 (2026-07-09 실물 확인 — 근접
# 사선 샷 OBB 크게 skew / 탑다운 샷 정상 = 윗면 아래 오염 증거).
_TOP_PERCENTILE = 25.0
_TOP_BAND_M = 0.010


def top_face_points(
    pts_base: np.ndarray | None,
    band_m: float = _TOP_BAND_M,
    percentile: float = _TOP_PERCENTILE,
) -> np.ndarray | None:
    """base 점군 → 윗면 band 점만 (z 상위 percentile 기준 band_m 아래까지).

    (N,3) 전제 — z 열 없으면(2열) 그대로 통과. 필터 후 빈 결과면 None.
    """
    if pts_base is None or pts_base.ndim != 2 or pts_base.shape[1] < 3:
        return pts_base
    z = pts_base[:, 2]
    top_ref = float(np.percentile(z, 100.0 - percentile))
    top = pts_base[z >= top_ref - band_m]
    return top if len(top) else None


@dataclass(frozen=True, slots=True)
class Obb:
    """base XY 회전 사각형. center_xy: base (x,y) m. footprint: (long, short) m.
    yaw_rad: 긴 변의 base X 기준 각도 [-π/2, π/2) — grasp yaw (base Z 회전)."""

    center_xy: tuple[float, float]
    footprint: tuple[float, float]
    yaw_rad: float


def obb_from_base_points(pts_base: np.ndarray | None) -> Obb | None:
    """(N,2|3) base 점 → base XY OBB. 점 3개 미만이면 None (축퇴).

    pts_base 는 base frame 좌표 (m) — projection.base_points_from_mask 출력. Z 는
    무시하고 XY 만 사용 (footprint 는 바닥 투영). grasp yaw = base Z 회전.
    """
    if pts_base is None or len(pts_base) < 3:
        return None
    xy = np.ascontiguousarray(pts_base[:, :2], dtype=np.float32)
    (cx, cy), (w, h), angle = cv2.minAreaRect(xy)
    # boxPoints 실측 코너 → 긴 변 벡터의 atan2. cv2 버전별 angle 필드 의미 차 회피.
    box = cv2.boxPoints(((cx, cy), (w, h), angle))  # (4,2), 순서대로 인접
    edges = box[[1, 2, 3, 0]] - box  # 각 변 벡터
    lengths = np.hypot(edges[:, 0], edges[:, 1])
    long_edge = edges[int(np.argmax(lengths))]
    yaw = math.atan2(float(long_edge[1]), float(long_edge[0]))
    # 사각형은 180° 대칭 → [-π/2, π/2) 로 wrap (긴 변 방향만 의미).
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    long_side, short_side = (float(w), float(h)) if w >= h else (float(h), float(w))
    return Obb(
        center_xy=(float(cx), float(cy)),
        footprint=(long_side, short_side),
        yaw_rad=float(yaw),
    )


def obb_corners(obb: Obb, z: float) -> np.ndarray:
    """OBB 를 평면 z 위 base 3D 코너 4개 (4,3) 로. 오버레이 reproject 입력.

    코너 순서 = 인접 (사각형 그리기용). center + R(yaw)·(±L/2, ±S/2). z 는 그릴 평면
    (보통 물체 윗면 중심 z) — footprint 를 그 높이에 놓고 카메라로 reproject.
    """
    (cx, cy) = obb.center_xy
    long_side, short_side = obb.footprint
    hl, hs = long_side / 2.0, short_side / 2.0
    local = np.array([[hl, hs], [hl, -hs], [-hl, -hs], [-hl, hs]])
    yaw = obb.yaw_rad
    rot = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    xy = local @ rot.T + np.array([cx, cy])
    return np.column_stack([xy, np.full(4, z)])


def mask_contour(mask: np.ndarray) -> np.ndarray | None:
    """SAM mask → 최대 외곽 윤곽 폴리곤 (M,2) px. 없으면 None. image-space (오버레이).

    approxPolyDP 로 단순화 — bitmap 통째가 아니라 점 수십 개만 wire 에 실어 카메라
    패널이 실루엣을 그린다 (mask 자체는 wire 에 안 나감, backend_v2.md 결정).
    """
    m = mask.astype(np.uint8)
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    eps = 0.01 * cv2.arcLength(cnt, True)
    poly = cv2.approxPolyDP(cnt, eps, True).reshape(-1, 2)
    return poly.astype(float)
