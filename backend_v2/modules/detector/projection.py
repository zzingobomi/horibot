"""base-투영 수학 — 옛 backend DetectorNode `_handle_grounded_detect` 포팅.

결정적 (모델/하드웨어 무관): bbox + depth + intrinsic + TCP pose + hand_eye →
base frame 3D 좌표. 순수 numpy — 회사에서 단위테스트로 검증 가능한 핵심.

좌표 규약 (옛 detector 와 동일):
  R_be, t_be : end-effector → base (TCP pose, MOTION TCP_SNAPSHOT 에서)
  R_ce, t_ce : camera → end-effector (hand_eye 캘)
  obj_base = R_be · (R_ce · obj_cam + t_ce) + t_be

position(base_z 포함) + size_m 을 후보별 산출 (§17.5 기하 prior 입력). prior 적용/선택은
소비자(task SelectTarget) — 여기는 결정적 기하 변환만.
"""

from __future__ import annotations

import numpy as np

# bbox 영역 depth 에서 "객체 윗면"(카메라에 가까운 쪽) 대표값을 뽑는 percentile.
# 옛 detector 와 동일 — 25 = 상위 25% 가까운 픽셀 (평면/노이즈보다 물체 top).
_TOP_PERCENTILE = 25.0


def z_cam_from_depth_bbox(
    depth_z16: np.ndarray,
    bbox: tuple[float, float, float, float],
    depth_scale: float,
    percentile: float = _TOP_PERCENTILE,
) -> float | None:
    """bbox ROI 의 valid depth 로 객체 윗면 Z_cam(m) 추정. valid 없으면 None.

    depth_z16: (H, W) uint16 raw depth. bbox: (x1, y1, x2, y2) px.
    """
    x1, y1, x2, y2 = bbox
    h, w = depth_z16.shape
    ix1, iy1 = max(0, int(round(x1))), max(0, int(round(y1)))
    ix2, iy2 = min(w, int(round(x2))), min(h, int(round(y2)))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    roi = depth_z16[iy1:iy2, ix1:ix2]
    valid = roi[roi > 0]
    if valid.size == 0:
        return None
    return float(np.percentile(valid, percentile)) * depth_scale


def unproject_to_base(
    u: float,
    v: float,
    z_cam: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    r_be: np.ndarray,  # (3,3) ee → base
    t_be: np.ndarray,  # (3,) ee → base
    r_ce: np.ndarray,  # (3,3) cam → ee (hand_eye)
    t_ce: np.ndarray,  # (3,) cam → ee (hand_eye)
) -> np.ndarray:
    """pixel (u,v) + Z_cam → base frame 3D 위치 (3,). 옛 detector 포팅."""
    obj_cam = np.array(
        [(u - cx) / fx * z_cam, (v - cy) / fy * z_cam, z_cam], dtype=float
    )
    obj_ee = r_ce @ obj_cam + t_ce
    obj_base = r_be @ obj_ee + t_be
    return obj_base


def base_points_from_mask(
    depth_z16: np.ndarray,
    mask: np.ndarray,
    depth_scale: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
) -> np.ndarray | None:
    """mask 픽셀 중 valid depth 를 전부 base frame 3D 로 unproject → (N, 3). 없으면 None.

    SAM mask(물체 픽셀) + aligned depth → base 점군. geometry.obb_from_base_points 의
    입력. bbox ROI 가 아니라 물체 픽셀만 골라 배경/책상을 애초에 배제 → OBB 안정.
    좌표 규약은 unproject_to_base 와 동일 (cam → ee → base). 순수 numpy — 결정적.

    mask 는 depth 와 같은 (H, W) (D405 color-aligned depth 전제 — module 이 같은
    intrinsic 을 depth 에 적용하는 것과 일관).
    """
    m = mask & (depth_z16 > 0)
    vs, us = np.nonzero(m)
    if us.size == 0:
        return None
    z = depth_z16[vs, us].astype(np.float64) * depth_scale
    x = (us.astype(np.float64) - cx) / fx * z
    y = (vs.astype(np.float64) - cy) / fy * z
    pts_cam = np.stack([x, y, z], axis=1)  # (N, 3)
    return (pts_cam @ r_ce.T + t_ce) @ r_be.T + t_be


def project_base_to_pixel(
    pts_base: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
) -> np.ndarray:
    """base frame 3D 점 (N,3) → color 이미지 픽셀 (N,2). unproject_to_base 의 역.

    오버레이 전용 — base OBB 코너를 카메라 이미지에 다시 그리기 위한 reproject.
    forward(unproject): base = (cam @ r_ce.T + t_ce) @ r_be.T + t_be. 역:
      ee  = (base - t_be) @ r_be
      cam = (ee   - t_ce) @ r_ce
      u = fx·camx/camz + cx,  v = fy·camy/camz + cy
    카메라 앞(camz>0) 가정 (검출된 물체). 순수 numpy — 결정적.
    """
    pts = np.asarray(pts_base, dtype=float)
    ee = (pts - t_be) @ r_be
    cam = (ee - t_ce) @ r_ce
    z = cam[:, 2]
    u = fx * cam[:, 0] / z + cx
    v = fy * cam[:, 1] / z + cy
    return np.stack([u, v], axis=1)


def object_top_center_base(
    depth_z16: np.ndarray,
    bbox: tuple[float, float, float, float],
    depth_scale: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
    top_band_m: float = 0.010,
    percentile: float = _TOP_PERCENTILE,
) -> np.ndarray | None:
    """bbox 물체 '윗면'의 base-frame 중심 (x, y, z). valid depth 없으면 None.

    옛 방식(bbox 중심 픽셀 + 윗면 depth 로 unproject)의 systematic 편향 fix:
    비스듬한 카메라에선 bbox 중심 픽셀이 윗면 중심이 아니라 실루엣(윗면+옆면) 중심 →
    윗면 depth 로 unproject 하면 파지 x/y 가 카메라 쪽 모서리로 밀린다. 여기서는
    **bbox 픽셀을 각자의 depth 로 base 에 unproject 한 뒤 윗면 band 만 골라 실제
    3D centroid** 를 취해 픽셀·depth 불일치를 없앤다 → 큐브 윗면 중심(=바닥 중심 x/y).

    윗면 = base_z 상위 (percentile 로 top_z 추정 후 top_band_m 아래까지). 픽셀·depth
    일관 → 편향 없음. 순수 numpy — 결정적.
    """
    h, w = depth_z16.shape
    ix1, iy1 = max(0, int(round(bbox[0]))), max(0, int(round(bbox[1])))
    ix2, iy2 = min(w, int(round(bbox[2]))), min(h, int(round(bbox[3])))
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    roi = depth_z16[iy1:iy2, ix1:ix2]
    vs_local, us_local = np.nonzero(roi)
    if us_local.size == 0:
        return None
    us = us_local.astype(np.float64) + ix1
    vs = vs_local.astype(np.float64) + iy1
    z = roi[vs_local, us_local].astype(np.float64) * depth_scale
    x = (us - cx) / fx * z
    y = (vs - cy) / fy * z
    pts_base = (np.stack([x, y, z], axis=1) @ r_ce.T + t_ce) @ r_be.T + t_be
    # 윗면 = base_z 상위 percentile 기준 band (카메라 향한 top face). floor/노이즈 배제.
    top_ref = float(np.percentile(pts_base[:, 2], 100.0 - percentile))
    top = pts_base[pts_base[:, 2] >= top_ref - top_band_m]
    if top.size == 0:
        return None
    return np.array([top[:, 0].mean(), top[:, 1].mean(), top[:, 2].mean()], dtype=float)


def floor_z_and_height(
    depth_z16: np.ndarray,
    bbox: tuple[float, float, float, float],
    depth_scale: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
    obj_top_base_z: float,
    percentile: float = _TOP_PERCENTILE,
) -> tuple[float, float]:
    """bbox 외곽 ring 의 base-z 로 책상 floor_z + 물체 height 추정 (§17.5, 옛 detector 포팅).

    ring = bbox 를 pad 만큼 확장한 테두리 (bbox 내부 제외). 그 valid depth 를 전부 base
    로 unproject → z percentile = floor_z (주변 책상). height = obj_top_base_z - floor_z.
    ring valid 없으면 (floor_z=obj_top_base_z, height=0). 순수 numpy — 결정적.
    """
    h, w = depth_z16.shape
    ix1 = max(0, int(round(bbox[0])))
    iy1 = max(0, int(round(bbox[1])))
    ix2 = min(w, int(round(bbox[2])))
    iy2 = min(h, int(round(bbox[3])))
    bbox_w, bbox_h = ix2 - ix1, iy2 - iy1
    if bbox_w <= 0 or bbox_h <= 0:
        return obj_top_base_z, 0.0
    pad = max(15, min(80, int(min(bbox_w, bbox_h) * 0.5)))
    ex1, ey1 = max(0, ix1 - pad), max(0, iy1 - pad)
    ex2, ey2 = min(w, ix2 + pad), min(h, iy2 + pad)
    ext = depth_z16[ey1:ey2, ex1:ex2].copy()
    ext[(iy1 - ey1) : (iy2 - ey1), (ix1 - ex1) : (ix2 - ex1)] = 0  # bbox 내부 제외
    vs_local, us_local = np.nonzero(ext)
    if us_local.size == 0:
        return obj_top_base_z, 0.0
    us = us_local.astype(np.float64) + ex1
    vs = vs_local.astype(np.float64) + ey1
    z = ext[vs_local, us_local].astype(np.float64) * depth_scale
    x = (us - cx) / fx * z
    y = (vs - cy) / fy * z
    pts_cam = np.stack([x, y, z], axis=1)  # (N, 3)
    pts_base = (pts_cam @ r_ce.T + t_ce) @ r_be.T + t_be
    floor_z = float(np.percentile(pts_base[:, 2], percentile))
    height = max(0.0, obj_top_base_z - floor_z)
    return floor_z, height
