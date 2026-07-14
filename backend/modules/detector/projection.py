"""base-투영 수학 — 옛 backend DetectorNode `_handle_grounded_detect` 포팅.

결정적 (모델/하드웨어 무관): bbox + depth + intrinsic + TCP pose + hand_eye →
base frame 3D 좌표. 순수 numpy — 회사에서 단위테스트로 검증 가능한 핵심.

좌표 규약 (옛 detector 와 동일):
  R_be, t_be : end-effector → base (TCP pose, MOTION TCP_SNAPSHOT 에서)
  R_ce, t_ce : camera → end-effector (hand_eye 캘)
  obj_base = R_be · (R_ce · obj_cam + t_ce) + t_be

여기는 결정적 좌표 변환만 (pixel+depth ↔ base). 물체 기하(위치/height/OBB)는
전부 geometry.py 가 **물체 자기 점군**(base_points_from_mask 출력)에서 산출 —
주변 바닥(ring floor) 추정은 폐기됐다 (grasp_redesign_journey.md §5.1,
object-centric: 책상 없어도 성립, 추측이 아니라 관측).
"""

from __future__ import annotations

import numpy as np


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

