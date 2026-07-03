"""base-투영 수학 — 옛 backend DetectorNode `_handle_grounded_detect` 포팅.

결정적 (모델/하드웨어 무관): bbox + depth + intrinsic + TCP pose + hand_eye →
base frame 3D 좌표. 순수 numpy — 회사에서 단위테스트로 검증 가능한 핵심.

좌표 규약 (옛 detector 와 동일):
  R_be, t_be : end-effector → base (TCP pose, MOTION TCP_SNAPSHOT 에서)
  R_ce, t_ce : camera → end-effector (hand_eye 캘)
  obj_base = R_be · (R_ce · obj_cam + t_ce) + t_be

Day-1 = position only. height/base_z/Top-K/geometric prior 는 §5.2 개선(실제 task 요구 시).
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
