"""base-투영 수학 — 옛 backend DetectorNode `_handle_grounded_detect` 포팅.

결정적 (모델/하드웨어 무관): bbox + depth + intrinsic + TCP pose + hand_eye →
base frame 3D 좌표. 순수 numpy — 회사에서 단위테스트로 검증 가능한 핵심.

좌표 규약 (옛 detector 와 동일):
  R_be, t_be : end-effector → base (TCP pose, MOTION TCP_SNAPSHOT 에서)
  R_ce, t_ce : camera → end-effector (hand_eye 캘)
  obj_base = R_be · (R_ce · obj_cam + t_ce) + t_be

여기는 결정적 좌표 변환만 (pixel+depth ↔ base). 물체 기하(위치/height/OBB)는
전부 geometry.py 가 **물체 자기 점군**(base_points_from_mask 출력)에서 산출 —
주변 바닥(ring floor) 추정은 폐기됐다 (grasping.md §1,
object-centric: 책상 없어도 성립, 추측이 아니라 관측).
"""

from __future__ import annotations

import cv2
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


def camera_pose_in_base(
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """카메라 optical frame 의 base pose — (R_bc (3,3), C (3,)).

    base←cam 회전과 카메라 원점: R_bc = R_be·R_ce, C = R_be·t_ce + t_be.
    (unproject_to_base 의 합성 회전을 이름 붙여 노출 — plane 역투영/observe 포즈
    계산 공용.)
    """
    r_bc = r_be @ r_ce
    c = r_be @ np.asarray(t_ce, dtype=float).reshape(3) + np.asarray(
        t_be, dtype=float
    ).reshape(3)
    return r_bc, c


def plane_points_from_pixels(
    us: np.ndarray,
    vs: np.ndarray,
    plane_z: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: np.ndarray | None,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
) -> np.ndarray | None:
    """pixel 들 → 카메라 ray ∩ (base z=plane_z) 평면 교점 (N,3). 없으면 None.

    **mono(depth 없음) 검출의 핵심 수학** (omx 웹캠 — docs/omx_handover_prep.md
    §5.3): unproject_to_base 의 z_cam 을 "ray 와 테이블 평면의 교점 파라미터"로
    대체한다. 물체가 그 평면 위에 놓여 있다는 전제가 계약 (테이블 위 얇은 물체).

    **dist_coeffs undistort 선행 필수** — omx 웹캠은 barrel distortion 이 큼
    (k1≈−0.48): 순진한 pinhole 로 pixel→ray 를 만들면 유효 화각이 94°→62° 로
    축소되고 엣지 물체 위치가 틀어진다 (§5.3). None/전부 0 이면 pinhole.

    필터 (침묵 오염 방지): ray 가 평면과 카메라 **앞쪽**에서 만나는 픽셀만
    (t>0). 카메라가 평면 아래에 있거나 ray 가 위를 보면 그 픽셀은 버려진다 —
    전부 버려지면 None (호출자가 명시 실패로).
    순수 numpy/cv2 — 결정적, 오피스 단위테스트 대상.
    """
    us = np.asarray(us, dtype=np.float64)
    vs = np.asarray(vs, dtype=np.float64)
    if us.size == 0:
        return None
    if dist_coeffs is not None and np.any(np.asarray(dist_coeffs) != 0.0):
        k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        pts = np.stack([us, vs], axis=1).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, k, np.asarray(dist_coeffs, dtype=np.float64))
        xn, yn = norm[:, 0, 0], norm[:, 0, 1]
    else:
        xn = (us - cx) / fx
        yn = (vs - cy) / fy
    dirs_cam = np.stack([xn, yn, np.ones_like(xn)], axis=1)  # (N,3) optical
    r_bc, c = camera_pose_in_base(r_be, t_be, r_ce, t_ce)
    dirs_base = dirs_cam @ r_bc.T
    denom = dirs_base[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (plane_z - c[2]) / denom
    valid = np.isfinite(t) & (t > 1e-6)
    if not np.any(valid):
        return None
    return c + t[valid, None] * dirs_base[valid]


def project_base_to_pixel_distorted(
    pts_base: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: np.ndarray | None,
    r_be: np.ndarray,
    t_be: np.ndarray,
    r_ce: np.ndarray,
    t_ce: np.ndarray,
) -> np.ndarray:
    """base 3D 점 (N,3) → **왜곡 반영** 픽셀 (N,2) — plane 역투영의 역 (오버레이).

    project_base_to_pixel 은 pinhole 전용 — 광각 웹캠(omx) 이미지에 그대로 그리면
    엣지에서 오버레이가 어긋난다. cv2.projectPoints 로 dist_coeffs 를 태운다.
    """
    pts = np.asarray(pts_base, dtype=np.float64)
    ee = (pts - np.asarray(t_be, dtype=float).reshape(3)) @ np.asarray(r_be)
    cam = (ee - np.asarray(t_ce, dtype=float).reshape(3)) @ np.asarray(r_ce)
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = (
        np.asarray(dist_coeffs, dtype=np.float64)
        if dist_coeffs is not None
        else np.zeros(5)
    )
    px, _ = cv2.projectPoints(
        cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, dist
    )
    return px.reshape(-1, 2)


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

