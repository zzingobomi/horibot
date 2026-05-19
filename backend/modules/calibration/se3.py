"""SE(3) 매니폴드 헬퍼.

번들 조정용으로 SE(3) (4x4 동차 변환) ↔ 6-vec (rodrigues 3 + translation 3)
변환과 log/exp 매핑을 제공.

scipy.spatial.transform.Rotation을 회전 부분의 검증된 backend로 사용.
SE(3)의 정확한 log/exp는 회전 axis-angle 크기가 작을 때 numerical 불안정이 있어
small-angle 근사로 분기 처리.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """3x3 R + 3-vec t → 4x4 homogeneous."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def split_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """4x4 T → (R 3x3, t 3-vec)."""
    return T[:3, :3].copy(), T[:3, 3].copy()


def invert_T(T: np.ndarray) -> np.ndarray:
    """4x4 T의 역 (직교 회전 활용)."""
    R, t = split_T(T)
    Tinv = np.eye(4)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def vec_to_T(xi: np.ndarray) -> np.ndarray:
    """6-vec [rx, ry, rz, tx, ty, tz] → 4x4 T.

    회전부는 axis-angle (rodrigues) — scipy의 Rotation.from_rotvec 사용.
    이 매핑은 SE(3) exp가 아닌 직접 매개변수화 (회전과 평행이동이 독립).
    번들 조정에서 변수 표현으로 충분 — exact SE(3) exp까지는 불필요.
    """
    xi = np.asarray(xi, dtype=np.float64).reshape(6)
    R = Rotation.from_rotvec(xi[:3]).as_matrix()
    return make_T(R, xi[3:6])


def T_to_vec(T: np.ndarray) -> np.ndarray:
    """4x4 T → 6-vec [rx, ry, rz, tx, ty, tz]. vec_to_T의 역."""
    R, t = split_T(T)
    rvec = Rotation.from_matrix(R).as_rotvec()
    return np.concatenate([rvec, t])


def se3_log(T: np.ndarray) -> np.ndarray:
    """잔차용 6-vec 표현 — [rvec(R), t].

    진짜 SE(3) 매니폴드 log은 V^-1 항을 포함하지만 small-angle에서 numerical
    불안정. 우리는 ||r||^2이 0 ↔ T = I 만 필요하므로 split-form으로 충분.
    Huber loss + LM의 minimum과 수렴 거동은 동일.

    scipy.Rotation.as_rotvec()이 small-angle을 안전하게 처리 (내부 Taylor).
    """
    R, t = split_T(T)
    rvec = Rotation.from_matrix(R).as_rotvec()
    return np.concatenate([rvec, t])
