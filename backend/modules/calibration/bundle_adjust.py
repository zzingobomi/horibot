"""Hand-Eye + joint offset Bundle Adjustment.

cv2.calibrateHandEye 결과를 seed로 받아, joint zero offset과 hand-eye 변환을
동시에 최적화. 비용함수는 T_base←board의 분산 (체커보드가 안 움직였으니까
모든 포즈에서 같은 값이 되어야 함).

변수 (총 11개, estimate_joint_offsets=True 기준):
    joint_offset[J] (FK 입력에 더해지는 각 조인트 보정, 기본 J=5)
    rod (3): R_cam2gripper의 Rodrigues 벡터
    t (3): t_cam2gripper (미터)

T_b(보드의 base-frame 포즈)는 명시 변수가 아니라 매 iteration에서 모든 포즈의
T_base←board 평균으로 계산. 이렇게 하면 X와 T_b 사이 gauge freedom이 사라져
LM이 잘못된 minimum에 빠지지 않음 (T_b를 변수로 두면 X·T_b 결합 gauge가 ridge로
잡히지 않아 X가 잘못된 방향으로 헤맴 — 실측으로 확인).

scipy.optimize.least_squares + Levenberg-Marquardt. 잔차는 포즈마다 6차원
(회전 axis-angle 편차 3 + 위치 편차 3, 미터). 31포즈면 186개 잔차로 11개 변수.

PyBullet FK는 thread-safe (PybulletSolver._sim_lock). LM 한 번 돌리는 데
≲ 1만 FK 호출, ~수 초.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

import cv2
import numpy as np
from scipy.optimize import least_squares

from .se3 import make_T

logger = logging.getLogger(__name__)

# angles (라디안 리스트) → (R 3x3, t 3-vec).
# 반환값은 np.array로 변환 가능한 어떤 형태든 OK (list-of-list, tuple, ndarray).
FkFn = Callable[[list[float]], tuple[Any, Any]]

# joint_offset hard limit (rad). 구버전 LM은 unbounded지만 인터페이스 호환용으로 인자는 유지.
JOINT_OFFSET_BOUND_RAD: float = np.deg2rad(10.0)

# ridge / huber 인자는 현 mean-based 구현에서 사용하지 않음.
# 시그니처 호환을 위해 default만 유지.
JOINT_OFFSET_RIDGE_LAMBDA: float = 0.0
HUBER_F_SCALE: float = 0.03


@dataclass
class BundleAdjustResult:
    R_cam2gripper: np.ndarray  # 3x3
    t_cam2gripper: np.ndarray  # (3,) meters
    T_board_base: np.ndarray  # 4x4 (mean으로 계산한 결과)
    joint_offset_rad: np.ndarray  # (J,) — 캡처 각도에 더해야 할 보정량
    cost: float
    n_iter: int
    success: bool
    message: str
    # 포즈당 잔차 norm (보고용)
    residual_rot_deg: np.ndarray  # (N,)
    residual_t_mm: np.ndarray  # (N,)
    # 동결 옵션 진단용
    n_joint_vars: int


def _mean_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    """Markley chordal mean (SVD 기반)."""
    M = np.zeros((3, 3), dtype=np.float64)
    for R in Rs:
        M += R
    U, _, Vt = np.linalg.svd(M)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean


def bundle_adjust_hand_eye(
    *,
    joint_angles_per_pose: list[list[float]],  # N × J — 캡처 시점 각도 (offset 적용 후)
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    X_init: tuple[np.ndarray, np.ndarray],
    fk_fn: FkFn,
    estimate_joint_offsets: bool = True,
    joint_offset_bound_rad: float = JOINT_OFFSET_BOUND_RAD,  # 인터페이스 호환 (LM은 unbounded)
    joint_offset_ridge_lambda: float = JOINT_OFFSET_RIDGE_LAMBDA,  # 사용 안 함
    huber_f_scale: float = HUBER_F_SCALE,  # 사용 안 함
    max_nfev: int = 500,
) -> BundleAdjustResult:
    """mean-based BA로 X (+ joint_offset) 동시 추정.

    인자
    ----
    joint_angles_per_pose : 캡처 시 JointStateCache가 반환한 각도 (기존 offset 적용된
        값이면 BA가 추정하는 건 delta offset). FK는 매 iteration에서 재계산.
    R/t_target2cam : PnP에서 얻은 체커보드 포즈.
    X_init : cv2.calibrateHandEye seed.
    fk_fn  : PybulletSolver.fk_to_matrix 래핑.
    estimate_joint_offsets : False면 11→6 DOF (offset 0 고정). 회귀 테스트용.

    반환
    ----
    BundleAdjustResult — joint_offset_rad는 입력 각도에 *더해야 할* delta.
    """
    del joint_offset_bound_rad, joint_offset_ridge_lambda, huber_f_scale  # 호환용 미사용

    N = len(joint_angles_per_pose)
    assert N == len(R_target2cam) == len(t_target2cam), "포즈 리스트 길이 불일치"
    if N < 3:
        raise ValueError(f"BA 최소 3 포즈 필요 (받은 {N}개)")

    J = len(joint_angles_per_pose[0])
    assert all(len(a) == J for a in joint_angles_per_pose), (
        f"포즈마다 joint angle 수가 다름 (기대 {J})"
    )

    angles_np = np.array(joint_angles_per_pose, dtype=np.float64)  # (N, J)
    R_tc_list = [np.asarray(R, dtype=np.float64) for R in R_target2cam]
    t_tc_list = [
        np.asarray(t, dtype=np.float64).reshape(3) for t in t_target2cam
    ]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    n_offset_vars = J if estimate_joint_offsets else 0
    rod_seed, _ = cv2.Rodrigues(np.asarray(X_init[0], dtype=np.float64))
    t_seed = np.asarray(X_init[1], dtype=np.float64).reshape(3)
    x0 = np.concatenate(
        [
            np.zeros(n_offset_vars),  # offset 0 시작
            rod_seed.flatten(),
            t_seed,
        ]
    )

    def compute_T_target_in_base(x: np.ndarray) -> list[np.ndarray]:
        if n_offset_vars > 0:
            offset = x[:J]
            rod = x[J : J + 3]
            t = x[J + 3 : J + 6]
        else:
            offset = np.zeros(J)
            rod = x[:3]
            t = x[3:6]
        R_x, _ = cv2.Rodrigues(rod)
        T_x = make_T(R_x, t)
        T_list: list[np.ndarray] = []
        for i in range(N):
            corrected = list(angles_np[i] + offset)
            R_gb, t_gb = fk_fn(corrected)
            T_gb = make_T(np.asarray(R_gb), np.asarray(t_gb).reshape(3))
            T_list.append(T_gb @ T_x @ T_tc_list[i])
        return T_list

    def residual(x: np.ndarray) -> np.ndarray:
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = _mean_rotation([T[:3, :3] for T in T_list])
        res = np.empty(6 * N, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()  # 라디안
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos  # 미터
        return res

    result = least_squares(
        residual,
        x0,
        method="lm",
        max_nfev=max_nfev,
        xtol=1e-10,
        ftol=1e-10,
    )

    if n_offset_vars > 0:
        offset_opt = result.x[:J].copy()
        rod_opt = result.x[J : J + 3]
        t_opt = result.x[J + 3 : J + 6]
    else:
        offset_opt = np.zeros(J, dtype=np.float64)
        rod_opt = result.x[:3]
        t_opt = result.x[3:6]
    R_opt, _ = cv2.Rodrigues(rod_opt)

    # 최종 잔차 + T_b 계산 (보고용)
    T_list_final = compute_T_target_in_base(result.x)
    positions = np.array([T[:3, 3] for T in T_list_final])
    mean_pos = positions.mean(axis=0)
    mean_R = _mean_rotation([T[:3, :3] for T in T_list_final])
    T_b_final = make_T(mean_R, mean_pos)

    rot_norms = np.empty(N, dtype=np.float64)
    t_norms = np.empty(N, dtype=np.float64)
    for i, T in enumerate(T_list_final):
        R_dev = T[:3, :3] @ mean_R.T
        rod_dev, _ = cv2.Rodrigues(R_dev)
        rot_norms[i] = float(np.linalg.norm(rod_dev))
        t_norms[i] = float(np.linalg.norm(T[:3, 3] - mean_pos))

    return BundleAdjustResult(
        R_cam2gripper=R_opt.copy(),
        t_cam2gripper=t_opt.reshape(3).copy(),
        T_board_base=T_b_final,
        joint_offset_rad=offset_opt,
        cost=float(result.cost),
        n_iter=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        residual_rot_deg=np.degrees(rot_norms),
        residual_t_mm=t_norms * 1000.0,
        n_joint_vars=n_offset_vars,
    )
