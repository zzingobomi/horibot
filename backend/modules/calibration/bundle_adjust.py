"""Hand-Eye + joint offset Bundle Adjustment.

cv2.calibrateHandEye는 (a) outlier에 끌려가고 (b) FK 자체에 systematic 오차가
있으면 그걸 X 추정에 leak시킴. 여기서는 Huber loss로 outlier에 robust하면서,
joint zero offset 5개를 함께 추정해 FK floor를 같이 해소.

변수 (17 free):
    X (hand-eye, 6 DOF: rvec 3 + tvec 3)
    T_b (보드의 base-frame 포즈, 6 DOF)
    joint_offset[J] (FK 입력에 더해지는 각 조인트 보정, 기본 J=5)

모델: 캡처된 joint_angles에 offset을 더해 FK를 다시 풀어 T_gripper2base를
재계산. 보드는 안 움직였으므로 모든 포즈에서
    T_target←base_i = T_gb(angles_i + offset) · X · T_target2cam_i = T_b

잔차 (포즈당 6-vec):
    r_i = se3_log(T_b^-1 · T_predicted_i)
평행이동(미터)과 회전(라디안) 단위 불일치는 TRANSLATION_SCALE로 정렬.

PyBullet FK는 thread-safe (PybulletSolver._sim_lock). LM 한 번 돌리는 데
≲ 1만 FK 호출, ~수 초.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np
from scipy.optimize import least_squares

from .se3 import T_to_vec, invert_T, make_T, se3_log, vec_to_T

logger = logging.getLogger(__name__)

# angles (라디안 리스트) → (R 3x3, t 3-vec).
# 반환값은 np.array로 변환 가능한 어떤 형태든 OK (list-of-list, tuple, ndarray).
FkFn = Callable[[list[float]], tuple[Any, Any]]

# 평행이동(m)을 회전(rad)과 비슷한 스케일로 (0.1 m ≈ 1 rad).
TRANSLATION_SCALE: float = 10.0

# Huber transition. scaled 6-vec 잔차에서 1 sigma 좋은 자세는 ≲ 0.02.
HUBER_F_SCALE: float = 0.03

# joint_offset의 hard limit (rad). 모터 zero가 통째로 어긋나도 ±10° 이상은 비현실적.
# scipy bounds로 강제해 overfitting 방지.
JOINT_OFFSET_BOUND_RAD: float = np.deg2rad(10.0)

# joint_offset에 대한 ridge prior 가중치.
# 베이스 yaw / 손목 roll은 T_b / X와 부분적으로 degenerate (1D gauge) — 데이터가
# 일관되더라도 optimizer가 ridge 위 어디나 landing 가능. 소수의 ridge prior로
# "최소 보정" 해석을 강제 (보정량이 정말 필요할 때만 0 외 값 채택).
#
# 가중치 정당화: lambda * offset_rad가 6-vec 잔차 한 항과 같은 단위.
# offset 0.01 rad (~0.6°)에 대해 잔차 기여 = lambda * 0.01.
# Huber f_scale=0.03이므로 lambda=2.0이면 0.01 rad offset이 ~0.02 residual 기여 (transition 미만).
# 너무 강하지 않게 — 진짜 큰 보정(예: 5°)이 필요할 땐 데이터 residual이 압도.
JOINT_OFFSET_RIDGE_LAMBDA: float = 2.0


@dataclass
class BundleAdjustResult:
    R_cam2gripper: np.ndarray  # 3x3
    t_cam2gripper: np.ndarray  # (3,) meters
    T_board_base: np.ndarray  # 4x4
    joint_offset_rad: np.ndarray  # (J,) — 캡처 각도에 더해야 할 보정량
    cost: float
    n_iter: int
    success: bool
    message: str
    # 포즈당 잔차 norm (scale 풀린 후, 보고용)
    residual_rot_deg: np.ndarray  # (N,)
    residual_t_mm: np.ndarray  # (N,)
    # 동결 옵션 진단용
    n_joint_vars: int


def bundle_adjust_hand_eye(
    *,
    joint_angles_per_pose: list[list[float]],  # N × J — 캡처 시점 각도 (offset 적용 후)
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    X_init: tuple[np.ndarray, np.ndarray],
    fk_fn: FkFn,
    estimate_joint_offsets: bool = True,
    joint_offset_bound_rad: float = JOINT_OFFSET_BOUND_RAD,
    joint_offset_ridge_lambda: float = JOINT_OFFSET_RIDGE_LAMBDA,
    huber_f_scale: float = HUBER_F_SCALE,
    max_nfev: int = 500,
) -> BundleAdjustResult:
    """robust BA로 X (+ joint_offset) 동시 추정.

    인자
    ----
    joint_angles_per_pose : 캡처 시 JointStateCache가 반환한 각도 (기존 offset 적용된
        값이면 BA가 추정하는 건 delta offset). FK는 매 iteration에서 재계산.
    R/t_target2cam : PnP에서 얻은 체커보드 포즈.
    X_init : cv2.calibrateHandEye seed.
    fk_fn  : PybulletSolver.fk_to_matrix 래핑.
    estimate_joint_offsets : False면 17→12 DOF (offset 0 고정). 회귀 테스트용.

    반환
    ----
    BundleAdjustResult — joint_offset_rad는 입력 각도에 *더해야 할* delta.
    """
    N = len(joint_angles_per_pose)
    assert N == len(R_target2cam) == len(t_target2cam), "포즈 리스트 길이 불일치"
    if N < 3:
        raise ValueError(f"BA 최소 3 포즈 필요 (받은 {N}개)")

    J = len(joint_angles_per_pose[0])
    assert all(len(a) == J for a in joint_angles_per_pose), (
        f"포즈마다 joint angle 수가 다름 (기대 {J})"
    )

    angles_np = np.array(joint_angles_per_pose, dtype=np.float64)  # (N, J)
    T_tc = [make_T(R, np.asarray(t).reshape(3)) for R, t in zip(R_target2cam, t_target2cam)]

    # 초기 X / T_b
    R_init, t_init = X_init
    X_init_T = make_T(R_init, np.asarray(t_init).reshape(3))
    R0, t0 = fk_fn(list(angles_np[0]))
    T_gb_0 = make_T(np.asarray(R0), np.asarray(t0))
    T_b_init = T_gb_0 @ X_init_T @ T_tc[0]

    n_offset_vars = J if estimate_joint_offsets else 0
    x0 = np.concatenate(
        [
            T_to_vec(X_init_T),
            T_to_vec(T_b_init),
            np.zeros(n_offset_vars),
        ]
    )  # (12 + n_offset_vars,)

    # Bounds: X, T_b은 자유, offset은 ±bound
    lb = np.full(x0.shape, -np.inf)
    ub = np.full(x0.shape, np.inf)
    if n_offset_vars > 0:
        lb[12:] = -joint_offset_bound_rad
        ub[12:] = joint_offset_bound_rad

    # 총 잔차 길이: 6N (데이터) + n_offset_vars (ridge)
    total_resid_len = 6 * N + n_offset_vars

    def residual(x: np.ndarray) -> np.ndarray:
        X = vec_to_T(x[:6])
        T_b = vec_to_T(x[6:12])
        offset = x[12:] if n_offset_vars > 0 else np.zeros(J)
        T_b_inv = invert_T(T_b)

        r = np.empty(total_resid_len, dtype=np.float64)
        for i in range(N):
            corrected = list(angles_np[i] + offset)
            R_gb, t_gb = fk_fn(corrected)
            T_gb = make_T(np.asarray(R_gb), np.asarray(t_gb))
            T_pred = T_gb @ X @ T_tc[i]
            r6 = se3_log(T_b_inv @ T_pred).copy()
            r6[3:] *= TRANSLATION_SCALE
            r[6 * i : 6 * (i + 1)] = r6
        # ridge prior on offsets — gauge 해소 + 과적합 방지
        if n_offset_vars > 0:
            r[6 * N :] = joint_offset_ridge_lambda * offset
        return r

    result = least_squares(
        residual,
        x0,
        bounds=(lb, ub),
        jac="2-point",
        method="trf",
        loss="huber",
        f_scale=huber_f_scale,
        max_nfev=max_nfev,
        verbose=0,
    )

    X_opt = vec_to_T(result.x[:6])
    T_b_opt = vec_to_T(result.x[6:12])
    offset_opt = (
        result.x[12:].copy() if n_offset_vars > 0 else np.zeros(J, dtype=np.float64)
    )

    # ridge 부분은 떼고 데이터 잔차만 분석
    r_final_full = residual(result.x)
    r_final = r_final_full[: 6 * N].reshape(N, 6)
    rot_norms = np.linalg.norm(r_final[:, :3], axis=1)
    t_norms = np.linalg.norm(r_final[:, 3:] / TRANSLATION_SCALE, axis=1)

    return BundleAdjustResult(
        R_cam2gripper=X_opt[:3, :3].copy(),
        t_cam2gripper=X_opt[:3, 3].reshape(3).copy(),
        T_board_base=T_b_opt,
        joint_offset_rad=offset_opt,
        cost=float(result.cost),
        n_iter=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        residual_rot_deg=np.degrees(rot_norms),
        residual_t_mm=t_norms * 1000.0,
        n_joint_vars=n_offset_vars,
    )
