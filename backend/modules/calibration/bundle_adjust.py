"""Hand-Eye + joint offset (+ link offset) Bundle Adjustment.

cv2.calibrateHandEye 결과를 seed로 받아, joint zero offset과 hand-eye 변환을
동시에 최적화. 비용함수는 T_base←board의 분산 (체커보드가 안 움직였으니까
모든 포즈에서 같은 값이 되어야 함).

세 가지 BA가 제공됨:
  - `bundle_adjust_hand_eye(...)` — 기존 11자유도 (joint_offset 5 + R/t).
    PyBullet FK 사용 (URDF 고정). DIY 5축에서 σ_rot floor ~1.5° / σ_t ~17mm.
  - `bundle_adjust_hand_eye_extended(...)` — 확장 41자유도 (위 + link_trans 15
    + link_rot 15). URDF의 link 미스매치도 같이 풀어 floor 깸 (~1.3°/9mm 검증).
    PyBullet 우회, numpy fk_chain 사용 (link_offset이 매 iter 변수라 PyBullet
    의 정적 URDF로는 표현 불가능).
  - `bundle_adjust_hand_eye_physical_sag(...)` — 위 + 자세 의존 중력 처짐 sag 모델
    2변수 (k_J2, k_J3) = 43자유도. lumped mass + 모멘트 암 기반 토크 → sag = k * τ.
    자세 의존 오차 (link offset이 잘못 흡수하던 부분)를 분리해 σ_rot 0.65°/σ_t 7.9mm
    달성 (lumped mass 물리 sag 모델 검증).
    PyBullet calculateInverseDynamics는 URDF mass의 D405 누락으로 lumped보다 σ 손해 →
    채택 X (PyBullet inverseDynamics와 비교 후 lumped 우월 확인).

T_b(보드의 base-frame 포즈)는 명시 변수가 아니라 매 iteration에서 모든 포즈의
T_base←board 평균으로 계산. 이렇게 하면 X와 T_b 사이 gauge freedom이 사라져
LM이 잘못된 minimum에 빠지지 않음 (T_b를 변수로 두면 X·T_b 결합 gauge가 ridge로
잡히지 않아 X가 잘못된 방향으로 헤맴 — 실측으로 확인).

scipy.optimize.least_squares + Levenberg-Marquardt. 잔차는 포즈마다 6차원
(회전 axis-angle 편차 3 + 위치 편차 3, 미터). 확장 BA는 reg 잔차도 추가
(joint_offset 5 + link_trans 15 + link_rot 15).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable

import cv2
import numpy as np
from scipy.optimize import least_squares

from .se3 import make_T
from modules.kinematics.fk_chain import gravity_torque_lumped

if TYPE_CHECKING:
    from modules.kinematics.fk_chain import FkChain

# OMX-F sag joint 의 arm 안 0-indexed position. J2/J3 = motor id 2, 3 → 0/1-indexed 1, 2.
# SO-101 sag 캘 진입 시 일반화 (storage_layer.md §13.6 (5.5)(a) follow-up).
_OMX_SAG_JOINT_ARM_INDICES: list[int] = [1, 2]

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
    fk_fn  : Kinematics.fk_to_matrix 래핑.
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


# ─── IRLS+Huber 공통 default — _physical_sag_irls 가 사용 ──────────────────

DEFAULT_IRLS_MAX_OUTER_ITER: int = 5
DEFAULT_IRLS_COST_REL_TOL: float = 1e-3
DEFAULT_HUBER_KAPPA_FACTOR: float = 1.345  # standard Huber: 95% efficiency at Gaussian


# ─── 확장 BA — joint_offset + link_trans + link_rot + X 동시 추정 ──────────


@dataclass
class BundleAdjustExtendedResult:
    """확장 BA 결과. 기존 BundleAdjustResult + link_trans/link_rot."""
    R_cam2gripper: np.ndarray  # 3x3
    t_cam2gripper: np.ndarray  # (3,) meters
    T_board_base: np.ndarray  # 4x4
    joint_offset_rad: np.ndarray  # (J,)
    link_trans_m: np.ndarray  # (J, 3) — joint i origin xyz에 더할 dx,dy,dz (m)
    link_rot_rad: np.ndarray  # (J, 3) — joint i origin frame에 적용할 rotvec
    cost: float
    n_iter: int
    success: bool
    message: str
    residual_rot_deg: np.ndarray  # (N,)
    residual_t_mm: np.ndarray  # (N,)


# extended BA reg sweep으로 검증된 값. 캘 데이터 32포즈
# 기준 σ_rot 1.30°/σ_t 9.3mm 달성, hold-out validation으로 generalize 확인.
# 더 강하게 (예 link_trans_reg=5) → link offset이 0 가까이 눌려 BA가 J2/J3 offset
# 으로 흡수, gauge freedom 제거 효과 작아짐.
DEFAULT_JOINT_OFFSET_REG: float = 0.5
DEFAULT_LINK_TRANS_REG: float = 1.0
DEFAULT_LINK_ROT_REG: float = 1.0


def bundle_adjust_hand_eye_extended(
    *,
    joint_angles_per_pose: list[list[float]],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    X_init: tuple[np.ndarray, np.ndarray],
    fk_chain: "FkChain",
    joint_offset_reg: float = DEFAULT_JOINT_OFFSET_REG,
    link_trans_reg: float = DEFAULT_LINK_TRANS_REG,
    link_rot_reg: float = DEFAULT_LINK_ROT_REG,
    max_nfev: int = 3000,
) -> BundleAdjustExtendedResult:
    """joint_offset + link_trans + link_rot + R/t 동시 BA.

    `fk_chain` (FkChain 인스턴스) 명시 주입 — URDF 의 joint origin 이 변수로 풀려야
    하는데 PyBullet 은 URDF 로드 후 동적 변경 불가라 numpy chain. caller (예:
    HandEyeCalibration) 가 `RobotRegistry.get_fk_chain(robot_id)` 로 받음.

    Args:
        joint_angles_per_pose: shape (N, J) — 캡처 시점 URDF rad (J = fk_chain.n_arm).
            (= raw_to_rad + 현재 joint_offsets 적용 후).
        R/t_target2cam: PnP로 얻은 체커보드 포즈.
        X_init: cv2.calibrateHandEye seed.
        fk_chain: per-robot FkChain — URDF parse 결과 기반 arm chain.
        *_reg: regularization weights. 기본값은 reg sweep으로 검증됨.
            link_trans_reg=1.0 → ~15mm 부근 자유, link_rot_reg=1.0 → ~2° 부근 자유.
        max_nfev: LM iteration 상한.

    Returns:
        BundleAdjustExtendedResult — link_trans/link_rot는 *original URDF 기준
        absolute total* 값 (delta 아님 — x0 = zeros 에서 출발해 fk_chain이
        original URDF + link_t를 사용하므로). 디스크 link_offsets.npz에 적용 시
        cumulative 가산 금지 — **overwrite**로 덮어써야 함
        (LinkCoordinates.commit_offsets가 2026-05-28 overwrite로 fix됨,
        참조: docs/accuracy_squeeze_plan.md §1.6).
    """
    N = len(joint_angles_per_pose)
    assert N == len(R_target2cam) == len(t_target2cam), "포즈 리스트 길이 불일치"
    if N < 3:
        raise ValueError(f"BA 최소 3 포즈 필요 (받은 {N}개)")

    J = fk_chain.n_arm
    assert all(len(a) == J for a in joint_angles_per_pose), (
        f"포즈마다 joint angle 수가 {J}이어야 함"
    )

    angles_arr = np.array(joint_angles_per_pose, dtype=np.float64)
    R_tc_list = [np.asarray(R, dtype=np.float64) for R in R_target2cam]
    t_tc_list = [np.asarray(t, dtype=np.float64).reshape(3) for t in t_target2cam]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    rod_seed, _ = cv2.Rodrigues(np.asarray(X_init[0], dtype=np.float64))
    t_seed = np.asarray(X_init[1], dtype=np.float64).reshape(3)

    # 변수 layout: [J] offset + [3*J] link_trans + [3*J] link_rot + [3] rod + [3] t
    n_off = J
    n_lt = 3 * J
    n_lr = 3 * J

    def unpack(x: np.ndarray):
        i = 0
        offset = x[i : i + n_off]
        i += n_off
        link_t = x[i : i + n_lt].reshape(J, 3)
        i += n_lt
        link_r = x[i : i + n_lr].reshape(J, 3)
        i += n_lr
        rod = x[i : i + 3]
        i += 3
        t_x = x[i : i + 3]
        return offset, link_t, link_r, rod, t_x

    def compute_T_target_in_base(x: np.ndarray) -> list[np.ndarray]:
        offset, link_t, link_r, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out: list[np.ndarray] = []
        for i in range(N):
            R_gb, t_gb = fk_chain.fk(angles_arr[i] + offset, link_t, link_r)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual(x: np.ndarray) -> np.ndarray:
        offset, link_t, link_r, _, _ = unpack(x)
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = _mean_rotation([T[:3, :3] for T in T_list])
        n_reg = n_off + n_lt + n_lr
        res = np.empty(6 * N + n_reg, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos
        off_start = 6 * N
        res[off_start : off_start + n_off] = joint_offset_reg * offset
        res[off_start + n_off : off_start + n_off + n_lt] = (
            link_trans_reg * link_t.flatten()
        )
        res[off_start + n_off + n_lt : off_start + n_off + n_lt + n_lr] = (
            link_rot_reg * link_r.flatten()
        )
        return res

    x0 = np.concatenate(
        [
            np.zeros(n_off),
            np.zeros(n_lt),
            np.zeros(n_lr),
            rod_seed.flatten(),
            t_seed,
        ]
    )

    result = least_squares(
        residual,
        x0,
        method="lm",
        max_nfev=max_nfev,
        xtol=1e-11,
        ftol=1e-11,
    )

    offset_opt, link_t_opt, link_r_opt, rod_opt, t_opt = unpack(result.x)
    R_opt, _ = cv2.Rodrigues(rod_opt)

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

    return BundleAdjustExtendedResult(
        R_cam2gripper=R_opt.copy(),
        t_cam2gripper=t_opt.reshape(3).copy(),
        T_board_base=T_b_final,
        joint_offset_rad=offset_opt.copy(),
        link_trans_m=link_t_opt.copy(),
        link_rot_rad=link_r_opt.copy(),
        cost=float(result.cost),
        n_iter=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        residual_rot_deg=np.degrees(rot_norms),
        residual_t_mm=t_norms * 1000.0,
    )


# ─── 물리 sag BA — extended + 자세 의존 중력 처짐 ────────────────────────


@dataclass
class BundleAdjustPhysicalSagResult:
    """확장 BA + 물리 sag 모델 결과.

    BundleAdjustExtendedResult + sag_k_rad_per_m (2,) + max_sag_deg (2,).
    sag는 J2, J3에만 적용 (DIY 5축에서 중력 부하 가장 큰 두 joint).

    IRLS 변형 (`bundle_adjust_hand_eye_physical_sag_irls`) 도 같은 type return —
    weights / outer_iter / history 가 추가로 채워짐. non-IRLS BA 가 호출하면 default.
    이렇게 한 type 으로 통일해 caller (`hand_eye._run_ba_physical_sag_lists` etc.) 가
    IRLS 여부와 무관하게 동일 처리 가능.
    """
    R_cam2gripper: np.ndarray  # 3x3
    t_cam2gripper: np.ndarray  # (3,) meters
    T_board_base: np.ndarray  # 4x4
    joint_offset_rad: np.ndarray  # (J,)
    link_trans_m: np.ndarray  # (J, 3) m
    link_rot_rad: np.ndarray  # (J, 3) rad rotvec
    sag_k_rad_per_m: np.ndarray  # (2,) — k_J2, k_J3. sag = k * τ
    max_sag_deg: np.ndarray  # (2,) — 캡처 자세들의 최대 sag (디버깅/UI 표시용)
    cost: float
    n_iter: int
    success: bool
    message: str
    residual_rot_deg: np.ndarray  # (N,)
    residual_t_mm: np.ndarray  # (N,)
    # IRLS-only 필드 — non-IRLS 호출 시 default. weights=None 이 IRLS 미사용 marker.
    weights: np.ndarray | None = None  # (N,) Huber w_i ∈ [0,1]
    outer_iter: int = 0
    cost_history: list[float] = field(default_factory=list)
    sigma_hat_history: list[float] = field(default_factory=list)
    huber_kappa_history: list[float] = field(default_factory=list)


# reg sweep으로 검증된 default.
# k_reg 0~0.1 sweet spot, 0.5↑부터 link_offset으로 흡수되어 σ 손해.
# 기본 0.0 = reg 없음 (변수 작아서 폭주 안 함, k_J2≈0.27, k_J3≈0.14 nominal).
DEFAULT_SAG_K_REG: float = 0.0


def bundle_adjust_hand_eye_physical_sag(
    *,
    joint_angles_per_pose: list[list[float]],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    X_init: tuple[np.ndarray, np.ndarray],
    fk_chain: "FkChain",
    joint_offset_reg: float = DEFAULT_JOINT_OFFSET_REG,
    link_trans_reg: float = DEFAULT_LINK_TRANS_REG,
    link_rot_reg: float = DEFAULT_LINK_ROT_REG,
    sag_k_reg: float = DEFAULT_SAG_K_REG,
    max_nfev: int = 5000,
) -> BundleAdjustPhysicalSagResult:
    """확장 BA + 물리 sag (43 DOF for OMX-F 5DOF / 51 DOF for SO-101 6DOF).

    `bundle_adjust_hand_eye_extended`와 동일 구조 + sag_k_rad_per_m 2개 추가.
    sag 는 J2/J3 (motor id 2,3 = arm idx 1,2) 에만 적용 (OMX-F 가정).
    SO-101 sag 캘 진입 시 sag joint 일반화 (storage_layer.md §13.6 (5.5)(a)
    follow-up).

    변수 layout (J = fk_chain.n_arm):
      [0:J]            joint_offset (rad)
      [J:J+3J]         link_translation (J×3, m)
      [J+3J:J+6J]      link_rotation (J×3, rad rotvec)
      [J+6J:J+6J+2]    sag_k (J2, J3) (rad / (m·g_unit))
      [J+6J+2:J+6J+5]  rod (cam2gripper)
      [J+6J+5:J+6J+8]  t (cam2gripper, m)

    σ_rot 0.65° / σ_t 7.9mm 달성 (32 포즈 검증). lumped mass 모델이므로 k 가
    (effective stiffness × effective mass) 비율을 통째로 흡수 → URDF mass 부정확성
    (D405 카메라 무게 누락 등) 에 robust.
    """
    N = len(joint_angles_per_pose)
    assert N == len(R_target2cam) == len(t_target2cam), "포즈 리스트 길이 불일치"
    if N < 3:
        raise ValueError(f"BA 최소 3 포즈 필요 (받은 {N}개)")

    J = fk_chain.n_arm
    assert all(len(a) == J for a in joint_angles_per_pose), (
        f"포즈마다 joint angle 수가 {J}이어야 함"
    )

    angles_arr = np.array(joint_angles_per_pose, dtype=np.float64)
    R_tc_list = [np.asarray(R, dtype=np.float64) for R in R_target2cam]
    t_tc_list = [np.asarray(t, dtype=np.float64).reshape(3) for t in t_target2cam]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    rod_seed, _ = cv2.Rodrigues(np.asarray(X_init[0], dtype=np.float64))
    t_seed = np.asarray(X_init[1], dtype=np.float64).reshape(3)

    n_off = J
    n_lt = 3 * J
    n_lr = 3 * J
    n_k = 2

    def unpack(x: np.ndarray):
        i = 0
        offset = x[i : i + n_off]
        i += n_off
        link_t = x[i : i + n_lt].reshape(J, 3)
        i += n_lt
        link_r = x[i : i + n_lr].reshape(J, 3)
        i += n_lr
        sag_k = x[i : i + n_k]
        i += n_k
        rod = x[i : i + 3]
        i += 3
        t_x = x[i : i + 3]
        return offset, link_t, link_r, sag_k, rod, t_x

    def compute_T_target_in_base(x: np.ndarray) -> list[np.ndarray]:
        offset, link_t, link_r, sag_k, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out: list[np.ndarray] = []
        for i in range(N):
            a_corr = fk_chain.apply_gravity_sag(
                angles_arr[i] + offset,
                sag_k,
                _OMX_SAG_JOINT_ARM_INDICES,
                link_t,
                link_r,
            )
            R_gb, t_gb = fk_chain.fk(a_corr, link_t, link_r)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual(x: np.ndarray) -> np.ndarray:
        offset, link_t, link_r, sag_k, _, _ = unpack(x)
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = _mean_rotation([T[:3, :3] for T in T_list])
        n_reg = n_off + n_lt + n_lr + n_k
        res = np.empty(6 * N + n_reg, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos
        k = 6 * N
        res[k : k + n_off] = joint_offset_reg * offset
        k += n_off
        res[k : k + n_lt] = link_trans_reg * link_t.flatten()
        k += n_lt
        res[k : k + n_lr] = link_rot_reg * link_r.flatten()
        k += n_lr
        res[k : k + n_k] = sag_k_reg * sag_k
        return res

    x0 = np.concatenate(
        [
            np.zeros(n_off),
            np.zeros(n_lt),
            np.zeros(n_lr),
            np.zeros(n_k),
            rod_seed.flatten(),
            t_seed,
        ]
    )

    result = least_squares(
        residual,
        x0,
        method="lm",
        max_nfev=max_nfev,
        xtol=1e-11,
        ftol=1e-11,
    )

    offset_opt, link_t_opt, link_r_opt, sag_k_opt, rod_opt, t_opt = unpack(result.x)
    R_opt, _ = cv2.Rodrigues(rod_opt)

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

    # 캡처 자세들의 최대 sag (deg) — UI/디버깅 표시용
    max_sag = np.zeros(2, dtype=np.float64)
    sag_idx_J2, sag_idx_J3 = _OMX_SAG_JOINT_ARM_INDICES
    for i in range(N):
        _, ee_pos, jo, ja = fk_chain.fk_with_axes(
            angles_arr[i] + offset_opt, link_t_opt, link_r_opt
        )
        tau2 = gravity_torque_lumped(ee_pos, jo[sag_idx_J2], ja[sag_idx_J2])
        tau3 = gravity_torque_lumped(ee_pos, jo[sag_idx_J3], ja[sag_idx_J3])
        s2 = abs(np.degrees(sag_k_opt[0] * tau2))
        s3 = abs(np.degrees(sag_k_opt[1] * tau3))
        max_sag[0] = max(max_sag[0], s2)
        max_sag[1] = max(max_sag[1], s3)

    return BundleAdjustPhysicalSagResult(
        R_cam2gripper=R_opt.copy(),
        t_cam2gripper=t_opt.reshape(3).copy(),
        T_board_base=T_b_final,
        joint_offset_rad=offset_opt.copy(),
        link_trans_m=link_t_opt.copy(),
        link_rot_rad=link_r_opt.copy(),
        sag_k_rad_per_m=sag_k_opt.copy(),
        max_sag_deg=max_sag,
        cost=float(result.cost),
        n_iter=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        residual_rot_deg=np.degrees(rot_norms),
        residual_t_mm=t_norms * 1000.0,
    )


# ─── _physical_sag + IRLS+Huber — robust 43 DOF BA ────────────────────────
# 결과 type 은 BundleAdjustPhysicalSagResult 와 동일 — weights / outer_iter /
# history 필드가 채워진 상태로 반환. caller 가 IRLS 여부로 분기할 필요 X.


def bundle_adjust_hand_eye_physical_sag_irls(
    *,
    joint_angles_per_pose: list[list[float]],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    X_init: tuple[np.ndarray, np.ndarray],
    fk_chain: "FkChain",
    joint_offset_reg: float = DEFAULT_JOINT_OFFSET_REG,
    link_trans_reg: float = DEFAULT_LINK_TRANS_REG,
    link_rot_reg: float = DEFAULT_LINK_ROT_REG,
    sag_k_reg: float = DEFAULT_SAG_K_REG,
    max_outer_iter: int = DEFAULT_IRLS_MAX_OUTER_ITER,
    cost_rel_tol: float = DEFAULT_IRLS_COST_REL_TOL,
    huber_kappa_factor: float = DEFAULT_HUBER_KAPPA_FACTOR,
    max_nfev: int = 5000,
) -> BundleAdjustPhysicalSagResult:
    """_physical_sag 의 IRLS+Huber 버전. _physical_sag 와 동일 DOF + outer loop.

    각 outer iter:
      1. sqrt(w_i) 로 per-pose 잔차에 weight (6N 부분만, reg 항은 weight 안 곱)
      2. LM 풀기
      3. per-pose 잔차 norm 계산
      4. σ̂ = MAD(r)/0.6745, κ = 1.345·σ̂, w_i = min(1, κ/r_i)
      5. 잔차 cost change < cost_rel_tol 면 수렴
    """
    N = len(joint_angles_per_pose)
    assert N == len(R_target2cam) == len(t_target2cam), "포즈 리스트 길이 불일치"
    if N < 3:
        raise ValueError(f"BA 최소 3 포즈 필요 (받은 {N}개)")

    J = fk_chain.n_arm
    assert all(len(a) == J for a in joint_angles_per_pose), (
        f"포즈마다 joint angle 수가 {J}이어야 함"
    )

    angles_arr = np.array(joint_angles_per_pose, dtype=np.float64)
    R_tc_list = [np.asarray(R, dtype=np.float64) for R in R_target2cam]
    t_tc_list = [np.asarray(t, dtype=np.float64).reshape(3) for t in t_target2cam]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    rod_seed, _ = cv2.Rodrigues(np.asarray(X_init[0], dtype=np.float64))
    t_seed = np.asarray(X_init[1], dtype=np.float64).reshape(3)

    n_off = J
    n_lt = 3 * J
    n_lr = 3 * J
    n_k = 2
    n_reg = n_off + n_lt + n_lr + n_k

    def unpack(x: np.ndarray):
        i = 0
        offset = x[i : i + n_off]
        i += n_off
        link_t = x[i : i + n_lt].reshape(J, 3)
        i += n_lt
        link_r = x[i : i + n_lr].reshape(J, 3)
        i += n_lr
        sag_k = x[i : i + n_k]
        i += n_k
        rod = x[i : i + 3]
        i += 3
        t_x = x[i : i + 3]
        return offset, link_t, link_r, sag_k, rod, t_x

    def compute_T_target_in_base(x: np.ndarray) -> list[np.ndarray]:
        offset, link_t, link_r, sag_k, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out: list[np.ndarray] = []
        for i in range(N):
            a_corr = fk_chain.apply_gravity_sag(
                angles_arr[i] + offset,
                sag_k,
                _OMX_SAG_JOINT_ARM_INDICES,
                link_t,
                link_r,
            )
            R_gb, t_gb = fk_chain.fk(a_corr, link_t, link_r)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual_unweighted(x: np.ndarray) -> np.ndarray:
        offset, link_t, link_r, sag_k, _, _ = unpack(x)
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = _mean_rotation([T[:3, :3] for T in T_list])
        res = np.empty(6 * N + n_reg, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos
        k = 6 * N
        res[k : k + n_off] = joint_offset_reg * offset
        k += n_off
        res[k : k + n_lt] = link_trans_reg * link_t.flatten()
        k += n_lt
        res[k : k + n_lr] = link_rot_reg * link_r.flatten()
        k += n_lr
        res[k : k + n_k] = sag_k_reg * sag_k
        return res

    x0 = np.concatenate(
        [
            np.zeros(n_off),
            np.zeros(n_lt),
            np.zeros(n_lr),
            np.zeros(n_k),
            rod_seed.flatten(),
            t_seed,
        ]
    )

    # ── IRLS outer loop ──
    w = np.ones(N, dtype=np.float64)
    x_current = x0.copy()
    prev_cost = float("inf")
    cost_history: list[float] = []
    sigma_hat_history: list[float] = []
    kappa_history: list[float] = []
    result = None
    outer_used = 0

    for outer in range(max_outer_iter):
        outer_used = outer + 1
        sqrt_w_per_pose = np.repeat(np.sqrt(w), 6)  # 6N
        sqrt_w_full = np.concatenate([sqrt_w_per_pose, np.ones(n_reg)])

        def residual_weighted(x: np.ndarray, _w=sqrt_w_full) -> np.ndarray:
            return residual_unweighted(x) * _w

        result = least_squares(
            residual_weighted,
            x_current,
            method="lm",
            max_nfev=max_nfev,
            xtol=1e-11,
            ftol=1e-11,
        )
        x_current = result.x.copy()

        # per-pose unweighted 잔차 norm
        r_full = residual_unweighted(result.x)
        r_pose_part = r_full[: 6 * N]
        r_per_pose = np.linalg.norm(r_pose_part.reshape(N, 6), axis=1)

        # σ̂ = MAD/0.6745
        median_r = float(np.median(r_per_pose))
        mad = float(np.median(np.abs(r_per_pose - median_r)))
        sigma_hat = max(mad / 0.6745, 1e-9)
        kappa = huber_kappa_factor * sigma_hat
        w_new = np.minimum(1.0, kappa / np.maximum(r_per_pose, 1e-9))

        unweighted_cost = float(0.5 * np.sum(r_per_pose**2))
        cost_history.append(unweighted_cost)
        sigma_hat_history.append(sigma_hat)
        kappa_history.append(kappa)

        if outer > 0:
            rel_change = abs(unweighted_cost - prev_cost) / max(prev_cost, 1e-12)
            if rel_change < cost_rel_tol:
                w = w_new
                break
        prev_cost = unweighted_cost
        w = w_new

    assert result is not None

    offset_opt, link_t_opt, link_r_opt, sag_k_opt, rod_opt, t_opt = unpack(result.x)
    R_opt, _ = cv2.Rodrigues(rod_opt)

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

    max_sag = np.zeros(2, dtype=np.float64)
    sag_idx_J2, sag_idx_J3 = _OMX_SAG_JOINT_ARM_INDICES
    for i in range(N):
        _, ee_pos, jo, ja = fk_chain.fk_with_axes(
            angles_arr[i] + offset_opt, link_t_opt, link_r_opt
        )
        tau2 = gravity_torque_lumped(ee_pos, jo[sag_idx_J2], ja[sag_idx_J2])
        tau3 = gravity_torque_lumped(ee_pos, jo[sag_idx_J3], ja[sag_idx_J3])
        s2 = abs(np.degrees(sag_k_opt[0] * tau2))
        s3 = abs(np.degrees(sag_k_opt[1] * tau3))
        max_sag[0] = max(max_sag[0], s2)
        max_sag[1] = max(max_sag[1], s3)

    return BundleAdjustPhysicalSagResult(
        R_cam2gripper=R_opt.copy(),
        t_cam2gripper=t_opt.reshape(3).copy(),
        T_board_base=T_b_final,
        joint_offset_rad=offset_opt.copy(),
        link_trans_m=link_t_opt.copy(),
        link_rot_rad=link_r_opt.copy(),
        sag_k_rad_per_m=sag_k_opt.copy(),
        max_sag_deg=max_sag,
        cost=float(result.cost),
        n_iter=int(result.nfev),
        success=bool(result.success),
        message=str(result.message),
        residual_rot_deg=np.degrees(rot_norms),
        residual_t_mm=t_norms * 1000.0,
        weights=w.copy(),
        outer_iter=outer_used,
        cost_history=cost_history,
        sigma_hat_history=sigma_hat_history,
        huber_kappa_history=kappa_history,
    )
