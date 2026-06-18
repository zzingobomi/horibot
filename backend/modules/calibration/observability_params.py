"""Per-parameter observability — BA 정보행렬(Fisher) 기반 식별성 분석.

스펙 v3 의 `{hand_eye_rotation, hand_eye_translation, joint_offset, link, sag}` 식별성을
산업표준 방식(파라미터 정규화된 identification Jacobian SVD + 직교 사영)으로 산출.
**capture 개수가 아니라 식별 가능성으로 BA 블록 unlock 을 gating** (docs/handeye_ux_solver_v3_plan.md §3).

원리 (Bayesian CRLB / 정보이득):
  data residual r(x) (`physical_sag_data_residual`, reg 제외) 의 Jacobian J = dr/dx.
  파라미터 정규화 (컬럼 × nominal 크기) → 무차원 J_scaled (블록 간 비교 가능).
  noise σ̂ = RMS(data residual) (self-calibrating).
  posterior 정보행렬  H = J_scaledᵀ J_scaled / σ̂²  +  I
    (I = weakly-informative prior: 정규화 파라미터 ~ N(0,1). 항상 SPD → 가역).
  posterior 공분산  C = H⁻¹.
  블록 b 의 observability = 1 − mean(diag(C)[b])  ∈ [0,1)
    = "데이터가 prior 대비 이 블록 불확실성을 얼마나 줄였나" (정보이득 / resolution).
  0 → 데이터가 이 블록에 정보 안 줌 (unobservable, prior 그대로).
  →1 → 데이터가 강하게 제약 (well-observed).
  "같은 회전축 50장" → sag/joint 컬럼이 residual 분산을 못 줄임 → score≈0
    (스펙 degeneracy 시나리오 그대로 검출. 개수 무관 — 식별성으로 판정).

왜 직교사영 σ 대신 Bayesian CRLB 인가: 본 BA 는 43 DOF / 8~30 포즈로 over-parameterized
+ regularized 영역 — naive 직교사영은 모든 블록이 서로 span 안에 들어 score≈0 으로 붕괴
(측정 확인). prior 가 H 를 가역으로 만들어 "데이터가 더한 정보" 만 깨끗이 분리.

reg 항을 residual 에서 제외하는 이유: reg 는 prior — Fisher 정보는 *측정* residual 만
봐야 함 (여기선 prior 를 I 로 명시 추가, 이중계상 방지).

Refs:
  - Cramér-Rao / Fisher information — JᵀJ 가 Gauss-Newton Hessian
  - Bayesian experimental design — posterior/prior 분산비 = information gain / resolution
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from . import thresholds as T
from .bundle_adjust import (
    BLOCK_HANDEYE_ROT,
    BLOCK_HANDEYE_TRANS,
    BLOCK_JOINT,
    BLOCK_LINK,
    BLOCK_SAG,
    physical_sag_block_indices,
    physical_sag_data_residual,
    physical_sag_n_params,
)

if TYPE_CHECKING:
    from modules.kinematics.fk_chain import FkChain


# 블록 이름은 bundle_adjust 가 SSOT (layout 과 함께) — 여기선 re-export 만.
ALL_BLOCKS = [
    BLOCK_HANDEYE_ROT,
    BLOCK_HANDEYE_TRANS,
    BLOCK_JOINT,
    BLOCK_LINK,
    BLOCK_SAG,
]

# gating 대상 (handeye 제외).
GATEABLE_BLOCKS = [BLOCK_JOINT, BLOCK_LINK, BLOCK_SAG]


@dataclass
class ParamObservability:
    """블록별 식별성 score + verdict.

    scores: 블록 → score ∈ [0,1]
    verdicts: 블록 → "OK" / "WEAK" / "INSUFFICIENT"
    unlocked: gate 통과한 블록 set (joint/link/sag 중). handeye 는 항상 포함 X (별도).
    """

    n_poses: int
    scores: dict[str, float] = field(default_factory=dict)
    verdicts: dict[str, str] = field(default_factory=dict)
    unlocked: set[str] = field(default_factory=set)

    def to_dict(self) -> dict:
        return {
            "n_poses": self.n_poses,
            "scores": {k: float(v) for k, v in self.scores.items()},
            "verdicts": dict(self.verdicts),
            "unlocked": sorted(self.unlocked),
        }


def _column_scales(J: int) -> np.ndarray:
    """파라미터 정규화 — 각 컬럼을 nominal 크기로 곱해 무차원화 (블록 간 비교)."""
    scales = np.empty(physical_sag_n_params(J), dtype=np.float64)
    sl = physical_sag_block_indices(J)
    scales[sl[BLOCK_JOINT]] = T.OBS_SCALE_ANGLE_RAD
    # link 블록 = link_t (앞 3J, m) + link_r (뒤 3J, rad)
    link = sl[BLOCK_LINK]
    scales[link.start : link.start + 3 * J] = T.OBS_SCALE_TRANS_M
    scales[link.start + 3 * J : link.stop] = T.OBS_SCALE_ANGLE_RAD
    scales[sl[BLOCK_SAG]] = T.OBS_SCALE_SAG_K
    scales[sl[BLOCK_HANDEYE_ROT]] = T.OBS_SCALE_ANGLE_RAD
    scales[sl[BLOCK_HANDEYE_TRANS]] = T.OBS_SCALE_TRANS_M
    return scales


def _data_jacobian(
    x: np.ndarray,
    J: int,
    angles_arr: np.ndarray,
    T_tc_list: list[np.ndarray],
    fk_chain: "FkChain",
    sag_indices: list[int],
    *,
    eps: float = 1e-6,
) -> np.ndarray:
    """data residual (6N) 의 Jacobian (6N × n_params), 중앙차분.

    BA solution x 주변에서 선형화 (Gauss-Newton). reg 항 제외 (Fisher 는 측정만).
    """
    n = physical_sag_n_params(J)

    def r(xx: np.ndarray) -> np.ndarray:
        return physical_sag_data_residual(
            xx, J, angles_arr, T_tc_list, fk_chain, sag_indices
        )

    r0 = r(x)
    jac = np.empty((r0.shape[0], n), dtype=np.float64)
    for k in range(n):
        dx = np.zeros(n, dtype=np.float64)
        dx[k] = eps
        jac[:, k] = (r(x + dx) - r(x - dx)) / (2.0 * eps)
    return jac


def _row_weights(n_poses: int) -> np.ndarray:
    """residual 행 정규화 weight (6N,) — pose 마다 [rot×3 / σ_rot, trans×3 / σ_trans].

    rot/trans 단위 통일 + 측정 노이즈로 Fisher 정보 스케일. 고정 floor (fitted
    residual 아님) — overfit 으로 residual→0 이어도 정보 과대평가 안 함.
    """
    w = np.empty(6 * n_poses, dtype=np.float64)
    wr = 1.0 / T.OBS_NOISE_ROT_RAD
    wt = 1.0 / T.OBS_NOISE_TRANS_M
    for i in range(n_poses):
        w[6 * i : 6 * i + 3] = wr
        w[6 * i + 3 : 6 * (i + 1)] = wt
    return w


def _posterior_diag(J_norm: np.ndarray) -> np.ndarray:
    """Bayesian posterior 공분산 C = (Jᵀ J + I)⁻¹ 의 대각 (정규화 단위, prior var=1).

    J_norm = 행(노이즈) + 열(nominal) 양쪽 정규화된 무차원 Jacobian → JᵀJ 가 Fisher 정보.
    prior I (정규화 파라미터 ~ N(0,1)) → 항상 SPD, 데이터가 더한 정보만 분리.
    """
    n = J_norm.shape[1]
    H = J_norm.T @ J_norm + np.eye(n)
    C = np.linalg.inv(H)
    return np.clip(np.diag(C), 0.0, 1.0)


def _block_score(diag_C: np.ndarray, block_cols: slice) -> float:
    """블록 observability = 1 − mean(posterior_var) = 정보이득 ∈ [0,1].

    prior var = 1 (정규화). posterior var → 0 이면 데이터가 강하게 제약 (score→1),
    → 1 이면 데이터가 정보 안 줌 (score→0).
    """
    block = diag_C[block_cols]
    if block.size == 0:
        return 0.0
    return float(np.clip(1.0 - float(np.mean(block)), 0.0, 1.0))


def compute_param_observability(
    *,
    x: np.ndarray,
    fk_chain: "FkChain",
    joint_angles_per_pose: list[list[float]],
    R_target2cam: list[np.ndarray],
    t_target2cam: list[np.ndarray],
    sag_indices: list[int],
) -> ParamObservability:
    """BA solution x 에서 블록별 observability score + verdict + unlock set 산출.

    Args:
        x: physical_sag BA 의 해 (bundle_adjust.physical_sag_unpack layout).
        fk_chain / joint_angles_per_pose / R/t_target2cam / sag_indices:
            BA 입력과 동일 — 같은 모델 위에서 Jacobian 선형화.

    Returns:
        ParamObservability — scores/verdicts/unlocked.
    """
    from .se3 import make_T

    J = fk_chain.n_arm
    angles_arr = np.array(joint_angles_per_pose, dtype=np.float64)
    T_tc_list = [
        make_T(np.asarray(R, dtype=np.float64), np.asarray(t, dtype=np.float64).reshape(3))
        for R, t in zip(R_target2cam, t_target2cam)
    ]
    n = len(angles_arr)

    jac = _data_jacobian(x, J, angles_arr, T_tc_list, fk_chain, sag_indices)
    col_scales = _column_scales(J)
    row_w = _row_weights(n)
    # 행(측정 노이즈) + 열(파라미터 nominal) 양쪽 정규화 → 무차원 Fisher Jacobian.
    J_norm = row_w[:, None] * jac * col_scales[None, :]

    diag_C = _posterior_diag(J_norm)

    slices = physical_sag_block_indices(J)
    scores: dict[str, float] = {}
    for block, sl in slices.items():
        scores[block] = _block_score(diag_C, sl)

    # gate (freeze) 임계 — gateable 블록만 (handeye 는 항상 on).
    unlock_thr = {
        BLOCK_JOINT: T.OBS_UNLOCK_JOINT,
        BLOCK_LINK: T.OBS_UNLOCK_LINK,
        BLOCK_SAG: T.OBS_UNLOCK_SAG,
    }
    verdicts: dict[str, str] = {}
    unlocked: set[str] = set()
    for block in ALL_BLOCKS:
        score = scores[block]
        # verdict (UI 안내) — gate 와 별개 band.
        if score >= T.OBS_VERDICT_OK:
            verdicts[block] = "OK"
        elif score >= T.OBS_VERDICT_WEAK:
            verdicts[block] = "WEAK"
        else:
            verdicts[block] = "INSUFFICIENT"
        # gate — score ≥ unlock 이면 BA 가 추정, 아니면 freeze.
        if block in GATEABLE_BLOCKS and score >= unlock_thr[block]:
            unlocked.add(block)

    return ParamObservability(
        n_poses=n, scores=scores, verdicts=verdicts, unlocked=unlocked
    )
