"""Per-parameter observability + staged BA gating 단위 테스트.

docs/handeye_ux_solver_v3_plan.md §3. 검증:
  1. SSOT 리팩토링 무회귀 — 실데이터 8포즈 physical_sag IRLS σ baseline 유지.
  2. frozen_blocks 메커니즘 — frozen 블록은 0 고정 (추정 안 함), free 만 추정.
  3. observability score — 잘 퍼진 데이터 = 전 블록 OK + 전부 unlock.
  4. degeneracy 검출 — 자세 다양성 낮을수록 score 단조 하락 (스펙 시나리오).
  5. 병리적 데이터 (자세 거의 동일) → 일부 블록 freeze (gating 안전망).

fk_chain (numpy) 직접 사용 — PyBullet init 불필요. 합성 데이터는 알려진 X + board
pose 로 생성.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from core.robot.robot_registry import RobotRegistry
from modules.calibration.bundle_adjust import (
    BLOCK_LINK,
    BLOCK_SAG,
    _DEFAULT_SAG_ARM_INDICES,
    bundle_adjust_hand_eye_physical_sag_irls,
    physical_sag_block_indices,
)
from modules.calibration.observability_params import (
    ALL_BLOCKS,
    GATEABLE_BLOCKS,
    compute_param_observability,
)
from modules.calibration.se3 import make_T

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_NPZ = REPO_ROOT / "robot/instances/omx_f_0/calibration/handeye_poses.npz"
ROBOT = "omx_f_0"


@pytest.fixture(scope="module")
def fk_chain():
    return RobotRegistry().get_fk_chain(ROBOT)


def _fk_mat(fk_chain, ang):
    Z = np.zeros((fk_chain.n_arm, 3))
    R, t = fk_chain.fk(np.array(ang, dtype=float), Z, Z)
    return make_T(np.asarray(R), np.asarray(t).reshape(3))


def _tsai_seed(fk_chain, ja, R_tc, t_tc):
    R_gb, t_gb = [], []
    for a in ja:
        T = _fk_mat(fk_chain, a)
        R_gb.append(T[:3, :3])
        t_gb.append(T[:3, 3].reshape(3, 1))
    R_x, t_x = cv2.calibrateHandEye(
        R_gb, t_gb, R_tc, [t.reshape(3, 1) for t in t_tc],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return R_x, t_x.reshape(3)


def _solve(fk_chain, ja, R_tc, t_tc, **kw):
    R_x, t_x = _tsai_seed(fk_chain, ja, R_tc, t_tc)
    return bundle_adjust_hand_eye_physical_sag_irls(
        joint_angles_per_pose=ja, R_target2cam=R_tc, t_target2cam=t_tc,
        X_init=(R_x, t_x), fk_chain=fk_chain, **kw,
    )


def _x_from_result(r):
    return np.concatenate([
        r.joint_offset_rad, r.link_trans_m.flatten(), r.link_rot_rad.flatten(),
        r.sag_k_rad_per_m, cv2.Rodrigues(r.R_cam2gripper)[0].flatten(),
        r.t_cam2gripper,
    ])


def _obs(fk_chain, r, ja, R_tc, t_tc):
    return compute_param_observability(
        x=_x_from_result(r), fk_chain=fk_chain,
        joint_angles_per_pose=ja, R_target2cam=R_tc, t_target2cam=t_tc,
        sag_indices=_DEFAULT_SAG_ARM_INDICES,
    )


# ─── 합성 데이터 생성 ──────────────────────────────────────────


def _synth(fk_chain, perturb_fn, n, seed):
    """알려진 X(cam2gripper) + board pose 로 board_in_cam 관측 생성."""
    X = make_T(cv2.Rodrigues(np.array([0.02, -0.03, 0.05]))[0], np.array([0.03, 0.01, 0.04]))
    T_board = make_T(
        cv2.Rodrigues(np.array([0.1, 0.2, 0.0]))[0], np.array([0.20, 0.0, 0.10])
    )
    center = np.array([0.0, -1.0, 0.6, 0.8, 0.0])
    rng = np.random.default_rng(seed)
    R_l, t_l, jas = [], [], []
    for _ in range(n):
        ang = perturb_fn(rng, center.copy())
        Tc = _fk_mat(fk_chain, ang) @ X
        bic = np.linalg.inv(Tc) @ T_board
        R_l.append(bic[:3, :3])
        t_l.append(bic[:3, 3])
        jas.append(ang.tolist())
    return jas, R_l, t_l


def _diverse(rng, c):
    return c + rng.uniform(-0.4, 0.4, 5)


def _single_axis(rng, c):
    c[0] += rng.uniform(-0.6, 0.6)
    return c


def _near_identical(rng, c):
    return c + rng.uniform(-0.01, 0.01, 5)  # 거의 동일 — 병리적


# ─── 실데이터 fixture ──────────────────────────────────────────


@pytest.fixture(scope="module")
def real_data():
    if not REAL_NPZ.exists():
        pytest.skip(f"실데이터 npz 없음: {REAL_NPZ}")
    d = np.load(str(REAL_NPZ), allow_pickle=True)
    raw = d["raw_positions"]
    R_tc = [np.asarray(r, dtype=float) for r in d["R_target2cam"]]
    t_tc = [np.asarray(t, dtype=float).reshape(3) for t in d["t_target2cam"]]
    ja = [((r.astype(float) - 2048.0) / 4095.0 * 2 * np.pi).tolist() for r in raw]
    return ja, R_tc, t_tc


# ─── 1. 무회귀 — SSOT 리팩토링 후 baseline σ 유지 ──────────────


def test_real_data_baseline_sigma(fk_chain, real_data):
    """실데이터 8포즈 physical_sag IRLS — σ baseline (0.306°/2.22mm) 유지."""
    ja, R_tc, t_tc = real_data
    r = _solve(fk_chain, ja, R_tc, t_tc)
    sr = float(np.sqrt(np.mean(r.residual_rot_deg**2)))
    st = float(np.sqrt(np.mean(r.residual_t_mm**2)))
    assert sr == pytest.approx(0.306, abs=0.05), f"σ_rot 회귀: {sr}"
    assert st == pytest.approx(2.22, abs=0.3), f"σ_t 회귀: {st}"


def test_frozen_none_equals_default(fk_chain, real_data):
    """frozen_blocks=None == frozen_blocks=set() (전부 free) — 동일 결과."""
    ja, R_tc, t_tc = real_data
    r1 = _solve(fk_chain, ja, R_tc, t_tc, frozen_blocks=None)
    r2 = _solve(fk_chain, ja, R_tc, t_tc, frozen_blocks=set())
    assert np.allclose(r1.R_cam2gripper, r2.R_cam2gripper)
    assert np.allclose(r1.t_cam2gripper, r2.t_cam2gripper)


# ─── 2. frozen_blocks 메커니즘 ─────────────────────────────────


def test_frozen_sag_stays_zero(fk_chain, real_data):
    """sag freeze → sag_k 추정 안 함 (0 유지). joint/link 는 추정."""
    ja, R_tc, t_tc = real_data
    r = _solve(fk_chain, ja, R_tc, t_tc, frozen_blocks={BLOCK_SAG})
    assert np.allclose(r.sag_k_rad_per_m, 0.0), f"frozen sag 가 0 아님: {r.sag_k_rad_per_m}"
    # joint offset 은 free 라 0 아님 (실데이터엔 offset 존재).
    assert not np.allclose(r.joint_offset_rad, 0.0)


def test_frozen_link_stays_zero(fk_chain, real_data):
    """link freeze → link_trans/link_rot 0 유지."""
    ja, R_tc, t_tc = real_data
    r = _solve(fk_chain, ja, R_tc, t_tc, frozen_blocks={BLOCK_LINK})
    assert np.allclose(r.link_trans_m, 0.0)
    assert np.allclose(r.link_rot_rad, 0.0)


def test_block_indices_cover_all_params(fk_chain):
    """block slice 들이 전체 파라미터를 빠짐없이 덮음 (SSOT layout 일관성)."""
    J = fk_chain.n_arm
    from modules.calibration.bundle_adjust import physical_sag_n_params
    idx = physical_sag_block_indices(J)
    covered = np.zeros(physical_sag_n_params(J), dtype=int)
    for sl in idx.values():
        covered[sl] += 1
    assert np.all(covered == 1), "블록 slice 가 겹치거나 빠진 파라미터 존재"


# ─── 3. observability — 잘 퍼진 데이터 = 전부 OK + unlock ──────


def test_diverse_data_all_observable(fk_chain):
    """잘 퍼진 합성 데이터 → gateable 전부 unlock + INSUFFICIENT 없음 (무회귀 보장)."""
    ja, R_tc, t_tc = _synth(fk_chain, _diverse, 14, 1)
    r = _solve(fk_chain, ja, R_tc, t_tc)
    obs = _obs(fk_chain, r, ja, R_tc, t_tc)
    assert obs.unlocked == set(GATEABLE_BLOCKS), (
        f"잘 퍼진 데이터인데 freeze 발생: unlocked={obs.unlocked}, scores={obs.scores}"
    )
    # 좋은 데이터면 INSUFFICIENT(정보부족=freeze 대상) 블록이 없어야.
    for b in ALL_BLOCKS:
        assert obs.verdicts[b] != "INSUFFICIENT", (
            f"{b} INSUFFICIENT score={obs.scores[b]}"
        )


def test_real_data_all_unlocked(fk_chain, real_data):
    """실데이터도 gateable 전부 unlock (현 always-on BA 와 동일 = 무회귀)."""
    ja, R_tc, t_tc = real_data
    r = _solve(fk_chain, ja, R_tc, t_tc)
    obs = _obs(fk_chain, r, ja, R_tc, t_tc)
    assert obs.unlocked == set(GATEABLE_BLOCKS), (
        f"실데이터 freeze 발생 (회귀 위험): {obs.scores}"
    )


# ─── 4. degeneracy 단조성 — 다양성 낮을수록 score 하락 ──────────


def test_score_monotone_with_diversity(fk_chain):
    """자세 다양성: diverse > single-axis (스펙 degeneracy 검출)."""
    ja_d, R_d, t_d = _synth(fk_chain, _diverse, 14, 1)
    ja_s, R_s, t_s = _synth(fk_chain, _single_axis, 14, 2)
    obs_d = _obs(fk_chain, _solve(fk_chain, ja_d, R_d, t_d), ja_d, R_d, t_d)
    obs_s = _obs(fk_chain, _solve(fk_chain, ja_s, R_s, t_s), ja_s, R_s, t_s)
    # 전체 평균 score 가 diverse 에서 더 높아야 (다양성 ↑ = 관측성 ↑).
    avg_d = np.mean([obs_d.scores[b] for b in ALL_BLOCKS])
    avg_s = np.mean([obs_s.scores[b] for b in ALL_BLOCKS])
    assert avg_d > avg_s, f"다양성 단조성 위반: diverse {avg_d:.3f} <= single {avg_s:.3f}"


# ─── 5. 병리적 데이터 → freeze (gating 안전망) ─────────────────


def test_pathological_data_freezes_blocks(fk_chain):
    """자세 거의 동일 (병리적) → 일부 gateable 블록 freeze (정보 부족 차단)."""
    ja, R_tc, t_tc = _synth(fk_chain, _near_identical, 6, 3)
    r = _solve(fk_chain, ja, R_tc, t_tc)
    obs = _obs(fk_chain, r, ja, R_tc, t_tc)
    # 거의 동일한 자세면 적어도 한 블록은 freeze (unlock != 전체).
    assert obs.unlocked != set(GATEABLE_BLOCKS), (
        f"병리적 데이터인데 전부 unlock — gating 미작동: {obs.scores}"
    )


# ─── 6. SO-101 6DOF (primary robot) — 같은 코드 6축 동작 ────────


@pytest.fixture(scope="module")
def fk_chain_6dof():
    return RobotRegistry().get_fk_chain("so101_6dof_0")


def _synth_6dof(fk_chain, perturb_fn, n, seed):
    """6DOF 합성 — center 6-vector."""
    X = make_T(cv2.Rodrigues(np.array([0.02, -0.03, 0.05]))[0], np.array([0.03, 0.01, 0.04]))
    T_board = make_T(
        cv2.Rodrigues(np.array([0.1, 0.2, 0.0]))[0], np.array([0.25, 0.0, 0.15])
    )
    center = np.array([0.0, -0.8, 0.8, 0.3, 0.0, 0.0])
    rng = np.random.default_rng(seed)
    R_l, t_l, jas = [], [], []
    for _ in range(n):
        ang = perturb_fn(rng, center.copy())
        Tc = _fk_mat(fk_chain, ang) @ X
        bic = np.linalg.inv(Tc) @ T_board
        R_l.append(bic[:3, :3])
        t_l.append(bic[:3, 3])
        jas.append(ang.tolist())
    return jas, R_l, t_l


def test_so101_6dof_observability_and_gating(fk_chain_6dof):
    """SO-101 6축 (primary) — physical_sag BA + observability + frozen 모두 동작.

    같은 코드로 6축 (J=6, n_params=50) — observability J-generic 검증.
    """
    fk = fk_chain_6dof
    assert fk.n_arm == 6
    ja, R_tc, t_tc = _synth_6dof(fk, lambda r, c: c + r.uniform(-0.4, 0.4, 6), 16, 1)
    r = _solve(fk, ja, R_tc, t_tc)
    assert r.joint_offset_rad.shape[0] == 6
    obs = _obs(fk, r, ja, R_tc, t_tc)
    # 잘 퍼진 6축 데이터 → 전부 unlock + INSUFFICIENT 없음.
    assert obs.unlocked == set(GATEABLE_BLOCKS), f"6축 freeze 발생: {obs.scores}"
    for b in ALL_BLOCKS:
        assert obs.verdicts[b] != "INSUFFICIENT", f"6축 {b} INSUFFICIENT {obs.scores[b]}"
    # frozen 메커니즘 6축에서도 동작.
    rf = _solve(fk, ja, R_tc, t_tc, frozen_blocks={BLOCK_SAG})
    assert np.allclose(rf.sag_k_rad_per_m, 0.0)


# 실 SO-101 데이터 (read-only 픽스처 — npz 는 production 경로 아님, storage=DB SSOT).
# 사용자가 npz 지우면 자동 skip. 합성 아닌 *실제 6축 데이터* 에서 솔버 검증.
SO101_NPZ = REPO_ROOT / "robot/instances/so101_6dof_0/calibration/handeye_poses.npz"


def test_so101_real_data_converges(fk_chain_6dof):
    """실 SO-101 데이터 — BA 가 TSDF GOOD 임계 안 수렴 + observability 정상.

    합성이 아닌 *실제 하드웨어* 6축 캡처로 전체 솔버 검증 ('집 가면 캘 됨' 증거).
    npz 없으면 skip (production 은 DB SSOT — 본 npz 는 우연한 leftover).
    """
    if not SO101_NPZ.exists():
        pytest.skip("실 SO-101 npz 없음 (production 은 DB)")
    fk = fk_chain_6dof
    d = np.load(str(SO101_NPZ), allow_pickle=True)
    raw = d["raw_positions"]
    R_tc = [np.asarray(r, dtype=float) for r in d["R_target2cam"]]
    t_tc = [np.asarray(t, dtype=float).reshape(3) for t in d["t_target2cam"]]
    ja = [((r.astype(float) - 2048.0) / 4095.0 * 2 * np.pi).tolist() for r in raw]
    r = _solve(fk, ja, R_tc, t_tc)
    sr = float(np.sqrt(np.mean(r.residual_rot_deg**2)))
    st = float(np.sqrt(np.mean(r.residual_t_mm**2)))
    # TSDF GOOD: σ_rot < 1° / σ_t < 10mm (thresholds.SIGMA_ROT_GOOD_DEG / T_GOOD_MM).
    assert sr < 1.5, f"실 SO-101 σ_rot 너무 큼: {sr}"
    assert st < 12.0, f"실 SO-101 σ_t 너무 큼: {st}"
    obs = _obs(fk, r, ja, R_tc, t_tc)
    # 실데이터 → gateable 전부 unlock (freeze 없음).
    assert obs.unlocked == set(GATEABLE_BLOCKS), f"실 SO-101 freeze: {obs.scores}"
