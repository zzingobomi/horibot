"""물리 기반 sag 모델 검증.

이전 sin/cos basis 모델은:
  - random hold-out 1.05× ✓
  - 연속 split (J2 큰 sag 영역으로 extrapolate) 6.6× ✗ — 폭주
  → "중력 처짐 없다"가 아니라 "sin/cos basis가 잘못된 형태" 진단.

물리 기반 모델 — *모멘트 암 ∝ 처짐*:
  τ_J = ((ee_pos - joint_origin) × gravity_dir) · joint_axis   [base frame]
  sag_J = k_J * τ_J   (k = 1/stiffness, BA가 푸는 변수)

핵심: τ는 *전체 자세*의 함수 (ee 위치는 J3/J4/J5 자세에도 의존)이므로 같은 J2
각도라도 팔 쭉 펴면 토크 ↑, 접으면 ↓. sin/cos basis가 못 잡던 자세 결합을
*물리적으로* 표현.

자유도 2개 (k_J2, k_J3)로 검증. 세 단계 진단:
  [F] k reg sweep — 안정성
  [G] x0 noise robustness — global min 확인
  [H] J2 연속 split — extrapolation. **여기가 진짜 시험**
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))

from diag_gravity_sag import (  # noqa: E402
    EE_ORIGIN,
    JOINT_AXES,
    JOINT_ORIGINS,
    axis_angle_to_R,
    fk_chain,
    fit_ba_with_sag,
    load_data,
    mean_rotation,
    rpy_to_R,
    sigma_vs_mean,
)
from modules.calibration.se3 import make_T  # noqa: E402


# ─── 물리 기반 fk + torque ─────────────────────────────────────────────────


def fk_chain_with_axes(angles, link_trans, link_rot):
    """fk_chain + 각 joint의 origin/axis (base frame).

    중력 토크 계산용 — joint i의 origin은 *그 joint의 회전 적용 전*의 base 좌표,
    axis는 base에서 본 회전축 방향.
    """
    T = np.eye(4)
    joint_origins_base = np.zeros((5, 3))
    joint_axes_base = np.zeros((5, 3))
    for i in range(5):
        T_o = np.eye(4)
        T_o[:3, :3] = rpy_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        # 이 시점에서 T가 joint i frame in base (회전 적용 전)
        joint_origins_base[i] = T[:3, 3]
        joint_axes_base[i] = T[:3, :3] @ JOINT_AXES[i]
        # joint i 회전 적용
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], angles[i])
        T = T @ T_r
    T_ee = np.eye(4)
    T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return (
        Tee[:3, :3].copy(),
        Tee[:3, 3].copy(),
        joint_origins_base,
        joint_axes_base,
    )


GRAVITY_DIR = np.array([0.0, 0.0, -1.0])  # base frame


def gravity_torque_lumped(ee_pos, joint_origin, joint_axis):
    """ee에 lumped mass 가정. joint에 작용하는 중력 토크 (sign + magnitude).

    τ = (r × g_dir) · axis   where r = ee - joint_origin (모멘트 암 벡터)
    Units: r은 m, g_dir은 unit (-z), axis는 unit → τ는 m. k(=1/stiffness) 곱하면 rad.
    """
    r = ee_pos - joint_origin
    return float(np.dot(np.cross(r, GRAVITY_DIR), joint_axis))


def apply_physical_sag(angles, link_trans, link_rot, k_stiff):
    """k_stiff: (2,) — k_J2, k_J3. sag = k * τ.

    1차 근사: commanded 자세에서 τ 계산. (sag가 작으니 일차 근사로 충분)
    """
    _, ee_pos, joint_origs, joint_axes = fk_chain_with_axes(
        angles, link_trans, link_rot
    )
    tau_J2 = gravity_torque_lumped(ee_pos, joint_origs[1], joint_axes[1])
    tau_J3 = gravity_torque_lumped(ee_pos, joint_origs[2], joint_axes[2])
    a = angles.copy()
    a[1] += k_stiff[0] * tau_J2
    a[2] += k_stiff[1] * tau_J3
    return a


# ─── 물리 sag BA ───────────────────────────────────────────────────────────


def fit_ba_with_physical_sag(
    angles_all,
    R_tc_all,
    t_tc_all,
    *,
    use_link_offsets: bool = True,
    k_reg: float = 0.0,
    joint_offset_reg: float = 0.5,
    link_trans_reg: float = 1.0,
    link_rot_reg: float = 1.0,
    k_init_noise: np.ndarray | None = None,
):
    """41 DOF 확장 BA + k_J2, k_J3 = 43 DOF.

    변수 layout:
      [0:5]    joint_offset
      [5:20]   link_trans (생략 시 0 고정)
      [20:35]  link_rot
      [35:37]  k_stiff (J2, J3)
      [37:40]  rod
      [40:43]  t
    """
    N = len(angles_all)
    angles_arr = np.array(angles_all, dtype=np.float64)
    R_tc_list = [np.asarray(R) for R in R_tc_all]
    t_tc_list = [np.asarray(t).reshape(3) for t in t_tc_all]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    # cv2 seed
    zero = np.zeros((5, 3))
    R_gb_seed, t_gb_seed = [], []
    for a in angles_arr:
        R, t = fk_chain(a, zero, zero)
        R_gb_seed.append(R)
        t_gb_seed.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb_seed, t_gb_seed, R_tc_list,
        [t.reshape(3, 1) for t in t_tc_list],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    rod_seed, _ = cv2.Rodrigues(R_seed)
    t_seed_v = np.asarray(t_seed).reshape(3)

    n_off = 5
    n_lt = 15 if use_link_offsets else 0
    n_lr = 15 if use_link_offsets else 0
    n_k = 2

    def unpack(x):
        i = 0
        off = x[i:i + n_off]; i += n_off
        if use_link_offsets:
            lt = x[i:i + n_lt].reshape(5, 3); i += n_lt
            lr = x[i:i + n_lr].reshape(5, 3); i += n_lr
        else:
            lt = np.zeros((5, 3)); lr = np.zeros((5, 3))
        k_stiff = x[i:i + n_k]; i += n_k
        rod = x[i:i + 3]; i += 3
        t_x = x[i:i + 3]
        return off, lt, lr, k_stiff, rod, t_x

    def compute_T(x):
        off, lt, lr, k_stiff, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out = []
        for i in range(N):
            a_corr = apply_physical_sag(angles_arr[i] + off, lt, lr, k_stiff)
            R_gb, t_gb = fk_chain(a_corr, lt, lr)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual(x):
        off, lt, lr, k_stiff, _, _ = unpack(x)
        T_list = compute_T(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        n_reg = n_off + n_lt + n_lr + n_k
        res = np.empty(6 * N + n_reg, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i:6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3:6 * (i + 1)] = T[:3, 3] - mean_pos
        k = 6 * N
        res[k:k + n_off] = joint_offset_reg * off; k += n_off
        if use_link_offsets:
            res[k:k + n_lt] = link_trans_reg * lt.flatten(); k += n_lt
            res[k:k + n_lr] = link_rot_reg * lr.flatten(); k += n_lr
        res[k:k + n_k] = k_reg * k_stiff
        return res

    n_x = n_off + n_lt + n_lr + n_k + 6
    x0 = np.zeros(n_x)
    if k_init_noise is not None:
        idx_k = n_off + n_lt + n_lr
        x0[idx_k:idx_k + n_k] = k_init_noise
    x0[-6:-3] = rod_seed.flatten()
    x0[-3:] = t_seed_v

    result = least_squares(
        residual, x0, method="lm", max_nfev=5000, xtol=1e-11, ftol=1e-11
    )
    off, lt, lr, k_stiff, rod, t_x = unpack(result.x)
    T_list = compute_T(result.x)
    sr, st, rot_per, t_per = sigma_vs_mean(T_list)

    # k 값을 *물리적 의미*로 해석 — 자세별 최대 sag (deg)
    max_tau_J2 = 0.0
    max_tau_J3 = 0.0
    for a in angles_arr:
        _, ee_pos, jo, ja = fk_chain_with_axes(a + off, lt, lr)
        t2 = abs(gravity_torque_lumped(ee_pos, jo[1], ja[1]))
        t3 = abs(gravity_torque_lumped(ee_pos, jo[2], ja[2]))
        max_tau_J2 = max(max_tau_J2, t2)
        max_tau_J3 = max(max_tau_J3, t3)
    max_sag_J2_deg = float(np.degrees(abs(k_stiff[0]) * max_tau_J2))
    max_sag_J3_deg = float(np.degrees(abs(k_stiff[1]) * max_tau_J3))

    return {
        "sigma_rot": sr,
        "sigma_t": st,
        "rot_per_pose_deg": rot_per,
        "t_per_pose_mm": t_per,
        "joint_offset_deg": np.degrees(off),
        "link_trans_mm_max": float(np.max(np.abs(lt)) * 1000.0)
        if use_link_offsets else 0.0,
        "link_rot_deg_max": float(np.degrees(np.max(np.abs(lr))))
        if use_link_offsets else 0.0,
        "k_stiff": k_stiff,
        "max_sag_J2_deg": max_sag_J2_deg,
        "max_sag_J3_deg": max_sag_J3_deg,
        "x": result.x,
        "cost": float(result.cost),
        "dof": n_x,
    }


# ─── 진단 [D'] 시나리오 비교 ───────────────────────────────────────────────


def compare_scenarios(angles_all, R_tc_all, t_tc_all):
    print("[D'] 모델 비교 — sin/cos vs 물리 기반")
    print(f"  {'시나리오':<32s} {'σ_rot':>7s} {'σ_t':>8s} {'DOF':>4s}  "
          f"{'sag 최대 (deg)':>14s}  {'link_t_max':>10s}")

    # baseline (현 production)
    r = fit_ba_with_sag(
        angles_all, R_tc_all, t_tc_all,
        sag_mode="none", use_link_offsets=True,
    )
    print(f"  {'(4) link on, sag off [prod]':<32s} "
          f"{r['sigma_rot']:>6.3f}° {r['sigma_t']:>6.2f}mm "
          f"{r['dof']:>4d}  {'—':>14s}  {r['link_trans_mm_max']:>8.1f}mm")

    # sin/cos sag (이전)
    r = fit_ba_with_sag(
        angles_all, R_tc_all, t_tc_all,
        sag_mode="j23_sincos", use_link_offsets=True,
    )
    print(f"  {'(6) link on, sag sincos':<32s} "
          f"{r['sigma_rot']:>6.3f}° {r['sigma_t']:>6.2f}mm "
          f"{r['dof']:>4d}  {'—':>14s}  {r['link_trans_mm_max']:>8.1f}mm")

    # 물리 기반
    r_phys = fit_ba_with_physical_sag(angles_all, R_tc_all, t_tc_all)
    print(f"  {'(P) link on, sag physical':<32s} "
          f"{r_phys['sigma_rot']:>6.3f}° {r_phys['sigma_t']:>6.2f}mm "
          f"{r_phys['dof']:>4d}  "
          f"J2={r_phys['max_sag_J2_deg']:+.2f}/J3={r_phys['max_sag_J3_deg']:+.2f}  "
          f"{r_phys['link_trans_mm_max']:>8.1f}mm")
    print(f"      k_stiff = {r_phys['k_stiff']}  (rad / (m·g_unit))")

    # link off 비교
    r_phys_nolink = fit_ba_with_physical_sag(
        angles_all, R_tc_all, t_tc_all, use_link_offsets=False
    )
    print(f"  {'(P-) link OFF, sag physical':<32s} "
          f"{r_phys_nolink['sigma_rot']:>6.3f}° {r_phys_nolink['sigma_t']:>6.2f}mm "
          f"{r_phys_nolink['dof']:>4d}  "
          f"J2={r_phys_nolink['max_sag_J2_deg']:+.2f}/J3={r_phys_nolink['max_sag_J3_deg']:+.2f}  "
          f"{r_phys_nolink['link_trans_mm_max']:>8.1f}mm")
    print(f"      k_stiff = {r_phys_nolink['k_stiff']}")
    print()


# ─── [F] k_reg sweep ───────────────────────────────────────────────────────


def sweep_k_reg(angles_all, R_tc_all, t_tc_all):
    print("[F] k_reg sweep (link on, 물리 sag)")
    print(f"  {'k_reg':>7s}  {'σ_rot':>7s}  {'σ_t':>7s}  "
          f"{'k_J2':>10s}  {'k_J3':>10s}  "
          f"{'max sag(deg)':>14s}  {'link_t_max':>10s}")
    regs = [0.0, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
    rows = []
    for reg in regs:
        r = fit_ba_with_physical_sag(
            angles_all, R_tc_all, t_tc_all,
            use_link_offsets=True, k_reg=reg,
        )
        print(
            f"  {reg:>7.3f}  {r['sigma_rot']:>6.3f}°  {r['sigma_t']:>6.2f}mm  "
            f"{r['k_stiff'][0]:>+9.5f}  {r['k_stiff'][1]:>+9.5f}  "
            f"J2={r['max_sag_J2_deg']:+.2f}/J3={r['max_sag_J3_deg']:+.2f}  "
            f"{r['link_trans_mm_max']:>8.1f}mm"
        )
        rows.append(r["k_stiff"])
    rows = np.array(rows)
    mask = np.array([(0.0 <= rr <= 1.0) for rr in regs])
    if mask.sum() >= 2:
        stable = rows[mask]
        spread = stable.max(axis=0) - stable.min(axis=0)
        # rad units small. percentage 단위로 평가
        mean_abs = np.mean(np.abs(stable), axis=0)
        rel_spread = spread / (mean_abs + 1e-9)
        print(f"  → reg 0~1.0 범위 k 값 변동: "
              f"k_J2 {spread[0]:.5f} ({rel_spread[0]*100:.1f}%), "
              f"k_J3 {spread[1]:.5f} ({rel_spread[1]*100:.1f}%)")
        ok = all(s < 0.5 for s in rel_spread)  # 50% 이내 변동
        print(f"  → {'✓ robust' if ok else '✗ unstable'} (기준 변동 < 50%)")
    print()


# ─── [G] 초기값 noise ──────────────────────────────────────────────────────


def init_robustness(angles_all, R_tc_all, t_tc_all, n_trials: int = 8):
    print(f"[G] 초기값 robustness (k x0 noise, {n_trials} trials)")
    rng_master = np.random.default_rng(123)
    sag_results = []
    sigma_results = []
    print(f"  {'trial':>5s}  {'init k':>22s}  {'σ_rot':>7s}  "
          f"{'σ_t':>7s}  {'k final':>24s}")
    for trial in range(n_trials):
        rng = np.random.default_rng(200 + trial)
        # k_J2, k_J3는 작은 양수 (~0.01~0.1) 예상. noise는 ±0.1 정도 범위
        noise = rng.uniform(-0.1, 0.1, 2)
        r = fit_ba_with_physical_sag(
            angles_all, R_tc_all, t_tc_all,
            use_link_offsets=True, k_init_noise=noise,
        )
        sag_results.append(r["k_stiff"])
        sigma_results.append((r["sigma_rot"], r["sigma_t"]))
        print(
            f"  {trial:>5d}  ({noise[0]:+.3f}, {noise[1]:+.3f})  "
            f"{r['sigma_rot']:>6.3f}°  {r['sigma_t']:>6.2f}mm  "
            f"({r['k_stiff'][0]:+.5f}, {r['k_stiff'][1]:+.5f})"
        )
    sag_results = np.array(sag_results)
    sigma_results = np.array(sigma_results)
    std_k = sag_results.std(axis=0)
    mean_k = np.mean(np.abs(sag_results), axis=0)
    rel_std = std_k / (mean_k + 1e-9)
    print(f"  → k_J2 std={std_k[0]:.6f} ({rel_std[0]*100:.2f}%)  "
          f"k_J3 std={std_k[1]:.6f} ({rel_std[1]*100:.2f}%)")
    print(f"  → σ_rot std: {sigma_results[:,0].std():.4f}°  "
          f"σ_t std: {sigma_results[:,1].std():.3f}mm")
    ok = all(rs < 0.05 for rs in rel_std)  # 5% 이내
    print(f"  → {'✓ global minimum' if ok else '✗ multi-modal cost'} "
          f"(기준 std < 5%)")
    print()


# ─── [H] J2 연속 split — 진짜 시험 ─────────────────────────────────────────


def continuous_split(angles_all, R_tc_all, t_tc_all):
    print("[H] J2 각도 기준 연속 split (extrapolation test)")
    n = len(angles_all)
    order = np.argsort(angles_all[:, 1])
    J2_sorted = np.degrees(angles_all[order, 1])
    print(f"  전체 J2 범위: [{J2_sorted[0]:.1f}, {J2_sorted[-1]:.1f}]°")

    splits = [
        ("lower 70% → upper 30% (큰 sag 영역 extrapolate)",
         order[: int(n * 0.7)], order[int(n * 0.7):]),
        ("upper 70% → lower 30% (작은 sag 영역 extrapolate)",
         order[int(n * 0.3):], order[: int(n * 0.3)]),
        ("middle 60% → edges 40% (양쪽 extrapolate)",
         order[int(n * 0.2):int(n * 0.8)],
         np.concatenate([order[: int(n * 0.2)], order[int(n * 0.8):]])),
    ]

    for label, tr_idx, te_idx in splits:
        ang_tr = angles_all[tr_idx]
        R_tr = [R_tc_all[i] for i in tr_idx]
        t_tr = [t_tc_all[i] for i in tr_idx]
        ang_te = angles_all[te_idx]
        R_te = [R_tc_all[i] for i in te_idx]
        t_te = [t_tc_all[i] for i in te_idx]
        j2_tr_deg = np.degrees(ang_tr[:, 1])
        j2_te_deg = np.degrees(ang_te[:, 1])
        print(f"\n  {label}")
        print(f"    train J2 [{j2_tr_deg.min():.1f}, {j2_tr_deg.max():.1f}]° (n={len(tr_idx)})  "
              f"test [{j2_te_deg.min():.1f}, {j2_te_deg.max():.1f}]° (n={len(te_idx)})")

        for mode_label, fitter in [
            ("sag off (현 prod)", "none"),
            ("sag sincos      ", "sincos"),
            ("sag physical    ", "physical"),
        ]:
            if fitter == "none":
                r_tr = fit_ba_with_sag(
                    ang_tr, R_tr, t_tr,
                    sag_mode="none", use_link_offsets=True,
                )
            elif fitter == "sincos":
                r_tr = fit_ba_with_sag(
                    ang_tr, R_tr, t_tr,
                    sag_mode="j23_sincos", use_link_offsets=True,
                )
            else:  # physical
                r_tr = fit_ba_with_physical_sag(
                    ang_tr, R_tr, t_tr, use_link_offsets=True,
                )

            sr_tr, st_tr, sr_te, st_te = _eval_split(
                r_tr["x"], fitter, ang_tr, R_tr, t_tr, ang_te, R_te, t_te,
            )
            rr = sr_te / sr_tr if sr_tr > 1e-6 else float("inf")
            rt = st_te / st_tr if st_tr > 1e-6 else float("inf")
            print(
                f"    {mode_label}: "
                f"train σ=({sr_tr:.3f}°, {st_tr:.2f}mm)  "
                f"test σ=({sr_te:.3f}°, {st_te:.2f}mm)  "
                f"ratio rot={rr:.2f}× t={rt:.2f}×"
            )
    print()


def _eval_split(x_tr, fitter, ang_tr, R_tr, t_tr, ang_te, R_te, t_te):
    """x_tr (train으로 fit한 변수)를 test에 적용해 σ 계산."""
    use_link = True
    n_off = 5
    n_lt = 15; n_lr = 15
    if fitter == "physical":
        n_sag = 2
    elif fitter == "sincos":
        n_sag = 4
    else:
        n_sag = 0

    i = 0
    off = x_tr[i:i + n_off]; i += n_off
    lt = x_tr[i:i + n_lt].reshape(5, 3); i += n_lt
    lr = x_tr[i:i + n_lr].reshape(5, 3); i += n_lr
    sag = x_tr[i:i + n_sag] if n_sag else np.zeros(0); i += n_sag
    rod = x_tr[i:i + 3]; i += 3
    t_x = x_tr[i:i + 3]
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)

    def apply_correction(a):
        a2 = a.copy()
        if fitter == "sincos":
            a2[1] += sag[0] * np.sin(a[1]) + sag[1] * np.cos(a[1])
            abs_J3 = a[1] + a[2]
            a2[2] += sag[2] * np.sin(abs_J3) + sag[3] * np.cos(abs_J3)
        elif fitter == "physical":
            _, ee_pos, jo, ja = fk_chain_with_axes(a, lt, lr)
            tau_J2 = gravity_torque_lumped(ee_pos, jo[1], ja[1])
            tau_J3 = gravity_torque_lumped(ee_pos, jo[2], ja[2])
            a2[1] += sag[0] * tau_J2
            a2[2] += sag[1] * tau_J3
        return a2

    def T_list_of(angs, R_list, t_list):
        out = []
        for a, R, t in zip(angs, R_list, t_list):
            T_tc = make_T(R, np.asarray(t).reshape(3))
            a_corr = apply_correction(a + off)
            R_gb, t_gb = fk_chain(a_corr, lt, lr)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc)
        return out

    T_train = T_list_of(ang_tr, R_tr, t_tr)
    T_test = T_list_of(ang_te, R_te, t_te)
    pos_tr = np.array([T[:3, 3] for T in T_train])
    mean_pos = pos_tr.mean(axis=0)
    mean_R_v = mean_rotation([T[:3, :3] for T in T_train])
    sr_tr, st_tr, _, _ = sigma_vs_mean(T_train, mean_pos, mean_R_v)
    sr_te, st_te, _, _ = sigma_vs_mean(T_test, mean_pos, mean_R_v)
    return sr_tr, st_tr, sr_te, st_te


# ─── main ─────────────────────────────────────────────────────────────────


def main():
    print("=" * 78)
    print("물리 기반 sag 모델 검증 — 모멘트 암 ∝ 처짐")
    print("=" * 78)
    n, angles_all, R_tc_all, t_tc_all, _ = load_data()
    print(f"포즈 {n}개\n")

    compare_scenarios(angles_all, R_tc_all, t_tc_all)
    sweep_k_reg(angles_all, R_tc_all, t_tc_all)
    init_robustness(angles_all, R_tc_all, t_tc_all)
    continuous_split(angles_all, R_tc_all, t_tc_all)

    print("=" * 78)
    print("판정 가이드:")
    print("  [D'] 물리 sag가 sin/cos보다 σ 비슷 + DOF 적으면 → 물리 신호 진짜")
    print("  [F]  k 값이 reg에 robust → 안정")
    print("  [G]  x0 noise에도 같은 minimum → cost surface OK")
    print("  [H]  ★ 연속 split도 ratio < 2× → extrapolation 작동, 통합 OK")
    print("    sin/cos는 lower→upper에서 6.6× 폭주. physical이 어떻게 나오나가 핵심.")
    print("=" * 78)


if __name__ == "__main__":
    main()
