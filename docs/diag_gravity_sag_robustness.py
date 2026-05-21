"""중력 처짐 가설 robustness 검증.

[diag_gravity_sag.py](diag_gravity_sag.py)에서 발견한 sag 효과(σ_rot 1.30°→0.63°)
가 진짜 system 신호인지 vs 좁은 데이터 + 자유도 4개로 인한 artifact인지.

세 가지 진단:
  [F] sag_reg sweep — regularization 강도 변화에 sag 파라미터가 안정적인가
  [G] 초기값 robustness — sag x0에 noise 줘도 같은 minimum 수렴하는가
  [H] J2 기반 *연속* split — train/test 자세 분포가 안 겹치는 가혹 test에서도 generalize

판정 기준:
  [F] sag_reg 0.01~1.0 범위에서 sag(deg) 값 ±0.3° 안 → robust
  [G] x0 noise ±2°에서 수렴값 std < 0.1° → global minimum
  [H] 연속 split ratio < 2.0× → extrapolation 작동 (random split 1.05× 대비 완화 기준)
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

# 기존 진단 스크립트에서 재사용
from diag_gravity_sag import (  # noqa: E402
    fit_ba_with_sag,
    fk_chain,
    load_data,
    mean_rotation,
    sigma_vs_mean,
)
from modules.calibration.se3 import make_T  # noqa: E402


# ─── [F] sag_reg sweep ────────────────────────────────────────────────────


def sweep_sag_reg(angles_all, R_tc_all, t_tc_all):
    """sag_reg 변화에 sag 파라미터가 안정적인가."""
    print("[F] sag_reg sweep (link on, sag J2+J3, 다른 인자는 기본)")
    print(f"  {'sag_reg':>8s}  {'σ_rot':>7s}  {'σ_t':>7s}  "
          f"{'sag J2 sin/cos':>16s}  {'sag J3 sin/cos':>16s}  "
          f"{'link_t_max':>10s}  {'jo max':>7s}")
    regs = [0.0, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0]
    rows = []
    for reg in regs:
        r = fit_ba_with_sag(
            angles_all, R_tc_all, t_tc_all,
            sag_mode="j23_sincos", use_link_offsets=True, sag_reg=reg,
        )
        sag_deg = np.degrees(r["sag_params"])
        jo_max = np.max(np.abs(r["joint_offset_deg"]))
        print(
            f"  {reg:>8.3f}  {r['sigma_rot']:>6.3f}°  {r['sigma_t']:>6.2f}mm  "
            f"  ({sag_deg[0]:+.2f}, {sag_deg[1]:+.2f})  "
            f"  ({sag_deg[2]:+.2f}, {sag_deg[3]:+.2f})  "
            f"  {r['link_trans_mm_max']:>8.1f}mm  "
            f"  {jo_max:>5.2f}°"
        )
        rows.append(sag_deg)
    rows = np.array(rows)  # (n_reg, 4)
    # reg 0.01~1.0 범위에서 안정성 — 그 안의 행 추리기
    mask = np.array([(0.01 <= r <= 1.0) for r in regs])
    if mask.sum() >= 2:
        stable = rows[mask]
        spread = stable.max(axis=0) - stable.min(axis=0)
        print(f"  → reg 0.01~1.0 범위 sag 값 spread (max-min): "
              f"J2 sin {spread[0]:.2f}°, J2 cos {spread[1]:.2f}°, "
              f"J3 sin {spread[2]:.2f}°, J3 cos {spread[3]:.2f}°")
        ok = all(s < 0.3 for s in spread)
        print(f"  → {'✓ robust' if ok else '✗ unstable'} (기준 spread < 0.3°)")
    print()


# ─── [G] 초기값 noise robustness ──────────────────────────────────────────


def init_robustness(angles_all, R_tc_all, t_tc_all, n_trials: int = 8):
    """sag x0에 noise를 주고 같은 minimum으로 수렴하는지."""
    print(f"[G] 초기값 robustness (sag x0 ±2° noise, {n_trials} trials)")
    print(f"  {'trial':>5s}  {'noise(deg)':>20s}  {'σ_rot':>7s}  "
          f"{'σ_t':>7s}  {'sag final (deg)':>30s}")
    sag_results = []
    sigma_results = []
    for trial in range(n_trials):
        rng = np.random.default_rng(100 + trial)
        # 초기값 noise — 직접 x0 조작 위해 fit 호출 후 다른 경로 필요
        # fit_ba_with_sag는 x0=zeros 고정이라 monkey-patch 어려움 → 같은 식으로
        # least_squares 재구현 대신 wrapper 활용: noise를 angle bias로 흘려보내기
        # 정확한 robustness 테스트를 위해 fit_ba_with_sag를 약간 수정한 함수로 재호출
        r = _fit_with_init_noise(
            angles_all, R_tc_all, t_tc_all, rng=rng,
        )
        sag_deg = np.degrees(r["sag_params"])
        noise_used = r["init_noise_deg"]
        sag_results.append(sag_deg)
        sigma_results.append((r["sigma_rot"], r["sigma_t"]))
        print(
            f"  {trial:>5d}  ({noise_used[0]:+.2f}, {noise_used[1]:+.2f}, "
            f"{noise_used[2]:+.2f}, {noise_used[3]:+.2f})  "
            f"{r['sigma_rot']:>6.3f}°  {r['sigma_t']:>6.2f}mm  "
            f"({sag_deg[0]:+.2f}, {sag_deg[1]:+.2f}, "
            f"{sag_deg[2]:+.2f}, {sag_deg[3]:+.2f})"
        )
    sag_results = np.array(sag_results)
    sigma_results = np.array(sigma_results)
    std_sag = sag_results.std(axis=0)
    std_sigma = sigma_results.std(axis=0)
    print(f"  → sag final std: "
          f"J2 sin {std_sag[0]:.3f}°, J2 cos {std_sag[1]:.3f}°, "
          f"J3 sin {std_sag[2]:.3f}°, J3 cos {std_sag[3]:.3f}°")
    print(f"  → σ_rot std: {std_sigma[0]:.4f}°  σ_t std: {std_sigma[1]:.3f}mm")
    ok = all(s < 0.1 for s in std_sag)
    print(f"  → {'✓ global minimum' if ok else '✗ multi-modal cost'} (기준 std < 0.1°)")
    print()


def _fit_with_init_noise(angles_all, R_tc_all, t_tc_all, rng):
    """fit_ba_with_sag 와 동일한 BA지만 x0 sag 부분에 noise 추가."""
    from scipy.optimize import least_squares  # local import

    N = len(angles_all)
    angles_arr = np.array(angles_all, dtype=np.float64)
    R_tc_list = [np.asarray(R) for R in R_tc_all]
    t_tc_list = [np.asarray(t).reshape(3) for t in t_tc_all]
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    # seed
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

    n_off, n_lt, n_lr, n_sag = 5, 15, 15, 4

    # noise: ±2° = ±0.0349 rad
    noise_deg = rng.uniform(-2.0, 2.0, n_sag)
    noise_rad = np.deg2rad(noise_deg)

    def unpack(x):
        i = 0
        off = x[i:i + n_off]; i += n_off
        lt = x[i:i + n_lt].reshape(5, 3); i += n_lt
        lr = x[i:i + n_lr].reshape(5, 3); i += n_lr
        sag = x[i:i + n_sag]; i += n_sag
        rod = x[i:i + 3]; i += 3
        t_x = x[i:i + 3]
        return off, lt, lr, sag, rod, t_x

    def apply_sag(a, sag):
        a2 = a.copy()
        a2[1] += sag[0] * np.sin(a[1]) + sag[1] * np.cos(a[1])
        abs_J3 = a[1] + a[2]
        a2[2] += sag[2] * np.sin(abs_J3) + sag[3] * np.cos(abs_J3)
        return a2

    def compute_T(x):
        off, lt, lr, sag, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out = []
        for i in range(N):
            a_corr = apply_sag(angles_arr[i] + off, sag)
            R_gb, t_gb = fk_chain(a_corr, lt, lr)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual(x):
        off, lt, lr, sag, _, _ = unpack(x)
        T_list = compute_T(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        res = np.empty(6 * N + n_off + n_lt + n_lr + n_sag, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i:6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3:6 * (i + 1)] = T[:3, 3] - mean_pos
        k = 6 * N
        res[k:k + n_off] = 0.5 * off; k += n_off
        res[k:k + n_lt] = 1.0 * lt.flatten(); k += n_lt
        res[k:k + n_lr] = 1.0 * lr.flatten(); k += n_lr
        res[k:k + n_sag] = 0.1 * sag
        return res

    x0 = np.zeros(n_off + n_lt + n_lr + n_sag + 6)
    x0[n_off + n_lt + n_lr : n_off + n_lt + n_lr + n_sag] = noise_rad  # sag noise
    x0[-6:-3] = rod_seed.flatten()
    x0[-3:] = t_seed_v

    result = least_squares(
        residual, x0, method="lm", max_nfev=5000, xtol=1e-11, ftol=1e-11
    )
    off, lt, lr, sag, rod, t_x = unpack(result.x)
    T_list = compute_T(result.x)
    sr, st, _, _ = sigma_vs_mean(T_list)
    return {
        "sigma_rot": sr, "sigma_t": st,
        "sag_params": sag, "init_noise_deg": noise_deg,
    }


# ─── [H] J2 기반 연속 split ────────────────────────────────────────────────


def continuous_split(angles_all, R_tc_all, t_tc_all):
    """J2 각도로 정렬해서 좁은 한쪽을 train, 다른 쪽을 test.

    random split보다 가혹: train 자세 범위 *밖*에서 sag 모델이 작동하나.
    sin/cos extrapolation 능력 직접 테스트.
    """
    print("[H] J2 각도 기준 연속 split (extrapolation test)")
    n = len(angles_all)
    # J2 각도로 정렬
    order = np.argsort(angles_all[:, 1])
    J2_sorted = np.degrees(angles_all[order, 1])

    splits = [
        ("lower 70% → train, upper 30% → test  (큰 sag 영역 extrapolate)",
         order[: int(n * 0.7)], order[int(n * 0.7):]),
        ("upper 70% → train, lower 30% → test  (작은 sag 영역 extrapolate)",
         order[int(n * 0.3):], order[: int(n * 0.3)]),
        ("middle 60% → train, edges 40% → test (양쪽 extrapolate)",
         order[int(n * 0.2):int(n * 0.8)],
         np.concatenate([order[: int(n * 0.2)], order[int(n * 0.8):]])),
    ]
    print(f"  J2 범위 전체: [{J2_sorted[0]:.1f}, {J2_sorted[-1]:.1f}]°")

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
        print(f"    train J2 ∈ [{j2_tr_deg.min():.1f}, {j2_tr_deg.max():.1f}]°  "
              f"(n={len(tr_idx)})")
        print(f"    test  J2 ∈ [{j2_te_deg.min():.1f}, {j2_te_deg.max():.1f}]°  "
              f"(n={len(te_idx)})")

        for mode_label, sag_mode in [("sag off (현재)", "none"),
                                      ("sag J2+J3   ", "j23_sincos")]:
            r_tr = fit_ba_with_sag(
                ang_tr, R_tr, t_tr,
                sag_mode=sag_mode, use_link_offsets=True,
            )
            # train으로 풀은 변수로 test 포즈 σ 계산
            x_tr = r_tr["x"]
            n_off, n_lt, n_lr = 5, 15, 15
            n_sag = 4 if sag_mode == "j23_sincos" else 0

            i = 0
            off = x_tr[i:i + n_off]; i += n_off
            lt = x_tr[i:i + n_lt].reshape(5, 3); i += n_lt
            lr = x_tr[i:i + n_lr].reshape(5, 3); i += n_lr
            sag = x_tr[i:i + n_sag] if n_sag else np.zeros(0); i += n_sag
            rod = x_tr[i:i + 3]; i += 3
            t_x = x_tr[i:i + 3]
            R_x = cv2.Rodrigues(rod)[0]
            T_x = make_T(R_x, t_x)

            def apply_sag(a):
                a2 = a.copy()
                if sag_mode == "j23_sincos":
                    a2[1] += sag[0] * np.sin(a[1]) + sag[1] * np.cos(a[1])
                    abs_J3 = a[1] + a[2]
                    a2[2] += sag[2] * np.sin(abs_J3) + sag[3] * np.cos(abs_J3)
                return a2

            def T_list_of(angs, R_list, t_list):
                out = []
                for a, R, t in zip(angs, R_list, t_list):
                    T_tc = make_T(R, np.asarray(t).reshape(3))
                    a_corr = apply_sag(a + off)
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
            rr = sr_te / sr_tr if sr_tr > 1e-6 else float("inf")
            rt = st_te / st_tr if st_tr > 1e-6 else float("inf")
            print(
                f"    {mode_label}: "
                f"train σ=({sr_tr:.3f}°, {st_tr:.2f}mm)  "
                f"test σ=({sr_te:.3f}°, {st_te:.2f}mm)  "
                f"ratio rot={rr:.2f}× t={rt:.2f}×"
            )
    print()


# ─── main ─────────────────────────────────────────────────────────────────


def main():
    print("=" * 78)
    print("중력 처짐 가설 robustness 검증")
    print("=" * 78)
    n, angles_all, R_tc_all, t_tc_all, _ = load_data()
    print(f"포즈 {n}개\n")

    sweep_sag_reg(angles_all, R_tc_all, t_tc_all)
    init_robustness(angles_all, R_tc_all, t_tc_all)
    continuous_split(angles_all, R_tc_all, t_tc_all)

    print("=" * 78)
    print("판정 가이드:")
    print("  [F] sag 값이 reg에 robust  →  진짜 system 신호")
    print("  [G] 초기값 noise에도 같은 minimum  →  cost surface 잘 정의됨")
    print("  [H] 연속 split도 ratio<2 →  자세 범위 밖으로 extrapolate 작동")
    print("  셋 다 통과 → production 통합 진행 OK")
    print("  하나라도 실패 → 좁은 데이터 의존 가능성, 새 캡처로 재현 후 결정")
    print("=" * 78)


if __name__ == "__main__":
    main()
