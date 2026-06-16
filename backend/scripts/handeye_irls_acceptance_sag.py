"""Hand-Eye IRLS Acceptance Test — _physical_sag 위 (운영 BA).

기존 handeye_irls_acceptance.py 는 11 자유도 hand_eye 위. 이건 *진짜 운영 BA*
(_physical_sag 43 DOF) 위에서 IRLS 가 trauma 차단하는지.

시나리오:
  S1. clean 8 poses — baseline _physical_sag vs _physical_sag_irls
  S2. 8 + pose7 에 5° rot + 20mm trans perturbation

성공 기준:
  - baseline σ_rot S1→S2: ~0.29° → 큰 폭 악화
  - IRLS σ_rot S1→S2:    ~0.29° → 유지 (outlier 자동 down-weight)
  - IRLS w_outlier(pose7) < 0.3
  - ΔX (cam2gripper) IRLS < baseline (절반 이하)

실행: cd backend && uv run --no-sync python -m scripts.handeye_irls_acceptance_sag
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np


def _ensure_utf8() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")


def raw_to_rad(raw):
    return (raw.astype(np.float64) - 2048.0) / 4095.0 * 2.0 * np.pi


def make_tsai_seed(joint_angles, R_tc, t_tc, fk_fn):
    R_g2b, t_g2b = [], []
    for ang in joint_angles:
        R, t = fk_fn(ang)
        R_g2b.append(np.asarray(R))
        t_g2b.append(np.asarray(t).reshape(3, 1))
    R_x, t_x = cv2.calibrateHandEye(
        R_gripper2base=R_g2b,
        t_gripper2base=t_g2b,
        R_target2cam=R_tc,
        t_target2cam=[t.reshape(3, 1) for t in t_tc],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return R_x, t_x.reshape(3)


def inject_perturbation(R_list, t_list, target_idx, *, rot_deg, trans_mm, seed=42):
    rng = np.random.default_rng(seed)
    axis = rng.standard_normal(3)
    axis /= np.linalg.norm(axis)
    rvec = axis * np.deg2rad(rot_deg)
    R_pert, _ = cv2.Rodrigues(rvec)
    dir_t = rng.standard_normal(3)
    dir_t /= np.linalg.norm(dir_t)
    t_pert = dir_t * (trans_mm / 1000.0)
    R_new = [R.copy() for R in R_list]
    t_new = [t.copy() for t in t_list]
    R_new[target_idx] = R_pert @ R_list[target_idx]
    t_new[target_idx] = t_list[target_idx] + t_pert
    return R_new, t_new


def delta_X(R_a, t_a, R_b, t_b):
    rvec, _ = cv2.Rodrigues(R_a @ R_b.T)
    return (
        float(np.degrees(np.linalg.norm(rvec))),
        float(np.linalg.norm(t_a - t_b) * 1000.0),
    )


def sigma_rms(arr):
    return float(np.sqrt(np.mean(arr**2)))


def main() -> None:
    _ensure_utf8()

    repo_root = Path(__file__).resolve().parents[2]
    npz = np.load(
        str(repo_root / "robot/instances/omx_f_0/calibration/handeye_poses.npz"),
        allow_pickle=True,
    )
    raw = npz["raw_positions"]
    R_tc = [np.asarray(r, dtype=np.float64) for r in npz["R_target2cam"]]
    t_tc = [np.asarray(t, dtype=np.float64).reshape(3) for t in npz["t_target2cam"]]
    joint_angles = [raw_to_rad(r).tolist() for r in raw]

    from modules.calibration.bundle_adjust import (
        bundle_adjust_hand_eye_physical_sag,
        bundle_adjust_hand_eye_physical_sag_irls,
    )
    from modules.kinematics.registry import RobotRegistry

    kin = RobotRegistry().get_kinematics("omx_f_0")
    fk_chain = RobotRegistry().get_fk_chain("omx_f_0")

    def fk_fn(angles):
        R, t = kin.fk_to_matrix(list(angles))
        return np.asarray(R), np.asarray(t).reshape(3)

    seed_R, seed_t = make_tsai_seed(joint_angles, R_tc, t_tc, fk_fn)

    # S1: clean
    r_s1_base = bundle_adjust_hand_eye_physical_sag(
        joint_angles_per_pose=joint_angles,
        R_target2cam=R_tc,
        t_target2cam=t_tc,
        X_init=(seed_R, seed_t),
        fk_chain=fk_chain,
    )
    r_s1_irls = bundle_adjust_hand_eye_physical_sag_irls(
        joint_angles_per_pose=joint_angles,
        R_target2cam=R_tc,
        t_target2cam=t_tc,
        X_init=(seed_R, seed_t),
        fk_chain=fk_chain,
    )

    # S2: perturb pose7
    R_p, t_p = inject_perturbation(
        R_tc, t_tc, target_idx=7, rot_deg=5.0, trans_mm=20.0
    )
    r_s2_base = bundle_adjust_hand_eye_physical_sag(
        joint_angles_per_pose=joint_angles,
        R_target2cam=R_p,
        t_target2cam=t_p,
        X_init=(seed_R, seed_t),
        fk_chain=fk_chain,
    )
    r_s2_irls = bundle_adjust_hand_eye_physical_sag_irls(
        joint_angles_per_pose=joint_angles,
        R_target2cam=R_p,
        t_target2cam=t_p,
        X_init=(seed_R, seed_t),
        fk_chain=fk_chain,
    )

    def row(name, res, w_idx=None):
        sr = sigma_rms(res.residual_rot_deg)
        st = sigma_rms(res.residual_t_mm)
        extra = ""
        if w_idx is not None and hasattr(res, "weights"):
            extra = (
                f" | w_{w_idx}={res.weights[w_idx]:.3f}"
                f" | outer={res.outer_iter}"
            )
        print(
            f"  {name:30s}  σ_rot={sr:6.3f}°  σ_t={st:6.2f}mm{extra}"
        )

    print("=" * 80)
    print("S1: clean 8 poses on _physical_sag (43 DOF)")
    print("=" * 80)
    row("baseline _physical_sag", r_s1_base)
    row("IRLS _physical_sag", r_s1_irls, w_idx=7)

    print()
    print("=" * 80)
    print("S2: 8 + pose7 perturbation (5°/20mm)")
    print("=" * 80)
    row("baseline _physical_sag", r_s2_base)
    row("IRLS _physical_sag", r_s2_irls, w_idx=7)

    print()
    print("=" * 80)
    print("ΔX (S1 → S2) — outlier 가 X 를 얼마나 끌어당겼나")
    print("=" * 80)
    dR_b, dt_b = delta_X(
        r_s1_base.R_cam2gripper, r_s1_base.t_cam2gripper,
        r_s2_base.R_cam2gripper, r_s2_base.t_cam2gripper,
    )
    dR_i, dt_i = delta_X(
        r_s1_irls.R_cam2gripper, r_s1_irls.t_cam2gripper,
        r_s2_irls.R_cam2gripper, r_s2_irls.t_cam2gripper,
    )
    print(f"  baseline _physical_sag   ΔR={dR_b:6.3f}°  Δt={dt_b:6.2f}mm")
    print(f"  IRLS _physical_sag       ΔR={dR_i:6.3f}°  Δt={dt_i:6.2f}mm")

    print()
    print("=" * 80)
    print("IRLS weights on S2 (pose7 = injected outlier)")
    print("=" * 80)
    for i, w in enumerate(r_s2_irls.weights):
        mark = "  <-- INJECTED" if i == 7 else ""
        print(f"  pose {i}:  w = {w:.3f}{mark}")

    print()
    print("=" * 80)
    print("IRLS outer iteration history (S2)")
    print("=" * 80)
    for k, (c, s, kp) in enumerate(
        zip(
            r_s2_irls.cost_history,
            r_s2_irls.sigma_hat_history,
            r_s2_irls.huber_kappa_history,
        )
    ):
        print(f"  iter {k + 1}: cost={c:.4e}  σ̂={s:.4e}  κ={kp:.4e}")

    print()
    print("=" * 80)
    print("판정")
    print("=" * 80)
    s1_b_rot = sigma_rms(r_s1_base.residual_rot_deg)
    s2_b_rot = sigma_rms(r_s2_base.residual_rot_deg)
    s1_i_rot = sigma_rms(r_s1_irls.residual_rot_deg)
    s2_i_rot = sigma_rms(r_s2_irls.residual_rot_deg)
    w7 = float(r_s2_irls.weights[7])

    sanity_match = abs(s1_b_rot - s1_i_rot) < 0.05
    base_degrades = s2_b_rot > s1_b_rot * 1.5
    irls_stable = s2_i_rot < s1_i_rot * 1.5
    outlier_down = w7 < 0.3
    irls_dx_smaller = dR_i < dR_b * 0.6

    print(
        f"  S1 sanity (baseline ≈ IRLS):       "
        f"{s1_b_rot:.3f}° vs {s1_i_rot:.3f}°  diff<0.05?  {sanity_match}"
    )
    print(
        f"  S2 baseline 악화 (>1.5× S1):       "
        f"{s2_b_rot:.3f}° > {s1_b_rot * 1.5:.3f}°  ?  {base_degrades}"
    )
    print(
        f"  S2 IRLS 안정 (<1.5× S1):           "
        f"{s2_i_rot:.3f}° < {s1_i_rot * 1.5:.3f}°  ?  {irls_stable}"
    )
    print(f"  IRLS w_outlier < 0.3:                  w_7={w7:.3f}  ?  {outlier_down}")
    print(
        f"  IRLS ΔR < baseline×0.6:                "
        f"{dR_i:.3f}° < {dR_b * 0.6:.3f}°  ?  {irls_dx_smaller}"
    )
    print()

    n_pass = sum([sanity_match, base_degrades, irls_stable, outlier_down, irls_dx_smaller])
    if n_pass >= 4:
        print(
            f"  >>> PASS ({n_pass}/5) — _physical_sag IRLS 가 trauma 차단."
            f" calibration_node 전환 정당."
        )
    elif n_pass >= 2:
        print(f"  >>> PARTIAL ({n_pass}/5) — 일부만. 추가 검증 필요.")
    else:
        print(f"  >>> FAIL ({n_pass}/5) — IRLS 효과 미흡. 다른 방향.")


if __name__ == "__main__":
    main()
