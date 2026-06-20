"""최고의 calibration 자리 push — multi-start + outlier 제거 + prior grid search.

전략:
  1. 5종 cv2 seed (TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS) 각각 Stage C BA → best LOOCV
  2. 명시 outlier pose 자리 제외 (pose #6 등) 후 refit
  3. Prior 강도 grid (loose ↔ strict) 자리 LOOCV optimal
  4. Bootstrap σ — 25 자리 자리 17-21 자리 resample 자리 BA × N → handeye 분포
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

import cv2  # noqa: E402
from core.robot.robot_registry import RobotRegistry  # noqa: E402
from modules.motor.motor_config import load_motor_layout  # noqa: E402

import scripts.calibrate_offline as co  # noqa: E402

ROBOT = "so101_6dof_0"


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore

    # ─── Load ─────────────────────────────────────────────────
    run, captures, intrinsic, arm_cfgs = co.load_data(
        BACKEND / "storage" / "horibot.db",
        BACKEND / "storage" / "blobs",
        ROBOT, None, load_depth=False,
    )
    fk_chain = RobotRegistry().get_fk_chain(ROBOT)
    K = intrinsic["camera_matrix"]
    sag_arm_indices = [m - 1 for m in RobotRegistry().get(ROBOT).sag_joint_motor_ids]
    print(f"=== Captures: {len(captures)} ===\n")

    # ─── Strategy 1: Multi-start (5 cv2 seeds × Stage C BA) ───
    print("=" * 90)
    print("[1] Multi-start — 5 cv2.calibrateHandEye seeds × Stage C BA")
    print("=" * 90)
    R_g2b = [fk_chain.fk(c.joint_angles_rad_raw)[0] for c in captures]
    t_g2b = [fk_chain.fk(c.joint_angles_rad_raw)[1].reshape(3, 1) for c in captures]
    R_t2c = [c.R_target2cam_seed for c in captures]
    t_t2c = [c.t_target2cam_seed.reshape(3, 1) for c in captures]

    seed_methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    cfg_c = co.BAConfig(estimate_joint=True, estimate_link=True)

    print(f"{'seed':12s} {'BA train':10s} {'LOOCV':10s} {'ratio':6s} {'|he_t|':10s}")
    best_loocv = float("inf")
    best_seed_name = None
    best_seed_result = None
    for name, method in seed_methods.items():
        try:
            R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
            t = np.asarray(t).reshape(3)
            res = co.run_ba_stage(
                captures, fk_chain, K, sag_arm_indices, cfg_c,
                name=f"C_{name}", seed_handeye_R=R, seed_handeye_t=t,
                arm_cfgs=arm_cfgs, irls_outer=3,
            )
            loocv = co.compute_loocv(
                captures, fk_chain, K, sag_arm_indices, cfg_c,
                R, t, arm_cfgs,
            )
            t_norm_mm = float(np.linalg.norm(res.handeye_t) * 1000)
            print(f"  {name:10s} {res.reproj_rms_px:8.3f} {loocv:8.3f} "
                  f"{loocv/res.reproj_rms_px:5.2f}× {t_norm_mm:7.2f}mm")
            if loocv < best_loocv:
                best_loocv = loocv
                best_seed_name = name
                best_seed_result = res
        except cv2.error:
            print(f"  {name:10s} FAILED")
    print(f"\n→ Best seed: {best_seed_name}  LOOCV={best_loocv:.3f}px\n")

    # ─── Strategy 2: 명시 outlier 제거 ─────────────────────────
    print("=" * 90)
    print("[2] Outlier explicit removal — pose #6 (20px RMS, 16 corners 자리 가장자리 잘림)")
    print("=" * 90)
    outlier_indices = [6]
    captures_clean = [c for i, c in enumerate(captures) if i not in outlier_indices]
    print(f"  Captures: {len(captures)} → {len(captures_clean)}")

    seed_R, seed_t, seed_name = co.seed_handeye(captures_clean, fk_chain)
    print(f"  Re-seed best: {seed_name}, t={(seed_t*1000).round(2)}mm")
    res_clean = co.run_ba_stage(
        captures_clean, fk_chain, K, sag_arm_indices, cfg_c,
        name="C_clean", seed_handeye_R=seed_R, seed_handeye_t=seed_t,
        arm_cfgs=arm_cfgs, irls_outer=3,
    )
    loocv_clean = co.compute_loocv(
        captures_clean, fk_chain, K, sag_arm_indices, cfg_c,
        seed_R, seed_t, arm_cfgs,
    )
    print(f"  BA train: {res_clean.reproj_rms_px:.3f}px  LOOCV: {loocv_clean:.3f}px  "
          f"ratio: {loocv_clean/res_clean.reproj_rms_px:.2f}×\n")

    # ─── Strategy 3: Prior strength grid ──────────────────────
    print("=" * 90)
    print("[3] Prior strength grid (loose ↔ strict 자리 LOOCV optimum)")
    print("=" * 90)
    base_seed_R, base_seed_t, _ = co.seed_handeye(captures, fk_chain)

    # 원래 strict 자리 (1mm / 0.2° / 1°). loose (5mm / 1° / 2°), tight (0.5mm / 0.1° / 0.5°).
    grid = [
        ("very_strict", 0.0005, np.deg2rad(0.1), np.deg2rad(0.5)),
        ("strict (current)", 0.001, np.deg2rad(0.2), np.deg2rad(1.0)),
        ("medium", 0.003, np.deg2rad(0.5), np.deg2rad(2.0)),
        ("loose", 0.005, np.deg2rad(1.0), np.deg2rad(3.0)),
    ]
    print(f"{'preset':22s} {'link_t':10s} {'link_r':8s} {'joint':7s} "
          f"{'train':8s} {'LOOCV':8s} {'ratio':6s} {'|link_t|max':10s} {'|joint|max':9s}")
    orig_priors = (co.PRIOR_LINK_T_M, co.PRIOR_LINK_R_RAD, co.PRIOR_JOINT_RAD)
    for name, lt, lr, joint in grid:
        co.PRIOR_LINK_T_M = lt
        co.PRIOR_LINK_R_RAD = lr
        co.PRIOR_JOINT_RAD = joint
        res = co.run_ba_stage(
            captures, fk_chain, K, sag_arm_indices, cfg_c,
            name=f"C_{name}", seed_handeye_R=base_seed_R, seed_handeye_t=base_seed_t,
            arm_cfgs=arm_cfgs, irls_outer=3,
        )
        loocv = co.compute_loocv(
            captures, fk_chain, K, sag_arm_indices, cfg_c,
            base_seed_R, base_seed_t, arm_cfgs,
        )
        link_max = (
            max(np.linalg.norm(v) for v in res.link_trans.values()) * 1000.0
            if res.link_trans else 0.0
        )
        joint_max = (
            max(abs(np.rad2deg(v)) for v in res.joint_offsets.values())
            if res.joint_offsets else 0.0
        )
        print(f"  {name:22s} {lt*1000:5.1f}mm   {np.rad2deg(lr):4.2f}°  "
              f"{np.rad2deg(joint):4.1f}°  "
              f"{res.reproj_rms_px:6.3f}  {loocv:6.3f}  "
              f"{loocv/res.reproj_rms_px:5.2f}× {link_max:7.2f}mm  {joint_max:5.2f}°")
    co.PRIOR_LINK_T_M, co.PRIOR_LINK_R_RAD, co.PRIOR_JOINT_RAD = orig_priors
    print()

    # ─── Strategy 4: Bootstrap σ ───────────────────────────────
    print("=" * 90)
    print("[4] Bootstrap σ — N=20 자리 17 자세 resample BA → handeye 분포")
    print("=" * 90)
    n_boot = 20
    boot_size = max(15, int(len(captures) * 0.7))
    rng = np.random.default_rng(42)
    he_ts = []
    he_Rs = []
    t0 = time.time()
    for b in range(n_boot):
        idx = rng.choice(len(captures), boot_size, replace=False)
        cap_b = [captures[int(i)] for i in idx]
        try:
            sR, st_, _ = co.seed_handeye(cap_b, fk_chain)
            res = co.run_ba_stage(
                cap_b, fk_chain, K, sag_arm_indices, cfg_c,
                name=f"boot_{b}", seed_handeye_R=sR, seed_handeye_t=st_,
                arm_cfgs=arm_cfgs, irls_outer=2, max_nfev=200,
            )
            he_ts.append(res.handeye_t)
            he_Rs.append(res.handeye_R)
        except Exception:
            continue
    print(f"  {n_boot} bootstrap samples 완료 ({time.time()-t0:.1f}s)")
    he_t_arr = np.array(he_ts)
    bs_t_std = he_t_arr.std(axis=0) * 1000.0
    bs_t_mean = he_t_arr.mean(axis=0) * 1000.0
    print(f"  Hand-eye t mean: [{bs_t_mean[0]:+.2f}, {bs_t_mean[1]:+.2f}, "
          f"{bs_t_mean[2]:+.2f}] mm")
    print(f"  Hand-eye t std:  [{bs_t_std[0]:.2f}, {bs_t_std[1]:.2f}, "
          f"{bs_t_std[2]:.2f}] mm  total={np.linalg.norm(bs_t_std):.2f}mm")
    # Rotation σ
    eulers = np.array(
        [Rot.from_matrix(R).as_euler("xyz", degrees=True) for R in he_Rs]
    )
    eul_std = eulers.std(axis=0)
    print(f"  Hand-eye R euler XYZ mean: [{eulers.mean(axis=0)[0]:+.2f}, "
          f"{eulers.mean(axis=0)[1]:+.2f}, {eulers.mean(axis=0)[2]:+.2f}]°")
    print(f"  Hand-eye R euler XYZ std:  [{eul_std[0]:.3f}, {eul_std[1]:.3f}, "
          f"{eul_std[2]:.3f}]° total={np.linalg.norm(eul_std):.3f}°")
    print()
    print("→ Bootstrap σ 는 데이터 자체에서 직접 측정 — Hessian σ 보다 honest.")


if __name__ == "__main__":
    main()
