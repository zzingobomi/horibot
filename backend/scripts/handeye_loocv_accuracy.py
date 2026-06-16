"""Hand-Eye 외부 정확도 검증 — LOOCV cross-validation.

self-consistency residual (σ_rot, σ_t) 는 *BA fit 자체의 잔차* 라 *진짜 정확도* 와
다를 수 있음. 외부 측정 도구 (pinpoint TCP probe 등) 없이 *자체* 외부 정확도 proxy
얻는 방법 = Leave-One-Out:

  - 8장 중 i 제외 → 7장 만으로 X (cam2gripper) 추정 → X_i
  - X_i + fk(i 자세) 로 *i 자세의 board pose* 예측
  - actual (i 의 PnP 결과) vs predicted 의 R/t 차이 = *외부 정확도 의 proxy*

해석:
  - LOOCV err ≈ σ           → σ 가 외부 정확도 신뢰. σ 0.3° 는 진짜 0.3°.
  - LOOCV err 1.5~3× σ      → 약간 over-fit. σ 약간 낙관적.
  - LOOCV err > 3× σ        → 큰 over-fit. 외부 측정 도구 (TCP probe) 필요.

진단 도구 — *상한* 만 보임. *진짜* 정확도는 외부 GT 와 비교 필요 (보드 외 known
marker, ruler 측정, TCP probe 등). 회사 환경에선 LOOCV 가 가능한 한계.

실행: cd backend && uv run --no-sync python -m scripts.handeye_loocv_accuracy
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np


def _ensure_utf8() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")


def raw_to_rad(raw: np.ndarray) -> np.ndarray:
    return (raw.astype(np.float64) - 2048.0) / 4095.0 * 2.0 * np.pi


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


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
    N = len(R_tc)
    joint_angles = [raw_to_rad(r).tolist() for r in raw]

    from modules.calibration.bundle_adjust import (
        bundle_adjust_hand_eye_physical_sag_irls,
    )
    from modules.kinematics.registry import RobotRegistry

    kin = RobotRegistry().get_kinematics("omx_f_0")
    fk_chain = RobotRegistry().get_fk_chain("omx_f_0")

    def fk_fn(angles):
        R, t = kin.fk_to_matrix(list(angles))
        return np.asarray(R), np.asarray(t).reshape(3)

    seed_R, seed_t = make_tsai_seed(joint_angles, R_tc, t_tc, fk_fn)

    # full 8장 BA — baseline σ
    res_full = bundle_adjust_hand_eye_physical_sag_irls(
        joint_angles_per_pose=joint_angles,
        R_target2cam=R_tc,
        t_target2cam=t_tc,
        X_init=(seed_R, seed_t),
        fk_chain=fk_chain,
    )
    sigma_rot_full = float(np.sqrt(np.mean(res_full.residual_rot_deg**2)))
    sigma_t_full = float(np.sqrt(np.mean(res_full.residual_t_mm**2)))

    print("=" * 90)
    print("Hand-Eye 외부 정확도 검증 — LOOCV cross-validation")
    print("=" * 90)
    print(f"Full 8장 BA (IRLS+_physical_sag)  σ_rot = {sigma_rot_full:.3f}°  "
          f"σ_t = {sigma_t_full:.2f}mm")
    print()
    print("LOOCV — 각 자세 i 제외 + i 의 board pose 예측 vs actual")
    print("-" * 90)
    print(f"{'idx':>3}  {'pred R err°':>12}  {'pred t err mm':>14}  "
          f"{'rot ratio /σ':>13}  {'t ratio /σ':>11}")

    loo_rot_errs: list[float] = []
    loo_t_errs: list[float] = []

    for i in range(N):
        keep = [j for j in range(N) if j != i]
        res_lo = bundle_adjust_hand_eye_physical_sag_irls(
            joint_angles_per_pose=[joint_angles[j] for j in keep],
            R_target2cam=[R_tc[j] for j in keep],
            t_target2cam=[t_tc[j] for j in keep],
            X_init=(seed_R, seed_t),
            fk_chain=fk_chain,
        )
        R_x = res_lo.R_cam2gripper
        t_x = res_lo.t_cam2gripper
        T_x = make_T(R_x, t_x)
        T_b = res_lo.T_board_base

        # i 자세에서 predicted board-in-cam:
        # T_cam_in_base = T_gripper_in_base @ T_cam_in_gripper(=X)
        # T_board_in_cam_pred = inv(T_cam_in_base) @ T_board_in_base
        R_g2b, t_g2b = fk_fn(joint_angles[i])
        T_g2b = make_T(R_g2b, t_g2b)
        T_cam_in_base = T_g2b @ T_x
        T_b_in_cam_pred = np.linalg.inv(T_cam_in_base) @ T_b
        R_pred = T_b_in_cam_pred[:3, :3]
        t_pred = T_b_in_cam_pred[:3, 3]

        R_actual = R_tc[i]
        t_actual = t_tc[i]

        rvec, _ = cv2.Rodrigues(R_pred @ R_actual.T)
        rot_err = float(np.degrees(np.linalg.norm(rvec)))
        t_err = float(np.linalg.norm(t_pred - t_actual)) * 1000.0

        loo_rot_errs.append(rot_err)
        loo_t_errs.append(t_err)
        ratio_rot = rot_err / max(sigma_rot_full, 1e-6)
        ratio_t = t_err / max(sigma_t_full, 1e-6)
        print(
            f"{i:>3}  {rot_err:>12.3f}  {t_err:>14.2f}  "
            f"{ratio_rot:>10.2f}×  {ratio_t:>9.2f}×"
        )

    rms_rot = float(np.sqrt(np.mean(np.array(loo_rot_errs) ** 2)))
    rms_t = float(np.sqrt(np.mean(np.array(loo_t_errs) ** 2)))

    print("-" * 90)
    print(f"LOOCV RMS  R = {rms_rot:.3f}°  t = {rms_t:.2f}mm")
    print()
    print("=" * 90)
    print("해석")
    print("=" * 90)
    print(f"  σ_rot (BA fit residual)      = {sigma_rot_full:.3f}°")
    print(f"  σ_rot (LOOCV — 외부 proxy)   = {rms_rot:.3f}°")
    print(f"  ratio                         = {rms_rot / max(sigma_rot_full, 1e-6):.2f}×")
    print()
    print(f"  σ_t (BA fit residual)        = {sigma_t_full:.2f}mm")
    print(f"  σ_t (LOOCV)                  = {rms_t:.2f}mm")
    print(f"  ratio                         = {rms_t / max(sigma_t_full, 1e-6):.2f}×")
    print()

    ratio = rms_rot / max(sigma_rot_full, 1e-6)
    if ratio < 1.5:
        print("  >>> 일치 — σ ≈ LOOCV. σ 가 외부 정확도 *신뢰 가능*.")
    elif ratio < 3.0:
        print("  >>> 약간 over-fit — LOOCV 가 σ 의 1.5~3× 큼. σ 약간 낙관적.")
    else:
        print(
            "  >>> 큰 over-fit — LOOCV ≫ σ. σ 가 *실제 정확도 보다 매우 낙관적*."
        )
        print("      외부 측정 도구 (TCP probe / known marker ruler) 필요.")


if __name__ == "__main__":
    main()
