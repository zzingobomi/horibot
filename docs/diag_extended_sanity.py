"""bundle_adjust_hand_eye_extended 호출이 diag_handeye_extended.py v3 final 재현하는지.

검증 기대치 (diag_handeye_extended.py reg=1.0/1.0/0.5 결과):
  σ_rot ≈ 1.296°, σ_t ≈ 9.29mm
  joint_offset_deg ≈ [0, +3.29, +2.96, +2.41, 0]
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from core.common import GRIPPER_ID  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration.bundle_adjust import (  # noqa: E402
    bundle_adjust_hand_eye_extended,
)
from modules.calibration.hand_eye import HandEyeCalibration  # noqa: E402
from modules.dynamixel.motor_config import load_motor_config  # noqa: E402
from modules.kinematics.fk_chain import fk_chain  # noqa: E402

POSES = Path(__file__).parents[1] / "robot" / "calibration" / "handeye_poses.npz"


def main():
    he = HandEyeCalibration()
    he.load_poses(POSES)
    print(f"포즈 {len(he.poses)}개 로드")

    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]

    # baseline=0 (디스크 offset 무시) — v3 final과 동일 조건
    angles = []
    R_tc = []
    t_tc = []
    for p in he.poses:
        a = [
            raw_to_rad(int(p.raw_motor_positions[c.id]), reverse=c.reverse)
            for c in arm_cfgs
        ]
        angles.append(a)
        R_tc.append(np.asarray(p.R_target2cam, dtype=np.float64))
        t_tc.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3, 1))

    # TSAI seed
    R_gb = []
    t_gb = []
    zero = np.zeros((5, 3))
    for a in angles:
        R, t = fk_chain(np.array(a), zero, zero)
        R_gb.append(R)
        t_gb.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb, t_gb, R_tc, t_tc, method=cv2.CALIB_HAND_EYE_TSAI
    )

    ba = bundle_adjust_hand_eye_extended(
        joint_angles_per_pose=angles,
        R_target2cam=R_tc,
        t_target2cam=[t.reshape(3) for t in t_tc],
        X_init=(R_seed, t_seed),
    )

    sigma_rot = float(np.sqrt(np.mean(ba.residual_rot_deg**2)))
    sigma_t = float(np.sqrt(np.mean(ba.residual_t_mm**2)))

    print(f"σ_rot={sigma_rot:.3f}° σ_t={sigma_t:.2f}mm iter={ba.n_iter}")
    print(
        f"joint_offset_deg = "
        f"{[round(float(np.degrees(o)), 3) for o in ba.joint_offset_rad]}"
    )
    print("link_trans_mm:")
    for i in range(5):
        t = ba.link_trans_m[i] * 1000.0
        print(f"  joint{i+1}: ({t[0]:+5.2f}, {t[1]:+5.2f}, {t[2]:+5.2f})")
    print("link_rot_deg:")
    for i in range(5):
        r = np.degrees(ba.link_rot_rad[i])
        print(f"  joint{i+1}: ({r[0]:+5.3f}, {r[1]:+5.3f}, {r[2]:+5.3f})")

    expected_rot, expected_t = 1.296, 9.29
    print()
    if abs(sigma_rot - expected_rot) < 0.05 and abs(sigma_t - expected_t) < 0.5:
        print(f"OK: diag_handeye_extended.py 결과 재현 (기대 {expected_rot}° / {expected_t}mm)")
    else:
        print(f"!! 기대 {expected_rot}° / {expected_t}mm와 차이 큼")


if __name__ == "__main__":
    main()
