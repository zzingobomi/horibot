"""Hand-Eye BA floor 진단 — joint_offset 추정 ON/OFF / baseline 0 vs 현재 commit 비교.

같은 32자세에 4가지 시나리오로 BA 돌려서 σ_rot/σ_t 비교:
  (1) baseline=0, estimate=True  : 1차 BA 재현 — 처음 봤던 J2 +5.75° 그림
  (2) baseline=0, estimate=False : joint_offset 끄고 hand-eye만 — 진짜 floor
  (3) baseline=현재 commit, estimate=True  : 다음 라운드가 보는 그림
  (4) baseline=현재 commit, estimate=False : 1차 baseline에서 순수 hand-eye

해석:
  (1) σ vs (2) σ — joint_offset이 σ 줄이면 systematic 잡은 것, 비슷하면 sink로 작동
  (3)의 offset_delta가 (1) 추정치와 비슷 부호/크기면 BA가 진동(시퀀스로 발산)
  (3)의 σ가 (1) σ보다 안 떨어지면 baseline 갱신 효과 없음 = J2/J3 sink fake
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from core.common import GRIPPER_ID  # noqa: E402
from core.joint_coordinates import JointCoordinates  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration.bundle_adjust import bundle_adjust_hand_eye  # noqa: E402
from modules.calibration.hand_eye import HandEyeCalibration  # noqa: E402
from modules.dynamixel.motor_config import load_motor_config  # noqa: E402
from modules.kinematics.solver import PybulletSolver  # noqa: E402

POSES_PATH = Path(__file__).parents[1] / "robot" / "calibration" / "handeye_poses.npz"


def main() -> None:
    he = HandEyeCalibration()
    n = he.load_poses(POSES_PATH)
    print(f"포즈 {n}개 로드")

    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
    solver = PybulletSolver()

    def fk(angles: list[float]):
        return solver.fk_to_matrix(list(angles))

    coords = JointCoordinates()
    current_offsets = coords.snapshot()
    print(
        "현재 디스크 offsets (deg):",
        [round(float(np.degrees(current_offsets.get(c.id, 0.0))), 3) for c in arm_cfgs],
    )

    angles_zero: list[list[float]] = []
    angles_current: list[list[float]] = []
    R_tc_list: list[np.ndarray] = []
    t_tc_list: list[np.ndarray] = []
    for p in he.poses:
        a0: list[float] = []
        ac: list[float] = []
        for cfg in arm_cfgs:
            raw = p.raw_motor_positions[cfg.id]
            r = raw_to_rad(int(raw), reverse=cfg.reverse)
            a0.append(r)
            ac.append(r + current_offsets.get(cfg.id, 0.0))
        angles_zero.append(a0)
        angles_current.append(ac)
        R_tc_list.append(np.asarray(p.R_target2cam, dtype=np.float64))
        t_tc_list.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3, 1))

    def tsai_seed(angles: list[list[float]]):
        R_gb = []
        t_gb = []
        for a in angles:
            R, pos = fk(a)
            R_gb.append(np.asarray(R, dtype=np.float64))
            t_gb.append(np.asarray(pos, dtype=np.float64).reshape(3, 1))
        R_x, t_x = cv2.calibrateHandEye(
            R_gb, t_gb, R_tc_list, t_tc_list, method=cv2.CALIB_HAND_EYE_TSAI
        )
        return R_x, t_x

    R_seed_zero, t_seed_zero = tsai_seed(angles_zero)
    R_seed_cur, t_seed_cur = tsai_seed(angles_current)

    def run(label, angles, R_seed, t_seed, estimate):
        ba = bundle_adjust_hand_eye(
            joint_angles_per_pose=angles,
            R_target2cam=R_tc_list,
            t_target2cam=t_tc_list,
            X_init=(R_seed, t_seed),
            fk_fn=fk,
            estimate_joint_offsets=estimate,
        )
        sigma_rot = float(np.sqrt(np.mean(ba.residual_rot_deg**2)))
        sigma_t = float(np.sqrt(np.mean(ba.residual_t_mm**2)))
        if ba.n_joint_vars > 0:
            offset_deg = [round(float(np.degrees(o)), 3) for o in ba.joint_offset_rad]
        else:
            offset_deg = ["-"] * 5
        print(
            f"  {label}  σ_rot={sigma_rot:6.3f}°  σ_t={sigma_t:5.2f}mm  "
            f"offset_delta_deg={offset_deg}  success={ba.success}"
        )

    print("\n--- baseline=0 (디스크 offset 무시) ---")
    run("(1) est=True ", angles_zero, R_seed_zero, t_seed_zero, True)
    run("(2) est=False", angles_zero, R_seed_zero, t_seed_zero, False)
    print("\n--- baseline=현재 commit (디스크 offset 적용) ---")
    run("(3) est=True ", angles_current, R_seed_cur, t_seed_cur, True)
    run("(4) est=False", angles_current, R_seed_cur, t_seed_cur, False)


if __name__ == "__main__":
    main()
