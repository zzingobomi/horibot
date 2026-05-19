"""Bundle Adjustment for hand-eye calibration.

cv2.calibrateHandEye 결과를 seed로 받아, joint zero offset과 hand-eye 변환을
동시에 최적화. 비용함수는 T_base←board의 분산 (체커보드가 안 움직였으니까
모든 포즈에서 같은 값이 되어야 함).

변수 (총 11개):
  - joint_zero_offset[5] (라디안): URDF zero 대비 모터 horn 조립 오차
  - rod (3): R_cam2gripper의 Rodrigues 벡터
  - t (3): t_cam2gripper (미터)

scipy.optimize.least_squares + Levenberg-Marquardt로 푼다. 잔차는 포즈마다
6차원 (회전 axis-angle 편차 3 + 위치 편차 3). 17포즈면 102개 잔차로 11개 변수.
"""

from __future__ import annotations
import logging
import time
import cv2
import numpy as np
from scipy.optimize import least_squares

logger = logging.getLogger(__name__)


def bundle_adjust(
    poses: list,
    solver,
    R_seed: np.ndarray,
    t_seed: np.ndarray,
    n_joints: int = 5,
    max_nfev: int = 200,
) -> dict:
    """BA 실행.

    Args:
        poses: HandEyeCalibration.poses (Pose dataclass 리스트). 각 Pose는
            joint_angles_rad / R_target2cam / t_target2cam 보유.
        solver: PybulletSolver (fk_to_matrix 호출).
        R_seed, t_seed: cv2.calibrateHandEye 결과 (보통 TSAI).
        n_joints: 5DOF arm.
        max_nfev: LM iteration 제한.

    Returns:
        dict with joint_offsets_rad, R/t_cam2gripper, sigma_rot_deg, sigma_t_mm,
        per_pose_residual, iterations, cost_initial/final, success.
    """
    if len(poses) < 3:
        raise ValueError(f"BA는 최소 3포즈 필요 (현재 {len(poses)})")

    # 초기값: offsets=0, R/t는 seed
    rod_seed, _ = cv2.Rodrigues(R_seed)
    t_seed_flat = np.asarray(t_seed, dtype=np.float64).reshape(3)
    x0 = np.concatenate(
        [
            np.zeros(n_joints, dtype=np.float64),
            rod_seed.flatten(),
            t_seed_flat,
        ]
    )

    joint_angles_per_pose = [
        np.asarray(p.joint_angles_rad, dtype=np.float64) for p in poses
    ]
    R_target2cam_per_pose = [
        np.asarray(p.R_target2cam, dtype=np.float64) for p in poses
    ]
    t_target2cam_per_pose = [
        np.asarray(p.t_target2cam, dtype=np.float64).reshape(3) for p in poses
    ]

    def residuals(x: np.ndarray) -> np.ndarray:
        offsets = x[:n_joints]
        rod = x[n_joints : n_joints + 3]
        t = x[n_joints + 3 : n_joints + 6]
        R_x, _ = cv2.Rodrigues(rod)
        T_x = _make_T(R_x, t)

        T_target2base_list = []
        for joints, R_t2c, t_t2c in zip(
            joint_angles_per_pose, R_target2cam_per_pose, t_target2cam_per_pose
        ):
            joints_corrected = (joints + offsets).tolist()
            R_g2b_list, t_g2b = solver.fk_to_matrix(joints_corrected)
            R_g2b = np.asarray(R_g2b_list, dtype=np.float64)
            T_g2b = _make_T(R_g2b, np.asarray(t_g2b, dtype=np.float64))
            T_t2c = _make_T(R_t2c, t_t2c)
            T_target2base_list.append(T_g2b @ T_x @ T_t2c)

        positions = np.array([T[:3, 3] for T in T_target2base_list])
        mean_pos = positions.mean(axis=0)
        mean_R = _mean_rotation([T[:3, :3] for T in T_target2base_list])

        res: list[np.ndarray] = []
        for T in T_target2base_list:
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res.append(rod_dev.flatten())  # 3
            res.append(T[:3, 3] - mean_pos)  # 3 (미터)
        return np.concatenate(res)

    t0 = time.time()
    initial_res = residuals(x0)
    cost_initial = float(0.5 * np.sum(initial_res**2))

    result = least_squares(
        residuals,
        x0,
        method="lm",
        max_nfev=max_nfev,
        xtol=1e-10,
        ftol=1e-10,
    )

    elapsed = time.time() - t0
    cost_final = float(result.cost)

    # 최적화 결과 추출
    offsets_opt = result.x[:n_joints]
    rod_opt = result.x[n_joints : n_joints + 3]
    t_opt = result.x[n_joints + 3 : n_joints + 6]
    R_opt, _ = cv2.Rodrigues(rod_opt)

    # 최종 per-pose 잔차 (단위 변환: 라디안→°, m→mm)
    final_res = result.fun.reshape(-1, 6)
    per_pose: list[dict] = []
    rot_devs_sq: list[float] = []
    pos_devs_sq: list[float] = []
    for pose, r in zip(poses, final_res):
        rod_dev = r[:3]
        pos_dev = r[3:]
        drot_deg = float(np.degrees(np.linalg.norm(rod_dev)))
        dt_mm = float(np.linalg.norm(pos_dev)) * 1000.0
        per_pose.append({"id": pose.id, "drot_deg": drot_deg, "dt_mm": dt_mm})
        rot_devs_sq.append(drot_deg**2)
        pos_devs_sq.append(dt_mm**2)

    sigma_rot_deg = float(np.sqrt(np.mean(rot_devs_sq))) if rot_devs_sq else 0.0
    sigma_t_mm = float(np.sqrt(np.mean(pos_devs_sq))) if pos_devs_sq else 0.0

    logger.info(
        "BA 완료: %.2fs, iter=%d, cost %.6f→%.6f, σ_rot=%.3f° σ_t=%.1fmm",
        elapsed,
        int(result.nfev),
        cost_initial,
        cost_final,
        sigma_rot_deg,
        sigma_t_mm,
    )
    logger.info(
        "joint_offsets [°]: %s",
        ", ".join(f"{np.degrees(o):+.3f}" for o in offsets_opt),
    )

    return {
        "joint_offsets_rad": offsets_opt,
        "R_cam2gripper": R_opt,
        "t_cam2gripper": t_opt,
        "sigma_rot_deg": sigma_rot_deg,
        "sigma_t_mm": sigma_t_mm,
        "per_pose_residual": per_pose,
        "iterations": int(result.nfev),
        "cost_initial": cost_initial,
        "cost_final": cost_final,
        "success": bool(result.success),
        "elapsed_sec": elapsed,
    }


# ── 유틸 (hand_eye.py와 중복되지만 의존 방향 단순화) ────────────


def _make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    """Shepperd's method (수치 안정)."""
    m = R
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    """Markley/Crassidis quaternion 평균."""
    if not rotations:
        return np.eye(3)
    M = np.zeros((4, 4))
    for R in rotations:
        q = _R_to_quat(R).reshape(4, 1)
        M += q @ q.T
    M /= len(rotations)
    _, eigvecs = np.linalg.eigh(M)
    q_mean = eigvecs[:, -1]
    return _quat_to_R(q_mean)
