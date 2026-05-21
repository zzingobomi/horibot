"""Hand-Eye BA 확장 v3 — link rotation + translation 둘 다 풀어 σ_rot/σ_t 동시 공략.

v1 (link translation만):
  σ_t 16.9→9.36mm 성공, σ_rot 1.5→1.4° 정체. link length는 위치만 영향이라 당연.

v2 (link rotation만):
  σ_rot 1.5→1.32° 성공! link frame 기울기가 회전 정보 제공. 값들이 ±1° 안 합리적.

v3 (둘 다, regularization 강하게):
  자유도 41 vs 잔차 192. 비율 4.7로 경계지만 reg로 gauge freedom 차단.
  - link_rot ±1° 자유 (v2에서 검증됨, weight 2.0)
  - link_trans ±5mm 자유 (v1 unreg 시 −61mm 비현실 → weight 5.0 강하게)
  - joint_offset ±3° 자유 (weight 1.0)

변수 layout (총 41):
  [0:5]    joint_offset (rad)
  [5:20]   link_translation (5 joint × dx,dy,dz, m)
  [20:35]  link_rotation (5 joint × rx,ry,rz, rad)
  [35:38]  rod (cam2gripper)
  [38:41]  t (cam2gripper, m)
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))

from core.common import GRIPPER_ID  # noqa: E402
from core.joint_coordinates import JointCoordinates  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration.hand_eye import HandEyeCalibration  # noqa: E402
from modules.calibration.se3 import make_T  # noqa: E402
from modules.dynamixel.motor_config import load_motor_config  # noqa: E402

POSES_PATH = Path(__file__).parents[1] / "robot" / "calibration" / "handeye_poses.npz"

JOINT_ORIGINS = np.array(
    [
        [-0.01125, 0.0, 0.034],
        [0.0, 0.0, 0.0635],
        [0.0415, 0.0, 0.11315],
        [0.162, 0.0, 0.0],
        [0.0287, 0.0, 0.0],
    ],
    dtype=np.float64,
)
JOINT_AXES = np.array(
    [
        [0, 0, 1],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [1, 0, 0],
    ],
    dtype=np.float64,
)
EE_ORIGIN = np.array([0.09193, -0.0016, 0.0], dtype=np.float64)


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    c = np.cos(angle)
    s = np.sin(angle)
    K = np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=np.float64
    )
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)


def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    """Small-angle rotation: Rodrigues with rpy as rotation vector.
    각 성분이 작을 때(<5°) ZYX/XYZ 순서와 차이 무시 가능."""
    angle = np.linalg.norm(rpy)
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_R(rpy, angle)


def fk_chain(
    angles: np.ndarray,
    link_trans: np.ndarray,
    link_rot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """link_trans shape=(5,3) — joint i origin xyz에 더할 dx,dy,dz (m).
    link_rot shape=(5,3) — joint i origin frame에 적용할 rotation vector (rad).
    URDF (rpy=0, origin=JOINT_ORIGINS)에 더한 perturbation.
    """
    T = np.eye(4)
    for i in range(5):
        T_origin = np.eye(4)
        T_origin[:3, :3] = rpy_to_R(link_rot[i])
        T_origin[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_origin
        T_rot = np.eye(4)
        T_rot[:3, :3] = axis_angle_to_R(JOINT_AXES[i], angles[i])
        T = T @ T_rot
    T_ee = np.eye(4)
    T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return Tee[:3, :3].copy(), Tee[:3, 3].copy()


def mean_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    U, _, Vt = np.linalg.svd(M)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean


def main() -> None:
    he = HandEyeCalibration()
    n = he.load_poses(POSES_PATH)
    print(f"포즈 {n}개 로드")

    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
    coords = JointCoordinates()
    _ = coords.snapshot()

    angles_list: list[np.ndarray] = []
    R_tc_list: list[np.ndarray] = []
    t_tc_list: list[np.ndarray] = []
    for p in he.poses:
        a = np.array(
            [
                raw_to_rad(int(p.raw_motor_positions[c.id]), reverse=c.reverse)
                for c in arm_cfgs
            ],
            dtype=np.float64,
        )
        angles_list.append(a)
        R_tc_list.append(np.asarray(p.R_target2cam, dtype=np.float64))
        t_tc_list.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3))

    angles_arr = np.stack(angles_list)
    N = len(angles_list)
    T_tc_list = [make_T(R, t) for R, t in zip(R_tc_list, t_tc_list)]

    # cv2 TSAI seed
    R_gb_seed: list[np.ndarray] = []
    t_gb_seed: list[np.ndarray] = []
    zero_trans = np.zeros((5, 3))
    zero_rot = np.zeros((5, 3))
    for a in angles_list:
        R, t = fk_chain(a, zero_trans, zero_rot)
        R_gb_seed.append(R)
        t_gb_seed.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb_seed,
        t_gb_seed,
        R_tc_list,
        [t.reshape(3, 1) for t in t_tc_list],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    rod_seed, _ = cv2.Rodrigues(R_seed)
    t_seed_v = np.asarray(t_seed).reshape(3)

    def unpack(x: np.ndarray):
        return (
            x[:5],                         # joint_offset
            x[5:20].reshape(5, 3),          # link_translation (m)
            x[20:35].reshape(5, 3),         # link_rotation (rad)
            x[35:38],                       # rod
            x[38:41],                       # t
        )

    def compute_T_target_in_base(x: np.ndarray) -> list[np.ndarray]:
        offset, link_trans, link_rot, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out: list[np.ndarray] = []
        for i in range(N):
            R_gb, t_gb = fk_chain(angles_arr[i] + offset, link_trans, link_rot)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    # Regularization
    JOINT_REG_WEIGHT = 0.5       # joint_offset rad — 5° 부근에서 잔차 0.04
    LINK_TRANS_REG_WEIGHT = 1.0  # link_trans m — 15mm 부근에서 잔차 0.015
    LINK_ROT_REG_WEIGHT = 1.0    # link_rot rad — 2° 부근에서 잔차 0.035

    def residual(x: np.ndarray) -> np.ndarray:
        offset, link_trans, link_rot, _, _ = unpack(x)
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        res = np.empty(6 * N + 5 + 15 + 15, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos
        res[6 * N : 6 * N + 5] = JOINT_REG_WEIGHT * offset
        res[6 * N + 5 : 6 * N + 20] = LINK_TRANS_REG_WEIGHT * link_trans.flatten()
        res[6 * N + 20 : 6 * N + 35] = LINK_ROT_REG_WEIGHT * link_rot.flatten()
        return res

    x0 = np.concatenate(
        [
            np.zeros(5),
            np.zeros(15),
            np.zeros(15),
            rod_seed.flatten(),
            t_seed_v,
        ]
    )

    def stats(x: np.ndarray, label: str):
        T_list = compute_T_target_in_base(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        rots, ts = [], []
        for T in T_list:
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            rots.append(np.degrees(np.linalg.norm(rod_dev)))
            ts.append(np.linalg.norm(T[:3, 3] - mean_pos) * 1000.0)
        rots = np.array(rots)
        ts = np.array(ts)
        sigma_rot = float(np.sqrt(np.mean(rots**2)))
        sigma_t = float(np.sqrt(np.mean(ts**2)))
        offset, link_trans, link_rot, _, _ = unpack(x)
        print(f"\n[{label}]")
        print(f"  σ_rot={sigma_rot:.3f}°  σ_t={sigma_t:.2f}mm")
        print(f"  joint_offset_deg = {[round(float(np.degrees(o)),3) for o in offset]}")
        print("  link_translation (mm) + link_rotation (deg):")
        for i in range(5):
            tr = link_trans[i] * 1000.0
            ro = np.degrees(link_rot[i])
            print(
                f"    joint{i+1}: t=({tr[0]:+5.2f},{tr[1]:+5.2f},{tr[2]:+5.2f})mm  "
                f"r=({ro[0]:+5.2f},{ro[1]:+5.2f},{ro[2]:+5.2f})°"
            )
        return sigma_rot, sigma_t

    stats(x0, "초기 시드 (확장 변수 0)")

    print("\nLM 최적화 중...")
    result = least_squares(
        residual,
        x0,
        method="lm",
        max_nfev=3000,
        xtol=1e-12,
        ftol=1e-12,
    )
    print(f"  iter={result.nfev}  success={result.success}  cost={result.cost:.5f}")

    stats(result.x, "확장 BA 결과 (joint_offset + link_rotation)")


if __name__ == "__main__":
    main()
