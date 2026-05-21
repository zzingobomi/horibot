"""v3 final hold-out validation — 41 DOF BA가 진짜 generalize하는지 검증.

가설: σ_t 9.3mm는 link offset과 hand-eye t의 trade-off로 trained 자세에만 fit
한 overfit일 수 있음. 진짜 system 보정이라면 새 자세에서도 σ 비슷.

방법:
  - 32포즈 random shuffle → train(24) / test(8) split
  - train으로 v3 BA 풀기
  - 추정된 X, joint_offset, link_trans, link_rot 고정한 채 test 포즈의 σ 계산
  - train σ ≈ test σ → generalize, BA가 진짜 system 파라미터 잡은 것
  - train σ << test σ → overfit, 자유도 과다로 train만 fit
  안정성 위해 3 seed로 반복.

해석 기준:
  test σ / train σ < 1.5 — 양호 (generalize 잘 됨)
  test σ / train σ 1.5~3.0 — 경계 (자유도 좀 과다, reg 더 강하게 또는 자세 추가)
  test σ / train σ > 3.0 — overfit 명확 (자유도 줄여야 함)
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
    [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0]], dtype=np.float64
)
EE_ORIGIN = np.array([0.09193, -0.0016, 0.0], dtype=np.float64)

JOINT_REG_WEIGHT = 0.5
LINK_TRANS_REG_WEIGHT = 1.0
LINK_ROT_REG_WEIGHT = 1.0

TRAIN_FRAC = 0.75
N_SEEDS = 3


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    c = np.cos(angle)
    s = np.sin(angle)
    K = np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=np.float64
    )
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)


def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(rpy)
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_R(rpy, angle)


def fk_chain(angles, link_trans, link_rot):
    T = np.eye(4)
    for i in range(5):
        T_o = np.eye(4)
        T_o[:3, :3] = rpy_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], angles[i])
        T = T @ T_r
    T_ee = np.eye(4)
    T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return Tee[:3, :3].copy(), Tee[:3, 3].copy()


def mean_rotation(Rs):
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    U, _, Vt = np.linalg.svd(M)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean


def unpack(x):
    return x[:5], x[5:20].reshape(5, 3), x[20:35].reshape(5, 3), x[35:38], x[38:41]


def compute_T_list(x, angles_arr, T_tc_list):
    offset, link_t, link_r, rod, t_x = unpack(x)
    R_x = cv2.Rodrigues(rod)[0]
    T_x = make_T(R_x, t_x)
    out = []
    for i in range(len(angles_arr)):
        R_gb, t_gb = fk_chain(angles_arr[i] + offset, link_t, link_r)
        T_gb = make_T(R_gb, t_gb)
        out.append(T_gb @ T_x @ T_tc_list[i])
    return out


def sigma(T_list):
    positions = np.array([T[:3, 3] for T in T_list])
    mean_pos = positions.mean(axis=0)
    mean_R = mean_rotation([T[:3, :3] for T in T_list])
    rots, ts = [], []
    for T in T_list:
        R_dev = T[:3, :3] @ mean_R.T
        rod_dev, _ = cv2.Rodrigues(R_dev)
        rots.append(np.degrees(np.linalg.norm(rod_dev)))
        ts.append(np.linalg.norm(T[:3, 3] - mean_pos) * 1000.0)
    return float(np.sqrt(np.mean(np.array(rots) ** 2))), float(
        np.sqrt(np.mean(np.array(ts) ** 2))
    )


def fit_train(angles_arr, R_tc_train, t_tc_train, T_tc_train):
    R_gb_seed, t_gb_seed = [], []
    zero = np.zeros((5, 3))
    for a in angles_arr:
        R, t = fk_chain(a, zero, zero)
        R_gb_seed.append(R)
        t_gb_seed.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb_seed, t_gb_seed, R_tc_train,
        [t.reshape(3, 1) for t in t_tc_train],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    rod_seed, _ = cv2.Rodrigues(R_seed)
    t_seed_v = np.asarray(t_seed).reshape(3)

    def residual(x):
        offset, link_t, link_r, _, _ = unpack(x)
        T_list = compute_T_list(x, angles_arr, T_tc_train)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        N = len(T_list)
        res = np.empty(6 * N + 35, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i : 6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3 : 6 * (i + 1)] = T[:3, 3] - mean_pos
        res[6 * N : 6 * N + 5] = JOINT_REG_WEIGHT * offset
        res[6 * N + 5 : 6 * N + 20] = LINK_TRANS_REG_WEIGHT * link_t.flatten()
        res[6 * N + 20 : 6 * N + 35] = LINK_ROT_REG_WEIGHT * link_r.flatten()
        return res

    x0 = np.concatenate([np.zeros(5), np.zeros(15), np.zeros(15), rod_seed.flatten(), t_seed_v])
    result = least_squares(residual, x0, method="lm", max_nfev=3000, xtol=1e-12, ftol=1e-12)
    return result.x


def main():
    he = HandEyeCalibration()
    n = he.load_poses(POSES_PATH)
    print(f"포즈 {n}개 로드, train_frac={TRAIN_FRAC}, seeds={N_SEEDS}")

    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
    _ = JointCoordinates().snapshot()

    angles_all = []
    R_tc_all = []
    t_tc_all = []
    for p in he.poses:
        a = np.array(
            [raw_to_rad(int(p.raw_motor_positions[c.id]), reverse=c.reverse) for c in arm_cfgs],
            dtype=np.float64,
        )
        angles_all.append(a)
        R_tc_all.append(np.asarray(p.R_target2cam, dtype=np.float64))
        t_tc_all.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3))
    angles_all = np.stack(angles_all)
    T_tc_all = [make_T(R, t) for R, t in zip(R_tc_all, t_tc_all)]

    n_train = int(n * TRAIN_FRAC)
    print(f"  train={n_train}, test={n - n_train}")
    print()

    rows = []
    for seed in range(N_SEEDS):
        rng = np.random.default_rng(seed)
        idx = rng.permutation(n)
        train_idx = idx[:n_train]
        test_idx = idx[n_train:]

        angles_tr = angles_all[train_idx]
        R_tc_tr = [R_tc_all[i] for i in train_idx]
        t_tc_tr = [t_tc_all[i] for i in train_idx]
        T_tc_tr = [T_tc_all[i] for i in train_idx]

        angles_te = angles_all[test_idx]
        T_tc_te = [T_tc_all[i] for i in test_idx]

        x_opt = fit_train(angles_tr, R_tc_tr, t_tc_tr, T_tc_tr)

        T_train = compute_T_list(x_opt, angles_tr, T_tc_tr)
        T_test = compute_T_list(x_opt, angles_te, T_tc_te)

        # 중요: test σ는 train과 *같은 mean_pos/mean_R* 기준이어야 의미 있음.
        # 체커보드는 안 움직였으니 train으로 추정한 board pose가 base of truth.
        positions_tr = np.array([T[:3, 3] for T in T_train])
        mean_pos = positions_tr.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_train])

        def sigma_against_train(T_list):
            rots, ts = [], []
            for T in T_list:
                R_dev = T[:3, :3] @ mean_R.T
                rod_dev, _ = cv2.Rodrigues(R_dev)
                rots.append(np.degrees(np.linalg.norm(rod_dev)))
                ts.append(np.linalg.norm(T[:3, 3] - mean_pos) * 1000.0)
            return (
                float(np.sqrt(np.mean(np.array(rots) ** 2))),
                float(np.sqrt(np.mean(np.array(ts) ** 2))),
            )

        sr_tr, st_tr = sigma_against_train(T_train)
        sr_te, st_te = sigma_against_train(T_test)
        ratio_rot = sr_te / sr_tr if sr_tr > 1e-6 else float("inf")
        ratio_t = st_te / st_tr if st_tr > 1e-6 else float("inf")

        rows.append((seed, sr_tr, st_tr, sr_te, st_te, ratio_rot, ratio_t))
        print(
            f"seed {seed}: "
            f"train σ_rot={sr_tr:.3f}° σ_t={st_tr:.2f}mm  "
            f"test σ_rot={sr_te:.3f}° σ_t={st_te:.2f}mm  "
            f"ratio rot={ratio_rot:.2f}× t={ratio_t:.2f}×"
        )

    # 평균
    avg_tr_r = np.mean([r[1] for r in rows])
    avg_tr_t = np.mean([r[2] for r in rows])
    avg_te_r = np.mean([r[3] for r in rows])
    avg_te_t = np.mean([r[4] for r in rows])
    avg_ratio_r = avg_te_r / avg_tr_r
    avg_ratio_t = avg_te_t / avg_tr_t

    print()
    print(f"평균: train σ=({avg_tr_r:.3f}°, {avg_tr_t:.2f}mm)  "
          f"test σ=({avg_te_r:.3f}°, {avg_te_t:.2f}mm)")
    print(f"평균 ratio: rot={avg_ratio_r:.2f}×  t={avg_ratio_t:.2f}×")
    print()
    if avg_ratio_r < 1.5 and avg_ratio_t < 1.5:
        print("✓ 양호 — BA가 진짜 system 파라미터 잡았음. 통합 진행 OK.")
    elif avg_ratio_r < 3.0 and avg_ratio_t < 3.0:
        print("△ 경계 — 자유도 좀 과다할 수 있음. reg 강하게 하거나 자세 추가 검토.")
    else:
        print("✗ overfit — 자유도 줄여야 함. v2(link_rot만) 또는 자세 더 캡처.")


if __name__ == "__main__":
    main()
