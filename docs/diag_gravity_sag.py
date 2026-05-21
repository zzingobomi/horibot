"""중력 처짐(자세 의존 오차) 가설 검증.

배경: 확장 BA(41 DOF)가 σ_rot 1.30° / σ_t 9.3mm에서 정체. 외부 의견 —
"link_offset이 흡수한 것 중 일부는 사실 자세 의존 중력 처짐일 수 있다.
J2/J3는 중력 부하 크고, 11V 운용으로 토크 마진 작아 자세마다 sag 다르게 발생.
확장 BA는 그걸 *constant* link offset으로 잘못 근사 중일 가능성."

이게 맞다면:
  1. per-pose 잔차가 J2/J3 각도와 systematic 상관을 가져야 함
  2. 자세 의존 sag 모델을 BA 변수로 추가하면 σ가 더 떨어져야 함
  3. 그게 hold-out으로 generalize되어야 함 (overfit 아닌 진짜 system 보정)
  4. sag 추가 시 link_offset 값이 *줄어들어야* 함 (둘이 분리됨)

이게 안 맞으면:
  - 잔차가 자세 무관 (noise floor)
  - sag 모델 추가해도 σ 변화 없음 또는 link_offset과 trade-off (gauge freedom)
  - § 15i robust loss 교훈 그대로: σ만 보지 말고 파라미터 폭주 함께 확인

총 5 단계:
  [A] baseline 41 DOF BA + per-pose 잔차 추출
  [B] 잔차의 자세 의존성 — J2, J3 각도 vs 잔차 norm Pearson 상관
  [C] 잔차의 자세 의존성 — 팔 펼침 정도(EE horizontal extent) vs 잔차 비교
  [D] sag 모델 추가 BA 4가지 시나리오 비교
  [E] hold-out validation (best sag model)
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

# backend/ 경로 추가 (다른 진단 스크립트와 동일 패턴)
sys.path.insert(0, str(Path(__file__).parents[1] / "backend"))

from core.common import GRIPPER_ID  # noqa: E402
from core.joint_coordinates import JointCoordinates  # noqa: E402
from core.units import raw_to_rad  # noqa: E402
from modules.calibration.bundle_adjust import (  # noqa: E402
    bundle_adjust_hand_eye_extended,
)
from modules.calibration.hand_eye import HandEyeCalibration  # noqa: E402
from modules.calibration.se3 import make_T  # noqa: E402
from modules.dynamixel.motor_config import load_motor_config  # noqa: E402

POSES_PATH = Path(__file__).parents[1] / "robot" / "calibration" / "handeye_poses.npz"

# fk_chain 상수 — diag_handeye_validation.py와 동일
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


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    K = np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=np.float64
    )
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)


def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rpy))
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_R(rpy, angle)


def fk_chain(angles, link_trans, link_rot):
    """numpy fk chain — link_trans/link_rot 매개로 BA 변수 받음."""
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


def sigma_vs_mean(T_list, mean_pos=None, mean_R=None):
    """T_list의 흩어짐 σ_rot(deg) / σ_t(mm). mean을 받으면 그 기준, 안 받으면 자체 평균."""
    positions = np.array([T[:3, 3] for T in T_list])
    Rs = [T[:3, :3] for T in T_list]
    if mean_pos is None:
        mean_pos = positions.mean(axis=0)
    if mean_R is None:
        mean_R = mean_rotation(Rs)
    rots, ts = [], []
    for T in T_list:
        R_dev = T[:3, :3] @ mean_R.T
        rod_dev, _ = cv2.Rodrigues(R_dev)
        rots.append(np.degrees(np.linalg.norm(rod_dev)))
        ts.append(np.linalg.norm(T[:3, 3] - mean_pos) * 1000.0)
    rots = np.array(rots)
    ts = np.array(ts)
    return (
        float(np.sqrt(np.mean(rots**2))),
        float(np.sqrt(np.mean(ts**2))),
        rots,
        ts,
    )


# ─── 데이터 로드 ─────────────────────────────────────────────────────────


def load_data():
    he = HandEyeCalibration()
    n = he.load_poses(POSES_PATH)
    _, motor_cfgs = load_motor_config()
    arm_cfgs = [m for m in motor_cfgs if m.id != GRIPPER_ID]
    # JoinOffsets 디스크에 이미 commit 되어 있으면 그 값을 적용한 각도 사용
    # (현재 production state와 동일)
    jc = JointCoordinates()
    _ = jc.snapshot()

    angles_all = []
    R_tc_all = []
    t_tc_all = []
    for p in he.poses:
        a = np.array(
            [
                raw_to_rad(int(p.raw_motor_positions[c.id]), reverse=c.reverse)
                for c in arm_cfgs
            ],
            dtype=np.float64,
        )
        angles_all.append(a)
        R_tc_all.append(np.asarray(p.R_target2cam, dtype=np.float64))
        t_tc_all.append(np.asarray(p.t_target2cam, dtype=np.float64).reshape(3))
    angles_all = np.stack(angles_all)
    T_tc_all = [make_T(R, t) for R, t in zip(R_tc_all, t_tc_all)]
    return n, angles_all, R_tc_all, t_tc_all, T_tc_all


# ─── 단계 [A] baseline 확장 BA ────────────────────────────────────────────


def run_baseline(angles_all, R_tc_all, t_tc_all):
    """현재 production 확장 BA. seed = cv2.calibrateHandEye TSAI."""
    zero = np.zeros((5, 3))
    R_gb_seed, t_gb_seed = [], []
    for a in angles_all:
        R, t = fk_chain(a, zero, zero)
        R_gb_seed.append(R)
        t_gb_seed.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb_seed,
        t_gb_seed,
        R_tc_all,
        [t.reshape(3, 1) for t in t_tc_all],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    ba = bundle_adjust_hand_eye_extended(
        joint_angles_per_pose=[list(a) for a in angles_all],
        R_target2cam=R_tc_all,
        t_target2cam=t_tc_all,
        X_init=(R_seed, t_seed),
    )
    return ba


# ─── 단계 [B,C] 잔차 vs 자세 분석 ──────────────────────────────────────────


def analyze_residual_correlation(ba, angles_all):
    """per-pose 잔차가 J2/J3 각도와 상관 있나?

    핵심 신호:
      - corr(|res|, sin(θ_J2)) 크면 J2 horizontal-amount 의존 = 중력 sag signature
      - corr(|res|, |θ_J2|) 크면 J2 절대각 의존
      - corr(|res|, EE horizontal extent) 크면 모멘트 암 의존
    """
    rot_dev = ba.residual_rot_deg
    t_dev = ba.residual_t_mm

    theta_J2 = angles_all[:, 1]
    theta_J3 = angles_all[:, 2]
    theta_J23 = theta_J2 + theta_J3  # J3 link 절대 각도 (URDF 기준 zero에 의존)

    # 팔의 horizontal extent — 단순 근사: 어깨~EE의 수평 거리.
    # link 길이 합 ≈ 0.0635(J2 z) + 0.11315(J3 z) + 0.162(J3 X arm) + 0.0287 + 0.09193
    # 정확한 EE 위치는 fk_chain으로.
    horiz_ee = []
    for a in angles_all:
        _, t_ee = fk_chain(a, np.zeros((5, 3)), np.zeros((5, 3)))
        horiz_ee.append(np.hypot(t_ee[0], t_ee[1]))
    horiz_ee = np.array(horiz_ee)

    def corr(x, y):
        if np.std(x) < 1e-9 or np.std(y) < 1e-9:
            return 0.0
        return float(np.corrcoef(x, y)[0, 1])

    feats = {
        "θ_J2 (rad)": theta_J2,
        "sin(θ_J2)": np.sin(theta_J2),
        "cos(θ_J2)": np.cos(theta_J2),
        "|θ_J2|": np.abs(theta_J2),
        "θ_J3 (rad)": theta_J3,
        "sin(θ_J2+θ_J3)": np.sin(theta_J23),
        "cos(θ_J2+θ_J3)": np.cos(theta_J23),
        "EE horiz dist (m)": horiz_ee,
    }

    print("  [상관도] per-pose 잔차 norm vs 자세 feature (Pearson r)")
    print(f"  {'feature':<22s} {'r(|σ_rot|)':>11s} {'r(|σ_t|)':>11s}")
    for name, x in feats.items():
        r_rot = corr(x, rot_dev)
        r_t = corr(x, t_dev)
        # |r| > 0.4 면 의미 있는 상관, > 0.6 면 강한 상관
        mark_rot = " *" if abs(r_rot) > 0.4 else ""
        mark_t = " *" if abs(r_t) > 0.4 else ""
        print(f"  {name:<22s} {r_rot:>+10.3f}{mark_rot:<2s} {r_t:>+10.3f}{mark_t:<2s}")
    print("  (|r| > 0.4 = 약한 상관, > 0.6 = 강한 상관. * 표시)")
    return feats


# ─── 단계 [D] sag 모델 추가 BA ─────────────────────────────────────────────


def fit_ba_with_sag(
    angles_all,
    R_tc_all,
    t_tc_all,
    *,
    sag_mode: str,  # "none" | "j2_sincos" | "j23_sincos"
    use_link_offsets: bool,
    link_trans_reg: float = 1.0,
    link_rot_reg: float = 1.0,
    joint_offset_reg: float = 0.5,
    sag_reg: float = 0.1,
):
    """sag 모델 추가 BA. link_offset on/off 토글 가능.

    sag_mode:
      "none"        — 자세 의존 sag 없음 (기존 확장 BA와 동일 + link 토글)
      "j2_sincos"   — J2에만 자세 의존: δJ2 = a*sin(θ_J2) + b*cos(θ_J2)        +2 params
      "j23_sincos"  — J2/J3에 자세 의존: 4 params
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

    # 변수 layout
    n_off = 5
    n_lt = 15 if use_link_offsets else 0
    n_lr = 15 if use_link_offsets else 0
    if sag_mode == "none":
        n_sag = 0
    elif sag_mode == "j2_sincos":
        n_sag = 2
    elif sag_mode == "j23_sincos":
        n_sag = 4
    else:
        raise ValueError(sag_mode)

    def unpack(x):
        i = 0
        off = x[i:i + n_off]; i += n_off
        if use_link_offsets:
            lt = x[i:i + n_lt].reshape(5, 3); i += n_lt
            lr = x[i:i + n_lr].reshape(5, 3); i += n_lr
        else:
            lt = np.zeros((5, 3))
            lr = np.zeros((5, 3))
        sag = x[i:i + n_sag] if n_sag else np.zeros(0); i += n_sag
        rod = x[i:i + 3]; i += 3
        t_x = x[i:i + 3]
        return off, lt, lr, sag, rod, t_x

    def apply_sag(a, sag):
        """a: (5,) joint angles. sag: (n_sag,)."""
        a_out = a.copy()
        if sag_mode == "none":
            return a_out
        # J2 sin+cos
        a_out[1] += sag[0] * np.sin(a[1]) + sag[1] * np.cos(a[1])
        if sag_mode == "j23_sincos":
            # J3 absolute angle = θ_J2 + θ_J3
            abs_J3 = a[1] + a[2]
            a_out[2] += sag[2] * np.sin(abs_J3) + sag[3] * np.cos(abs_J3)
        return a_out

    def compute_T_list(x):
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
        T_list = compute_T_list(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        n_reg = n_off + n_lt + n_lr + n_sag
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
        if n_sag:
            res[k:k + n_sag] = sag_reg * sag; k += n_sag
        return res

    n_x = n_off + n_lt + n_lr + n_sag + 6
    x0 = np.zeros(n_x)
    x0[-6:-3] = rod_seed.flatten()
    x0[-3:] = t_seed_v

    result = least_squares(
        residual, x0, method="lm", max_nfev=5000, xtol=1e-11, ftol=1e-11
    )
    off, lt, lr, sag, rod, t_x = unpack(result.x)
    T_list = compute_T_list(result.x)
    sr, st, rot_per, t_per = sigma_vs_mean(T_list)
    return {
        "sigma_rot": sr,
        "sigma_t": st,
        "rot_per_pose_deg": rot_per,
        "t_per_pose_mm": t_per,
        "joint_offset_deg": np.degrees(off),
        "link_trans_mm_max": float(np.max(np.abs(lt)) * 1000.0) if use_link_offsets else 0.0,
        "link_rot_deg_max": float(np.degrees(np.max(np.abs(lr)))) if use_link_offsets else 0.0,
        "sag_params": sag,
        "x": result.x,
        "cost": float(result.cost),
        "dof": n_x,
        "n_data": 6 * N,
    }


# ─── 단계 [E] hold-out validation ──────────────────────────────────────────


def holdout_validate(
    angles_all,
    R_tc_all,
    t_tc_all,
    *,
    sag_mode: str,
    use_link_offsets: bool,
    n_seeds: int = 3,
    train_frac: float = 0.75,
):
    n = len(angles_all)
    n_train = int(n * train_frac)
    ratios = []
    print(f"  hold-out validation (train={n_train}, test={n - n_train}, seeds={n_seeds})")
    for seed in range(n_seeds):
        rng = np.random.default_rng(seed)
        idx = rng.permutation(n)
        tr_idx = idx[:n_train]
        te_idx = idx[n_train:]
        ang_tr = angles_all[tr_idx]
        R_tr = [R_tc_all[i] for i in tr_idx]
        t_tr = [t_tc_all[i] for i in tr_idx]
        ang_te = angles_all[te_idx]
        R_te = [R_tc_all[i] for i in te_idx]
        t_te = [t_tc_all[i] for i in te_idx]

        out_tr = fit_ba_with_sag(
            ang_tr, R_tr, t_tr, sag_mode=sag_mode, use_link_offsets=use_link_offsets
        )
        x_tr = out_tr["x"]

        # train으로 풀은 변수를 test 포즈에 그대로 적용 → T_list 계산
        # apply_sag/fk_chain/unpack 다시 호출하기 위해 fit_ba_with_sag 내부 로직 재현
        T_tc_tr = [make_T(R, t.reshape(3) if t.ndim == 2 else t) for R, t in zip(R_tr, t_tr)]
        T_tc_te = [make_T(R, t.reshape(3) if t.ndim == 2 else t) for R, t in zip(R_te, t_te)]

        n_off = 5
        n_lt = 15 if use_link_offsets else 0
        n_lr = 15 if use_link_offsets else 0
        n_sag = {"none": 0, "j2_sincos": 2, "j23_sincos": 4}[sag_mode]

        i = 0
        off = x_tr[i:i + n_off]; i += n_off
        if use_link_offsets:
            lt = x_tr[i:i + n_lt].reshape(5, 3); i += n_lt
            lr = x_tr[i:i + n_lr].reshape(5, 3); i += n_lr
        else:
            lt = np.zeros((5, 3)); lr = np.zeros((5, 3))
        sag = x_tr[i:i + n_sag] if n_sag else np.zeros(0); i += n_sag
        rod = x_tr[i:i + 3]; i += 3
        t_x = x_tr[i:i + 3]
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)

        def apply_sag(a):
            a2 = a.copy()
            if sag_mode == "none":
                return a2
            a2[1] += sag[0] * np.sin(a[1]) + sag[1] * np.cos(a[1])
            if sag_mode == "j23_sincos":
                abs_J3 = a[1] + a[2]
                a2[2] += sag[2] * np.sin(abs_J3) + sag[3] * np.cos(abs_J3)
            return a2

        def T_list_of(angs, T_tcs):
            out = []
            for a, T_tc in zip(angs, T_tcs):
                a_corr = apply_sag(a + off)
                R_gb, t_gb = fk_chain(a_corr, lt, lr)
                T_gb = make_T(R_gb, t_gb)
                out.append(T_gb @ T_x @ T_tc)
            return out

        T_train = T_list_of(ang_tr, T_tc_tr)
        T_test = T_list_of(ang_te, T_tc_te)
        positions_tr = np.array([T[:3, 3] for T in T_train])
        mean_pos = positions_tr.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_train])
        sr_tr, st_tr, _, _ = sigma_vs_mean(T_train, mean_pos, mean_R)
        sr_te, st_te, _, _ = sigma_vs_mean(T_test, mean_pos, mean_R)
        rr = sr_te / sr_tr if sr_tr > 1e-6 else float("inf")
        rt = st_te / st_tr if st_tr > 1e-6 else float("inf")
        ratios.append((sr_tr, st_tr, sr_te, st_te, rr, rt))
        print(
            f"    seed {seed}: train σ=({sr_tr:.3f}°, {st_tr:.2f}mm) "
            f"test σ=({sr_te:.3f}°, {st_te:.2f}mm) "
            f"ratio rot={rr:.2f}× t={rt:.2f}×"
        )
    avg = np.mean(ratios, axis=0)
    print(
        f"    평균:    train σ=({avg[0]:.3f}°, {avg[1]:.2f}mm) "
        f"test σ=({avg[2]:.3f}°, {avg[3]:.2f}mm) "
        f"ratio rot={avg[4]:.2f}× t={avg[5]:.2f}×"
    )
    return avg


# ─── main ─────────────────────────────────────────────────────────────────


def main():
    print("=" * 78)
    print("중력 처짐 가설 검증 — per-pose 자세 의존성 + sag 모델 추가 BA")
    print("=" * 78)
    n, angles_all, R_tc_all, t_tc_all, T_tc_all = load_data()
    print(f"포즈 {n}개 로드, J2 range [{np.degrees(angles_all[:,1]).min():.1f}, "
          f"{np.degrees(angles_all[:,1]).max():.1f}]°, "
          f"J3 range [{np.degrees(angles_all[:,2]).min():.1f}, "
          f"{np.degrees(angles_all[:,2]).max():.1f}]°")
    print()

    # ──[A]──
    print("[A] baseline 확장 BA (41 DOF, link on, sag off)")
    ba = run_baseline(angles_all, R_tc_all, t_tc_all)
    print(f"  σ_rot = {np.sqrt(np.mean(ba.residual_rot_deg**2)):.3f}°  "
          f"σ_t = {np.sqrt(np.mean(ba.residual_t_mm**2)):.2f}mm")
    print(f"  joint_offset (deg): "
          f"{', '.join(f'{np.degrees(v):+.2f}' for v in ba.joint_offset_rad)}")
    print(f"  link_trans max |Δ|: {np.max(np.abs(ba.link_trans_m))*1000:.2f}mm  "
          f"link_rot max |Δ|: {np.degrees(np.max(np.abs(ba.link_rot_rad))):.2f}°")
    print()

    # ──[B,C]──
    print("[B,C] per-pose 잔차의 자세 의존성")
    analyze_residual_correlation(ba, angles_all)
    print()

    # ──[D]──
    print("[D] sag 모델 추가 BA 시나리오 비교")
    scenarios = [
        ("(1) link off, sag off", "none", False),
        ("(2) link off, sag J2  ", "j2_sincos", False),
        ("(3) link off, sag J2+J3", "j23_sincos", False),
        ("(4) link on,  sag off  ", "none", True),
        ("(5) link on,  sag J2   ", "j2_sincos", True),
        ("(6) link on,  sag J2+J3", "j23_sincos", True),
    ]
    results = {}
    for label, sag_mode, use_link in scenarios:
        r = fit_ba_with_sag(
            angles_all, R_tc_all, t_tc_all,
            sag_mode=sag_mode, use_link_offsets=use_link,
        )
        results[label] = r
        sag_str = ""
        if r["sag_params"].size > 0:
            sag_deg = np.degrees(r["sag_params"])
            sag_str = "  sag(deg)=[" + ", ".join(f"{v:+.2f}" for v in sag_deg) + "]"
        print(
            f"  {label}  σ_rot={r['sigma_rot']:.3f}°  σ_t={r['sigma_t']:.2f}mm  "
            f"DOF={r['dof']:<3d}  "
            f"link_t_max={r['link_trans_mm_max']:.1f}mm  "
            f"link_r_max={r['link_rot_deg_max']:.2f}°{sag_str}"
        )
    print()

    # ──[E]──
    print("[E] hold-out validation")
    print("  (4) link on, sag off  ← 현재 production")
    holdout_validate(angles_all, R_tc_all, t_tc_all,
                      sag_mode="none", use_link_offsets=True)
    print("  (5) link on, sag J2")
    holdout_validate(angles_all, R_tc_all, t_tc_all,
                      sag_mode="j2_sincos", use_link_offsets=True)
    print("  (6) link on, sag J2+J3")
    holdout_validate(angles_all, R_tc_all, t_tc_all,
                      sag_mode="j23_sincos", use_link_offsets=True)
    print()
    print("=" * 78)
    print("판정:")
    print("  - 상관도(B,C)에서 |r| > 0.4 짜리 거의 없음        → 자세 의존 신호 약함")
    print("  - (5)/(6) σ가 (4) 대비 거의 안 떨어지면          → sag 모델 추가 효과 X")
    print("  - σ는 떨어지는데 link_t_max도 함께 커지면        → gauge freedom 흡수 (§15i)")
    print("  - hold-out ratio가 (4) 대비 (5)/(6)에서 크게 늘면 → overfit")
    print("  - 위 3 중 하나라도 해당하면 → 중력 처짐 가설 reject, intrinsic + 체커보드로")
    print("=" * 78)


if __name__ == "__main__":
    main()
