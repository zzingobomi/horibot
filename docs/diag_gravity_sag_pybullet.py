"""PyBullet 동역학 기반 sag 모델 — 검증된 라이브러리 사용.

이전 [diag_gravity_sag_physical.py](diag_gravity_sag_physical.py)는 lumped mass 가정
+ 직접 cross product 짜서 토크 계산. URDF가 정확한 mass/inertia(소수점 10자리)를
들고 있는데 그걸 활용 안 하는 건 reinvent. PyBullet에 `calculateInverseDynamics`가
있으니 그걸로 *distributed mass + Jacobian 정확* 동역학.

핵심 API:
    tau = p.calculateInverseDynamics(robot, q, v=[0]*n, a=[0]*n)
    # 정적 자세(v=a=0) → tau = pure gravity torque vector (각 joint별)

전략:
  - link_offset 변수와 *동시*로 풀기는 불가능 (PyBullet은 URDF 정적 로드)
  - 그래서 두 단계:
    Stage 1: 기존 확장 BA로 link/joint/X 풀고 patched URDF 생성
    Stage 2: patched URDF를 PyBullet에 로드 → gravity torque 함수 정의
             → sag k_J2, k_J3 + joint_offset 미세조정 + X 재최적화 (link 고정)

이 스크립트는 Stage 1+2를 같은 41 포즈로 돌리고 lumped 모델과 σ 비교.

판정:
  - PyBullet σ_rot < lumped σ_rot → 정확한 mass 모델이 의미 있음. 통합 채택.
  - 비슷하면 → URDF mass의 정확도와 lumped 단순화 효과 비슷. 단순성 위해 lumped 유지.
  - PyBullet σ가 더 나쁘면 → 단순 lumped가 우월하거나 URDF mass 부정확. 분석 필요.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np
import pybullet as p
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1] / "backend"))

from core.link_coordinates import LinkCoordinates  # noqa: E402
from core.urdf_patcher import write_patched_urdf  # noqa: E402
from diag_gravity_sag import (  # noqa: E402
    fk_chain,
    fit_ba_with_sag,
    load_data,
    mean_rotation,
    sigma_vs_mean,
)
from diag_gravity_sag_physical import fit_ba_with_physical_sag  # noqa: E402
from modules.calibration.bundle_adjust import (  # noqa: E402
    bundle_adjust_hand_eye_extended,
)
from modules.calibration.link_offsets import LinkOffsets  # noqa: E402
from modules.calibration.se3 import make_T  # noqa: E402

URDF_PATH = Path(__file__).parents[1] / "robot" / "urdf" / "omx_f" / "omx_f.urdf"


# ─── PyBullet 환경 ─────────────────────────────────────────────────────────


class PybulletDynamics:
    """patched URDF 로드 + calculateInverseDynamics 래퍼.

    BA의 한 단계가 끝날 때마다 새 link_offset으로 다시 만듦.
    """

    def __init__(self, link_trans: np.ndarray, link_rot: np.ndarray):
        # link_trans/link_rot → LinkOffsets dataclass → patched URDF
        # joint id는 1~5 (motor id와 동일)
        offsets = LinkOffsets(
            trans={i + 1: link_trans[i].copy() for i in range(5)},
            rot={i + 1: link_rot[i].copy() for i in range(5)},
        )
        self._urdf_path = write_patched_urdf(URDF_PATH, offsets)

        self._client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        self._robot = p.loadURDF(
            str(self._urdf_path),
            useFixedBase=True,
            physicsClientId=self._client,
        )

        # arm joint indices (revolute joint들 처음 5개)
        self._joint_indices: list[int] = []
        num = p.getNumJoints(self._robot, physicsClientId=self._client)
        for i in range(num):
            info = p.getJointInfo(self._robot, i, physicsClientId=self._client)
            if info[2] == p.JOINT_REVOLUTE:
                self._joint_indices.append(i)
        # arm은 처음 5개 + gripper 6번째
        self._n_revolute = len(self._joint_indices)

    def gravity_torque(self, angles_5: np.ndarray) -> np.ndarray:
        """5축 arm angle → 5축 gravity torque (Nm)."""
        # PyBullet은 *모든* 운동학 자유도 (gripper 포함)를 받아야 함
        q = [0.0] * self._n_revolute
        for i in range(min(5, self._n_revolute)):
            q[i] = float(angles_5[i])
        v = [0.0] * self._n_revolute
        a = [0.0] * self._n_revolute
        tau = p.calculateInverseDynamics(
            self._robot, q, v, a, physicsClientId=self._client
        )
        return np.array(tau[:5], dtype=np.float64)

    def close(self):
        if p.isConnected(self._client):
            p.disconnect(self._client)


# ─── Stage 2: PyBullet 토크 기반 sag BA ────────────────────────────────────


def fit_sag_pybullet(
    angles_all,
    R_tc_all,
    t_tc_all,
    *,
    link_trans_init: np.ndarray,
    link_rot_init: np.ndarray,
    joint_offset_init: np.ndarray,
    X_init: tuple[np.ndarray, np.ndarray],
    k_reg: float = 0.0,
    joint_offset_reg: float = 0.5,
    optimize_joint_offset: bool = True,
):
    """Stage 2 — link/sag/joint offset/X를 PyBullet 동역학으로 풀기.

    link offset은 *상수*로 고정 (Stage 1에서 풀은 값). 변수:
      [0:5]    joint_offset (optimize_joint_offset=True 시)
      [5:7]    sag k_J2, k_J3
      [7:10]   rod (cam2gripper)
      [10:13]  t (cam2gripper)
    총 13 DOF.
    """
    N = len(angles_all)
    angles_arr = np.array(angles_all, dtype=np.float64)
    T_tc_list = [
        make_T(np.asarray(R), np.asarray(t).reshape(3))
        for R, t in zip(R_tc_all, t_tc_all)
    ]

    # PyBullet 동역학 인스턴스 (link offset 고정으로 1회 생성)
    dyn = PybulletDynamics(link_trans_init, link_rot_init)

    rod_seed, _ = cv2.Rodrigues(np.asarray(X_init[0]))
    t_seed_v = np.asarray(X_init[1]).reshape(3)

    n_off = 5 if optimize_joint_offset else 0
    n_k = 2

    def unpack(x):
        i = 0
        if optimize_joint_offset:
            off = x[i:i + n_off]; i += n_off
        else:
            off = joint_offset_init.copy()
        k_stiff = x[i:i + n_k]; i += n_k
        rod = x[i:i + 3]; i += 3
        t_x = x[i:i + 3]
        return off, k_stiff, rod, t_x

    def apply_sag_pybullet(angles, k_stiff):
        """PyBullet으로 gravity torque 계산 → sag = k * tau."""
        tau = dyn.gravity_torque(angles)
        a = angles.copy()
        a[1] += k_stiff[0] * tau[1]   # J2 (index 1)
        a[2] += k_stiff[1] * tau[2]   # J3 (index 2)
        return a

    def compute_T(x):
        off, k_stiff, rod, t_x = unpack(x)
        R_x = cv2.Rodrigues(rod)[0]
        T_x = make_T(R_x, t_x)
        out = []
        for i in range(N):
            a_corr = apply_sag_pybullet(angles_arr[i] + off, k_stiff)
            R_gb, t_gb = fk_chain(a_corr, link_trans_init, link_rot_init)
            T_gb = make_T(R_gb, t_gb)
            out.append(T_gb @ T_x @ T_tc_list[i])
        return out

    def residual(x):
        off, k_stiff, _, _ = unpack(x)
        T_list = compute_T(x)
        positions = np.array([T[:3, 3] for T in T_list])
        mean_pos = positions.mean(axis=0)
        mean_R = mean_rotation([T[:3, :3] for T in T_list])
        n_reg = n_off + n_k
        res = np.empty(6 * N + n_reg, dtype=np.float64)
        for i, T in enumerate(T_list):
            R_dev = T[:3, :3] @ mean_R.T
            rod_dev, _ = cv2.Rodrigues(R_dev)
            res[6 * i:6 * i + 3] = rod_dev.flatten()
            res[6 * i + 3:6 * (i + 1)] = T[:3, 3] - mean_pos
        k = 6 * N
        if optimize_joint_offset:
            res[k:k + n_off] = joint_offset_reg * off; k += n_off
        res[k:k + n_k] = k_reg * k_stiff
        return res

    n_x = n_off + n_k + 6
    x0 = np.zeros(n_x)
    # joint_offset init은 0 (delta) — Stage 1 결과는 이미 link_offset에 흡수됐고
    # Stage 2의 joint_offset은 *추가 미세조정*
    x0[-6:-3] = rod_seed.flatten()
    x0[-3:] = t_seed_v

    result = least_squares(
        residual, x0, method="lm", max_nfev=5000, xtol=1e-11, ftol=1e-11
    )
    off, k_stiff, rod, t_x = unpack(result.x)
    T_list = compute_T(result.x)
    sr, st, _, _ = sigma_vs_mean(T_list)

    # 최대 sag (deg) — PyBullet 토크 기준
    max_sag_J2_deg = 0.0
    max_sag_J3_deg = 0.0
    for a in angles_arr:
        tau = dyn.gravity_torque(a + off)
        s2 = abs(np.degrees(k_stiff[0] * tau[1]))
        s3 = abs(np.degrees(k_stiff[1] * tau[2]))
        max_sag_J2_deg = max(max_sag_J2_deg, s2)
        max_sag_J3_deg = max(max_sag_J3_deg, s3)

    dyn.close()

    return {
        "sigma_rot": sr,
        "sigma_t": st,
        "joint_offset_deg": np.degrees(off),
        "k_stiff": k_stiff,
        "max_sag_J2_deg": max_sag_J2_deg,
        "max_sag_J3_deg": max_sag_J3_deg,
        "x": result.x,
        "dof": n_x,
        "cost": float(result.cost),
    }


# ─── main 비교 ─────────────────────────────────────────────────────────────


def main():
    print("=" * 78)
    print("PyBullet 동역학 기반 sag 모델 vs lumped mass 모델")
    print("=" * 78)
    n, angles_all, R_tc_all, t_tc_all, _ = load_data()
    print(f"포즈 {n}개 로드\n")

    # ──[1] 기존 확장 BA (Stage 1) — link/joint/X 풀기 ────────────
    print("[Stage 1] 기존 확장 BA — link_offset + joint_offset + X 풀기")
    R_gb_seed, t_gb_seed = [], []
    zero = np.zeros((5, 3))
    for a in angles_all:
        R, t = fk_chain(a, zero, zero)
        R_gb_seed.append(R)
        t_gb_seed.append(t.reshape(3, 1))
    R_seed, t_seed = cv2.calibrateHandEye(
        R_gb_seed, t_gb_seed, R_tc_all,
        [t.reshape(3, 1) for t in t_tc_all],
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    ba1 = bundle_adjust_hand_eye_extended(
        joint_angles_per_pose=[list(a) for a in angles_all],
        R_target2cam=R_tc_all,
        t_target2cam=t_tc_all,
        X_init=(R_seed, t_seed),
    )
    sigma1_rot = float(np.sqrt(np.mean(ba1.residual_rot_deg**2)))
    sigma1_t = float(np.sqrt(np.mean(ba1.residual_t_mm**2)))
    print(f"  Stage 1 결과: σ_rot={sigma1_rot:.3f}°  σ_t={sigma1_t:.2f}mm  "
          f"link_t_max={np.max(np.abs(ba1.link_trans_m))*1000:.1f}mm")
    print()

    # ──[2] Stage 2: PyBullet 동역학 기반 sag ─────────────────────
    print("[Stage 2] PyBullet calculateInverseDynamics → sag k_J2, k_J3 풀기")
    print("          (Stage 1의 link/joint_offset/X 시드로)")
    res_pyb = fit_sag_pybullet(
        angles_all, R_tc_all, t_tc_all,
        link_trans_init=ba1.link_trans_m,
        link_rot_init=ba1.link_rot_rad,
        joint_offset_init=ba1.joint_offset_rad,
        X_init=(ba1.R_cam2gripper, ba1.t_cam2gripper),
    )
    print(f"  σ_rot={res_pyb['sigma_rot']:.3f}°  σ_t={res_pyb['sigma_t']:.2f}mm  "
          f"DOF={res_pyb['dof']}")
    print(f"  k_stiff = ({res_pyb['k_stiff'][0]:+.5f}, "
          f"{res_pyb['k_stiff'][1]:+.5f})  (rad/Nm)")
    print(f"  최대 sag: J2={res_pyb['max_sag_J2_deg']:+.2f}°, "
          f"J3={res_pyb['max_sag_J3_deg']:+.2f}°")
    print(f"  추가 joint_offset (deg): "
          f"{', '.join(f'{v:+.3f}' for v in res_pyb['joint_offset_deg'])}")
    print()

    # ──[3] lumped mass 모델 비교 ─────────────────────────────────
    print("[비교] lumped mass 모델 (이전 [diag_gravity_sag_physical.py])")
    res_lumped = fit_ba_with_physical_sag(
        angles_all, R_tc_all, t_tc_all,
        use_link_offsets=True,
    )
    print(f"  σ_rot={res_lumped['sigma_rot']:.3f}°  "
          f"σ_t={res_lumped['sigma_t']:.2f}mm  DOF={res_lumped['dof']}")
    print(f"  k_stiff = ({res_lumped['k_stiff'][0]:+.5f}, "
          f"{res_lumped['k_stiff'][1]:+.5f})  (rad/(m·g_unit))")
    print(f"  최대 sag: J2={res_lumped['max_sag_J2_deg']:+.2f}°, "
          f"J3={res_lumped['max_sag_J3_deg']:+.2f}°")
    print()

    # ──[4] sag off baseline ────────────────────────────────────────
    print("[참고] 현재 production (sag off)")
    res_baseline = fit_ba_with_sag(
        angles_all, R_tc_all, t_tc_all,
        sag_mode="none", use_link_offsets=True,
    )
    print(f"  σ_rot={res_baseline['sigma_rot']:.3f}°  "
          f"σ_t={res_baseline['sigma_t']:.2f}mm")
    print()

    # ──[5] 요약 ──────────────────────────────────────────────────
    print("=" * 78)
    print("요약")
    print("=" * 78)
    print(f"  {'모델':<35s} {'σ_rot':>8s} {'σ_t':>8s}  비고")
    print(f"  {'sag off (현 prod)':<35s} {res_baseline['sigma_rot']:>7.3f}° "
          f"{res_baseline['sigma_t']:>7.2f}mm  baseline")
    print(f"  {'lumped mass (직접 cross product)':<35s} "
          f"{res_lumped['sigma_rot']:>7.3f}° "
          f"{res_lumped['sigma_t']:>7.2f}mm  단순 모델")
    print(f"  {'PyBullet (calculateInverseDynamics)':<35s} "
          f"{res_pyb['sigma_rot']:>7.3f}° "
          f"{res_pyb['sigma_t']:>7.2f}mm  distributed mass")
    print()
    diff_rot = res_pyb["sigma_rot"] - res_lumped["sigma_rot"]
    diff_t = res_pyb["sigma_t"] - res_lumped["sigma_t"]
    print(f"  PyBullet vs lumped: Δσ_rot={diff_rot:+.3f}°, "
          f"Δσ_t={diff_t:+.2f}mm")
    if diff_rot < -0.05:
        print("  → PyBullet이 σ_rot 0.05° 이상 우월. URDF mass 정확 → 채택 가치 있음.")
    elif abs(diff_rot) < 0.05:
        print("  → 비슷. URDF mass 정확하지만 lumped 단순화 효과 작음.")
        print("     단순성 위해 lumped 유지 고려 (PyBullet은 2단계 BA가 복잡).")
    else:
        print("  → lumped가 더 좋음. URDF mass 의심 또는 distributed mass의 추가 자유도가")
        print("     데이터 적게 fit 어렵게 만들 가능성. 분석 필요.")


if __name__ == "__main__":
    main()
