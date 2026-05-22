"""OMX_F의 URDF chain을 numpy로 직접 구현한 FK.

PyBullet은 URDF 파일을 *정적으로* 로드해 link transform이 부팅 후 고정.
Hand-Eye BA는 link origin offset을 *변수*로 풀어야 하므로 매 LM iteration마다
다른 link_offset으로 FK를 호출해야 함 — PyBullet 우회 필요.

이 모듈의 사용처:
    (a) `bundle_adjust.py`의 확장 BA — link offset 자유도와 함께 FK 평가
    (b) PybulletSolver / urdf_patcher가 같은 URDF 상수를 공유 (single source)

URDF 변경 시 sync 필요:
    URDF의 모든 <joint><origin rpy/> 가 "0 0 0" 가정.  rpy 비0 joint가 추가되면
    `RPY_BASE` 같이 base 회전을 명시해 chain에 곱하도록 확장.

[robot/urdf/omx_f/omx_f.urdf](robot/urdf/omx_f/omx_f.urdf) chain:
    world → link0 → [j1, z] → link1 → [j2, y] → link2 → [j3, y] → link3
          → [j4, y] → link4 → [j5, x] → link5 → (fixed) → end_effector_link
"""

from __future__ import annotations

import numpy as np

# joint i origin xyz (m). URDF의 <joint><origin xyz="..."/>.  motor id 1~5와 일치.
JOINT_ORIGINS: np.ndarray = np.array(
    [
        [-0.01125, 0.0, 0.034],     # joint1 (link0 → link1)
        [0.0, 0.0, 0.0635],          # joint2
        [0.0415, 0.0, 0.11315],      # joint3
        [0.162, 0.0, 0.0],            # joint4
        [0.0287, 0.0, 0.0],           # joint5
    ],
    dtype=np.float64,
)

# joint i axis. URDF의 <joint><axis xyz="..."/>.
JOINT_AXES: np.ndarray = np.array(
    [
        [0, 0, 1],  # joint1: z
        [0, 1, 0],  # joint2: y
        [0, 1, 0],  # joint3: y
        [0, 1, 0],  # joint4: y
        [1, 0, 0],  # joint5: x
    ],
    dtype=np.float64,
)

# link5 → end_effector_link fixed transform (URDF의 end_effector_joint).
EE_ORIGIN: np.ndarray = np.array([0.09193, -0.0016, 0.0], dtype=np.float64)

# arm joint 개수 (gripper 제외 — IK/FK 대상).
N_JOINTS: int = 5


def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues — axis는 정규화 안 돼있어도 OK."""
    a = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(a))
    if norm < 1e-12:
        return np.eye(3)
    a = a / norm
    c = np.cos(angle)
    s = np.sin(angle)
    K = np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=np.float64
    )
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)


def rotvec_to_R(rotvec: np.ndarray) -> np.ndarray:
    """rotation vector(= axis * angle) → 3x3. 작은 각(<5°)에서 정확."""
    rv = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_to_R(rv, angle)


def fk_chain(
    joint_angles: np.ndarray,
    link_trans: np.ndarray | None = None,
    link_rot: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """OMX_F FK with optional link offset patch.

    Args:
        joint_angles: shape (5,) — joint 1~5 각도 (rad, URDF 기준).
        link_trans: shape (5,3) or None — joint i origin xyz에 더할 dx,dy,dz (m).
        link_rot: shape (5,3) or None — joint i origin frame에 적용할 rotvec (rad).

    Returns:
        (R, t) — end_effector_link의 world frame 자세. R is 3x3, t is (3,).
    """
    if link_trans is None:
        link_trans = np.zeros((N_JOINTS, 3), dtype=np.float64)
    if link_rot is None:
        link_rot = np.zeros((N_JOINTS, 3), dtype=np.float64)

    angles = np.asarray(joint_angles, dtype=np.float64)
    T = np.eye(4)
    for i in range(N_JOINTS):
        # joint i origin transform (URDF base + offset patch)
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        # joint i revolute rotation
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], float(angles[i]))
        T = T @ T_r

    # fixed end_effector_joint
    T_ee = np.eye(4)
    T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return Tee[:3, :3].copy(), Tee[:3, 3].copy()


def fk_chain_with_axes(
    joint_angles: np.ndarray,
    link_trans: np.ndarray | None = None,
    link_rot: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """fk_chain + 각 joint origin/axis (base frame). 중력 토크 계산에 사용.

    joint i의 origin은 *그 joint의 회전 적용 전* base 좌표 (회전축 위치).
    axis는 base에서 본 단위 회전축 방향.

    Returns:
        (R_ee, t_ee, joint_origins_base, joint_axes_base)
        joint_origins_base: shape (5, 3) m
        joint_axes_base: shape (5, 3) unit vectors
    """
    if link_trans is None:
        link_trans = np.zeros((N_JOINTS, 3), dtype=np.float64)
    if link_rot is None:
        link_rot = np.zeros((N_JOINTS, 3), dtype=np.float64)

    angles = np.asarray(joint_angles, dtype=np.float64)
    T = np.eye(4)
    joint_origins_base = np.zeros((N_JOINTS, 3), dtype=np.float64)
    joint_axes_base = np.zeros((N_JOINTS, 3), dtype=np.float64)
    for i in range(N_JOINTS):
        T_o = np.eye(4)
        T_o[:3, :3] = rotvec_to_R(link_rot[i])
        T_o[:3, 3] = JOINT_ORIGINS[i] + link_trans[i]
        T = T @ T_o
        # 이 시점에서 T는 joint i frame in base (회전 적용 전)
        joint_origins_base[i] = T[:3, 3]
        joint_axes_base[i] = T[:3, :3] @ JOINT_AXES[i]
        T_r = np.eye(4)
        T_r[:3, :3] = axis_angle_to_R(JOINT_AXES[i], float(angles[i]))
        T = T @ T_r

    T_ee = np.eye(4)
    T_ee[:3, 3] = EE_ORIGIN
    Tee = T @ T_ee
    return (
        Tee[:3, :3].copy(),
        Tee[:3, 3].copy(),
        joint_origins_base,
        joint_axes_base,
    )


# 중력 방향 — base frame의 -z. SI 단위 g·m로 normalize (k가 rad/(m·g_unit) 단위 흡수).
_GRAVITY_DIR = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def gravity_torque_lumped(
    ee_pos_base: np.ndarray,
    joint_origin_base: np.ndarray,
    joint_axis_base: np.ndarray,
) -> float:
    """ee에 lumped mass 가정. joint 회전축에 작용하는 중력 토크 (sign + magnitude).

    τ = (r × g_dir) · axis   where r = ee - joint_origin   (모멘트 암 벡터)

    Units: r은 m, g_dir/axis는 unit → τ는 m. k(=1/effective_stiffness) 곱하면 rad.

    URDF의 distributed mass 대신 lumped 가정을 쓰는 이유:
        - URDF mass가 D405 카메라 교체 무게를 반영 못 함 (44g vs D405 42g)
        - PyBullet calculateInverseDynamics보다 σ에서 우월 (0.65° vs 0.77°)
        - k가 effective (stiffness × mass) 비율을 통째로 흡수해 mass 부정확성에 robust
    """
    r = np.asarray(ee_pos_base) - np.asarray(joint_origin_base)
    return float(np.dot(np.cross(r, _GRAVITY_DIR), np.asarray(joint_axis_base)))


def apply_gravity_sag(
    joint_angles: np.ndarray,
    k_stiff: np.ndarray,
    link_trans: np.ndarray | None = None,
    link_rot: np.ndarray | None = None,
) -> np.ndarray:
    """commanded joint angles → sag 적용된 actual angles.

    모델: actual = commanded + sag_offset(commanded)
        sag_offset_J = k_J * τ_J   where τ_J = gravity_torque_lumped(ee, joint_J)

    현재 모델은 J2, J3에만 sag (DIY 5축에서 중력 부하 가장 큰 두 joint).
    J1/J4/J5의 sag는 측정 noise 수준이라 모델 단순성 위해 제외.

    Args:
        joint_angles: shape (5,) — commanded angles (rad).
        k_stiff: shape (2,) — k_J2, k_J3 (rad/(m·g_unit)). 빈 배열 또는 [0,0]이면 no-op.
        link_trans/link_rot: fk_chain_with_axes로 전달.

    Returns:
        shape (5,) — sag 적용 angles. J2/J3 외엔 그대로.
    """
    k = np.asarray(k_stiff, dtype=np.float64)
    if k.size == 0 or float(np.max(np.abs(k))) < 1e-12:
        return np.asarray(joint_angles, dtype=np.float64).copy()
    _, ee_pos, joint_origins, joint_axes = fk_chain_with_axes(
        joint_angles, link_trans, link_rot
    )
    tau_J2 = gravity_torque_lumped(ee_pos, joint_origins[1], joint_axes[1])
    tau_J3 = gravity_torque_lumped(ee_pos, joint_origins[2], joint_axes[2])
    out = np.asarray(joint_angles, dtype=np.float64).copy()
    out[1] += k[0] * tau_J2
    out[2] += k[1] * tau_J3
    return out


def actual_to_commanded(
    actual_angles: np.ndarray,
    k_stiff: np.ndarray,
    link_trans: np.ndarray | None = None,
    link_rot: np.ndarray | None = None,
) -> np.ndarray:
    """`apply_gravity_sag`의 역방향 (IK용). 1차 근사.

    모델: actual = commanded + sag_offset(commanded)
    Implicit: commanded = actual - sag_offset(commanded)
    1st-order: commanded ≈ actual - sag_offset(actual)
              (sag_offset varies slowly, commanded ≈ actual이라 OK)

    sag ~2° 수준에서 1차 근사 오차는 < 0.05° (Taylor 잔차). fixed-point 1 iter면
    완전 수렴하지만 perf vs 정확도 트레이드오프상 1차로 충분.

    Args:
        actual_angles: PyBullet IK 결과 (URDF의 static fk가 target에 도달하는 angle).
        k_stiff, link_trans, link_rot: apply_gravity_sag와 동일.

    Returns:
        commanded angles — 모터에 이 값 명령하면 sag 후 actual에 도달, 즉 target_ee 달성.
    """
    a = np.asarray(actual_angles, dtype=np.float64)
    sag_at_actual = apply_gravity_sag(a, k_stiff, link_trans, link_rot) - a
    return a - sag_at_actual
