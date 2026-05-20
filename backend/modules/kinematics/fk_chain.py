"""OMX_F의 URDF chain을 numpy로 직접 구현한 FK.

PyBullet은 URDF 파일을 *정적으로* 로드해 link transform이 부팅 후 고정.
Hand-Eye BA는 link origin offset을 *변수*로 풀어야 하므로 매 LM iteration마다
다른 link_offset으로 FK를 호출해야 함 — PyBullet 우회 필요.

이 모듈의 사용처:
    (a) [diag_handeye_extended.py](backend/diag_handeye_extended.py) 같은
        진단/검증 스크립트
    (b) `bundle_adjust.py`의 확장 BA — link offset 자유도와 함께 FK 평가
    (c) PybulletSolver / urdf_patcher가 같은 URDF 상수를 공유 (single source)

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
