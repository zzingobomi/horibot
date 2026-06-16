"""FkChain 의 ground truth 검증 — PyBullet `loadURDF` 의 정적 FK 와 결과 일치.

본 검증이 PR 의 핵심 — 자체 numpy chain build 의 *수치 정확성* 보장.
- omx_f (5DOF, rpy=0 가정 chain) + so101_6dof (6DOF, rpy 비0 광범위)
- joint angle limits 안에서 랜덤 N=20 자세 → 두 FK 결과 (R, t) 비교
- tolerance: pos 1e-4 m, rot 1e-4 rad

so101 의 rpy 비0 자리에서도 일치 = yourdfpy parse (rpy 처리 + 4x4 origin) + 우리
chain build 가 정확.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from modules.kinematics.fk_chain import FkChain
from modules.motor.motor_config import load_motor_layout
from core.robot.robot_registry import RobotRegistry


REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def rng():
    return np.random.default_rng(42)


def _pybullet_fk(
    urdf_path: Path,
    tcp_link: str,
    joint_angles: list[float],
    arm_joint_names: list[str],
):
    """PyBullet 의 정적 FK — link_offset 0 가정에서 ground truth.

    arm_joint_names 으로 PyBullet 의 actuated joint 중 arm 만 골라 angles 매핑.
    gripper / 추가 fixed-frame joint 는 0 으로 둠 (default reset state).
    """
    import pybullet as p

    client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=client)
        # PyBullet 의 모든 joint info → name → index 매핑
        n_joints_pyb = p.getNumJoints(robot, physicsClientId=client)
        name_to_idx: dict[str, int] = {}
        tcp_link_idx: int | None = None
        for i in range(n_joints_pyb):
            info = p.getJointInfo(robot, i, physicsClientId=client)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            name_to_idx[joint_name] = i
            if link_name == tcp_link:
                tcp_link_idx = i
        if tcp_link_idx is None:
            raise RuntimeError(f"PyBullet: link '{tcp_link}' 못 찾음")

        for jname, q in zip(arm_joint_names, joint_angles):
            if jname not in name_to_idx:
                raise RuntimeError(f"PyBullet: joint '{jname}' 못 찾음")
            p.resetJointState(robot, name_to_idx[jname], q, physicsClientId=client)
        ls = p.getLinkState(
            robot, tcp_link_idx, computeForwardKinematics=True, physicsClientId=client
        )
        pos = np.array(ls[4], dtype=np.float64)  # worldLinkFramePosition
        orn = ls[5]  # worldLinkFrameOrientation (xyzw quaternion)
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return R, pos
    finally:
        p.disconnect(physicsClientId=client)


@pytest.mark.parametrize("robot_id", ["omx_f_0", "so101_6dof_0"])
def test_fk_chain_matches_pybullet(robot_id: str, rng):
    """FkChain.fk() vs PybulletKinematics 정적 FK — 동일 결과 (link_offset=0)."""
    cfg = RobotRegistry().get(robot_id)
    layout = load_motor_layout(robot_id)
    arm_cfgs = layout.arm
    arm_joint_names = [m.name for m in arm_cfgs]

    fc = FkChain(cfg.urdf_path, arm_joint_names)
    assert fc.n_arm == len(arm_cfgs)

    # joint limits 안에서 랜덤 자세 N=20
    limits_rad = [
        ((m.limit_min - 2048) / 4095.0 * 2 * np.pi,
         (m.limit_max - 2048) / 4095.0 * 2 * np.pi)
        for m in arm_cfgs
    ]

    max_pos_err = 0.0
    max_rot_err = 0.0
    for _ in range(20):
        angles = [rng.uniform(lo, hi) for (lo, hi) in limits_rad]
        R_fc, t_fc = fc.fk(np.array(angles))
        R_pyb, t_pyb = _pybullet_fk(cfg.urdf_path, "tcp", angles, arm_joint_names)

        pos_err = float(np.linalg.norm(t_fc - t_pyb))
        # rotation error = ||rotvec(R_fc @ R_pyb.T)||
        import cv2
        rvec, _ = cv2.Rodrigues(R_fc @ R_pyb.T)
        rot_err = float(np.linalg.norm(rvec))

        max_pos_err = max(max_pos_err, pos_err)
        max_rot_err = max(max_rot_err, rot_err)

    # tolerance: 1e-4 m / 1e-4 rad (~0.006°)
    # PyBullet 내부 quat→matrix 변환 floating point noise 감안.
    assert max_pos_err < 1e-4, (
        f"{robot_id}: pos error {max_pos_err*1000:.4f}mm > 0.1mm tolerance"
    )
    assert max_rot_err < 1e-4, (
        f"{robot_id}: rot error {np.degrees(max_rot_err):.5f}° > 0.006° tolerance"
    )


def test_fk_chain_link_offset_zero_no_change():
    """link_trans=0 / link_rot=0 인자 명시 vs None → 결과 일치."""
    cfg = RobotRegistry().get("omx_f_0")
    layout = load_motor_layout("omx_f_0")
    arm_joint_names = [m.name for m in layout.arm]
    fc = FkChain(cfg.urdf_path, arm_joint_names)

    angles = np.array([0.1, -0.3, 0.5, 0.0, -0.2])
    R1, t1 = fc.fk(angles)
    R2, t2 = fc.fk(
        angles,
        link_trans=np.zeros((5, 3)),
        link_rot=np.zeros((5, 3)),
    )
    np.testing.assert_allclose(R1, R2, atol=1e-12)
    np.testing.assert_allclose(t1, t2, atol=1e-12)


def test_fk_chain_link_offset_translates_ee():
    """link_trans[i] 가산 → ee position 변화 확인 (적절한 방향)."""
    cfg = RobotRegistry().get("omx_f_0")
    layout = load_motor_layout("omx_f_0")
    arm_joint_names = [m.name for m in layout.arm]
    fc = FkChain(cfg.urdf_path, arm_joint_names)

    angles = np.zeros(5)
    _, t0 = fc.fk(angles)

    # 마지막 arm joint 의 origin xyz 에 +0.01m x 가산 → ee x 위치 +0.01m (chain 끝)
    link_t = np.zeros((5, 3))
    link_t[4, 0] = 0.01
    _, t1 = fc.fk(angles, link_trans=link_t)
    # base joint angle=0 이라 chain 의 +x 가 그대로 base +x.
    assert abs((t1[0] - t0[0]) - 0.01) < 1e-9
