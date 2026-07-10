"""PybulletKinematics (Motion D1) test — robot_v2 so101_6dof URDF.

순수 compute (PyBullet DIRECT) — 하드웨어 불필요, 회사 검증 가능.
검증: dof=6 (gripper 제외) / FK·IK roundtrip / joint_limits / unreachable→None.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from modules.motion.adapters.pybullet import PybulletKinematics

_URDF = (
    Path(__file__).resolve().parents[3]  # tests/modules → backend → repo root
    / "robot"
    / "so101_6dof"
    / "urdf"
    / "so101_6dof.urdf"
)


@pytest.fixture
def kin():
    k = PybulletKinematics(_URDF)
    k.initialize()
    yield k
    k.close()


def test_dof_is_6_gripper_excluded(kin: PybulletKinematics):
    # tcp ancestor chain = joint1..6. gripper(joint7)는 sibling 가지 → 제외.
    assert kin.dof == 6
    assert kin.tcp_link_name == "tcp"
    assert len(kin.joint_limits()) == 6


def test_fk_ik_roundtrip(kin: PybulletKinematics):
    joints = [0.1, 0.3, -0.4, 0.1, 0.2, 0.0]
    pos, quat = kin.fk(joints)
    assert len(pos) == 3 and len(quat) == 4

    solved = kin.ik(pos, quat, current_joint_angles=joints)
    assert solved is not None, "reachable pose IK 실패"
    assert len(solved) == 6

    # redundancy 로 joint 값은 다를 수 있음 → POSE 일치로 검증
    pos2, _ = kin.fk(solved)
    err = float(np.linalg.norm(np.array(pos) - np.array(pos2)))
    assert err < 1e-2, f"FK/IK roundtrip pose 오차 {err}"


def test_ik_unreachable_returns_none(kin: PybulletKinematics):
    # 팔 길이 밖 (2m) → 수렴 실패 (seed + restart 모두) → None
    assert kin.ik((2.0, 2.0, 2.0), None) is None


def test_ik_reachable_from_bad_seed_solves(kin: PybulletKinematics):
    """도달 가능한 target 은 seed 가 나빠도 IK 가 해를 찾아야 함 (multi-restart).

    single-seed local 솔버는 나쁜 basin 의 seed 에서 존재하는 해를 놓친다 —
    "잡을 수 있는 위치인데 IK 실패" 회귀. J2 를 앞으로 접은 낮은 target 을 FK 로
    만들어 확실히 reachable 하게 하고, 정반대 zero-seed 로 요청해 재시도가
    실제로 해를 살리는지 검증. multi-restart 를 seeded-only 로 되돌리면 실패.
    """
    reachable = [0.5, 2.0, -2.5, 1.0, -1.0, 1.5]  # J2 앞으로 접은 낮은 자세
    target_pos, _ = kin.fk(reachable)

    sol = kin.ik(target_pos, None, current_joint_angles=[0.0] * 6)  # 나쁜 seed
    assert sol is not None, "reachable target 인데 IK 실패 (multi-restart 회귀?)"
    pos2, _ = kin.fk(sol)
    err = float(np.linalg.norm(np.array(target_pos) - np.array(pos2)))
    assert err < 1e-2, f"IK 해의 FK 오차 {err}"


def test_fk_to_matrix_shape(kin: PybulletKinematics):
    rot, pos = kin.fk_to_matrix([0.0] * 6)
    assert len(rot) == 3 and all(len(r) == 3 for r in rot)
    assert len(pos) == 3
