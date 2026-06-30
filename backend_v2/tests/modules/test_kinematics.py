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
    Path(__file__).resolve().parents[3]  # tests/modules → backend_v2 → repo root
    / "robot_v2"
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
    # 팔 길이 밖 (2m) → 수렴 실패 → None
    assert kin.ik((2.0, 2.0, 2.0), None) is None


def test_fk_to_matrix_shape(kin: PybulletKinematics):
    rot, pos = kin.fk_to_matrix([0.0] * 6)
    assert len(rot) == 3 and all(len(r) == 3 for r in rot)
    assert len(pos) == 3
