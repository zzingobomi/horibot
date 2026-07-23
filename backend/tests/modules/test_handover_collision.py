"""CrossRobotChecker — 실 URDF 두 대를 한 세계에 놓는 sim 검증 (PyBullet 부팅).

잠그는 것: ① 겹친 배치 = 충돌 True (판정이 실제로 뭔가를 잡는다) ② 멀리 떨어진
배치 = False (게이트가 정상 배치를 안 죽인다) ③ 경로 표본 검사가 중간 구성의
충돌을 잡는다. margin/실 base_pose 의 실물 유효성은 여기서 증명되지 않는다 —
**실물 특성화 필요** (collision.py TODO).
"""

from __future__ import annotations

import pytest

from apps.config import _ROBOT_DIR, load_robots
from modules.tasks.handover.collision import BasePose, CrossRobotChecker

pytestmark = pytest.mark.sim  # PyBullet/URDF 부팅 — fast loop 제외


def _urdf(robot_type: str):
    return _ROBOT_DIR / robot_type / "urdf" / f"{robot_type}.urdf"


@pytest.fixture(scope="module")
def types() -> tuple[str, str]:
    robots = load_robots()
    return robots["so101_6dof_0"].type, robots["omx_f_0"].type


def test_overlapping_bases_collide(types):
    so, omx = types
    checker = CrossRobotChecker(
        _urdf(so), _urdf(omx), BasePose(0.0, 0.0, 0.0, 0.0)
    )
    try:
        assert checker.in_collision([0.0] * 6, [0.0] * 4) is True
    finally:
        checker.close()


def test_distant_bases_clear(types):
    so, omx = types
    checker = CrossRobotChecker(
        _urdf(so), _urdf(omx), BasePose(1.0, 1.0, 0.0, 0.0)
    )
    try:
        assert checker.in_collision([0.0] * 6, [0.0] * 4) is False
        # 경로 표본 검사 — 멀리 있는 b 에 대해 a 의 임의 관절 경로는 전 구간 통과
        assert checker.path_in_collision(
            [[0.0] * 6, [0.5, 0.3, -0.5, 0.2, 0.1, 0.0]], [0.0] * 4
        ) is False
    finally:
        checker.close()


def test_real_base_pose_loads_and_answers(types):
    """실 크로스캘 base_pose 로 로드/판정이 도는지 (값 유효성은 실물 몫)."""
    so, omx = types
    robots = load_robots()
    bp = robots["omx_f_0"].base_pose
    import math

    checker = CrossRobotChecker(
        _urdf(so), _urdf(omx),
        BasePose(bp.x, bp.y, bp.z, math.radians(bp.yaw_deg)),
    )
    try:
        out = checker.in_collision([0.0] * 6, [0.0] * 4)
        assert out in (True, False)  # 부팅/판정 무결성만 잠금
    finally:
        checker.close()


# ─── 2026-07-23 정밀화 (근접 핸드오프) ───────────────────────────────


def test_mimic_follower_tracks_leader(types):
    """omx 미러 손가락(gripper_joint_2, multiplier=−1)이 URDF mimic 을 따른다.

    옛 "전부 상한" 배치는 미러 관절에서 손가락 교차(비물리 자세)였다 — grip
    fraction 을 주면 leader=lower+frac·range, follower=−leader 여야 한다.
    """
    import pybullet as p

    so, omx = types
    checker = CrossRobotChecker(
        _urdf(so), _urdf(omx), BasePose(1.0, 1.0, 0.0, 0.0)
    )
    try:
        checker.in_collision([0.0] * 6, [0.0] * 5, grip_b=1.0)
        b = checker._b
        assert b is not None
        assert b.mimic.get("gripper_joint_2") == ("gripper_joint_1", -1.0, 0.0)
        idx = {n: j for n, j in zip(b.names, b.movable)}
        leader = p.getJointState(
            b.body, idx["gripper_joint_1"], physicsClientId=checker._client
        )[0]
        follower = p.getJointState(
            b.body, idx["gripper_joint_2"], physicsClientId=checker._client
        )[0]
        # grip 1.0 → leader = upper(0.8469 = 벌림), follower = −leader
        assert leader == pytest.approx(0.8469, abs=1e-4)
        assert follower == pytest.approx(-leader, abs=1e-9)
        # grip 0.0 → leader = lower (닫힘 반대편) — fraction 배선 확인
        checker.in_collision([0.0] * 6, [0.0] * 5, grip_b=0.0)
        leader0 = p.getJointState(
            b.body, idx["gripper_joint_1"], physicsClientId=checker._client
        )[0]
        assert leader0 == pytest.approx(-0.3805, abs=1e-4)
    finally:
        checker.close()


def test_margin_override_per_query(types):
    """판정별 margin override — 근접 핸드오프 국면이 기본 2cm 를 좁혀 부른다.
    멀리 떨어진 배치도 margin 을 크게 주면 True (plumbing 잠금)."""
    so, omx = types
    checker = CrossRobotChecker(
        _urdf(so), _urdf(omx), BasePose(0.9, 0.0, 0.0, 0.0)
    )
    try:
        assert checker.in_collision([0.0] * 6, [0.0] * 5) is False
        assert checker.in_collision(
            [0.0] * 6, [0.0] * 5, margin_m=2.0
        ) is True
        assert checker.path_in_collision(
            [[0.0] * 6], [0.0] * 5, margin_m=2.0
        ) is True
    finally:
        checker.close()
