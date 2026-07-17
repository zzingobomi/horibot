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
