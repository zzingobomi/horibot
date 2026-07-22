"""PLAN_PATH sim test — 실 so101 URDF + PyBullet 충돌 세계 위에서 RRT 검증.

의미 있는 검증 (통과용 X): planner 순수 테스트(합성 2-DOF)는 알고리즘만 잠근다
— 여기는 "실 로봇 기구학에서 직선이 장애물에 막힌 두 자세 사이의 우회를 찾고,
그 경로가 같은 충돌 모델로 재검사해도 무충돌"을 잠근다 (transit 이 이 경로를
그대로 MoveJ 하므로 이게 실행 안전의 실체). e2e 는 서비스 결선(현재 관절 시작 +
obstacle lifecycle)을 잠근다.

Runtime/PyBullet/URDF 부팅 — 마커 정의 그대로 sim.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from apps.config import DeploymentConfig, DriverMode, load_robots
from apps.resolve import resolve_robot_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.motion.adapters.pybullet import PybulletKinematics
from modules.motion.contract import (
    Motion,
    PlanPathRequest,
    PlanPathResponse,
)
from modules.motion.module import MotionModule
from modules.motion.planner import STEP_RAD, _edge_free, plan_joint_path
from modules.motor.contract import MotorKind
from modules.motor.drivers.mock import MockMotorBackend
from modules.motor.module import MotorDriverModule

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_SO101 = "so101_6dof_0"

pytestmark = pytest.mark.sim


@pytest.fixture(scope="module")
def kin():
    robot = load_robots()[_SO101]
    deps = resolve_robot_deps("motion", robot, DeploymentConfig(driver_mode=DriverMode.MOCK))
    k = PybulletKinematics(deps["urdf_path"])
    k.initialize()
    yield k
    k.close()


# 장애물 벽 — 로봇 정면 (x 0.14~0.30, y=0) 에 세운 점군 커튼. 좌→우 스윕의
# 직선 관절 보간이 이 높이(z≤0.12)를 통과하므로 위로 넘는 우회가 유일한 해.
def _wall_points() -> list[tuple[float, float, float]]:
    pts = []
    x = 0.14
    while x <= 0.30:
        z = 0.0
        while z <= 0.12:
            pts.append((round(x, 3), 0.0, round(z, 3)))
            z += 0.01
        x += 0.01
    return pts


def _side_configs(kin) -> tuple[list[float], list[float]]:
    """벽 좌/우의 reached-out 자세 (position-only IK, 결정적 seed)."""
    qa = kin.ik((0.20, -0.15, 0.05), None, [0.0] * 6)
    qb = kin.ik((0.20, 0.15, 0.05), None, [0.0] * 6)
    assert qa is not None and qb is not None, "측면 자세 IK 실패 — 테스트 전제 붕괴"
    return qa, qb


def test_plan_detours_around_wall_on_real_urdf(kin):
    qa, qb = _side_configs(kin)
    kin.set_obstacle_points(_wall_points())
    try:
        def coll(q: list[float]) -> bool:
            return (
                kin.self_collision(q)
                or kin.floor_collision(q, -0.01)
                or kin.obstacle_collision(q)
            )

        # 전제 검증 — 직선이 실제로 벽에 막혀야 이 테스트가 유의미 (vacuous 방지)
        assert not _edge_free(qa, qb, coll, STEP_RAD), (
            "직선이 벽을 안 지나감 — 벽/자세 배치를 조정해야 테스트가 유효"
        )
        t0 = time.perf_counter()
        result = plan_joint_path(qa, qb, kin.joint_limits(), coll)
        dt = time.perf_counter() - t0
        assert result.path is not None, f"우회 못 찾음: {result.reason}"
        assert not result.direct
        # 반환 경로 독립 재검사 — 더 촘촘한 해상도로 전 엣지 무충돌
        for a, b in zip(result.path, result.path[1:]):
            assert _edge_free(a, b, coll, STEP_RAD / 2), "계획 경로에 충돌 엣지"
        assert result.path[0] == [float(v) for v in qa]
        assert result.path[-1] == [float(v) for v in qb]
        # 관측성 기준선 (PC) — Pi 계측의 비교점으로 출력만, 단정은 느슨하게
        print(f"plan_detour: {dt * 1000:.0f}ms, checks={result.checks}, "
              f"waypoints={len(result.path)}")
    finally:
        kin.set_obstacle_points(None)


def test_plan_direct_when_free_on_real_urdf(kin):
    """장애물 없는 자유 이동 = 직선 fast-path (RRT 폭주 없음 — transit 의
    지배적 케이스가 싸다는 계약)."""
    qa, qb = _side_configs(kin)

    def coll(q: list[float]) -> bool:
        return kin.self_collision(q) or kin.floor_collision(q, -0.01)

    result = plan_joint_path(qa, qb, kin.joint_limits(), coll)
    assert result.path is not None
    assert result.direct and result.path == [
        [float(v) for v in qa], [float(v) for v in qb]
    ]


# ─── e2e — 서비스 결선 (현재 관절 시작 + 응답 shape) ─────────────────


@pytest.fixture
async def stack():
    robot = load_robots()[_SO101]
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    driver = MockMotorBackend(motors=robot.motors)
    runtime.add_module(MotorDriverModule, robot_id=_SO101, driver=driver)
    deps = resolve_robot_deps(
        "motion", robot, DeploymentConfig(driver_mode=DriverMode.MOCK)
    )
    runtime.add_module(MotionModule, robot_id=_SO101, **deps)
    await runtime.start()
    yield runtime, robot
    await runtime.stop()
    transport.close()


async def test_plan_path_service_from_current_joints(stack):
    runtime, robot = stack
    arm_dof = len([s for s in robot.motors if s.kind != MotorKind.GRIPPER])
    # 목표 = 베이스(J1)만 0.4 rad 회전 — zero-pose 대비 기하 불변이라 확실히
    # 무충돌 (all-0.1 벡터는 실 URDF 에서 self-collision — 첫 구동에서 확인).
    goal = [0.0] * arm_dof
    goal[0] = 0.4
    # motor state 도달 대기 (start_joints=None → 현재 관절)
    res: PlanPathResponse | None = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        try:
            got = await runtime.module_runtime.call(
                Motion.Service.PLAN_PATH,
                PlanPathRequest(goal_joints=goal),
                PlanPathResponse,
                robot_id=_SO101,
            )
        except Exception:
            continue
        if not isinstance(got, PlanPathResponse):
            continue  # 픽스처가 Any — isinstance 로 타입 확정 (pyright 협조)
        res = got
        if res.found or "motor state" not in res.message:
            break
    assert res is not None, "응답 없음"
    assert res.found, res.message
    assert res.direct  # zero 근방 → J1 회전 — 자유 직선
    assert res.waypoints == []  # 직선 = 중간 경유점 없음
    assert res.checks > 0 and res.planning_ms >= 0.0
