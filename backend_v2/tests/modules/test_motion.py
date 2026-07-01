"""MotionModule (D2b) test — MoveJ e2e + TCP snapshot.

의미 있는 검증 (통과용 X): MoveJ → Ruckig trajectory → Motor.Stream.COMMAND →
mock motor 가 실제로 target 에 도달 (rad→raw round-trip 포함). 순수 compute +
mock motor → 회사 검증 가능, 실 모터는 집.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from apps.config import DriverMode, load_robots
from apps.resolve import resolve_deps
from framework.runtime.app import Runtime
from infra.transport.zenoh import ZenohTransport
from modules.motion import units
from modules.motion.contract import (
    JogJInput,
    JogTcpInput,
    Motion,
    MoveJRequest,
    MoveJResponse,
    TcpSnapshotRequest,
    TcpState,
)
from modules.motion.module import MotionModule
from modules.motor.contract import MotorKind
from modules.motor.drivers.mock import MockMotorBackend
from modules.motor.module import MotorDriverModule

_LOCAL_CFG = {"mode": "peer", "scouting": {"multicast": {"enabled": False}}}
_SO101 = "so101_6dof_0"


@pytest.fixture
def robot():
    return load_robots()[_SO101]


@pytest.fixture
async def stack(robot):
    """motor(mock) + motion 한 runtime — pi_motor 동거 등가."""
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    driver = MockMotorBackend(motors=robot.motors)
    runtime.add_module(MotorDriverModule, robot_id=_SO101, driver=driver)
    motion_deps = resolve_deps("motion", robot, _deploy_mock())
    runtime.add_module(MotionModule, robot_id=_SO101, **motion_deps)
    await runtime.start()
    yield runtime, driver, robot
    await runtime.stop()
    transport.close()


def _deploy_mock():
    from apps.config import DeploymentConfig

    return DeploymentConfig(driver_mode=DriverMode.MOCK)


def _arm_specs(robot):
    return [s for s in robot.motors if s.kind != MotorKind.GRIPPER]


async def test_move_j_drives_mock_motor_to_target(stack):
    runtime, driver, robot = stack
    arm = _arm_specs(robot)

    # motor state(20Hz) 가 motion 까지 도달해 _latest_arm_rad 세팅될 때까지
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None, "motion 이 motor state 못 받음"

    target_rad = [0.1, 0.3, -0.4, 0.1, 0.2, 0.0]
    res = await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target_joints=target_rad),
        MoveJResponse,
        robot_id=_SO101,
    )
    assert res.accepted, res.message

    # trajectory → command stream → mock motor. target raw 도달까지 poll
    expected = units.joints_rad_to_raw(target_rad, arm)
    reached = False
    for _ in range(150):  # ~3s
        await asyncio.sleep(0.02)
        if all(abs(a - b) <= 2 for a, b in zip(driver.read_positions()[:6], expected)):
            reached = True
            break
    assert reached, (
        f"MoveJ 가 target 에 도달 못함: {driver.read_positions()[:6]} != {expected}"
    )
    # gripper 는 MoveJ 대상 아님 → home 유지
    assert driver.read_positions()[6] == 2048


def test_motion_resolve_rejects_non_prefix_arm(robot):
    # gripper 가 arm joint 앞/사이에 있으면 positional raw 매핑이 깨짐 → boot fail-fast
    reordered = robot.model_copy(
        update={"motors": [robot.motors[-1], *robot.motors[:-1]]}  # gripper 맨 앞
    )
    with pytest.raises(ValueError, match="prefix"):
        resolve_deps("motion", reordered, _deploy_mock())


async def test_move_j_rejects_wrong_dof(stack):
    runtime, _driver, _robot = stack
    res = await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target_joints=[0.0, 0.0, 0.0]),  # 3 != 6
        MoveJResponse,
        robot_id=_SO101,
    )
    assert not res.accepted


async def test_tcp_snapshot_returns_fk_pose(stack):
    runtime, _driver, _robot = stack
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None
    assert len(snap.position) == 3
    assert len(snap.quaternion) == 4
    assert len(snap.joints) == 6  # arm only
    # joint_names 계약 = motors.yaml arm prefix 순서 SSOT (URDF 파일 순서와 무관).
    # frontend 는 이 name list 로 URDF joint 를 찾아 매핑 — 순서 회귀 원천 차단.
    assert snap.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert len(snap.joint_names) == len(snap.joints)


async def _wait_motion_ready(runtime) -> bool:
    for _ in range(50):
        await asyncio.sleep(0.02)
        if await _try_snapshot(runtime) is not None:
            return True
    return False


async def test_jog_j_moves_motor(stack):
    # backend e2e jog: JogJ 50Hz 입력 → 적분 → command → mock motor 이동
    runtime, driver, _robot = stack
    assert await _wait_motion_ready(runtime)
    for _ in range(30):  # ~0.6s @ 50Hz
        runtime.module_runtime.publish(
            Motion.Stream.JOG_J,
            JogJInput(robot_id=_SO101, velocities=[0.5, 0, 0, 0, 0, 0]),
        )
        await asyncio.sleep(0.02)
    # joint1 이 home(2048)에서 + 방향 이동 (적분 결과)
    assert driver.read_positions()[0] > 2060


async def test_jog_j_clamps_to_joint_limit(stack):
    # 안전: 한계 넘게 오래 jog (속도 3.0 > max_vel 1.5 → cap) → motor limit clamp
    runtime, driver, robot = stack
    arm = _arm_specs(robot)
    assert await _wait_motion_ready(runtime)
    for _ in range(120):  # ~2.4s — 한계 도달에 충분
        runtime.module_runtime.publish(
            Motion.Stream.JOG_J,
            JogJInput(robot_id=_SO101, velocities=[3.0, 0, 0, 0, 0, 0]),
        )
        await asyncio.sleep(0.02)
    j1 = driver.read_positions()[0]
    assert j1 <= arm[0].limit_max, f"joint1 이 limit 넘음: {j1} > {arm[0].limit_max}"
    assert j1 >= arm[0].limit_max - 5, "clamp 가 멈춘 게 아니라 아예 안 움직임"


async def test_jog_tcp_moves_motor(stack):
    # JogTcp: cartesian twist → SE(3) 적분 → IK → mock motor 이동
    runtime, driver, _robot = stack
    assert await _wait_motion_ready(runtime)
    before = driver.read_positions()[:6]
    for _ in range(30):
        runtime.module_runtime.publish(
            Motion.Stream.JOG_TCP,
            JogTcpInput(
                robot_id=_SO101, linear=(0.02, 0.0, 0.0), angular=(0.0, 0.0, 0.0)
            ),
        )
        await asyncio.sleep(0.02)
    assert driver.read_positions()[:6] != before, "JogTcp IK → 모터 이동 없음"


async def test_jog_tcp_pure_translation_uses_position_only_ik(stack):
    """회귀 잡음 — 2026-07-01 SO-101 실 hardware Z+ jog IK reject 사건.
    orientation constraint 를 매 프레임 pin 하면 arm 최대 reach 근처 자리 orientation
    exact solve 가 실패 (reason=orientation-only-fail). teach-pendant 표준 자리
    pure translation (angular=0) = position-only IK (옛 backend cartesian path 원칙
    `servo_tcp(pos, None, angles)` 과 동일).

    이 assert 뒤집으면 회귀 즉시 잡힘 — angular=0 자리 kin.ik 가 quaternion=None 로
    호출되는지 spy 로 계약 검증.
    """
    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)

    # motion module 의 kinematics 를 spy 로 감싸 ik 호출 인자 캡처
    from unittest.mock import MagicMock

    # runtime 안 module 찾아 spy 주입
    motion_mod = next(
        m for m in runtime._modules if type(m).__name__ == "MotionModule"
    )
    orig_ik = motion_mod._kin.ik
    ik_calls: list[tuple] = []

    def spy_ik(pos, quat, cur):
        ik_calls.append((pos, quat, tuple(cur)))
        return orig_ik(pos, quat, cur)

    motion_mod._kin.ik = MagicMock(side_effect=spy_ik)  # type: ignore[method-assign]

    # pure Z+ translation — 계약: quaternion 인자 = None 이어야 함
    for _ in range(15):
        runtime.module_runtime.publish(
            Motion.Stream.JOG_TCP,
            JogTcpInput(
                robot_id=_SO101, linear=(0.0, 0.0, 0.02), angular=(0.0, 0.0, 0.0)
            ),
        )
        await asyncio.sleep(0.02)

    # idle re-latch (첫 호출) 은 quat 포함 (fresh FK latch) — 이후 non-idle 호출 자리
    # angular=0 이면 quat=None 이어야. non-idle 호출 최소 1건 확인.
    non_idle = [c for c in ik_calls if c[1] is None]
    assert len(non_idle) > 0, (
        f"pure translation jog 인데 position-only IK (quat=None) 호출이 0건 — "
        f"모든 호출: {[(c[1] is None) for c in ik_calls]}"
    )


async def test_jog_tcp_with_angular_uses_6dof_ik(stack):
    """대응 계약 — angular 성분 있으면 quaternion 을 IK 에 넘겨 6DOF exact solve.
    사용자가 명시적으로 orientation jog 하는 자리는 orientation 유지가 의도."""
    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)

    from unittest.mock import MagicMock

    motion_mod = next(
        m for m in runtime._modules if type(m).__name__ == "MotionModule"
    )
    orig_ik = motion_mod._kin.ik
    ik_calls: list[tuple] = []

    def spy_ik(pos, quat, cur):
        ik_calls.append((pos, quat, tuple(cur)))
        return orig_ik(pos, quat, cur)

    motion_mod._kin.ik = MagicMock(side_effect=spy_ik)  # type: ignore[method-assign]

    # angular Rz jog — quaternion 인자 포함 되어야
    for _ in range(15):
        runtime.module_runtime.publish(
            Motion.Stream.JOG_TCP,
            JogTcpInput(
                robot_id=_SO101, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.1)
            ),
        )
        await asyncio.sleep(0.02)

    with_quat = [c for c in ik_calls if c[1] is not None]
    assert len(with_quat) > 0, (
        f"angular jog 인데 quaternion 넘긴 IK 호출이 0건 — "
        f"6DOF exact solve 계약 위반"
    )


async def _try_snapshot(runtime) -> TcpState | None:
    try:
        return await runtime.module_runtime.call(
            Motion.Service.TCP_SNAPSHOT,
            TcpSnapshotRequest(),
            TcpState,
            robot_id=_SO101,
        )
    except Exception:
        return None
