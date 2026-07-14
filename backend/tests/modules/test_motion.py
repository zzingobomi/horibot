"""MotionModule (D2b) test — MoveJ e2e + TCP snapshot.

의미 있는 검증 (통과용 X): MoveJ → Ruckig trajectory → Motor.Stream.COMMAND →
mock motor 가 실제로 target 에 도달 (rad→raw round-trip 포함). 순수 compute +
mock motor → 회사 검증 가능, 실 모터는 집.
"""

from __future__ import annotations

import asyncio
import math
import time

import pytest

from apps.config import DriverMode, load_robots
from apps.resolve import resolve_robot_deps
from framework.runtime.app import Runtime
from framework.transport.protocol import RemoteError
from infra.transport.zenoh import ZenohTransport
from modules.motion import units
from modules.motion.contract import (
    JogJInput,
    JogTcpInput,
    JointTarget,
    Motion,
    MoveJRequest,
    MoveJResponse,
    MoveLRequest,
    MoveLResponse,
    PoseTarget,
    ResolveReachableRequest,
    ResolveReachableResponse,
    TcpPose,
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
    """motor(mock) + motion 한 runtime — pi_hori1 동거 등가."""
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    driver = MockMotorBackend(motors=robot.motors)
    runtime.add_module(MotorDriverModule, robot_id=_SO101, driver=driver)
    motion_deps = resolve_robot_deps("motion", robot, _deploy_mock())
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
    # 성공 = 반환 (거부/실패는 raise — RemoteError 로 전파되는 계약)
    await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=target_rad)),
        MoveJResponse,
        robot_id=_SO101,
    )

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
    with pytest.raises(ValueError, match="앞쪽에 연속으로 배치"):
        resolve_robot_deps("motion", reordered, _deploy_mock())


async def test_move_j_rejects_wrong_dof(stack):
    # 거부 = raise — wire 를 건너 RemoteError("MotionRejected", 사유) 로 도달
    runtime, _driver, _robot = stack
    with pytest.raises(RemoteError, match="MotionRejected") as exc_info:
        await runtime.module_runtime.call(
            Motion.Service.MOVE_J,
            MoveJRequest(target=JointTarget(kind="joint", joints=[0.0, 0.0, 0.0])),
            MoveJResponse,
            robot_id=_SO101,
        )
    assert "dof 불일치" in exc_info.value.message  # 사유 보존 (침묵 금지)


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
    # gripper 는 arm(joints)과 분리된 별도 필드로 report — URDF open/close 시각화용.
    # arm(6)에 섞이면 waypoint(state.joints 소비)/MoveJ dof 가 깨짐 → 별도 필드가 계약.
    assert snap.gripper_joint_name == "joint7"
    assert snap.gripper_rad is not None and isinstance(snap.gripper_rad, float)


async def _wait_motion_ready(runtime) -> bool:
    for _ in range(50):
        await asyncio.sleep(0.02)
        if await _try_snapshot(runtime) is not None:
            return True
    return False


async def test_move_l_reaches_target_position(stack):
    # MoveL (position-only v1): home 자세로 MoveJ 후 직선 이동 검증 — 실제 task 흐름
    # (시작 → home → 작업)과 동일. await 완료 대기 + TCP 가 target 근처 도달 확인.
    # home = robot_poses.yaml 값 (검증됨: 6축 limit 안, IK OK). orientation 미검증(v1).
    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)

    home_rad = [math.radians(d) for d in (0.0, 15.0, -45.0, 85.0, -5.0, 90.0)]
    await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=home_rad)),
        MoveJResponse,
        robot_id=_SO101,
    )
    await asyncio.sleep(0.2)  # 20Hz 피드백이 home 반영할 시간 (snapshot 안정화)

    start = await _try_snapshot(runtime)
    assert start is not None
    # 작은 직선 이동 (reach 안, base frame). +2cm x / -2cm z (하강, pick 접근 유사).
    target = (start.position[0] + 0.02, start.position[1], start.position[2] - 0.02)

    await runtime.module_runtime.call(
        Motion.Service.MOVE_L,
        MoveLRequest(target=PoseTarget(kind="pose", position=target)),
        MoveLResponse,
        robot_id=_SO101,
    )

    # await 완료 후 TCP 가 target 근처 (20Hz 피드백 lag 흡수 위해 짧게 poll)
    end = None
    err = 1.0
    for _ in range(50):
        await asyncio.sleep(0.02)
        end = await _try_snapshot(runtime)
        if end is None:
            continue
        err = max(abs(end.position[i] - target[i]) for i in range(3))
        if err < 0.01:
            break
    assert end is not None
    assert err < 0.01, (
        f"MoveL 도달 오차 {err * 1000:.1f}mm > 10mm: {end.position} vs {target}"
    )


async def test_move_j_pose_reaches_target_via_joint_space(stack):
    # MOVE_J + PoseTarget: pose → IK → 관절 이동 (Cartesian 직선 아님). pick 접근/승강.
    # home 후 target pose(위치) 로 MoveJ(pose) → TCP 가 target 근처 도달. 자세는 IK 자유
    # (position-only) 라 위치만 검증. MoveL 과 달리 "자세 고정 경로" 제약이 없음.
    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)

    home_rad = [math.radians(d) for d in (0.0, 15.0, -45.0, 85.0, -5.0, 90.0)]
    await runtime.module_runtime.call(
        Motion.Service.MOVE_J, MoveJRequest(target=JointTarget(kind="joint", joints=home_rad)),
        MoveJResponse, robot_id=_SO101,
    )
    await asyncio.sleep(0.2)

    start = await _try_snapshot(runtime)
    assert start is not None
    target = (start.position[0] + 0.02, start.position[1], start.position[2] - 0.02)

    await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=PoseTarget(kind="pose", position=target)),
        MoveJResponse, robot_id=_SO101,
    )

    end = None
    err = 1.0
    for _ in range(50):
        await asyncio.sleep(0.02)
        end = await _try_snapshot(runtime)
        if end is None:
            continue
        err = max(abs(end.position[i] - target[i]) for i in range(3))
        if err < 0.01:
            break
    assert end is not None
    assert err < 0.01, (
        f"MoveJPose 도달 오차 {err * 1000:.1f}mm > 10mm: {end.position} vs {target}"
    )


async def test_move_l_holds_orientation_along_tool_axis(stack):
    # MoveL constant-orientation — PnP 접근축 진입의 코어 (45° 사선 하강이 큐브를
    # 밀던 실패 fix). home 자세에서 접근축(tcp x) 을 따라 3cm 슬라이드하며 자세
    # 고정 → 완료 후 orientation 이 시작과 동일한지 검증. (position-only 였다면
    # IK 가 자세를 흘려보내 dot 이 떨어짐 — target_quaternion 전파 증명.)
    import numpy as np
    from scipy.spatial.transform import Rotation

    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)

    home_rad = [math.radians(d) for d in (0.0, 15.0, -45.0, 85.0, -5.0, 90.0)]
    await runtime.module_runtime.call(
        Motion.Service.MOVE_J,
        MoveJRequest(target=JointTarget(kind="joint", joints=home_rad)),
        MoveJResponse,
        robot_id=_SO101,
    )
    await asyncio.sleep(0.2)

    start = await _try_snapshot(runtime)
    assert start is not None
    r_start = Rotation.from_quat(list(start.quaternion))
    adir = r_start.apply([1.0, 0.0, 0.0])  # 접근축 (그리퍼 pointing)
    target = (
        float(start.position[0] + 0.03 * adir[0]),
        float(start.position[1] + 0.03 * adir[1]),
        float(start.position[2] + 0.03 * adir[2]),
    )

    await runtime.module_runtime.call(
        Motion.Service.MOVE_L,
        MoveLRequest(
            target=PoseTarget(
                kind="pose", position=target, quaternion=start.quaternion
            )
        ),
        MoveLResponse,
        robot_id=_SO101,
        timeout=15.0,
    )

    ok = False
    pos_err = float("inf")
    dangle = float("inf")
    for _ in range(50):
        await asyncio.sleep(0.02)
        end = await _try_snapshot(runtime)
        if end is None:
            continue
        pos_err = max(abs(end.position[i] - target[i]) for i in range(3))
        r_end = Rotation.from_quat(list(end.quaternion))
        # 자세 유지 — 시작/끝 orientation 차이 각도
        dangle = float(np.linalg.norm((r_end * r_start.inv()).as_rotvec()))
        if pos_err < 0.01 and dangle < math.radians(5.0):
            ok = True
            break
    assert ok, (
        f"자세고정 MoveL 실패 — pos_err={pos_err * 1000:.1f}mm "
        f"dangle={math.degrees(dangle):.1f}°"
    )


async def test_move_l_rejects_without_motor_state(robot):
    # motor 없이 motion 만 → motor state 없음 → pre-flight 거부 = raise
    # (RemoteError("MotionRejected", 사유) — 모션 0, 로봇 안 움직임)
    transport = ZenohTransport(_LOCAL_CFG)
    time.sleep(0.05)
    runtime = Runtime(transport)
    motion_deps = resolve_robot_deps("motion", robot, _deploy_mock())
    runtime.add_module(MotionModule, robot_id=_SO101, **motion_deps)
    await runtime.start()
    try:
        with pytest.raises(RemoteError, match="MotionRejected") as exc_info:
            await runtime.module_runtime.call(
                Motion.Service.MOVE_L,
                MoveLRequest(target=PoseTarget(kind="pose", position=(0.2, 0.0, 0.2))),
                MoveLResponse,
                robot_id=_SO101,
            )
        assert "motor state" in exc_info.value.message
    finally:
        await runtime.stop()
        transport.close()


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
        "angular jog 인데 quaternion 넘긴 IK 호출이 0건 — "
        "6DOF exact solve 계약 위반"
    )


async def test_resolve_reachable_orders_and_rejects(stack):
    """배치 IK 판정 계약 — (a) 불가 그룹(멀리) 건너뛰고 첫 가용 그룹 index,
    (b) 전부 불가면 -1, (c) 모션 0 (motor 명령 안 나감). 뒤집으면: early-exit
    순서를 무시하거나 불가를 가용으로 판정하면 즉시 깨짐."""
    runtime, driver, _robot = stack
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None
    # 현재 TCP(가용 확실) vs workspace 밖(불가 확실 — reach ~0.4m 대비 1.5m)
    here = TcpPose(position=snap.position)
    far = TcpPose(position=(1.5, 0.0, 0.5))
    res = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=[[far], [far, here], [here, here], [here]]),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    # group0: far 불가 / group1: far 불가 (그룹 내 전 pose 필요) / group2: 첫 가용
    assert res.index == 2, res
    # 채택 그룹의 IK 해 반환 — 실행부 재계산 제거 계약 (grasp_redesign §5.5).
    # 해의 FK 가 요청 pose 위치로 돌아와야 진짜 해다.
    assert len(res.solutions) == 2
    motion_mod = next(
        m for m in runtime._modules if type(m).__name__ == "MotionModule"
    )
    import numpy as np

    for sol in res.solutions:
        pos, _ = motion_mod._kin.fk(sol)
        assert np.linalg.norm(np.array(pos) - np.array(snap.position)) < 0.02
    res_none = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=[[far], [far]]),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert res_none.index == -1
    assert res_none.solutions == []
    assert res_none.message  # 사유 침묵 금지 (UI 표시)


async def test_resolve_reachable_floor_and_linear_gates(stack):
    """게이트 계약 — (a) 바닥 평면이 해 자세 링크를 침투하면 기각 (+사유 표기),
    (b) linear=True 는 연속 pose 직선 경로까지 검증 (여기선 제자리 = 통과),
    (c) floor 가 로봇 아래 멀리면 통과 (게이트가 정상 후보를 안 죽임)."""
    runtime, _driver, _robot = stack
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None
    here = TcpPose(position=snap.position)

    # (c) 바닥이 한참 아래 + linear — 정상 후보 통과
    ok = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=[[here, here]], floor_z=snap.position[2] - 0.5, linear=True
        ),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert ok.index == 0 and len(ok.solutions) == 2

    # (a) 바닥 평면을 로봇 전체 위(z+1m)로 — 모든 해가 침투 → 전멸 + 사유
    blocked = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=[[here]], floor_z=snap.position[2] + 1.0),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert blocked.index == -1
    assert "바닥" in blocked.message


async def test_resolve_reachable_obstacle_and_path_gates(stack):
    """③b/④ 게이트 계약 (grasp_redesign §10.4-3/4) — (a) 장애물 점군이 해 자세의
    로봇을 감싸면 전멸 + 사유 표기, (b) 먼 장애물 + path_from(현재 관절 ≈ 해)은
    통과 (게이트가 정상 후보를 안 죽임), (c) path_from dof 불일치 = 기술적 실패
    (데이터 -1 이 아니라 raise — 잘못된 요청의 침묵 전멸 금지)."""
    runtime, _driver, _robot = stack
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None
    here = TcpPose(position=snap.position)

    # (a) 현재 TCP 를 감싸는 점군 뭉치 — 해 자세에서 침투 확실 → 전멸 + 사유
    import numpy as np

    grid = np.linspace(-0.02, 0.02, 6)
    ball = [
        (snap.position[0] + a, snap.position[1] + b, snap.position[2] + c)
        for a in grid for b in grid for c in grid
    ]
    blocked = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(groups=[[here]], obstacle_points=ball),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert blocked.index == -1
    assert "장애물" in blocked.message

    # (b) 먼 장애물 + path_from=현재 관절 (경로 ≈ 제자리) → 통과 + 잔존 장애물
    # 없음 (lifecycle — 앞 판정의 점군이 남아 이 판정을 오염하면 여기서 걸림)
    ok = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=[[here]],
            obstacle_points=[(0.5, 0.5, 0.5)],
            path_from=list(snap.joints),
            gripper_open=True,
        ),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert ok.index == 0 and len(ok.solutions) == 1

    # (c) path_from dof 불일치 → MotionRejected (RemoteError 로 전파)
    with pytest.raises(RemoteError, match="dof"):
        await runtime.module_runtime.call(
            Motion.Service.RESOLVE_REACHABLE,
            ResolveReachableRequest(groups=[[here]], path_from=[0.0]),
            ResolveReachableResponse,
            robot_id=_SO101,
            timeout=60.0,
        )


async def test_obstacle_points_kinematics_gate(stack):
    """Kinematics obstacle scene 계약 — 점군을 감싼 자세는 침투 판정(그리퍼 벌림
    포함), 먼 점군/해제 후엔 False, gripper_open 검사가 이후 질의 상태를 오염하지
    않는다 (그리퍼 원위치 복원)."""
    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)
    motion_mod = next(
        m for m in runtime._modules if type(m).__name__ == "MotionModule"
    )
    kin = motion_mod._kin
    assert kin is not None

    joints = [0.1, 0.3, -0.4, 0.1, 0.2, 0.0]
    pos, _ = kin.fk(joints)
    import numpy as np

    grid = np.linspace(-0.02, 0.02, 6)
    ball = [
        (pos[0] + a, pos[1] + b, pos[2] + c)
        for a in grid for b in grid for c in grid
    ]
    kin.set_obstacle_points(ball)
    assert kin.obstacle_collision(joints) is True
    assert kin.obstacle_collision(joints, gripper_open=True) is True
    before = kin.self_collision(joints)
    # gripper_open 검사 후에도 self_collision 판정 재현 (그리퍼 복원 확인)
    assert kin.self_collision(joints) == before

    kin.set_obstacle_points([(0.6, 0.6, 0.6)])  # 먼 점군 — 통과
    assert kin.obstacle_collision(joints) is False
    kin.set_obstacle_points(None)  # 해제 — 점군 없음 = False
    assert kin.obstacle_collision(joints) is False


async def test_resolve_reachable_real_grasp_plan_ik_passes(stack):
    """★ 프로덕션 서비스 경로 e2e — 실 URDF IK 가 실 grasp 후보에서 진짜 풀리나.

    verify_production_pipeline.py 는 게이트를 스크립트가 재현했다 (kin.ik 직접
    호출). 여기서는 **실제 MotionModule 의 resolve_reachable 서비스**에 **실
    antipodal → plan_grasp 후보 그룹**을 던진다 — steps.try_plan_grasp 가 던지는
    것과 동일한 인자(floor_z / linear / obstacle_points=점군 / gripper_open /
    path_from). "로직만 짜고 IK 는 canned 로 넘어간 것 아니냐"를 실행으로 닫는다.

    합성 큐브(양쪽 옆면 + 윗면)를 워크스페이스 안(0.24,0.10)에 놓고 실 antipodal
    쌍 → 후보 가족 → 서비스. index≥0 이면 실 IK 가 그 후보를 실제로 풀었다는
    뜻이고, solutions FK 가 grasp pose 위치로 복귀해야 진짜 해다.
    """
    import numpy as np

    from modules.tasks.pick_and_place import antipodal, geometry

    runtime, _driver, _robot = stack
    assert await _wait_motion_ready(runtime)
    snap = None
    for _ in range(50):
        await asyncio.sleep(0.02)
        snap = await _try_snapshot(runtime)
        if snap is not None:
            break
    assert snap is not None

    # 합성 큐브 점군 — 옆면 ±x(간격 2.2cm) + 윗면. object-centric 관측 흉내.
    cx, cy, base_z, top_z = 0.24, 0.10, -0.045, -0.022
    ys = np.linspace(cy - 0.011, cy + 0.011, 12)
    zs = np.linspace(base_z, top_z, 12)
    xs = np.linspace(cx - 0.011, cx + 0.011, 12)
    pts: list[tuple[float, float, float]] = []
    for x in (cx + 0.011, cx - 0.011):
        for y in ys:
            for z in zs:
                pts.append((float(x), float(y), float(z)))
    for x in xs:
        for y in ys:
            pts.append((float(x), float(y), top_z))
    cloud = np.array(pts, dtype=float)

    # 실 antipodal → 실 plan_grasp (canned 아님 — 실제 선택기/기하)
    pairs = antipodal.horizontal_antipodal_pairs(cloud)
    assert pairs, "합성 큐브에서 antipodal 쌍 0 — 선택기 자체가 안 돎"
    plan = geometry.plan_grasp(pairs)
    assert plan

    # 실 resolve_reachable 서비스 — steps.try_plan_grasp 와 동일 인자
    res = await runtime.module_runtime.call(
        Motion.Service.RESOLVE_REACHABLE,
        ResolveReachableRequest(
            groups=geometry.grasp_ik_groups(plan),
            floor_z=base_z - 0.005,
            linear=True,
            obstacle_points=[tuple(p) for p in cloud],
            gripper_open=True,
            path_from=list(snap.joints),
        ),
        ResolveReachableResponse,
        robot_id=_SO101,
        timeout=60.0,
    )
    assert res.index >= 0, (
        f"실 URDF IK 가 grasp 후보 {len(plan)}개 전멸 — {res.message}"
    )
    # solutions FK 가 후보 pre/grasp pose 로 복귀해야 진짜 해 (재계산 없이 실행부가 씀)
    motion_mod = next(
        m for m in runtime._modules if type(m).__name__ == "MotionModule"
    )
    chosen = plan[res.index]
    assert len(res.solutions) == 2
    for sol, want in zip(res.solutions, (chosen.pre, chosen.grasp)):
        pos, _ = motion_mod._kin.fk(sol)
        assert np.linalg.norm(np.array(pos) - np.array(want)) < 0.02, (
            f"IK 해 FK 가 목표 pose 와 어긋남: {pos} != {want}"
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
