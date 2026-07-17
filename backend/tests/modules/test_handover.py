"""handover task 검증 — mock(FakeContext) 레벨 (2026-07-17 신설, 실물 미검증).

잠그는 것:
  ① frame 변환 왕복 (base_pose 크로스캘 규약)
  ② 시나리오 happy path 의 호출 경로 — 특히 **수취 순서 불변식**: so101 이
     close + held 판정한 뒤에만 omx 가 연다 (뒤집히면 물체 낙하)
  ③ 수취 계획의 cross-robot 충돌 게이트 — 충돌 그룹 제외 재시도 / 전멸 명시 실패
  ④ 티칭 자산 fail-fast (handover waypoint 없음 = 모션 0 시점 실패)
  ⑤ module 배선 (preview 정적 트리 / list_robots)
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import pytest
from pydantic import BaseModel

from modules.detector.contract import (
    DetectOrientedResponse,
    Detector,
    OrientedDetection,
)
from modules.motion.contract import (
    Motion,
    MoveJResponse,
    MoveLResponse,
    ResolveReachableResponse,
    TcpState,
)
from modules.motor.contract import JointState, Motor, SetGripperResponse
from modules.tasks.core.contract import PreviewRequest
from modules.tasks.core.errors import NoReachableGrasp, TaskError
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.handover import steps
from modules.tasks.handover.collision import BasePose
from modules.tasks.handover.contract import ListRobotsRequest
from modules.tasks.handover.module import HandoverModule
from modules.waypoint.contract import (
    ListGroupMembersResponse,
    ListGroupsResponse,
    ListWaypointsResponse,
    Waypoint,
    WaypointGroupRecord,
    WaypointRecord,
)

SO = "so101_6dof_0"
OMX = "omx_f_0"
_TS = datetime.fromtimestamp(0, UTC)

_DETECT = str(Detector.Service.DETECT_ORIENTED)
_SELECT = str(Motion.Service.RESOLVE_REACHABLE)
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)
_READ_STATE = str(Motor.Service.READ_STATE)
_TCP_SNAP = str(Motion.Service.TCP_SNAPSHOT)
_LIST_WP = str(Waypoint.Service.LIST)
_LIST_GROUPS = str(Waypoint.Service.LIST_GROUPS)
_LIST_MEMBERS = str(Waypoint.Service.LIST_GROUP_MEMBERS)

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186, gripper_close_raw=1935,
    gripper_index=5, gripper_held_threshold_raw=2100,
)
_SPECS = {SO: _SPEC, OMX: _SPEC}
_BASE_OMX = BasePose(x=0.0342, y=0.2702, z=-0.0094, yaw_rad=math.radians(-3.33))
_HELD_RAW = 2400  # gap > margin → HELD


@pytest.fixture(autouse=True)
def _fast(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)
    monkeypatch.setattr(steps, "_SEARCH_SETTLE_S", 0.0)


def _wp(robot: str, name: str, rid: int = 1) -> WaypointRecord:
    return WaypointRecord(
        id=rid, robot_id=robot, name=name,
        joint_values=[0.1 * rid] * 6, joint_names=[], created_at=_TS,
    )


def _det(
    position=(0.20, 0.05, 0.024), score=0.85, base_z=0.0, height=0.024,
    footprint=(0.024, 0.022),
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=0.0, footprint=footprint,
        points=[(0.2, 0.05, 0.01)] * 60,
    )


def _joint_state(gripper_raw: int) -> JointState:
    pos = [0] * 6
    pos[_SPEC.gripper_index] = gripper_raw
    return JointState(
        robot_id=SO, seq=0, timestamp_unix=0.0,
        positions_raw=pos, loads_raw=None,
    )


def _tcp(position, joints) -> TcpState:
    return TcpState(
        robot_id=OMX, seq=0, timestamp_unix=0.0, position=position,
        quaternion=(0.0, 0.0, 0.0, 1.0), joint_names=[], joints=list(joints),
    )


class _Rt:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001, ANN201
        raise AssertionError("module runtime 호출 금지 — ctx 로만")


def _module(checker=None) -> HandoverModule:  # noqa: ANN001
    return HandoverModule(
        _Rt(), _SPECS, omx_base_pose=_BASE_OMX, checker=checker
    )  # type: ignore[arg-type]


def _happy_script() -> dict:
    """happy path 스크립트 — place_object="" (수취까지, 적치 생략)."""
    so_wps = ListWaypointsResponse(waypoints=[_wp(SO, "home", 1)])
    omx_wps = ListWaypointsResponse(
        waypoints=[_wp(OMX, "home", 2), _wp(OMX, "handover", 3)]
    )
    return {
        _LIST_WP: [so_wps, omx_wps, omx_wps],
        _LIST_GROUPS: [ListGroupsResponse(groups=[
            WaypointGroupRecord(id=1, robot_id=SO, name="search")
        ])],
        _LIST_MEMBERS: [ListGroupMembersResponse(waypoints=[_wp(SO, "s0", 4)])],
        _DETECT: [DetectOrientedResponse(found=True, candidates=[_det()])],
        _SELECT: [
            # omx pick: [pre, grasp, lift] 3해 / so 수취: [pre, grasp] 2해
            ResolveReachableResponse(
                index=0, solutions=[[0.1] * 6, [0.2] * 6, [0.3] * 6]
            ),
            ResolveReachableResponse(index=0, solutions=[[0.4] * 6, [0.5] * 6]),
        ],
        # so home / omx home / sweep / omx pre / so home / omx handover /
        # so pre / omx retreat home / so 종료 home = 9
        _MOVE_J: [MoveJResponse()] * 9,
        # omx grasp / omx lift / so obj / so withdraw = 4
        _MOVE_L: [MoveLResponse()] * 4,
        # omx open / omx close / so open / so close / omx release open = 5
        _GRIP: [SetGripperResponse()] * 5,
        # omx close후 / omx lift후 / handover 도달 / so close후 / so 이탈후 = 5
        _READ_STATE: [_joint_state(_HELD_RAW)] * 5,
        _TCP_SNAP: [_tcp((0.12, -0.08, 0.14), [0.1, 0.2, 0.3, 0.4])],
    }


def _ctx(script: dict) -> FakeContext:
    return FakeContext(robots=[SO, OMX], specs=_SPECS, service_script=script)


# ─── ① frame 변환 ────────────────────────────────────────────────────


def test_base_pose_transform_roundtrip():
    p_world = (0.21, -0.09, 0.05)
    p_omx = steps.world_to_robot(p_world, _BASE_OMX)
    back = steps.robot_to_world(p_omx, _BASE_OMX)
    assert back == pytest.approx(p_world, abs=1e-12)
    # 회전 방향 sanity: omx base 는 world (0.034, 0.270) — omx 원점의 world 좌표
    assert steps.robot_to_world((0.0, 0.0, 0.0), _BASE_OMX) == pytest.approx(
        (0.0342, 0.2702, -0.0094)
    )


# ─── ② happy path + 수취 순서 불변식 ─────────────────────────────────


async def test_scenario_happy_path_and_release_order():
    ctx = _ctx(_happy_script())
    await _module().scenario(ctx, pick_object="white cube")
    # 수취 순서 불변식: so101 close → so101 held 판정 → 그 뒤에만 omx open.
    log = ctx.wire.call_log
    grip_events = [
        (i, c["robot_id"], c["req"].position_raw)
        for i, c in enumerate(log) if c["key"] == _GRIP
    ]
    # 순서: omx open(준비) → omx close(집기) → so open(수취 준비) →
    #       so close(수취) → omx open(release)
    robots = [(r, raw == _SPEC.gripper_open_raw) for _, r, raw in grip_events]
    assert robots == [
        (OMX, True), (OMX, False), (SO, True), (SO, False), (OMX, True)
    ], grip_events
    so_close_i = grip_events[3][0]
    omx_release_i = grip_events[4][0]
    # so close 와 omx release 사이에 so101 held 판정(READ_STATE)이 있어야 한다
    between = [
        c for c in log[so_close_i:omx_release_i]
        if c["key"] == _READ_STATE and c["robot_id"] == SO
    ]
    assert between, "so101 held 판정 전에 omx 가 열림 — 낙하 위험 순서 위반"
    # robot-scoped 라우팅: omx 명령이 omx 로 갔는지 (참여 명부 검증 경유)
    assert {c["robot_id"] for c in ctx.calls(_MOVE_L)} == {SO, OMX}


async def test_scenario_fails_fast_without_handover_waypoint():
    script = _happy_script()
    omx_home_only = ListWaypointsResponse(waypoints=[_wp(OMX, "home", 2)])
    script[_LIST_WP] = [
        ListWaypointsResponse(waypoints=[_wp(SO, "home", 1)]),
        omx_home_only, omx_home_only,
    ]
    ctx = _ctx(script)
    with pytest.raises(TaskError, match="handover"):
        await _module().scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []  # 모션 0 시점 실패


# ─── ③ 수취 충돌 게이트 ──────────────────────────────────────────────


class _FakeChecker:
    margin_m = 0.02

    def __init__(self, hits: list[bool]) -> None:
        self.hits = hits
        self.calls = 0

    def path_in_collision(self, path, joints_b) -> bool:  # noqa: ANN001
        self.calls += 1
        return self.hits.pop(0)

    def in_collision(self, ja, jb) -> bool:  # noqa: ANN001
        return False


async def test_plan_receive_retries_past_colliding_group():
    checker = _FakeChecker(hits=[True, False])
    ctx = _ctx({
        _TCP_SNAP: [_tcp((0.12, -0.08, 0.14), [0.1] * 4)],
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
            ResolveReachableResponse(index=0, solutions=[[0.3] * 6, [0.4] * 6]),
        ],
    })
    sols, _quat, _obj, _oj = await steps.plan_receive(
        ctx, SO, OMX, _BASE_OMX, checker  # type: ignore[arg-type]
    )
    assert checker.calls == 2  # 1차 충돌 → 그룹 제외 재-resolve → 2차 통과
    assert len(ctx.calls(_SELECT)) == 2
    assert sols[0] == [0.3] * 6


async def test_plan_receive_all_colliding_fails_explicitly():
    checker = _FakeChecker(hits=[True, True, True])
    ctx = _ctx({
        _TCP_SNAP: [_tcp((0.12, -0.08, 0.14), [0.1] * 4)],
        _SELECT: [
            ResolveReachableResponse(index=0, solutions=[[0.1] * 6, [0.2] * 6]),
        ] * 3,
    })
    with pytest.raises(NoReachableGrasp, match="충돌"):
        await steps.plan_receive(
            ctx, SO, OMX, _BASE_OMX, checker  # type: ignore[arg-type]
        )


# ─── ⑤ module 배선 ───────────────────────────────────────────────────


async def test_module_list_robots_and_preview():
    mod = _module()
    robots = await mod.list_robots(ListRobotsRequest())
    assert robots.robot_ids == [SO, OMX]
    res = await mod.preview(PreviewRequest())
    top = [e.name for e in res.entries if e.depth == 0]
    # 시나리오 골격 잠금 — 구조를 바꾸면 이 목록도 같이 (계약 잠금, PnP 동형)
    assert top == [
        "named_waypoint", "named_waypoint", "named_waypoint",
        "go_home", "go_home", "set_gripper", "detect",
        "plan_omx_pick", "omx_pick", "go_home", "omx_present",
        "plan_receive", "set_gripper", "receive", "omx_retreat",
        "place_into", "go_home",
    ]
