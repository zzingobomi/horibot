"""Day-1 primitive step 단위테스트 (§17.1 표: MoveJ/MoveTCP/Gripper/VerifyGrasp).

fake runtime 으로 서비스 call 캡처 — 실 motion/motor 없이 step 내부 로직 검증.
각 assert 는 "뒤집으면 잡히는 회귀":
  - MoveJ: 이름→그 waypoint 의 joint_values 로 MOVE_J (엉뚱한 자세 / lookup 누락)
  - MoveJ: 없는 이름 = fail-fast (silent no-op 방지, motion 안 침)
  - Gripper: open/close ↔ spec open/close raw (open/close 뒤바뀜)
  - VerifyGrasp: raw < threshold = 빈손 raise (비교 방향 뒤집힘)
  - MoveTCP: target + offset = MOVE_L position (offset 미적용/축 오류)
"""

from __future__ import annotations

from datetime import UTC, datetime

import pytest
from pydantic import BaseModel

from modules.motion.contract import Motion, MoveJResponse, MoveLResponse
from modules.motor.contract import Motor, SetGripperResponse
from modules.task.schema import Position3
from modules.task.spec import TaskRobotSpec
from modules.task.step import StepContext
from modules.task.steps import Gripper, MoveJ, MoveTCP, VerifyGrasp
from modules.waypoint.contract import ListWaypointsResponse, WaypointRecord

_ROBOT = "so101_6dof_0"
_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=6,
    gripper_held_threshold_raw=2123,
)


class _FakeRuntime:
    """runtime.call 캡처 + canned 응답 (res_cls dispatch). robot-scoped/agnostic 모두
    runtime.call 로 수렴하므로 하나로 충분."""

    def __init__(self, waypoints: list[WaypointRecord] | None = None) -> None:
        self.calls: list[tuple[str, BaseModel]] = []
        self._waypoints = waypoints or []

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001, ANN201
        self.calls.append((str(key), req))
        if res_cls is ListWaypointsResponse:
            return ListWaypointsResponse(waypoints=self._waypoints)
        if res_cls is MoveJResponse:
            return MoveJResponse(accepted=True)
        if res_cls is MoveLResponse:
            return MoveLResponse(accepted=True)
        if res_cls is SetGripperResponse:
            return SetGripperResponse(ok=True)
        raise AssertionError(f"예상 못한 call: {key} / {res_cls.__name__}")


def _wp(name: str, joints: list[float]) -> WaypointRecord:
    return WaypointRecord(
        robot_id=_ROBOT,
        name=name,
        joint_values=joints,
        joint_names=[f"j{i}" for i in range(len(joints))],
        created_at=datetime(2026, 1, 1, tzinfo=UTC),
    )


def _ctx(rt, *, spec=None, gripper_raw=None) -> StepContext:  # noqa: ANN001
    return StepContext(rt, _ROBOT, spec, gripper_raw)


def _reqs(rt: _FakeRuntime, key) -> list[BaseModel]:  # noqa: ANN001
    return [r for k, r in rt.calls if k == str(key)]


async def test_movej_resolves_waypoint_name_to_that_pose():
    wps = [_wp("home", [0.0] * 6), _wp("search_1", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])]
    rt = _FakeRuntime(waypoints=wps)
    await MoveJ(waypoint="search_1").execute(_ctx(rt))

    movej = _reqs(rt, Motion.Service.MOVE_J)
    assert len(movej) == 1
    # home(0,0,..) 아니라 search_1 의 joint_values 여야 함
    assert movej[0].target_joints == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # type: ignore[attr-defined]


async def test_movej_unknown_waypoint_fails_without_moving():
    rt = _FakeRuntime(waypoints=[_wp("home", [0.0] * 6)])
    with pytest.raises(RuntimeError, match="없음"):
        await MoveJ(waypoint="nope").execute(_ctx(rt))
    # lookup 실패면 MOVE_J 를 아예 안 쳐야 함 (엉뚱한 자세로 가는 것 방지)
    assert _reqs(rt, Motion.Service.MOVE_J) == []


async def test_gripper_open_close_map_to_spec_raw():
    rt = _FakeRuntime()
    await Gripper(action="close").execute(_ctx(rt, spec=_SPEC))
    await Gripper(action="open").execute(_ctx(rt, spec=_SPEC))

    grip = _reqs(rt, Motor.Service.SET_GRIPPER)
    # close→close_raw, open→open_raw (뒤바뀌면 잡힘)
    assert [g.position_raw for g in grip] == [1935, 3186]  # type: ignore[attr-defined]


async def test_gripper_without_spec_fails_fast():
    rt = _FakeRuntime()
    with pytest.raises(RuntimeError, match="TaskRobotSpec"):
        await Gripper(action="close").execute(_ctx(rt))  # spec 미주입
    assert _reqs(rt, Motor.Service.SET_GRIPPER) == []


async def test_verifygrasp_below_threshold_is_empty():
    rt = _FakeRuntime()
    # raw < threshold(2123) = 빈손
    with pytest.raises(RuntimeError, match="빈손"):
        await VerifyGrasp().execute(_ctx(rt, spec=_SPEC, gripper_raw=lambda: 2000))
    # raw >= threshold = 잡힘 (예외 없음)
    await VerifyGrasp().execute(_ctx(rt, spec=_SPEC, gripper_raw=lambda: 2200))


async def test_verifygrasp_no_state_fails():
    rt = _FakeRuntime()
    with pytest.raises(RuntimeError, match="미수신"):
        await VerifyGrasp().execute(_ctx(rt, spec=_SPEC, gripper_raw=lambda: None))


async def test_movetcp_applies_offset_to_target():
    rt = _FakeRuntime()
    await MoveTCP(
        target=Position3(x=0.1, y=0.2, z=0.3),
        offset=Position3(x=0.0, y=0.0, z=0.06),
    ).execute(_ctx(rt))

    movel = _reqs(rt, Motion.Service.MOVE_L)
    assert len(movel) == 1
    # z 에 offset 더해진 (0.1, 0.2, 0.36)
    assert movel[0].target_position == pytest.approx((0.1, 0.2, 0.36))  # type: ignore[attr-defined]
