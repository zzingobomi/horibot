"""pick-and-place task #1 mock-runtime e2e (§17.5 recipe). 실 하드웨어/모델 0.

fake runtime 이 waypoint group / MoveJ / Detect Top-K / gripper 를 canned 응답 →
runner 가 전 step 실행. 의미(뒤집으면 회귀):
  - Waypoint Group 전 자세 순회하며 Detect (첫 자세 break 아님) — DETECT 호출 수
  - SelectTarget 이 누적 후보 중 **최고 score** 선택 (grasp x 가 low 후보 것이면 실패)
  - GraspPolicy grasp_z = base_z + height·0.5 (§17.5 순수 계산) — grasp MOVE_L 좌표
  - open→close→open(release) gripper raw 시퀀스 (spec 값)
  - 전체 SUCCESS 도달
"""

from __future__ import annotations

from datetime import UTC, datetime
from typing import Any

import pytest
from pydantic import BaseModel

from modules.detector.contract import DetectResponse, Detection, Detector
from modules.motion.contract import Motion, MoveJResponse, MoveLResponse
from modules.motor.contract import Motor, SetGripperResponse
from modules.task.contract import TaskStatus
from modules.task.runner import TaskRunner
from modules.task.spec import TaskRobotSpec
from modules.task.tasks import build_task
from modules.waypoint.contract import (
    ListGroupMembersResponse,
    ListGroupsResponse,
    ListWaypointsResponse,
    WaypointGroupRecord,
    WaypointRecord,
)

_ROBOT = "so101_6dof_0"
_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=6,
    gripper_held_threshold_raw=2000,
)
# 검출 후보 2개 — high(0.3,0,0.05 score .9) / low(0.1,0.2,0.05 score .4). base_z 0, h .05.
_HIGH = Detection(
    prompt="white cube", position=(0.3, 0.0, 0.05), score=0.9, base_z=0.0, height=0.05
)
_LOW = Detection(
    prompt="white cube", position=(0.1, 0.2, 0.05), score=0.4, base_z=0.0, height=0.05
)


def _wp(name: str, joints: list[float]) -> WaypointRecord:
    return WaypointRecord(
        robot_id=_ROBOT, name=name, joint_values=joints,
        joint_names=[f"j{i}" for i in range(len(joints))],
        created_at=datetime(2026, 1, 1, tzinfo=UTC),
    )


class _FakeRuntime:
    """canned 응답 + call/publish 캡처. DETECT 는 매번 [high, low] (자세마다 누적)."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, BaseModel]] = []
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key, event) -> None:  # noqa: ANN001
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0) -> Any:  # noqa: ANN001
        self.calls.append((str(key), req))
        if res_cls is ListGroupsResponse:
            return ListGroupsResponse(
                groups=[WaypointGroupRecord(id=1, robot_id=_ROBOT, name="search")]
            )
        if res_cls is ListGroupMembersResponse:
            return ListGroupMembersResponse(
                waypoints=[_wp("s1", [0.1] * 6), _wp("s2", [0.2] * 6)]
            )
        if res_cls is ListWaypointsResponse:  # MoveJ("home") lookup
            return ListWaypointsResponse(waypoints=[_wp("home", [0.0] * 6)])
        if res_cls is MoveJResponse:
            return MoveJResponse(accepted=True)
        if res_cls is MoveLResponse:
            return MoveLResponse(accepted=True)
        if res_cls is DetectResponse:
            return DetectResponse(found=True, candidates=[_HIGH, _LOW])
        if res_cls is SetGripperResponse:
            return SetGripperResponse(ok=True)
        raise AssertionError(f"예상 못한 call: {key} / {res_cls.__name__}")


def _reqs(rt: _FakeRuntime, key) -> list[BaseModel]:  # noqa: ANN001
    return [r for k, r in rt.calls if k == str(key)]


def _last_status(rt: _FakeRuntime) -> TaskStatus | None:
    states = [e for k, e in rt.published if k.endswith("/state")]
    return states[-1].status if states else None  # type: ignore[attr-defined]


async def test_pick_and_place_runs_to_success():
    rt = _FakeRuntime()
    runner = TaskRunner(rt, _ROBOT, _SPEC, gripper_raw=lambda: 2500)
    task = build_task(
        "pick_and_place",
        {"pick_object": "white cube", "place_object": "blue box"},
    )
    assert runner.run(task) is True
    assert runner._handle is not None
    await runner._handle

    assert _last_status(rt) == TaskStatus.SUCCESS

    # Waypoint Group 순회: pick 2자세 + place 2자세 = DETECT 4회 (첫 검출 break 아님)
    assert len(_reqs(rt, Detector.Service.DETECT)) == 4

    # SelectTarget 이 최고 score(_HIGH, x=0.3) 선택 → GraspPolicy grasp_z=0+0.05·0.5=0.025.
    # grasp MOVE_L 좌표 = (0.3, 0.0, 0.025). low 후보(x=0.1)면 selection 회귀.
    movel = [r.target_position for r in _reqs(rt, Motion.Service.MOVE_L)]  # type: ignore[attr-defined]
    assert any(
        pos == pytest.approx((0.3, 0.0, 0.025)) for pos in movel
    ), movel

    # gripper open→close→open(release) raw 시퀀스 (spec 값)
    grips = [r.position_raw for r in _reqs(rt, Motor.Service.SET_GRIPPER)]  # type: ignore[attr-defined]
    assert grips == [3186, 1935, 3186], grips


async def test_pick_only_no_place_skips_place_steps():
    rt = _FakeRuntime()
    runner = TaskRunner(rt, _ROBOT, _SPEC, gripper_raw=lambda: 2500)
    task = build_task("pick_and_place", {"pick_object": "white cube"})
    runner.run(task)
    assert runner._handle is not None
    await runner._handle

    assert _last_status(rt) == TaskStatus.SUCCESS
    # place 없음 → DETECT 2회 (pick 2자세만), gripper open→close (release 없음)
    assert len(_reqs(rt, Detector.Service.DETECT)) == 2
    grips = [r.position_raw for r in _reqs(rt, Motor.Service.SET_GRIPPER)]  # type: ignore[attr-defined]
    assert grips == [3186, 1935], grips


def test_pick_and_place_requires_pick_object():
    import pytest as _pytest

    with _pytest.raises(ValueError, match="pick_object"):
        build_task("pick_and_place", {})
