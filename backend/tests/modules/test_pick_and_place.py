"""Pick & Place task 테스트 — 순수 함수(geometry) / FakeContext 시나리오 / module wire.

의미 (뒤집으면 회귀): height prior 무시 / 후보 가족(tilt×yaw×flip) 수·보정 부호 변질 /
place 분기가 pick-only 에서 실행 / 실패가 침묵 성공 / place 검출 실패 시 release
(물체 낙하) / RUN 동시 실행 허용 / gripper raw 가 spec 아닌 추측값 / 도달 전멸(-1)
침묵 통과.
"""

from __future__ import annotations

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
    StopResponse,
)
from modules.motor.contract import Motor, SetGripperResponse
from modules.tasks.core.errors import DetectionNotFound, NoReachableGrasp
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.core.contract import TaskState, TaskStatus
from modules.tasks.pick_and_place import geometry, steps
from modules.tasks.core.contract import ControlRequest
from modules.tasks.pick_and_place.contract import ListRobotsRequest, RunRequest
from modules.tasks.pick_and_place.module import PickAndPlaceModule

_BOT = "so101_6dof_0"

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=5,
    gripper_held_threshold_raw=2100,
)

_DETECT = str(Detector.Service.DETECT_ORIENTED)
_SELECT = str(Motion.Service.RESOLVE_REACHABLE)
_MOVE_J = str(Motion.Service.MOVE_J)
_MOVE_L = str(Motion.Service.MOVE_L)
_GRIP = str(Motor.Service.SET_GRIPPER)


@pytest.fixture(autouse=True)
def _no_settle(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(steps, "_GRIPPER_SETTLE_S", 0.0)  # 테스트 즉시 진행


def _det(
    score: float = 0.9,
    height: float = 0.023,
    position: tuple[float, float, float] = (0.2, 0.05, 0.023),
    base_z: float = 0.0,
    footprint: tuple[float, float] = (0.023, 0.022),
    grasp_yaw: float = 0.3,
) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube", position=position, score=score, base_z=base_z,
        height=height, grasp_yaw=grasp_yaw, footprint=footprint,
    )


# ─── geometry 순수 함수 ──────────────────────────────────────────────


def test_select_pick_target_prior_and_score():
    best = geometry.select_pick_target(
        # 3번째는 score 최고지만 height prior 탈락 — confidence 무관 reject
        [_det(score=0.5), _det(score=0.9), _det(score=0.99, height=0.5)],
        prompt="cube",
    )
    assert best.score == 0.9

    with pytest.raises(DetectionNotFound, match="검출 0건"):
        geometry.select_pick_target([], prompt="cube")
    with pytest.raises(DetectionNotFound, match="height prior"):
        geometry.select_pick_target([_det(height=0.5)], prompt="cube")


def test_plan_grasp_family_and_lateral():
    target = _det()
    plan = geometry.plan_grasp(target)
    assert len(plan) == 11 * 2 * 2  # tilt × yaw(짧/긴 변) × flip

    first = plan[0]  # tilt=0 이 가장 앞 (작은 tilt 우선 probe)
    assert "tilt=+0" in first.label
    # 단일 가동 조 보정 — 짧은 변(0.022) 기준: across/2 + 여유 − TCP→고정조
    assert first.lateral == pytest.approx(0.022 / 2 + 0.005 - 0.0079)
    # pre 는 윗면 + 0.06, grasp 는 중간 높이 (테이블 여유 floor 위)
    assert first.pre[2] == pytest.approx(0.023 + 0.06)
    assert first.grasp[2] == pytest.approx(0.023 - 0.023 * 0.5)


def test_plan_grasp_z_floor_keeps_finger_off_table():
    thin = _det(height=0.016, position=(0.2, 0.05, 0.016))
    plan = geometry.plan_grasp(thin)
    # 얇은 물체 — 중간 높이가 테이블 여유(base_z+0.008) 밑으로 못 내려감
    assert all(c.grasp[2] >= thin.base_z + 0.008 - 1e-9 for c in plan)


def test_grasp_ik_groups_pair_pre_and_grasp():
    plan = geometry.plan_grasp(_det())
    groups = geometry.grasp_ik_groups(plan)
    assert len(groups) == len(plan)
    assert groups[0][0].position == plan[0].pre
    assert groups[0][1].position == plan[0].grasp
    assert groups[0][0].quaternion == plan[0].quat


def test_plan_place_release_height():
    spot = _det(position=(0.25, -0.05, 0.04), height=0.04)
    held = _det(height=0.023)
    pplan = geometry.plan_place(spot, held=held, lateral=0.008)
    # release z = spot 상면 + held/2 + 여유(0.005) — 물체 바닥이 상면에 닿게
    assert pplan[0].place[2] == pytest.approx(0.04 + 0.023 / 2 + 0.005)
    assert pplan[0].pre[2] == pytest.approx(pplan[0].place[2] + 0.06)
    assert len(pplan) == 11 * 2 * 2


# ─── 시나리오 (FakeContext — 하드웨어/wire 없음, step 은 게이트 없이 실행) ──


def _module_for_scenario() -> PickAndPlaceModule:
    class _Rt:
        def publish(self, k: str, e: BaseModel) -> None: ...
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return PickAndPlaceModule(_Rt(), {})  # type: ignore[arg-type]


def _pick_script(**overrides) -> dict:
    """pick 경로 성공 스크립트 — 서비스 키별 응답 (실패 주입은 overrides)."""
    script = {
        _DETECT: [DetectOrientedResponse(found=True, candidates=[_det()])],
        _SELECT: [ResolveReachableResponse(index=0)],
        _MOVE_J: [MoveJResponse()],
        _MOVE_L: [MoveLResponse()] * 2,  # descend + lift
        _GRIP: [SetGripperResponse()] * 2,  # open + close
    }
    script.update(overrides)
    return script


async def test_scenario_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = FakeContext(robots=[_BOT], specs={_BOT: _SPEC}, service_script=_pick_script())

    await mod.scenario(ctx, pick_object="white cube")

    # 서비스 호출 순서 = 검출 → 선별 → 접근 → open → 하강 → close → 들어올림
    assert ctx.keys() == [
        _DETECT, _SELECT, _MOVE_J, _GRIP, _MOVE_L, _GRIP, _MOVE_L,
    ]
    # place 분기 안 탐 (detect 1회뿐) + 든 채 종료 (마지막 gripper = close raw)
    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [_SPEC.gripper_open_raw, _SPEC.gripper_close_raw]


async def test_scenario_with_place_branch():
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[_det()]),
                DetectOrientedResponse(
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [ResolveReachableResponse(index=0)] * 2,
            _MOVE_J: [MoveJResponse()] * 2,  # pre_grasp + pre_place
            _MOVE_L: [MoveLResponse()] * 4,  # descend/lift + lower/retreat
            _GRIP: [SetGripperResponse()] * 3,  # open/close + release
        },
    )

    await mod.scenario(ctx, pick_object="white cube", place_object="red box")

    grips = [c["req"].position_raw for c in ctx.calls(_GRIP)]
    assert grips == [
        _SPEC.gripper_open_raw, _SPEC.gripper_close_raw,
        _SPEC.gripper_open_raw,  # 마지막 open = release
    ]
    assert len(ctx.calls(_MOVE_L)) == 4
    # 계획 먼저, 실행 나중 (#2): 검출·선별(DETECT/SELECT)이 전부 끝난 뒤에야 첫
    # 모션(MOVE_J)이 나간다 — 못 놓을 물체를 집는 일이 없도록.
    keys = ctx.keys()
    first_motion = keys.index(_MOVE_J)
    planning = keys[:first_motion]
    assert planning == [_DETECT, _SELECT, _DETECT, _SELECT]  # 집기·놓기 둘 다 계획


async def test_scenario_detect_fail_raises_before_motion():
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT], specs={_BOT: _SPEC},
        service_script={
            _DETECT: [DetectOrientedResponse(found=False, candidates=[])],
        },
    )
    with pytest.raises(DetectionNotFound):
        await mod.scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []  # 검출 실패면 모션 0


async def test_scenario_ik_exhausted_raises():
    """RESOLVE_REACHABLE 의 -1 은 데이터 — step 이 치명 판정 (침묵 -1 통과 금지)."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT], specs={_BOT: _SPEC},
        service_script=_pick_script(
            **{_SELECT: [ResolveReachableResponse(index=-1, message="전멸")]}
        ),
    )
    with pytest.raises(NoReachableGrasp, match="전멸"):
        await mod.scenario(ctx, pick_object="white cube")
    assert ctx.calls(_MOVE_J) == []  # 전멸이면 모션 0


async def test_scenario_place_unreachable_fails_before_pick():
    """놓을 곳 IK 불가 → 집기 **전에** 실패 (#2). 물체를 쥔 채 멈추는 corrupt 상태를
    막는다 — 실물 실패 로그(resolve_place IK 불가) 그대로가 회귀 시나리오.

    계획 단계(집기·놓기 검출+IK)가 모션 0 이라, 놓기 IK 가 -1 이면 아무 모션도
    나가기 전에 raise → gripper·MOVE 호출 0 (아무것도 안 집음)."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        specs={_BOT: _SPEC},
        service_script={
            _DETECT: [
                DetectOrientedResponse(found=True, candidates=[_det()]),  # 집기 검출 OK
                DetectOrientedResponse(  # 놓기 검출 OK
                    found=True,
                    candidates=[_det(position=(0.25, -0.05, 0.04), height=0.04)],
                ),
            ],
            _SELECT: [
                ResolveReachableResponse(index=0),  # 집기 도달 가능
                ResolveReachableResponse(index=-1, message="놓기 IK 전멸"),  # 놓기 불가
            ],
        },
    )
    with pytest.raises(NoReachableGrasp, match="놓기 IK 전멸"):
        await mod.scenario(ctx, pick_object="white cube", place_object="red box")
    # 핵심 (#2): 물리 동작이 하나도 안 나감 — 아무것도 안 집었으니 든 채 멈춤 없음.
    assert ctx.calls(_MOVE_J) == []
    assert ctx.calls(_MOVE_L) == []
    assert ctx.calls(_GRIP) == []


async def test_preview_lists_full_steps_without_motion():
    """PREVIEW(#1): dry-run 으로 전체 step 목록 수집 — 실 runtime·모션 없음.

    imperative 시나리오라 정적 목록이 없어 canned dry-run 으로 traverse. 놓기 포함
    최대 경로의 step title 이 실행 순서대로 나와야 (뒤집으면 목록 누락 회귀)."""
    from modules.tasks.pick_and_place.contract import PreviewRequest

    mod = _module_for_scenario()  # 빈 specs → dummy 로 채워 dry-run 진행
    res = await mod.preview(PreviewRequest())

    titles = [s.title for s in res.steps]
    assert titles == [
        "집기 계획", "검출", "파지 후보 선별",
        "놓기 계획", "검출", "적치 후보 선별",
        "집기 실행", "파지 접근", "그리퍼 열기", "하강", "그리퍼 닫기", "들어올리기",
        "놓기 실행", "적치 접근", "내리기", "내려놓기", "후퇴",
    ]
    # 중첩 depth — 계획/실행은 0, 그 안 step 은 1
    assert res.steps[0].depth == 0 and res.steps[1].depth == 1


# ─── module wire (runner 결합 e2e) ───────────────────────────────────


class _WireStub:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []
        self.responses: dict[str, BaseModel] = {}

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001, ANN201
        r = self.responses.get(str(key))
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


async def test_module_run_reports_failure_reason_and_allows_rerun():
    rt = _WireStub()
    rt.responses[_DETECT] = DetectOrientedResponse(found=False, candidates=[])
    rt.responses[str(Motion.Service.STOP)] = StopResponse(ok=True)  # abort 안전 경로
    mod = PickAndPlaceModule(rt, {})  # type: ignore[arg-type]

    res = await mod.run(RunRequest(pick_object="white cube"))
    assert res.accepted
    assert mod.task._run is not None and mod.task._run.handle is not None
    await mod.task._run.handle

    states = [e for k, e in rt.published if k.endswith("/state")]
    final = states[-1]
    assert isinstance(final, TaskState)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "white cube" in final.error  # 사유 표시

    # 실패 후 재실행 가능 (상태 corrupt 없음)
    res2 = await mod.run(RunRequest(pick_object="white cube"))
    assert res2.accepted


async def test_module_control_without_run_says_why():
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    r = await mod.pause(ControlRequest())
    assert not r.ok and r.message  # 침묵 금지


def test_task_robots_constant_matches_scenario_binding():
    """TASK_ROBOTS = 바인딩 SSOT (scenario 도 여기서 파생) — 값이 바뀌면 프론트
    스트림 키/실 robot 대상이 같이 바뀌므로 명시 잠금."""
    assert PickAndPlaceModule.TASK_ROBOTS == ("so101_6dof_0",)


async def test_list_robots_returns_task_robots():
    """LIST_ROBOTS = 프론트가 {robot_id} 를 채우는 유일한 채널 — TASK_ROBOTS 와
    어긋나면 프론트가 존재하지 않는 스트림을 구독한다 (침묵 무데이터)."""
    mod = PickAndPlaceModule(_WireStub(), {})  # type: ignore[arg-type]
    res = await mod.list_robots(ListRobotsRequest())
    assert res.robot_ids == list(PickAndPlaceModule.TASK_ROBOTS)
