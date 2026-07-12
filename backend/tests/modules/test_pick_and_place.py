"""Pick & Place task 테스트 — 순수 함수(geometry) / FakeContext 시나리오 / module wire.

의미 (뒤집으면 회귀): height prior 무시 / 후보 가족(tilt×yaw×flip) 수·보정 부호 변질 /
place 분기가 pick-only 에서 실행 / 실패가 침묵 성공 / place 검출 실패 시 release
(물체 낙하) / RUN 동시 실행 허용.
"""

from __future__ import annotations

import pytest
from pydantic import BaseModel

from modules.detector.contract import DetectOrientedResponse, OrientedDetection
from modules.tasks.core.errors import DetectionNotFound, NoReachableGrasp
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.contract import TaskState, TaskStatus
from modules.tasks.pick_and_place import geometry
from modules.tasks.pick_and_place.contract import ControlRequest, RunRequest
from modules.tasks.pick_and_place.module import TASK_INFO, PickAndPlaceModule

_BOT = "so101_6dof_0"


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


# ─── 시나리오 (FakeContext — 하드웨어/wire 없음) ─────────────────────


def _module_for_scenario() -> PickAndPlaceModule:
    class _Rt:
        def publish(self, k: str, e: BaseModel) -> None: ...
        async def call(self, *a, **kw): ...  # noqa: ANN002, ANN003, ANN201

    return PickAndPlaceModule(_Rt(), {})  # type: ignore[arg-type]


async def test_scenario_pick_only_sequence():
    mod = _module_for_scenario()
    ctx = FakeContext(robots=[_BOT], detect_script={"white cube": [[_det()]]})

    await mod._scenario(ctx, pick_object="white cube")

    assert ctx.kinds() == [
        "detect_oriented", "select_reachable",
        "move_j_pose", "gripper", "move_l", "gripper", "move_l",
    ]
    labels = ctx.labels()
    assert [lb for lb in labels if lb in ("pre_grasp", "descend", "lift")] == [
        "pre_grasp", "descend", "lift",
    ]
    assert "detect_place" not in labels  # place 분기 안 탐
    grips = [c["action"] for c in ctx.calls("gripper")]
    assert grips == ["open", "close"]  # 든 채 종료 (release 없음)
    assert ctx.result_log[-1][0] == "grasp"  # 파지 계획 씬 노출


async def test_scenario_with_place_branch():
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        detect_script={
            "white cube": [[_det()]],
            "red box": [[_det(position=(0.25, -0.05, 0.04), height=0.04)]],
        },
    )

    await mod._scenario(ctx, pick_object="white cube", place_object="red box")

    labels = ctx.labels()
    for lb in ("pre_place", "lower", "retreat"):
        assert lb in labels
    grips = [c["action"] for c in ctx.calls("gripper")]
    assert grips == ["open", "close", "open"]  # 마지막 open = release


async def test_scenario_detect_fail_raises_before_motion():
    mod = _module_for_scenario()
    ctx = FakeContext(robots=[_BOT], detect_script={"white cube": [[]]})
    with pytest.raises(DetectionNotFound):
        await mod._scenario(ctx, pick_object="white cube")
    assert ctx.calls("move_j_pose") == []  # 검출 실패면 모션 0


async def test_scenario_ik_exhausted_raises():
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        detect_script={"white cube": [[_det()]]},
        reachable_script=[-1],
    )
    with pytest.raises(NoReachableGrasp):
        await mod._scenario(ctx, pick_object="white cube")


async def test_scenario_place_detect_fail_keeps_holding():
    """place 검출 실패 → raise (release 금지 — 물체 낙하 방지, 든 채 FAILED)."""
    mod = _module_for_scenario()
    ctx = FakeContext(
        robots=[_BOT],
        detect_script={"white cube": [[_det()]], "red box": [[]]},
    )
    with pytest.raises(DetectionNotFound):
        await mod._scenario(ctx, pick_object="white cube", place_object="red box")
    grips = [c["action"] for c in ctx.calls("gripper")]
    assert grips[-1] == "close"  # release 안 함


# ─── module wire (runner 결합 e2e) ───────────────────────────────────


class _WireStub:
    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []
        self.responses: dict[str, BaseModel] = {}

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001, ANN201
        r = self.responses.get(str(key))
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


async def test_module_run_reports_failure_reason_and_allows_rerun():
    rt = _WireStub()
    rt.responses["srv/detector/detect_oriented"] = DetectOrientedResponse(
        found=False, candidates=[]
    )
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


def test_task_info_params_derive_from_run_request():
    specs = TASK_INFO.param_specs()
    assert [(s.name, s.type, s.required) for s in specs] == [
        ("pick_object", "str", True),
        ("place_object", "str", False),
    ]
