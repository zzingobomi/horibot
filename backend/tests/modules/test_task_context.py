"""TaskContext / RobotHandle / FakeContext / TaskMetadata 단위테스트.

의미 (뒤집으면 회귀): primitive 가 잘못된 wire 키/robot_id 로 나감 / 거부 응답이
침묵 통과 / on_abort 가 안 움직인 robot 까지 정지 / gripper raw 가 spec 아닌 추측값 /
FakeContext 표면이 실 ctx 와 드리프트 / param 스펙이 RunRequest 와 어긋남.
"""

from __future__ import annotations

from typing import Any

import pytest
from pydantic import BaseModel

from modules.detector.contract import DetectOrientedResponse, OrientedDetection
from modules.motion.contract import (
    MoveLResponse,
    SelectReachableResponse,
    StopResponse,
    TcpPose,
)
from modules.motor.contract import SetGripperResponse
from modules.tasks.core import context as context_mod
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import (
    MotionRejected,
    NoReachableGrasp,
    TaskError,
)
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.metadata import TaskMetadata, TaskParamSpec
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.core.contract import TraceEntry

_BOT = "test_bot_0"

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=5,
    gripper_held_threshold_raw=2100,
)


class _WireStub:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []
        self.responses: dict[str, Any] = {}
        self.published: list[tuple[str, BaseModel]] = []

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001, ANN201
        self.calls.append(
            {"key": str(key), "req": req, "robot_id": robot_id, "timeout": timeout}
        )
        r = self.responses.get(str(key))
        if isinstance(r, Exception):
            raise r
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


class _LinkStub:
    """runner 없이 ctx primitive 를 굴리는 관측 훅 stub."""

    def __init__(self) -> None:
        self.entries: list[TraceEntry] = []
        self.results: list[tuple[str, str, Any]] = []

    async def enter(self, kind: str, label: str) -> TraceEntry:
        entry = TraceEntry(
            label=label or f"{kind}#{len(self.entries) + 1}",
            kind=kind,
            status="running",
            started_unix=0.0,
        )
        self.entries.append(entry)
        return entry

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        entry.status = "completed"
        entry.detail = detail

    def fail(self, entry: TraceEntry, detail: str) -> None:
        entry.status = "failed"
        entry.detail = detail

    def emit_result(self, label: str, type_name: str, value: Any) -> None:
        self.results.append((label, type_name, value))


def _ctx(rt: _WireStub) -> tuple[TaskContext, _LinkStub]:
    ctx = TaskContext(rt, {_BOT: _SPEC})
    link = _LinkStub()
    ctx.bind_run(link, [_BOT])
    return ctx, link


def _det(score: float = 0.9) -> OrientedDetection:
    return OrientedDetection(
        prompt="cube",
        position=(0.2, 0.0, 0.05),
        score=score,
        base_z=0.0,
        height=0.05,
        grasp_yaw=0.1,
        footprint=(0.03, 0.02),
    )


# ─── RobotHandle primitive ───────────────────────────────────────────


async def test_move_l_calls_scoped_key_and_rejection_raises():
    rt = _WireStub()
    rt.responses["srv/motion/{robot_id}/move_l"] = MoveLResponse(accepted=True)
    ctx, link = _ctx(rt)
    bot = ctx.robot(_BOT)

    await bot.move_l((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0), label="descend")
    call = rt.calls[-1]
    assert call["robot_id"] == _BOT  # robot-scoped 키에 자동 주입
    assert call["req"].target_position == (0.1, 0.2, 0.3)
    assert link.entries[-1].status == "completed"

    rt.responses["srv/motion/{robot_id}/move_l"] = MoveLResponse(
        accepted=False, message="IK 불가"
    )
    with pytest.raises(MotionRejected, match="IK 불가"):
        await bot.move_l((9.9, 9.9, 9.9))
    assert link.entries[-1].status == "failed"  # 침묵 통과 금지


async def test_detect_oriented_returns_candidates_and_emits_result():
    rt = _WireStub()
    rt.responses["srv/detector/detect_oriented"] = DetectOrientedResponse(
        found=True, candidates=[_det()]
    )
    ctx, link = _ctx(rt)

    cands = await ctx.robot(_BOT).detect_oriented("cube", top_k=3, label="find")
    assert len(cands) == 1
    assert rt.calls[-1]["req"].robot_id == _BOT  # robot-agnostic — req 필드로
    assert rt.calls[-1]["req"].top_k == 3
    label, type_name, value = link.results[-1]  # 값 primitive 자동 노출
    assert label == "find" and type_name == "list" and len(value) == 1


async def test_select_reachable_negative_raises():
    rt = _WireStub()
    rt.responses["srv/motion/{robot_id}/select_reachable"] = SelectReachableResponse(
        index=-1, message="전멸"
    )
    ctx, _ = _ctx(rt)
    with pytest.raises(NoReachableGrasp, match="전멸"):
        await ctx.robot(_BOT).select_reachable(
            [[TcpPose(position=(0.1, 0.0, 0.1))]]
        )


async def test_gripper_uses_spec_raw_not_guess(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.setattr(context_mod, "_GRIPPER_SETTLE_S", 0.0)  # 테스트 즉시
    rt = _WireStub()
    rt.responses["srv/motor/{robot_id}/set_gripper"] = SetGripperResponse(ok=True)
    ctx, _ = _ctx(rt)
    bot = ctx.robot(_BOT)

    await bot.gripper("open")
    assert rt.calls[-1]["req"].position_raw == _SPEC.gripper_open_raw
    await bot.gripper("close")
    assert rt.calls[-1]["req"].position_raw == _SPEC.gripper_close_raw


async def test_gripper_without_spec_fails_fast():
    rt = _WireStub()
    ctx = TaskContext(rt, {})  # spec 미주입
    ctx.bind_run(_LinkStub(), [_BOT])
    with pytest.raises(TaskError, match="gripper spec"):
        await ctx.robot(_BOT).gripper("open")


async def test_on_abort_stops_only_moved_robots():
    rt = _WireStub()
    rt.responses["srv/motion/{robot_id}/move_l"] = MoveLResponse(accepted=True)
    rt.responses["srv/detector/detect_oriented"] = DetectOrientedResponse(
        found=True, candidates=[]
    )
    rt.responses["srv/motion/{robot_id}/stop"] = StopResponse(ok=True)
    ctx = TaskContext(rt, {})
    ctx.bind_run(_LinkStub(), ["bot_a", "bot_b"])

    await ctx.robot("bot_a").move_l((0.1, 0.0, 0.1))  # bot_a 만 모션
    await ctx.robot("bot_b").detect_oriented("cube")  # bot_b 는 검출만
    await ctx.on_abort()

    stops = [c for c in rt.calls if c["key"].endswith("/stop")]
    assert [c["robot_id"] for c in stops] == ["bot_a"]  # 안 움직인 robot 정지 금지


async def test_robot_not_in_declaration_raises_at_call_site():
    ctx, _ = _ctx(_WireStub())
    with pytest.raises(TaskError, match="undeclared_bot"):
        ctx.robot("undeclared_bot")


# ─── FakeContext (시나리오 테스트 표면) ──────────────────────────────


async def test_fake_context_scripts_and_records():
    ctx = FakeContext(
        robots=[_BOT],
        detect_script={"cube": [[], [_det()]]},
        reachable_script=[1],
    )
    bot = ctx.robot(_BOT)

    assert await bot.detect_oriented("cube") == []  # 1차 자세 — 미검출
    assert len(await bot.detect_oriented("cube")) == 1  # 2차 자세 — 검출
    idx = await bot.select_reachable([[TcpPose(position=(0.1, 0.0, 0.1))]])
    assert idx == 1
    await bot.move_l((0.1, 0.0, 0.1), label="descend")
    await bot.gripper("close")
    await ctx.wait(0.5)
    ctx.record("grasp", _det())

    assert ctx.kinds() == [
        "detect_oriented", "detect_oriented", "select_reachable",
        "move_l", "gripper", "wait",
    ]
    assert "descend" in ctx.labels()
    assert ctx.result_log[-1][0] == "grasp"


async def test_fake_context_reachable_minus_one_raises():
    ctx = FakeContext(robots=[_BOT], reachable_script=[-1])
    with pytest.raises(NoReachableGrasp):
        await ctx.robot(_BOT).select_reachable([[TcpPose(position=(0, 0, 0))]])


async def test_fake_context_fail_labels_inject_failure():
    ctx = FakeContext(robots=[_BOT], fail_labels={"descend"})
    with pytest.raises(MotionRejected):
        await ctx.robot(_BOT).move_l((0.1, 0.0, 0.1), label="descend")


# ─── TaskMetadata (GET /tasks param 파생) ────────────────────────────


class _RunReq(BaseModel):
    pick_object: str
    place_object: str = ""
    top_k: int = 5


def test_metadata_params_derive_from_run_request():
    meta = TaskMetadata(
        name="t", robots=[_BOT], description="d", run="srv/t/run",
        params_model=_RunReq,
    )
    assert meta.param_specs() == [
        TaskParamSpec(name="pick_object", type="str", required=True),
        TaskParamSpec(name="place_object", type="str", required=False, default=""),
        TaskParamSpec(name="top_k", type="int", required=False, default="5"),
    ]


def test_metadata_rejects_unsupported_param_type():
    class _Bad(BaseModel):
        poses: list[str] = []

    meta = TaskMetadata(
        name="bad", robots=[], description="", run="srv/bad/run", params_model=_Bad
    )
    with pytest.raises(TypeError, match="poses"):
        meta.param_specs()
