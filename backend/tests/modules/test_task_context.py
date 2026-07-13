"""TaskContext / @step / FakeContext 단위테스트.

의미 (뒤집으면 회귀): 참여 선언 밖 robot 명령 허용 (STOP 커버리지 구멍) /
agnostic 키에 robot_id= 침묵 허용 (거짓 라우팅) / on_abort 가 참여 robot 정지
누락 / spec 없는 gripper 류가 침묵 진행 / @step 게이트·depth·실패 경로가 어긋남 /
run 밖 step 이 게이트를 요구.
"""

from __future__ import annotations

import pytest

from framework.transport.protocol import RemoteError
from modules.motion.contract import (
    Motion,
    MoveLRequest,
    MoveLResponse,
    PoseTarget,
)
from modules.tasks.core.errors import TaskError
from modules.tasks.core.fake import FakeContext
from modules.tasks.core.spec import TaskRobotSpec
from modules.tasks.core.step import bind_link, reset_link, step
from modules.tasks.core.contract import TraceEntry

_BOT = "test_bot_0"

_SPEC = TaskRobotSpec(
    gripper_open_raw=3186,
    gripper_close_raw=1935,
    gripper_index=5,
    gripper_held_threshold_raw=2100,
)


class _LinkStub:
    """runner 없이 @step 을 굴리는 관측 훅 stub (RunLink 구현)."""

    def __init__(self) -> None:
        self.entries: list[TraceEntry] = []

    async def enter(self, name: str, depth: int, title: str = "") -> TraceEntry:
        entry = TraceEntry(
            name=name, title=title, depth=depth, status="running", started_unix=0.0
        )
        self.entries.append(entry)
        return entry

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        entry.status = "completed"
        if detail:
            entry.detail = detail

    def fail(self, entry: TraceEntry, detail: str) -> None:
        entry.status = "failed"
        entry.detail = detail


# ─── ctx.call — 단일 호출 표면 (참여 검증 + robot-scoped 키 주입) ────


async def test_call_scoped_key_passes_robot_id_after_validation():
    ctx = FakeContext(
        robots=[_BOT],
        service_script={Motion.Service.MOVE_L: [MoveLResponse()]},
    )
    await ctx.call(
        Motion.Service.MOVE_L,
        MoveLRequest(target=PoseTarget(kind="pose", position=(0.1, 0.2, 0.3))),
        MoveLResponse,
        robot_id=_BOT,
    )
    call = ctx.wire.call_log[-1]
    assert call["robot_id"] == _BOT  # 검증 통과 후 키 주입용으로 전달
    assert call["req"].target.position == (0.1, 0.2, 0.3)
    assert call["timeout"] is None  # 미지정 → contract 선언 기본값에 위임


async def test_call_undeclared_robot_rejected():
    """선언 밖 robot 명령 = on_abort STOP 커버리지 구멍 — 모든 명령에서 검증."""
    ctx = FakeContext(robots=[_BOT])
    with pytest.raises(TaskError, match="undeclared_bot"):
        await ctx.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target=PoseTarget(kind="pose", position=(0, 0, 0))),
            MoveLResponse,
            robot_id="undeclared_bot",
        )


async def test_call_agnostic_key_rejects_robot_id_kwarg():
    """agnostic 키에 robot_id= = 라우팅되는 척하는 거짓 코드 — fail-fast (§2.7)."""
    from modules.detector.contract import (
        DetectOrientedResponse,
        DetectRequest,
        Detector,
    )

    ctx = FakeContext(robots=[_BOT])
    with pytest.raises(TaskError, match="robot-agnostic"):
        await ctx.call(
            Detector.Service.DETECT_ORIENTED,
            DetectRequest(robot_id=_BOT, prompt="cube"),
            DetectOrientedResponse,
            robot_id=_BOT,
        )


async def test_spec_lookup_and_fail_fast():
    ctx = FakeContext(robots=[_BOT], specs={_BOT: _SPEC})
    assert ctx.spec(_BOT).gripper_open_raw == 3186
    with pytest.raises(TaskError, match="spec 없음"):
        ctx.spec("spec_없는_bot")


# ─── on_abort — 참여 robot 전원 STOP ─────────────────────────────────


async def test_on_abort_stops_all_participating_robots():
    """moved 추적 폐기 (2026-07-13) — 참여 선언한 robot 전원 STOP (보수적 안전)."""
    ctx = FakeContext(robots=["bot_a", "bot_b"])
    await ctx.on_abort()
    stops = ctx.calls(Motion.Service.STOP)
    assert [c["robot_id"] for c in stops] == ["bot_a", "bot_b"]
    assert ctx.aborted


async def test_on_abort_continues_after_one_stop_failure():
    """bot_a STOP 실패해도 bot_b 정지는 진행 (best-effort — 안전 경로 직렬 차단 금지)."""
    ctx = FakeContext(
        robots=["bot_a", "bot_b"],
        service_script={
            Motion.Service.STOP: [RemoteError("Boom", "bot_a 죽음")],
        },
    )
    await ctx.on_abort()  # raise 없이 완료
    stops = ctx.calls(Motion.Service.STOP)
    assert [c["robot_id"] for c in stops] == ["bot_a", "bot_b"]


# ─── @step — 게이트/trace/depth/실패 경로 ────────────────────────────


async def test_step_outside_run_is_plain_function():
    @step
    async def double(x: int) -> int:
        return x * 2

    assert await double(3) == 6  # 링크 없음 — 게이트/trace 없이 본문만


async def test_step_records_label_depth_and_title():
    link = _LinkStub()

    @step(title="자식 표시명")
    async def child() -> None: ...

    @step(title="부모 표시명")
    async def parent() -> None:
        await child()

    token = bind_link(link)
    try:
        await parent()
    finally:
        reset_link(token)

    # name = 식별자 (= 함수 이름, breakpoint/run_to 키), title = UI 표시 문구 — 분리
    assert [(e.name, e.title, e.depth, e.status) for e in link.entries] == [
        ("parent", "부모 표시명", 0, "completed"),
        ("child", "자식 표시명", 1, "completed"),
    ]


async def test_step_failure_marks_path_root_to_leaf():
    """안쪽 step 실패 → 그 entry + 바깥 entry 전부 failed (실패 경로 가시화)."""
    link = _LinkStub()

    @step
    async def inner() -> None:
        raise TaskError("안쪽 실패")

    @step
    async def outer() -> None:
        await inner()

    token = bind_link(link)
    try:
        with pytest.raises(TaskError, match="안쪽 실패"):
            await outer()
    finally:
        reset_link(token)

    assert [(e.name, e.depth, e.status) for e in link.entries] == [
        ("outer", 0, "failed"),
        ("inner", 1, "failed"),
    ]
    assert "안쪽 실패" in link.entries[0].detail  # 사유가 경로 전체에 보존


async def test_step_depth_restores_after_child():
    """자식 step 종료 후 depth 원복 — 형제 step 이 자식 depth 로 찍히면 회귀."""
    link = _LinkStub()

    @step
    async def child() -> None: ...

    @step
    async def parent() -> None:
        await child()
        await child()

    token = bind_link(link)
    try:
        await parent()
    finally:
        reset_link(token)

    assert [(e.name, e.depth) for e in link.entries] == [
        ("parent", 0), ("child", 1), ("child", 1),
    ]


def test_step_rejects_sync_function():
    with pytest.raises(TypeError, match="async"):
        step(lambda: None)  # type: ignore[arg-type]


# ─── FakeContext — 스크립트/실패 주입 ────────────────────────────────


async def test_fake_context_scripts_by_key_and_injects_failure():
    ctx = FakeContext(
        robots=[_BOT],
        service_script={
            Motion.Service.MOVE_L: [
                MoveLResponse(),
                RemoteError("MotionRejected", "IK 불가"),
            ],
        },
    )
    req = MoveLRequest(target=PoseTarget(kind="pose", position=(0.1, 0.0, 0.1)))
    await ctx.call(Motion.Service.MOVE_L, req, MoveLResponse, robot_id=_BOT)  # 성공
    with pytest.raises(RemoteError, match="IK 불가"):
        await ctx.call(  # 2차 — 실패 주입
            Motion.Service.MOVE_L, req, MoveLResponse, robot_id=_BOT
        )


async def test_fake_context_unscripted_key_asserts():
    ctx = FakeContext(robots=[_BOT])
    with pytest.raises(AssertionError, match="미등록"):
        await ctx.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target=PoseTarget(kind="pose", position=(0, 0, 0))),
            MoveLResponse,
            robot_id=_BOT,
        )


# (TaskMetadata/registry 는 2026-07-13 삭제 — task 의 정보 채널은 계약이 유일.)
