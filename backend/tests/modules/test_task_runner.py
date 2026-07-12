"""TaskRunner 단위테스트 — 실행 생명주기 + 디버거 게이트 + exception filter.

실 TaskContext + fake runtime (wire stub) 로 runner e2e. 도메인 없는 primitive
(ctx.wait) 가 게이트를 타므로 하드웨어/motion 없이 검증 가능. 의미 (뒤집으면 회귀):
SUCCESS 미도달 / cancel 이 in-flight await 못 끊음 / 실패가 침묵 / breakpoint 무시 /
활성 중 이중 start 허용 / 중단·실패 시 on_abort 미호출.
"""

from __future__ import annotations

import asyncio
from enum import StrEnum
from typing import Any, cast

from pydantic import BaseModel

from framework.transport.protocol import RemoteError
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import DetectionNotFound
from modules.tasks.core.runner import TaskRunner
from modules.tasks.core.contract import TaskState, TaskStatus, TaskTrace

_BOT = "test_bot_0"


class _Streams(StrEnum):
    STATE = "stream/testtask/{robot_id}/state"
    TRACE = "stream/testtask/{robot_id}/trace"
    STEP_RESULT = "stream/testtask/{robot_id}/step_result"


class _WireStub:
    """publish 캡처 + call 스크립트 (키 → 응답 모델 또는 예외)."""

    def __init__(self) -> None:
        self.published: list[tuple[str, BaseModel]] = []
        self.responses: dict[str, Any] = {}

    def publish(self, wire_key: str, event: BaseModel) -> None:
        self.published.append((str(wire_key), event))

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=5.0):  # noqa: ANN001, ANN201
        r = self.responses.get(str(key))
        if isinstance(r, Exception):
            raise r
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


class _SpyCtx(TaskContext):
    """on_abort 호출 여부 기록 — Motion.STOP 위임 검증은 test_task_context 쪽."""

    def __init__(self, rt: Any) -> None:
        super().__init__(rt, {})
        self.abort_called = False

    async def on_abort(self) -> None:
        self.abort_called = True


def _make(rt: _WireStub | None = None) -> tuple[_WireStub, TaskRunner, _SpyCtx]:
    rt = rt or _WireStub()
    return rt, TaskRunner(rt, streams=_Streams), _SpyCtx(rt)


def _states(rt: _WireStub) -> list[TaskState]:
    return [cast(TaskState, e) for k, e in rt.published if k.endswith("/state")]


def _last_state(rt: _WireStub) -> TaskState:
    states = _states(rt)
    assert states, "STATE 미발행"
    return states[-1]


def _last_trace(rt: _WireStub) -> TaskTrace:
    traces = [cast(TaskTrace, e) for k, e in rt.published if k.endswith("/trace")]
    assert traces, "TRACE 미발행"
    return traces[-1]


async def _wait_status(rt: _WireStub, status: TaskStatus) -> None:
    for _ in range(500):
        await asyncio.sleep(0)
        if _states(rt) and _states(rt)[-1].status == status:
            return
    raise AssertionError(f"status {status} 미도달 (last={_last_state(rt).status})")


async def _noop3(ctx: TaskContext) -> None:
    await ctx.wait(0, label="w0")
    await ctx.wait(0, label="w1")
    await ctx.wait(0, label="w2")


async def test_success_flow_publishes_state_and_trace():
    rt, runner, ctx = _make()
    r = runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    assert r.accepted
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle

    assert _last_state(rt).status == TaskStatus.SUCCESS
    trace = _last_trace(rt)
    assert [e.label for e in trace.entries] == ["w0", "w1", "w2"]
    assert all(e.status == "completed" for e in trace.entries)
    assert not ctx.abort_called  # 정상 종료엔 Motion.STOP 불필요


async def test_start_rejected_while_active_and_after_finish_ok():
    rt, runner, ctx = _make()
    release = asyncio.Event()

    async def blocked(c: TaskContext) -> None:
        await c.wait(0, label="a")
        await release.wait()

    assert runner.start(blocked, ctx=ctx, robot_ids=[_BOT]).accepted
    dup = runner.start(_noop3, ctx=_SpyCtx(rt), robot_ids=[_BOT])
    assert not dup.accepted and "이미 실행 중" in dup.message

    release.set()
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert runner.start(_noop3, ctx=_SpyCtx(rt), robot_ids=[_BOT]).accepted


async def test_cancel_cuts_inflight_await_and_calls_on_abort():
    """옛 runner 결함 (step 경계에서만 stop 검사) 회귀 차단 — 비행 중 await 즉시 끊김."""
    rt, runner, ctx = _make()
    never = asyncio.Event()

    async def stuck(c: TaskContext) -> None:
        await c.wait(0, label="before")
        await never.wait()  # 60s MoveL 대기 등가 — cancel 만이 끊을 수 있음

    runner.start(stuck, ctx=ctx, robot_ids=[_BOT])
    for _ in range(50):
        await asyncio.sleep(0)  # primitive 하나 통과할 시간
    assert runner.cancel().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(rt).status == TaskStatus.STOPPED
    assert ctx.abort_called  # 중단 시 모션 정지 경로 호출


async def test_cancel_without_run_reports_reason():
    _, runner, _ = _make()
    r = runner.cancel()
    assert not r.ok and r.message  # 침묵 금지 — 사유 있어야


async def test_pause_holds_at_next_primitive_then_resume():
    rt, runner, ctx = _make()
    step_gate = asyncio.Event()

    async def slow(c: TaskContext) -> None:
        await c.wait(0, label="w0")
        await step_gate.wait()
        await c.wait(0, label="w1")

    runner.start(slow, ctx=ctx, robot_ids=[_BOT])
    for _ in range(50):
        await asyncio.sleep(0)
    assert runner.pause().ok
    step_gate.set()
    await _wait_status(rt, TaskStatus.PAUSED)
    assert _last_state(rt).current_label == "w1"  # 다음 primitive 앞에서 hold

    assert runner.resume().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(rt).status == TaskStatus.SUCCESS


async def test_breakpoint_step_once_run_to():
    rt, runner, ctx = _make()
    runner.toggle_breakpoint("w0")  # run 전에 미리 — 보존돼야
    runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    await _wait_status(rt, TaskStatus.PAUSED)
    assert _last_state(rt).current_label == "w0"

    assert runner.step_once().ok  # w0 하나만 실행 후 w1 앞 hold
    await _wait_status(rt, TaskStatus.PAUSED)
    assert _last_state(rt).current_label == "w1"
    done = [e.label for e in _last_trace(rt).entries if e.status == "completed"]
    assert done == ["w0"]

    assert runner.run_to("w2").ok  # w1 실행, w2 앞 hold
    await _wait_status(rt, TaskStatus.PAUSED)
    assert _last_state(rt).current_label == "w2"

    assert runner.resume().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(rt).status == TaskStatus.SUCCESS


async def test_cancel_while_paused_stops_immediately():
    rt, runner, ctx = _make()
    runner.toggle_breakpoint("w1")
    runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    await _wait_status(rt, TaskStatus.PAUSED)
    assert runner.cancel().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(rt).status == TaskStatus.STOPPED


async def test_task_error_becomes_failed_with_readable_reason():
    rt, runner, ctx = _make()

    async def failing(c: TaskContext) -> None:
        await c.wait(0, label="ok")
        raise DetectionNotFound("white cube", candidates=0)

    runner.start(failing, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(rt)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "white cube" in final.error
    assert ctx.abort_called


async def test_primitive_remote_error_composes_failed_label():
    """primitive 안에서 터진 실패 — 사유에 [label] 이 붙어 어디서 실패했는지 보임."""
    rt = _WireStub()
    rt.responses["srv/motion/{robot_id}/move_l"] = RemoteError("IkError", "IK 불가")
    _, runner, ctx = _make(rt)

    async def move_fails(c: TaskContext) -> None:
        await c.robot(_BOT).move_l((0.1, 0.0, 0.1), label="descend")

    runner.start(move_fails, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(rt)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and final.error.startswith("[descend]")
    trace = _last_trace(rt)
    assert trace.entries[-1].status == "failed"


async def test_unexpected_exception_becomes_failed():
    rt, runner, ctx = _make()

    async def bug(c: TaskContext) -> None:
        raise KeyError("oops")

    runner.start(bug, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(rt)
    assert final.status == TaskStatus.FAILED and final.error


async def test_ctx_robot_not_declared_rejected():
    rt, runner, ctx = _make()

    async def wrong_robot(c: TaskContext) -> None:
        c.robot("other_bot")

    runner.start(wrong_robot, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(rt)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "other_bot" in final.error


async def test_toggle_breakpoint_without_run_says_next_run():
    _, runner, _ = _make()
    r = runner.toggle_breakpoint("w0")
    assert r.ok and "다음 실행" in r.message
