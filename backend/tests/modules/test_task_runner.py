"""TaskRunner 테스트 — 생명주기 + 디버거 게이트 + 콜백 통지 (wire 무지 계약).

runner 는 wire 를 모른다 — 콜백(on_state/on_trace) 캡처로 검증하고,
wire payload 조립(TaskState 발행 등)은 module 테스트(test_pick_and_place)가 검증.
도메인 없는 @step 함수로 하드웨어/motion 없이. 의미 (뒤집으면 회귀): SUCCESS
미도달 / cancel 이 in-flight await 못 끊음 / 실패가 침묵 / breakpoint 무시 /
활성 중 이중 start 허용 / 중단·실패 시 on_abort 미호출 / 중첩 step 의 depth·
게이트(step-into) 어긋남 / 콜백 미장착(headless) 실행 불가 / 콜백 예외가 run 을 죽임.
"""

from __future__ import annotations

import asyncio
from typing import Any

from framework.transport.protocol import RemoteError
from modules.motion.contract import Motion, MoveLRequest, MoveLResponse, PoseTarget
from modules.tasks.core.context import TaskContext
from modules.tasks.core.errors import DetectionNotFound
from modules.tasks.core.runner import RunState, TaskRunner
from modules.tasks.core.step import step
from modules.tasks.core.contract import TaskStatus, TraceEntry

_BOT = "test_bot_0"


class _WireStub:
    """ctx 의 서비스 call 스크립트 (키 → 응답 모델 또는 예외) — runner 는 안 씀."""

    def __init__(self) -> None:
        self.responses: dict[str, Any] = {}

    def publish(self, wire_key: str, event: Any) -> None:  # ctx 표면 충족용
        raise AssertionError(f"runner 경로에서 publish 발생: {wire_key}")

    async def call(self, key, req, res_cls, *, robot_id=None, timeout=None):  # noqa: ANN001, ANN201
        r = self.responses.get(str(key))
        if isinstance(r, Exception):
            raise r
        if r is None:
            raise AssertionError(f"call 스크립트 없음: {key}")
        return r


class _Obs:
    """runner 콜백 캡처 — 실 모듈이 발행 메서드를 달듯 테스트는 기록 메서드를 담."""

    def __init__(self) -> None:
        self.states: list[RunState] = []
        self.traces: list[list[TraceEntry]] = []

    def on_state(self, s: RunState) -> None:
        self.states.append(s)

    def on_trace(self, s: RunState, entries: list[TraceEntry]) -> None:
        self.traces.append(entries)


class _SpyCtx(TaskContext):
    """on_abort 호출 여부 기록 — Motion.STOP 위임 검증은 test_task_context 쪽."""

    def __init__(self, rt: Any) -> None:
        super().__init__(rt, {})
        self.abort_called = False

    async def on_abort(self) -> None:
        self.abort_called = True


def _make(
    rt: _WireStub | None = None,
) -> tuple[_WireStub, _Obs, TaskRunner, _SpyCtx]:
    rt = rt or _WireStub()
    obs = _Obs()
    runner = TaskRunner(on_state=obs.on_state, on_trace=obs.on_trace)
    return rt, obs, runner, _SpyCtx(rt)


def _last_state(obs: _Obs) -> RunState:
    assert obs.states, "STATE 통지 없음"
    return obs.states[-1]


def _last_trace(obs: _Obs) -> list[TraceEntry]:
    assert obs.traces, "TRACE 통지 없음"
    return obs.traces[-1]


async def _wait_status(obs: _Obs, status: TaskStatus) -> None:
    for _ in range(500):
        await asyncio.sleep(0)
        if obs.states and obs.states[-1].status == status:
            return
    raise AssertionError(f"status {status} 미도달 (last={_last_state(obs).status})")


# 도메인 없는 step 들 — runner 게이트/trace 검증용. name = 함수 이름이므로
# 검증 편의상 함수 이름 자체를 w0/w1/w2 로 (breakpoint 대상 문자열과 일치).
@step
async def w0() -> None: ...


@step
async def w1() -> None: ...


@step
async def w2() -> None: ...


async def _noop3(ctx: TaskContext) -> None:
    await w0()
    await w1()
    await w2()


async def test_success_flow_notifies_state_and_trace():
    _, obs, runner, ctx = _make()
    r = runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    assert r.accepted
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle

    assert _last_state(obs).status == TaskStatus.SUCCESS
    assert _last_state(obs).robot_ids == (_BOT,)  # 콜백이 라우팅에 쓸 스냅샷
    entries = _last_trace(obs)
    assert [e.name for e in entries] == ["w0", "w1", "w2"]
    assert all(e.status == "completed" for e in entries)
    assert all(e.depth == 0 for e in entries)  # 최상위 step
    assert not ctx.abort_called  # 정상 종료엔 Motion.STOP 불필요


async def test_start_rejected_while_active_and_after_finish_ok():
    rt, obs, runner, ctx = _make()
    release = asyncio.Event()

    async def blocked(c: TaskContext) -> None:
        await w0()
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
    _, obs, runner, ctx = _make()
    never = asyncio.Event()

    async def stuck(c: TaskContext) -> None:
        await w0()
        await never.wait()  # 60s MoveL 대기 등가 — cancel 만이 끊을 수 있음

    runner.start(stuck, ctx=ctx, robot_ids=[_BOT])
    for _ in range(50):
        await asyncio.sleep(0)  # step 하나 통과할 시간
    assert runner.cancel().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.STOPPED
    assert ctx.abort_called  # 중단 시 모션 정지 경로 호출


async def test_cancel_without_run_reports_reason():
    _, _, runner, _ = _make()
    r = runner.cancel()
    assert not r.ok and r.message  # 침묵 금지 — 사유 있어야


async def test_pause_holds_at_next_step_then_resume():
    _, obs, runner, ctx = _make()
    step_gate = asyncio.Event()

    async def slow(c: TaskContext) -> None:
        await w0()
        await step_gate.wait()
        await w1()

    runner.start(slow, ctx=ctx, robot_ids=[_BOT])
    for _ in range(50):
        await asyncio.sleep(0)
    assert runner.pause().ok
    step_gate.set()
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "w1"  # 다음 step 앞에서 hold

    assert runner.resume().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.SUCCESS


async def test_breakpoint_step_once_run_to():
    _, obs, runner, ctx = _make()
    runner.toggle_breakpoint("w0")  # run 전에 미리 — 보존돼야
    runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "w0"

    assert runner.step_once().ok  # w0 하나만 실행 후 w1 앞 hold
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "w1"
    done = [e.name for e in _last_trace(obs) if e.status == "completed"]
    assert done == ["w0"]

    assert runner.run_to("w2").ok  # w1 실행, w2 앞 hold
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "w2"

    assert runner.resume().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.SUCCESS


async def test_cancel_while_paused_stops_immediately():
    _, obs, runner, ctx = _make()
    runner.toggle_breakpoint("w1")
    runner.start(_noop3, ctx=ctx, robot_ids=[_BOT])
    await _wait_status(obs, TaskStatus.PAUSED)
    assert runner.cancel().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.STOPPED


async def test_task_error_becomes_failed_with_readable_reason():
    _, obs, runner, ctx = _make()

    async def failing(c: TaskContext) -> None:
        await w0()
        raise DetectionNotFound("white cube", candidates=0)

    runner.start(failing, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(obs)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "white cube" in final.error
    assert ctx.abort_called


async def test_step_remote_error_composes_failed_name():
    """step 안에서 터진 원격 실패 — 사유에 [name] 이 붙어 어디서 실패했는지 보임."""
    rt = _WireStub()
    rt.responses["srv/motion/{robot_id}/move_l"] = RemoteError(
        "MotionRejected", "IK 불가"
    )
    _, obs, runner, ctx = _make(rt)

    @step
    async def descend(c: TaskContext) -> None:
        await c.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target=PoseTarget(kind="pose", position=(0.1, 0.0, 0.1))),
            MoveLResponse,
            robot_id=_BOT,
        )

    async def move_fails(c: TaskContext) -> None:
        await descend(c)

    runner.start(move_fails, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(obs)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and final.error.startswith("[descend]")
    assert "IK 불가" in final.error  # 서비스 raise 사유 보존
    assert _last_trace(obs)[-1].status == "failed"


async def test_nested_steps_trace_depth_and_step_into():
    """중첩 step — trace 는 flat 리스트 + depth, 게이트는 모든 진입점 (step-into)."""
    _, obs, runner, ctx = _make()

    @step
    async def child_a() -> None: ...

    @step
    async def child_b() -> None: ...

    @step
    async def parent() -> None:
        await child_a()
        await child_b()

    async def scenario(c: TaskContext) -> None:
        await parent()
        await w2()

    runner.toggle_breakpoint("parent")
    runner.start(scenario, ctx=ctx, robot_ids=[_BOT])
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "parent"

    assert runner.step_once().ok  # parent 진입 → child_a 앞 hold (step-into)
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "child_a"

    assert runner.run_to("w2").ok  # 나머지 자식 통과, 형제 w2 앞 hold
    await _wait_status(obs, TaskStatus.PAUSED)
    assert _last_state(obs).current_name == "w2"

    assert runner.resume().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.SUCCESS

    assert [(e.name, e.depth) for e in _last_trace(obs)] == [
        ("parent", 0), ("child_a", 1), ("child_b", 1), ("w2", 0),
    ]
    assert all(e.status == "completed" for e in _last_trace(obs))


async def test_nested_step_failure_marks_whole_path():
    """자식 실패 → 자식/부모 entry 전부 failed + 사유는 가장 깊은 지점 name."""
    _, obs, runner, ctx = _make()

    @step
    async def leaf() -> None:
        raise DetectionNotFound("cube", candidates=0)

    @step
    async def trunk() -> None:
        await leaf()

    async def scenario(c: TaskContext) -> None:
        await trunk()

    runner.start(scenario, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle

    final = _last_state(obs)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and final.error.startswith("[leaf]")
    assert [(e.name, e.status) for e in _last_trace(obs)] == [
        ("trunk", "failed"), ("leaf", "failed"),
    ]


async def test_unexpected_exception_becomes_failed():
    _, obs, runner, ctx = _make()

    async def bug(c: TaskContext) -> None:
        raise KeyError("oops")

    runner.start(bug, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(obs)
    assert final.status == TaskStatus.FAILED and final.error


async def test_ctx_call_undeclared_robot_rejected():
    _, obs, runner, ctx = _make()

    async def wrong_robot(c: TaskContext) -> None:
        await c.call(
            Motion.Service.MOVE_L,
            MoveLRequest(target=PoseTarget(kind="pose", position=(0, 0, 0))),
            MoveLResponse,
            robot_id="other_bot",
        )

    runner.start(wrong_robot, ctx=ctx, robot_ids=[_BOT])
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    final = _last_state(obs)
    assert final.status == TaskStatus.FAILED
    assert final.error is not None and "other_bot" in final.error


async def test_toggle_breakpoint_without_run_says_next_run():
    _, _, runner, _ = _make()
    r = runner.toggle_breakpoint("w0")
    assert r.ok and "다음 실행" in r.message


async def test_headless_run_without_callbacks():
    """콜백 미장착 = headless 감독 — 발행/통지 없이도 실행·종료 정상 (wire 무지 계약)."""
    runner = TaskRunner()  # 콜백 없음
    ctx = _SpyCtx(_WireStub())
    assert runner.start(_noop3, ctx=ctx, robot_ids=[_BOT]).accepted
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert runner.status == TaskStatus.SUCCESS


async def test_parallel_branches_all_stop_pause_and_trace():
    """병렬 가지 (asyncio.gather — 협동 robot 동시 모션의 골격) all-stop 의미론.

    뒤집으면 회귀: pause 가 일부 가지만 잡음(다른 팔이 계속 움직임 — 위험) /
    가지별 depth 가 섞임 (ContextVar 상속 깨짐) / 병렬 trace 유실.
    """
    _, obs, runner, ctx = _make()
    left_gate = asyncio.Event()
    right_gate = asyncio.Event()

    @step
    async def left_child() -> None: ...

    @step
    async def left_parent() -> None:
        await left_child()  # 가지 안 중첩 — depth 가 가지별로 독립이어야

    @step
    async def left_b() -> None: ...

    @step
    async def right_a() -> None: ...

    @step
    async def right_b() -> None: ...

    async def scenario(c: TaskContext) -> None:
        async def left() -> None:
            await left_parent()
            await left_gate.wait()
            await left_b()

        async def right() -> None:
            await right_a()
            await right_gate.wait()
            await right_b()

        await asyncio.gather(left(), right())

    runner.start(scenario, ctx=ctx, robot_ids=[_BOT])
    # 두 가지가 첫 step 들을 끝내고 이벤트 대기 지점까지 진행
    for _ in range(200):
        await asyncio.sleep(0)
        done = {e.name for e in (obs.traces[-1] if obs.traces else []) if e.status == "completed"}
        if {"left_parent", "left_child", "right_a"} <= done:
            break

    assert runner.pause().ok  # all-stop — 이후 모든 가지가 다음 경계에서 hold
    left_gate.set()
    right_gate.set()
    await _wait_status(obs, TaskStatus.PAUSED)
    for _ in range(100):
        await asyncio.sleep(0)  # 양 가지가 경계에 도달할 시간
    # 둘 다 경계에서 멈췄어야 — left_b/right_b 는 아직 진입 못 함 (entry 없음)
    names = [e.name for e in _last_trace(obs)]
    assert "left_b" not in names and "right_b" not in names

    assert runner.resume().ok  # 게이트 하나로 전 가지 재개
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.SUCCESS

    # trace: 병렬 interleave 여도 전부 기록 + 가지별 depth 정확 (ContextVar 상속)
    by_name = {e.name: e for e in _last_trace(obs)}
    assert set(by_name) == {"left_parent", "left_child", "left_b", "right_a", "right_b"}
    assert all(e.status == "completed" for e in by_name.values())
    assert by_name["left_parent"].depth == 0
    assert by_name["left_child"].depth == 1  # left 가지의 중첩
    assert by_name["right_a"].depth == 0  # right 가지는 left 중첩에 안 물듦
    assert by_name["right_b"].depth == 0


async def test_parallel_branches_cancel_propagates():
    """cancel — gather 자식 가지 전부 취소 전파 → STOPPED + on_abort (전원 STOP)."""
    _, obs, runner, ctx = _make()
    never = asyncio.Event()

    async def scenario(c: TaskContext) -> None:
        async def branch() -> None:
            await w0()
            await never.wait()  # in-flight 모션 등가

        await asyncio.gather(branch(), branch())

    runner.start(scenario, ctx=ctx, robot_ids=[_BOT])
    for _ in range(50):
        await asyncio.sleep(0)
    assert runner.cancel().ok
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert _last_state(obs).status == TaskStatus.STOPPED
    assert ctx.abort_called  # 참여 robot 전원 STOP 경로


async def test_callback_failure_does_not_kill_run():
    """관측이 실행을 죽이면 안 됨 — 콜백 예외는 삼키고 run 은 SUCCESS."""

    def boom(*a: Any) -> None:
        raise RuntimeError("콜백 죽음")

    runner = TaskRunner(on_state=boom, on_trace=boom)
    ctx = _SpyCtx(_WireStub())
    assert runner.start(_noop3, ctx=ctx, robot_ids=[_BOT]).accepted
    assert runner._run is not None and runner._run.handle is not None
    await runner._run.handle
    assert runner.status == TaskStatus.SUCCESS
