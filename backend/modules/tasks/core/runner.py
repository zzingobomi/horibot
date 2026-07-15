"""TaskRunner — 범용 long-running 시나리오 감독기 (wire 무지, 2026-07-13 확정).

두 클래스 (Mirror/MirrorState 와 같은 분리 — 2026-07-14 데코레이터 DX 통일):
  - TaskRunner  = 클래스 바디 **선언부** (descriptor). @task.on_state /
    @task.on_trace 로 훅을 @service/@publishes/@mirror.on_change 와 같은
    데코레이터 리듬으로 연결한다. 선언만 가지므로 인스턴스 간 공유돼도 무해.
  - TaskRunnerState = 인스턴스별 **감독기 본체** (_run/_breakpoints/게이트 —
    실행 상태). descriptor __get__ 이 인스턴스 __dict__ 에 lazy 생성. 모듈
    인스턴스 둘이 run/breakpoint 를 공유하면 "이미 실행 중" 오거부/상태 오염 —
    Mirror 가 상태를 MirrorState 로 쪼갠 바로 그 이유.

책임 (이것만): start/cancel/pause/resume/step_once/run_to/toggle_breakpoint,
@step 진입 게이트, 예외 → FAILED + 사유 조립, 상태/trace **데이터** 추적.

**wire 를 모른다** — runtime/zenoh/키/robot/계약 무지. 변화가 생기면 모듈이 달아둔
**훅**(on_state/on_trace — 전부 선택)을 부를 뿐이다. publish 할지,
어떤 키/payload 로 할지는 훅을 단 모듈의 코드 — 필요 없으면 안 달면 된다
(headless). robot_ids 는 불투명 문자열 목록으로만 통과 (ctx 허용 목록 + 훅에
스냅샷으로 전달). 훅 해석은 이름 지연 bind (getattr — MRO 를 타므로 서브클래스
override 가 이긴다) — descriptor 로 바뀌어도 본체는 여전히 함수만 받는 순수 부품.

실행 규약:
  - start = fire-and-monitor: asyncio.create_task 감독 시작 후 accepted 즉시 반환.
    아무 async 함수나 받는다 (트리거 중립 — 서비스 핸들러/@subscriber/내부 호출
    어디서든 호출 가능). 동시 1 run (활성 중 start 거부).
  - cancel = task.cancel() — in-flight await (60s MoveL 등) 즉시 끊김.
    중단/실패 시 finally 에서 ctx.on_abort() → 참여 robot 전원 Motion.STOP.
  - 시나리오 정상 반환 = SUCCESS / TaskError = FAILED + 읽을 수 있는 사유 /
    RemoteError = FAILED (서비스가 raise 한 기술적 실패 — MotionRejected 등) /
    그 외 예외 = FAILED + 로그 (버그 유형).
  - pause/step_once/run_to/breakpoint = @step 진입 게이트 (step name 기준).
    게이트는 모든 depth 의 진입점 (step_once = step-into). 비행 중 급정지는 cancel.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Protocol, TypeVar, overload

from framework.transport.protocol import RemoteError

from .errors import TaskError
from .contract import (
    TRACE_COMPLETED,
    TRACE_FAILED,
    TRACE_RUNNING,
    TaskStatus,
    TraceEntry,
)
from .step import bind_link, reset_link

logger = logging.getLogger(__name__)

_F = TypeVar("_F", bound=Callable[..., Any])


@dataclass(frozen=True)
class StartResult:
    accepted: bool
    message: str = ""


@dataclass(frozen=True)
class ControlResult:
    ok: bool
    message: str = ""


@dataclass(frozen=True)
class RunState:
    """observer 에 전달되는 실행 상태 스냅샷 (wire payload 아님 — 순수 데이터)."""

    task_name: str
    robot_ids: tuple[str, ...]
    status: TaskStatus
    current_name: str = ""
    current_title: str = ""
    error: str | None = None
    breakpoints: tuple[str, ...] = ()


class RunContext(Protocol):
    """runner 가 ctx 에 요구하는 최소 표면 — 도메인은 안 봄 (TaskContext 가 구현)."""

    def bind_run(self, robot_ids: list[str]) -> None: ...

    async def on_abort(self) -> None: ...


class _DebugMode(Enum):
    AUTO = "auto"
    STEP_ONCE = "step_once"  # 다음 step 앞에서 hold
    RUN_TO = "run_to"  # target step name 앞에서 hold


@dataclass
class _Run:
    ctx: RunContext
    robot_ids: list[str]
    task_name: str
    status: TaskStatus = TaskStatus.RUNNING
    error: str | None = None
    current_name: str = ""
    current_title: str = ""
    trace: list[TraceEntry] = field(default_factory=list)
    gate: asyncio.Event = field(default_factory=asyncio.Event)
    mode: _DebugMode = _DebugMode.AUTO
    run_to_target: str | None = None
    handle: asyncio.Task[None] | None = None

    @property
    def active(self) -> bool:
        return self.status in (TaskStatus.RUNNING, TaskStatus.PAUSED)


class TaskRunnerState:
    """
    TaskRunner의 실제 런타임 구현체.

    task 실행 상태와 생명주기를 관리하며, 실행·중단·일시정지·재개 및
    디버깅 기능을 담당한다.
    """

    def __init__(
        self,
        *,
        on_state: Callable[[RunState], None] | None = None,
        on_trace: Callable[[RunState, list[TraceEntry]], None] | None = None,
    ) -> None:
        self._on_state = on_state
        self._on_trace = on_trace
        self._run: _Run | None = None
        self._breakpoints: set[str] = set()

    def start(
        self,
        fn: Callable[..., Awaitable[None]],
        *,
        ctx: RunContext,
        robot_ids: list[str],
        task_name: str = "",
        **kwargs: Any,
    ) -> StartResult:
        if self._run is not None and self._run.active:
            return StartResult(False, f"이미 실행 중 ({self._run.task_name})")
        if not robot_ids:
            return StartResult(False, "robot_ids 필요 (참여 robot)")

        run = _Run(
            ctx=ctx,
            robot_ids=list(robot_ids),
            task_name=task_name or getattr(fn, "__name__", "task").lstrip("_"),
        )
        run.gate.set()  # set = 진행, clear = hold
        ctx.bind_run(list(robot_ids))
        self._run = run
        self._notify_state(run)  # RUNNING 진입
        self._notify_trace(run)  # 빈 trace — 이전 run 표시 clear
        run.handle = asyncio.create_task(self._supervise(run, fn, kwargs))
        return StartResult(True)

    def cancel(self) -> ControlResult:
        """활성 run 취소 — in-flight await 즉시 끊김. pause 상태에서도 즉시."""
        run = self._run
        if run is None or not run.active or run.handle is None:
            return ControlResult(False, "실행 중인 run 없음")
        run.handle.cancel()
        return ControlResult(True)

    # ─── 디버거 게이트 조작 ──────────────────────────────────────────

    def pause(self) -> ControlResult:
        run = self._run
        if run is None or run.status != TaskStatus.RUNNING:
            return ControlResult(False, "실행 중인 run 없음")
        run.mode = _DebugMode.AUTO
        run.gate.clear()
        run.status = TaskStatus.PAUSED  # 다음 step 경계에서 hold
        self._notify_state(run)
        return ControlResult(True)

    def resume(self) -> ControlResult:
        return self._release(_DebugMode.AUTO, None)

    def step_once(self) -> ControlResult:
        """다음 step 진입점 하나만 통과 후 다시 hold (step-into 의미론)."""
        return self._release(_DebugMode.STEP_ONCE, None)

    def run_to(self, name: str) -> ControlResult:
        """name(step 식별자) 직전까지 진행 후 hold (run to cursor)."""
        return self._release(_DebugMode.RUN_TO, name)

    def toggle_breakpoint(self, name: str) -> ControlResult:
        if name in self._breakpoints:
            self._breakpoints.discard(name)
        else:
            self._breakpoints.add(name)
        run = self._run
        # 항상 통지 — run 밖 토글(프리뷰에서 미리 박기)도 UI 에 보여야 한다
        # (침묵 금지). run 이 끝난 뒤면 최종 status 스냅샷 유지, run 자체가
        # 없으면 IDLE 스냅샷 (robot_ids 없음 — 라우팅은 훅을 단 모듈 몫).
        self._notify_state(run)
        if run is not None and run.active:
            return ControlResult(True)
        return ControlResult(True, "다음 실행부터 적용")

    def _release(self, mode: _DebugMode, target: str | None) -> ControlResult:
        run = self._run
        if run is None or run.status != TaskStatus.PAUSED:
            return ControlResult(False, "일시정지 상태 아님")
        run.mode = mode
        run.run_to_target = target
        run.status = TaskStatus.RUNNING
        self._notify_state(run)
        run.gate.set()
        return ControlResult(True)

    # ─── 상태 조회 (모듈 커스텀 로직용) ──────────────────────────────

    @property
    def status(self) -> TaskStatus:
        return self._run.status if self._run is not None else TaskStatus.IDLE

    # ─── 감독 (exception filter) ─────────────────────────────────────

    async def _supervise(
        self, run: _Run, fn: Callable[..., Awaitable[None]], kwargs: dict[str, Any]
    ) -> None:
        # @step wrapper 가 이 링크를 ContextVar 로 본다 — 감독 task 안에서 bind
        # 하므로 시나리오(fn)와 그 자식 task 들에 상속된다.
        token = bind_link(_Link(self, run))
        try:
            await fn(run.ctx, **kwargs)
            run.status = TaskStatus.SUCCESS
            run.current_name = ""
            run.current_title = ""
        except asyncio.CancelledError:
            # cancel() 에 의한 중단 — 감독 태스크가 최종 소비 (재raise 안 함).
            run.status = TaskStatus.STOPPED
        except TaskError as exc:
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.warning("task '%s' 실패: %s", run.task_name, run.error)
        except RemoteError as exc:
            # 서비스가 raise 한 기술적 실패 (MotionRejected/MotionFailed/...) —
            # type 이름 + 사유가 그대로 사용자 표시 문장이 된다.
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.warning("task '%s' 원격 실패: %s", run.task_name, run.error)
        except Exception as exc:
            # 도메인 실패가 아닌 버그 유형 — 사유는 남기되 stacktrace 로그.
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.exception("task '%s' 예외 (버그 의심)", run.task_name)
        finally:
            reset_link(token)
            if run.status in (TaskStatus.STOPPED, TaskStatus.FAILED):
                # 모션 비행 중 중단 가능 — 참여 robot 전원 정지 (ctx 가 수행).
                try:
                    await run.ctx.on_abort()
                except Exception:
                    logger.exception("ctx.on_abort 실패 — shutdown 계속")
            self._notify_state(run)

    @staticmethod
    def _compose_error(run: _Run, exc: BaseException) -> str:
        msg = str(exc) or type(exc).__name__
        # 가장 안쪽(깊은) 실패 entry = 실제 실패 지점 — 사유 접두에 박는다.
        failed = [e for e in run.trace if e.status == TRACE_FAILED]
        if failed:
            deepest = max(failed, key=lambda e: e.depth)
            return f"[{deepest.name}] {msg}"
        return msg

    # ─── 게이트 + trace (_Link 가 위임) ──────────────────────────────

    async def _enter_step(
        self, run: _Run, name: str, depth: int, title: str = ""
    ) -> TraceEntry:
        # hold 조건: 게이트 판정 (step_once/run_to/breakpoint) 또는 pause() 가
        # 이미 게이트를 내린 상태. 모든 depth 의 진입점에서 판정 (step-into).
        if self._should_hold(run, name) or not run.gate.is_set():
            run.mode = _DebugMode.AUTO
            run.run_to_target = None
            run.status = TaskStatus.PAUSED
            run.current_name = name
            run.current_title = title
            self._notify_state(run)
            run.gate.clear()
        await run.gate.wait()

        run.current_name = name
        run.current_title = title
        entry = TraceEntry(
            name=name,
            title=title,
            depth=depth,
            status=TRACE_RUNNING,
            started_unix=time.time(),
        )
        run.trace.append(entry)
        self._notify_state(run)
        self._notify_trace(run)
        return entry

    def _should_hold(self, run: _Run, name: str) -> bool:
        if run.mode is _DebugMode.STEP_ONCE:
            return True
        if run.mode is _DebugMode.RUN_TO and run.run_to_target == name:
            return True
        return name in self._breakpoints

    def _finish_entry(
        self, run: _Run, entry: TraceEntry, status: str, detail: str
    ) -> None:
        entry.status = status
        if detail:  # 실패 사유(fail)만 채운다 — complete 는 빈 값
            entry.detail = detail
        entry.ended_unix = time.time()
        self._notify_trace(run)

    # ─── 훅 통지 (wire 무관 — 발행 여부/방법은 훅을 단 모듈 몫) ──────

    def _snapshot(self, run: _Run | None) -> RunState:
        if run is None:  # run 밖 통지 (idle breakpoint 토글) — 상태는 IDLE
            return RunState(
                task_name="",
                robot_ids=(),
                status=TaskStatus.IDLE,
                breakpoints=tuple(sorted(self._breakpoints)),
            )
        return RunState(
            task_name=run.task_name,
            robot_ids=tuple(run.robot_ids),
            status=run.status,
            current_name=run.current_name,
            current_title=run.current_title,
            error=run.error,
            breakpoints=tuple(sorted(self._breakpoints)),
        )

    def _notify_state(self, run: _Run | None) -> None:
        if self._on_state is None:
            return
        try:
            self._on_state(self._snapshot(run))
        except Exception:
            logger.exception("on_state 훅 실패 — 실행 계속")

    def _notify_trace(self, run: _Run) -> None:
        if self._on_trace is None:
            return
        try:
            self._on_trace(self._snapshot(run), list(run.trace))
        except Exception:
            logger.exception("on_trace 훅 실패 — 실행 계속")


class TaskRunner:
    """
    TaskRunnerState를 인스턴스별로 제공하는 descriptor.

    선언 정보만 보관하며, 실행 상태는 공유되지 않도록 TaskRunnerState에
    분리한다.
    """

    def __init__(self) -> None:
        self._attr_name: str | None = None
        self._on_state_name: str | None = None
        self._on_trace_name: str | None = None

    def __set_name__(self, owner: type, name: str) -> None:
        self._attr_name = name

    # ─── 훅 선언 (클래스 바디 — 이름만 저장, bind 는 __get__ 에서) ────

    def on_state(self, fn: _F) -> _F:
        self._on_state_name = fn.__name__
        return fn

    def on_trace(self, fn: _F) -> _F:
        self._on_trace_name = fn.__name__
        return fn

    # ─── 인스턴스별 상태 (Mirror.__get__ 동형 — lazy, __dict__ 격리) ──

    @overload
    def __get__(self, instance: None, owner: type) -> TaskRunner: ...

    @overload
    def __get__(self, instance: Any, owner: type | None = ...) -> TaskRunnerState: ...

    def __get__(
        self, instance: Any, owner: type | None = None
    ) -> TaskRunnerState | TaskRunner:
        if instance is None:
            return self
        if self._attr_name is None:
            raise RuntimeError(
                f"TaskRunner descriptor missing __set_name__ "
                f"(owner={owner}, instance={type(instance).__name__})"
            )
        state_key = f"_taskrunner_{self._attr_name}"
        state = instance.__dict__.get(state_key)
        if state is None:
            state = TaskRunnerState(
                on_state=(
                    getattr(instance, self._on_state_name)
                    if self._on_state_name
                    else None
                ),
                on_trace=(
                    getattr(instance, self._on_trace_name)
                    if self._on_trace_name
                    else None
                ),
            )
            instance.__dict__[state_key] = state
        return state


class _Link:
    """@step wrapper 가 ContextVar 로 보는 게이트/관측 훅 — RunLink (step.py) 구현.

    _supervise 가 bind_link 로 심고, @step wrapper 가 enter/complete/fail 을 호출.
    (ctx 로는 주입하지 않는다 — ctx 는 참여 robot 만 받는다, 2026-07-15 정리.)"""

    def __init__(self, runner: TaskRunnerState, run: _Run) -> None:
        self._runner = runner
        self._run = run

    async def enter(self, name: str, depth: int, title: str = "") -> TraceEntry:
        return await self._runner._enter_step(self._run, name, depth, title)

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        self._runner._finish_entry(self._run, entry, TRACE_COMPLETED, detail)

    def fail(self, entry: TraceEntry, detail: str) -> None:
        self._runner._finish_entry(self._run, entry, TRACE_FAILED, detail)
