"""TaskRunner — 시나리오 실행 생명주기 (task 모듈이 소유하는 부품).

책임 (이것만): start/cancel/pause/resume/step_once/run_to/toggle_breakpoint,
primitive 진입 게이트, 예외 → FAILED + 사유 조립, STATE/TRACE/STEP_RESULT 발행.
robot 에 대해 아는 것은 **id 문자열뿐** (스트림 {robot_id} 라우팅용 불투명 값) —
robot spec / Motion.STOP 등 도메인 접근은 전부 TaskContext (책임 분리, 2026-07-12).

실행 규약:
  - start = fire-and-monitor: asyncio.create_task 감독 시작 후 accepted 즉시 반환.
    진행/종료는 STATE/TRACE 스트림 관찰. 동시 1 run (활성 중 start 거부).
  - cancel = task.cancel() — in-flight primitive await (60s MoveL 등) 즉시 끊김
    (옛 DSL runner 의 "step 경계에서만 stop 검사" 결함 해소). 중단/실패 시
    finally 에서 ctx.on_abort() → 실제 모션 보낸 robot 에만 Motion.STOP.
  - 시나리오 정상 반환 = SUCCESS / TaskError = FAILED + 읽을 수 있는 사유 /
    그 외 예외 = FAILED + 로그 (버그 유형).
  - pause/step_once/run_to/breakpoint = primitive 진입 게이트 (label 기준).
    모션 비행 중 급정지는 cancel 의 몫 (게이트는 경계에서만).

트리거 중립: RUN 서비스는 어댑터 하나일 뿐 — @subscriber 콜백/내부 모듈 호출
어디서든 start() 를 부를 수 있다.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Awaitable, Callable, Protocol

from framework.runtime.api import ModuleRuntime
from framework.transport.protocol import RemoteError

from .errors import TaskError
from .contract import (
    TRACE_COMPLETED,
    TRACE_FAILED,
    TRACE_RUNNING,
    TaskState,
    TaskStatus,
    TaskStepResult,
    TaskTrace,
    TraceEntry,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class StartResult:
    accepted: bool
    message: str = ""


@dataclass(frozen=True)
class ControlResult:
    ok: bool
    message: str = ""


class RunContext(Protocol):
    """runner 가 ctx 에 요구하는 최소 표면 — 도메인은 안 봄 (TaskContext 가 구현)."""

    def bind_run(self, link: Any, robot_ids: list[str]) -> None:
        ...

    async def on_abort(self) -> None:
        ...


class _DebugMode(Enum):
    AUTO = "auto"
    STEP_ONCE = "step_once"  # 다음 primitive 앞에서 hold
    RUN_TO = "run_to"  # target label 앞에서 hold


@dataclass
class _Run:
    ctx: RunContext
    robot_ids: list[str]
    task_name: str
    status: TaskStatus = TaskStatus.RUNNING
    error: str | None = None
    current_label: str = ""
    trace: list[TraceEntry] = field(default_factory=list)
    kind_counts: dict[str, int] = field(default_factory=dict)
    gate: asyncio.Event = field(default_factory=asyncio.Event)
    mode: _DebugMode = _DebugMode.AUTO
    run_to_target: str | None = None
    handle: asyncio.Task[None] | None = None

    @property
    def active(self) -> bool:
        return self.status in (TaskStatus.RUNNING, TaskStatus.PAUSED)


class TaskRunner:
    """task 모듈당 1개 보유. streams = 그 모듈 contract 의 Stream enum
    (STATE/TRACE/STEP_RESULT 멤버 필수 — 규약 payload 는 core/wire.py)."""

    def __init__(self, runtime: ModuleRuntime, *, streams: Any) -> None:
        self._runtime = runtime
        try:
            self._state_key = str(streams.STATE)
            self._trace_key = str(streams.TRACE)
            self._result_key = str(streams.STEP_RESULT)
        except AttributeError as exc:  # fail-fast: 규약 키 누락
            raise TypeError(
                "TaskRunner streams 에 STATE/TRACE/STEP_RESULT 멤버 필요 "
                f"(got {streams!r})"
            ) from exc
        self._run: _Run | None = None
        self._breakpoints: set[str] = set()  # label — run 간 보존 (미리 박기 허용)
        self._seq = {"state": 0, "trace": 0, "result": 0}

    # ─── 시작/중단 (트리거 어댑터들이 호출) ──────────────────────────

    def start(
        self,
        fn: Callable[..., Awaitable[None]],
        *,
        ctx: RunContext,
        robot_ids: list[str],
        task_name: str = "",
        **kwargs: Any,
    ) -> StartResult:
        """시나리오 감독 실행 시작 — accepted 즉시 반환 (fire-and-monitor).

        robot_ids = 참여 robot (스트림 라우팅 + ctx.robot() 허용 목록). kwargs 는
        시나리오 함수 인자로 그대로 전달 (typed req → kwargs 매핑은 호출부 몫).
        """
        if self._run is not None and self._run.active:
            return StartResult(False, f"이미 실행 중 ({self._run.task_name})")
        if not robot_ids:
            return StartResult(False, "robot_ids 필요 (스트림 라우팅 대상)")

        run = _Run(
            ctx=ctx,
            robot_ids=list(robot_ids),
            task_name=task_name or getattr(fn, "__name__", "task").lstrip("_"),
        )
        run.gate.set()  # set = 진행, clear = hold
        ctx.bind_run(_Link(self, run), list(robot_ids))
        self._run = run
        self._publish_state(run)  # RUNNING 진입
        self._publish_trace(run)  # 빈 trace — 이전 run 표시 clear
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
        run.status = TaskStatus.PAUSED  # 다음 primitive 경계에서 hold
        self._publish_state(run)
        return ControlResult(True)

    def resume(self) -> ControlResult:
        return self._release(_DebugMode.AUTO, None)

    def step_once(self) -> ControlResult:
        """한 primitive 만 진행 후 다시 hold."""
        return self._release(_DebugMode.STEP_ONCE, None)

    def run_to(self, label: str) -> ControlResult:
        """label 직전까지 진행 후 hold (run to cursor)."""
        return self._release(_DebugMode.RUN_TO, label)

    def toggle_breakpoint(self, label: str) -> ControlResult:
        if label in self._breakpoints:
            self._breakpoints.discard(label)
        else:
            self._breakpoints.add(label)
        run = self._run
        if run is not None and run.active:
            self._publish_state(run)  # breakpoints 변경 broadcast
            return ControlResult(True)
        return ControlResult(True, "다음 실행부터 적용")

    def _release(self, mode: _DebugMode, target: str | None) -> ControlResult:
        run = self._run
        if run is None or run.status != TaskStatus.PAUSED:
            return ControlResult(False, "일시정지 상태 아님")
        run.mode = mode
        run.run_to_target = target
        run.status = TaskStatus.RUNNING
        self._publish_state(run)
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
        try:
            await fn(run.ctx, **kwargs)
            run.status = TaskStatus.SUCCESS
            run.current_label = ""
        except asyncio.CancelledError:
            # cancel() 에 의한 중단 — 감독 태스크가 최종 소비 (재raise 안 함).
            run.status = TaskStatus.STOPPED
        except TaskError as exc:
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.warning("task '%s' 실패: %s", run.task_name, run.error)
        except RemoteError as exc:
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.warning("task '%s' 원격 실패: %s", run.task_name, run.error)
        except Exception as exc:
            # 도메인 실패가 아닌 버그 유형 — 사유는 남기되 stacktrace 로그.
            run.status = TaskStatus.FAILED
            run.error = self._compose_error(run, exc)
            logger.exception("task '%s' 예외 (버그 의심)", run.task_name)
        finally:
            if run.status in (TaskStatus.STOPPED, TaskStatus.FAILED):
                # 모션 비행 중 중단 가능 — 움직인 robot 정지 (ctx 가 대상을 앎).
                try:
                    await run.ctx.on_abort()
                except Exception:
                    logger.exception("ctx.on_abort 실패 — shutdown 계속")
            self._publish_state(run)

    @staticmethod
    def _compose_error(run: _Run, exc: BaseException) -> str:
        msg = str(exc) or type(exc).__name__
        failed = next(
            (e for e in reversed(run.trace) if e.status == TRACE_FAILED), None
        )
        if failed is not None:
            return f"[{failed.label}] {msg}"
        return msg

    # ─── 게이트 + trace (_Link 가 위임) ──────────────────────────────

    async def _enter_primitive(self, run: _Run, kind: str, label: str) -> TraceEntry:
        n = run.kind_counts.get(kind, 0) + 1
        run.kind_counts[kind] = n
        label = label or f"{kind}#{n}"

        # hold 조건: 게이트 판정 (step_once/run_to/breakpoint) 또는 pause() 가
        # 이미 게이트를 내린 상태 (다음 경계에서 hold — 모션 중 급정지는 cancel).
        if self._should_hold(run, label) or not run.gate.is_set():
            run.mode = _DebugMode.AUTO
            run.run_to_target = None
            run.status = TaskStatus.PAUSED
            run.current_label = label
            self._publish_state(run)
            run.gate.clear()
        await run.gate.wait()

        run.current_label = label
        entry = TraceEntry(
            label=label, kind=kind, status=TRACE_RUNNING, started_unix=time.time()
        )
        run.trace.append(entry)
        self._publish_state(run)
        self._publish_trace(run)
        return entry

    def _should_hold(self, run: _Run, label: str) -> bool:
        if run.mode is _DebugMode.STEP_ONCE:
            return True
        if run.mode is _DebugMode.RUN_TO and run.run_to_target == label:
            return True
        return label in self._breakpoints

    def _finish_entry(
        self, run: _Run, entry: TraceEntry, status: str, detail: str
    ) -> None:
        entry.status = status
        entry.detail = detail
        entry.ended_unix = time.time()
        self._publish_trace(run)

    # ─── publish (robot-scoped 키 = payload robot_id 라우팅) ─────────

    def _publish_state(self, run: _Run) -> None:
        for robot_id in run.robot_ids:
            self._runtime.publish(
                self._state_key,
                TaskState(
                    robot_id=robot_id,
                    seq=self._next_seq("state"),
                    timestamp_unix=time.time(),
                    status=run.status,
                    task_name=run.task_name,
                    current_label=run.current_label,
                    error=run.error,
                    breakpoints=sorted(self._breakpoints),
                ),
            )

    def _publish_trace(self, run: _Run) -> None:
        for robot_id in run.robot_ids:
            self._runtime.publish(
                self._trace_key,
                TaskTrace(
                    robot_id=robot_id,
                    seq=self._next_seq("trace"),
                    timestamp_unix=time.time(),
                    task_name=run.task_name,
                    entries=list(run.trace),
                ),
            )

    def _publish_result(
        self, run: _Run, label: str, type_name: str, value: Any
    ) -> None:
        for robot_id in run.robot_ids:
            self._runtime.publish(
                self._result_key,
                TaskStepResult(
                    robot_id=robot_id,
                    seq=self._next_seq("result"),
                    timestamp_unix=time.time(),
                    label=label,
                    type=type_name,
                    value=value,
                ),
            )

    def _next_seq(self, stream: str) -> int:
        seq = self._seq[stream]
        self._seq[stream] = seq + 1
        return seq


class _Link:
    """ctx 에 주입되는 게이트/관측 훅 — RunLink (context.py Protocol) 구현."""

    def __init__(self, runner: TaskRunner, run: _Run) -> None:
        self._runner = runner
        self._run = run

    async def enter(self, kind: str, label: str) -> TraceEntry:
        return await self._runner._enter_primitive(self._run, kind, label)

    def complete(self, entry: TraceEntry, detail: str = "") -> None:
        self._runner._finish_entry(self._run, entry, TRACE_COMPLETED, detail)

    def fail(self, entry: TraceEntry, detail: str) -> None:
        self._runner._finish_entry(self._run, entry, TRACE_FAILED, detail)

    def emit_result(self, label: str, type_name: str, value: Any) -> None:
        self._runner._publish_result(self._run, label, type_name, value)
