"""TaskRunner — Step list async 순차 실행 + pause/resume/step/run_to/breakpoint 디버거.

옛 backend/modules/task/task_runner.py 를 v2 async 로 재구성 (§17.4):
  - threading.Thread/Event → asyncio.Task/Event. 단일 event loop 협조 스케줄링이라
    v1 의 _state_lock 불필요 (control 메서드와 _run_task 가 진짜 동시 실행 X).
  - step.execute → await. motion 완료는 step 내부 await (§17.3) — traj-event 제거.
  - _execute_one_step = 단일 step 실행 (디버거 게이트/status/publish). flat step list
    직접 실행. control flow(ForEach/Try) 재진입은 §17.1 Orchestration → defer.

runner = per-robot (TaskModule 이 robot_id 별 1개). STATE/STEP_RESULT 는 runtime.publish
(robot-scoped 키 = payload robot_id 라우팅, scan 동형). TREE 는 module 이 발행.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Callable

from .contract import (
    STEP_COMPLETED,
    STEP_FAILED,
    STEP_PENDING,
    STEP_RUNNING,
    Task,
    TaskState,
    TaskStatus,
    TaskStepResult,
)
from .schema import StepResult
from .step import Step, StepContext, TaskSpec, collect_step_ids

if TYPE_CHECKING:
    from framework.runtime.api import ModuleRuntime

    from .spec import TaskRobotSpec

logger = logging.getLogger(__name__)


class _StopRequested(Exception):
    """외부 stop() — _execute_one_step 이 raise → _run_task 가 STOPPED 로."""


class _StepFailed(Exception):
    """step 실행 중 일반 예외 wrap → _run_task 가 FAILED 로."""


class DebugMode(str, Enum):
    AUTO = "auto"  # 다음 breakpoint / 끝까지
    STEP_ONCE = "step"  # 1 step 후 pause
    RUN_TO = "run_to"  # target step 직전까지


@dataclass
class _RunnerState:
    status: TaskStatus = TaskStatus.IDLE
    task_name: str = ""
    current_step: int = 0
    total_steps: int = 0
    current_label: str = ""
    current_step_id: str = ""
    error: str | None = None
    step_statuses: dict[str, str] = field(default_factory=dict)


class TaskRunner:
    def __init__(
        self,
        runtime: "ModuleRuntime",
        robot_id: str,
        robot_spec: "TaskRobotSpec | None" = None,
        gripper_raw: Callable[[], int | None] | None = None,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self._state = _RunnerState()
        self._stop = asyncio.Event()
        self._gate = asyncio.Event()
        self._gate.set()  # set = 일시정지 아님
        self._mode = DebugMode.AUTO
        self._run_to_target: str | None = None
        self._breakpoints: set[str] = set()
        self._handle: asyncio.Task[None] | None = None
        self._ctx = StepContext(runtime, robot_id, robot_spec, gripper_raw)
        self._state_seq = 0
        self._result_seq = 0

    def is_running(self) -> bool:
        return self._state.status in (TaskStatus.RUNNING, TaskStatus.PAUSED)

    # ─── 외부 API (서비스 핸들러가 호출, 같은 event loop) ─────────────

    def run(self, spec: TaskSpec) -> bool:
        if self._state.status == TaskStatus.RUNNING:
            return False
        # nested 포함 전 step pending 초기화. breakpoint 는 보존 (미리 박을 수 있음).
        self._state = _RunnerState(
            step_statuses={sid: STEP_PENDING for sid in collect_step_ids(spec.steps)}
        )
        self._stop.clear()
        self._gate.set()
        self._mode = DebugMode.AUTO
        self._run_to_target = None
        self._handle = asyncio.create_task(self._run_task(spec))
        return True

    def stop(self) -> None:
        self._stop.set()
        self._gate.set()  # paused 였으면 깨워서 stop 전파

    def pause(self) -> bool:
        if self._state.status != TaskStatus.RUNNING:
            return False
        self._gate.clear()
        self._update(status=TaskStatus.PAUSED)
        return True

    def resume(self) -> bool:
        if self._state.status != TaskStatus.PAUSED:
            return False
        self._mode = DebugMode.AUTO
        self._run_to_target = None
        self._update(status=TaskStatus.RUNNING)
        self._gate.set()
        return True

    def step_once(self) -> bool:
        if self._state.status != TaskStatus.PAUSED:
            return False
        self._mode = DebugMode.STEP_ONCE
        self._run_to_target = None
        self._update(status=TaskStatus.RUNNING)
        self._gate.set()
        return True

    def run_to(self, target_step_id: str) -> bool:
        if self._state.status != TaskStatus.PAUSED:
            return False
        self._mode = DebugMode.RUN_TO
        self._run_to_target = target_step_id
        self._update(status=TaskStatus.RUNNING)
        self._gate.set()
        return True

    def toggle_breakpoint(self, step_id: str) -> bool:
        if step_id in self._breakpoints:
            self._breakpoints.discard(step_id)
        else:
            self._breakpoints.add(step_id)
        self._emit_state()  # breakpoints 변경 broadcast
        return True

    # ─── internal ────────────────────────────────────────────────────

    async def _run_task(self, spec: TaskSpec) -> None:
        self._ctx.results.clear()  # 이전 task Slot 결과 누출 방지
        self._update(
            status=TaskStatus.RUNNING,
            task_name=spec.name,
            current_step=0,
            total_steps=len(spec.steps),
            current_label="",
            current_step_id="",
            error=None,
        )
        try:
            for i, step in enumerate(spec.steps):
                self._update(current_step=i + 1)
                await self._execute_one_step(step)
        except _StopRequested:
            self._update(status=TaskStatus.STOPPED)
            return
        except _StepFailed as exc:
            self._update(status=TaskStatus.FAILED, error=str(exc))
            return
        self._update(
            status=TaskStatus.SUCCESS,
            current_step=len(spec.steps),
            current_label="",
            current_step_id="",
        )

    async def _execute_one_step(self, step: Step) -> object:
        """단일 step 실행 — 디버거 게이트 + status + step_result publish.

        _run_task 의 top-level 루프가 flat step 마다 호출. (control flow nested 재진입은
        §17.1 Orchestration → defer.)
        """
        if self._stop.is_set():
            raise _StopRequested()

        if self._should_pause_before(step):
            self._update(
                status=TaskStatus.PAUSED,
                current_label=step.label or step.type_name,
                current_step_id=step.id,
            )
            self._gate.clear()
        await self._gate.wait()

        if self._stop.is_set():
            raise _StopRequested()

        label = step.label or step.type_name
        self._update(
            status=TaskStatus.RUNNING, current_label=label, current_step_id=step.id
        )
        self._set_step_status(step.id, STEP_RUNNING)

        try:
            result = await step.execute(self._ctx)
        except (_StopRequested, _StepFailed):
            raise
        except Exception as exc:
            self._set_step_status(step.id, STEP_FAILED)
            logger.exception("step 실행 예외 [%s]", label)
            raise _StepFailed(f"[{label}] {type(exc).__name__}: {exc}") from exc

        self._ctx.store(step.id, result)
        self._emit_step_result(step, result)
        self._set_step_status(step.id, STEP_COMPLETED)
        return result

    def _should_pause_before(self, step: Step) -> bool:
        if self._mode == DebugMode.STEP_ONCE:
            return True
        if self._mode == DebugMode.RUN_TO and self._run_to_target == step.id:
            return True
        return step.id in self._breakpoints

    # ─── publish (robot-scoped 키 = payload robot_id 라우팅) ──────────

    def _update(self, **kwargs: object) -> None:
        for k, v in kwargs.items():
            setattr(self._state, k, v)
        self._emit_state()

    def _set_step_status(self, step_id: str, status: str) -> None:
        self._state.step_statuses[step_id] = status
        self._emit_state()

    def _emit_state(self) -> None:
        self.runtime.publish(
            Task.Stream.STATE,
            TaskState(
                robot_id=self.robot_id,
                seq=self._state_seq,
                timestamp_unix=time.time(),
                status=self._state.status,
                task_name=self._state.task_name,
                current_step=self._state.current_step,
                total_steps=self._state.total_steps,
                current_label=self._state.current_label,
                current_step_id=self._state.current_step_id,
                error=self._state.error,
                step_statuses=dict(self._state.step_statuses),
                breakpoints=sorted(self._breakpoints),
            ),
        )
        self._state_seq += 1

    def _emit_step_result(self, step: Step, value: object | None) -> None:
        sr = StepResult(
            step_id=step.id,
            type_name=type(value).__name__ if value is not None else "None",
            value=value,
        ).to_dict()
        self.runtime.publish(
            Task.Stream.STEP_RESULT,
            TaskStepResult(
                robot_id=self.robot_id,
                seq=self._result_seq,
                timestamp_unix=time.time(),
                step_id=sr["step_id"],
                type=sr["type"],
                value=sr["value"],
            ),
        )
        self._result_seq += 1
