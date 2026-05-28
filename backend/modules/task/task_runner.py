from dataclasses import dataclass, field
from enum import Enum
import threading
from typing import TYPE_CHECKING, Callable

from .step_types import Task, TaskContext

if TYPE_CHECKING:
    from .step_executor import StepExecutor


class TaskStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCESS = "success"
    FAILED = "failed"
    STOPPED = "stopped"


# Step 별 실행 상태 — TaskState.step_statuses 에 step_id → 이 값으로 보관.
STEP_PENDING = "pending"
STEP_RUNNING = "running"
STEP_COMPLETED = "completed"
STEP_FAILED = "failed"


# 디버거 실행 모드 — PAUSED 해제 시 다음 동작 결정. 외부 publish 안 함 (TaskRunner 내부).
class DebugMode(str, Enum):
    AUTO = "auto"        # 다음 breakpoint 또는 끝까지 진행
    STEP_ONCE = "step"   # 1 step 만 실행 후 pause
    RUN_TO = "run_to"    # 특정 step.id 직전까지 진행 후 pause


@dataclass
class TaskState:
    status: TaskStatus = TaskStatus.IDLE
    task_name: str = ""
    current_step: int = 0
    total_steps: int = 0
    current_label: str = ""
    current_step_id: str = ""
    error: str | None = None
    step_statuses: dict[str, str] = field(default_factory=dict)
    breakpoints: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "status": self.status.value,
            "task_name": self.task_name,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "current_label": self.current_label,
            "current_step_id": self.current_step_id,
            "error": self.error,
            "step_statuses": dict(self.step_statuses),
            "breakpoints": list(self.breakpoints),
        }


OnStateChange = Callable[[TaskState], None]


class TaskRunner:
    def __init__(
        self,
        executor: "StepExecutor",
        on_state_change: OnStateChange | None = None,
    ) -> None:
        self._executor = executor
        self._on_state_change = on_state_change or (lambda _: None)

        self._state = TaskState()
        self._state_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # 초기: 일시정지 아님

        # 디버거 상태 — _state_lock 으로 보호
        self._mode: DebugMode = DebugMode.AUTO
        self._run_to_target: str | None = None
        self._breakpoints: set[str] = set()

        self._thread: threading.Thread | None = None

    @property
    def state(self) -> TaskState:
        with self._state_lock:
            return TaskState(
                status=self._state.status,
                task_name=self._state.task_name,
                current_step=self._state.current_step,
                total_steps=self._state.total_steps,
                current_label=self._state.current_label,
                current_step_id=self._state.current_step_id,
                error=self._state.error,
                step_statuses=dict(self._state.step_statuses),
                breakpoints=sorted(self._breakpoints),
            )

    # ─── 외부 API ─────────────────────────────────────────────────

    def run(self, task: Task) -> bool:
        with self._state_lock:
            if self._state.status == TaskStatus.RUNNING:
                return False
            # 새 task — 모든 step 을 pending 으로 초기화. breakpoint set 은 보존
            # (사용자가 task 시작 전에 미리 박아둘 수 있음).
            self._state.step_statuses = {s.id: STEP_PENDING for s in task.steps}

        self._stop_event.clear()
        self._pause_event.set()
        self._mode = DebugMode.AUTO
        self._run_to_target = None

        self._thread = threading.Thread(
            target=self._run_task,
            args=(task,),
            daemon=True,
            name=f"task-{task.name}",
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.set()

    def pause(self) -> bool:
        with self._state_lock:
            if self._state.status != TaskStatus.RUNNING:
                return False
        self._pause_event.clear()
        self._update_state(status=TaskStatus.PAUSED)
        return True

    def resume(self) -> bool:
        """auto 모드로 재개 — 다음 breakpoint 또는 끝까지."""
        with self._state_lock:
            if self._state.status != TaskStatus.PAUSED:
                return False
            self._mode = DebugMode.AUTO
            self._run_to_target = None
        self._update_state(status=TaskStatus.RUNNING)
        self._pause_event.set()
        return True

    def step_once(self) -> bool:
        """1 step 만 실행 후 다시 pause. PAUSED 상태에서만 동작."""
        with self._state_lock:
            if self._state.status != TaskStatus.PAUSED:
                return False
            self._mode = DebugMode.STEP_ONCE
            self._run_to_target = None
        self._update_state(status=TaskStatus.RUNNING)
        self._pause_event.set()
        return True

    def run_to(self, target_step_id: str) -> bool:
        """target step *직전*까지 진행 후 pause. VSCode 'Run to cursor' 와 동일."""
        with self._state_lock:
            if self._state.status != TaskStatus.PAUSED:
                return False
            self._mode = DebugMode.RUN_TO
            self._run_to_target = target_step_id
        self._update_state(status=TaskStatus.RUNNING)
        self._pause_event.set()
        return True

    def toggle_breakpoint(self, step_id: str) -> bool:
        """breakpoint 토글 — set 에 있으면 제거, 없으면 추가. 항상 success."""
        with self._state_lock:
            if step_id in self._breakpoints:
                self._breakpoints.discard(step_id)
            else:
                self._breakpoints.add(step_id)
        # PAUSED / IDLE 어디서든 호출 가능 → 현재 state publish 로 frontend 동기화
        self._update_state()
        return True

    def is_running(self) -> bool:
        with self._state_lock:
            return self._state.status in (TaskStatus.RUNNING, TaskStatus.PAUSED)

    # ─── Internal ─────────────────────────────────────────────────

    def _run_task(self, task: Task) -> None:
        context = TaskContext()

        self._update_state(
            status=TaskStatus.RUNNING,
            task_name=task.name,
            current_step=0,
            total_steps=len(task.steps),
            current_label="",
            current_step_id="",
            error=None,
        )

        for i, step in enumerate(task.steps):
            # stop 체크
            if self._stop_event.is_set():
                self._update_state(status=TaskStatus.STOPPED)
                return

            # ─── 디버거 게이트: 이 step 직전에 멈출지 결정 ──────
            should_pause = self._should_pause_before(step)
            if should_pause:
                # 다음 step.id 와 label 을 PAUSED 상태에 반영 — UI 가 "지금 여기"
                # 표시할 위치.
                label = getattr(step, "label", "") or step.type
                self._update_state(
                    status=TaskStatus.PAUSED,
                    current_step=i,  # 아직 실행 안 함 → 0-based "다음" 위치
                    current_label=label,
                    current_step_id=step.id,
                )
                self._pause_event.clear()

            # pause 대기 (외부 pause() 호출이나 위 디버거 게이트 둘 다 처리)
            self._pause_event.wait()

            # pause 해제와 동시에 stop 이 들어올 수 있으므로 재확인
            if self._stop_event.is_set():
                self._update_state(status=TaskStatus.STOPPED)
                return

            # mode 가 RUN_TO 이고 target == 현재 step 이면 여기서 한 번 더 멈춰야 함.
            # _should_pause_before 가 처리했어야 하는데 race 방지로 한번 더.
            # (실제로는 _should_pause_before 가 mode 를 AUTO 로 reset 하지 않음 →
            # 두 번 멈추는 일은 없음. 명시적 가드만.)

            label = getattr(step, "label", "") or step.type
            self._update_state(
                status=TaskStatus.RUNNING,
                current_step=i + 1,
                current_label=label,
                current_step_id=step.id,
            )
            self._set_step_status(step.id, STEP_RUNNING)

            try:
                ok = self._executor.execute(step, context, self._stop_event)
            except Exception as exc:
                self._set_step_status(step.id, STEP_FAILED)
                self._update_state(
                    status=TaskStatus.FAILED,
                    error=f"[{label}] Exception: {exc}",
                )
                return

            if not ok:
                self._set_step_status(step.id, STEP_FAILED)
                self._update_state(
                    status=TaskStatus.FAILED,
                    error=f"[{label}] step 실패 ({i + 1}/{len(task.steps)})",
                )
                return

            self._set_step_status(step.id, STEP_COMPLETED)

            # STEP_ONCE 모드: 1 step 실행 후 다음 step 직전에서 멈추도록 mode 유지.
            # _should_pause_before 가 STEP_ONCE 면 pause 처리.
            # AUTO 면 그냥 다음으로 진행.

        self._update_state(
            status=TaskStatus.SUCCESS,
            current_step=len(task.steps),
            current_label="",
            current_step_id="",
        )

    def _should_pause_before(self, step) -> bool:
        """다음 step 실행 *직전* 에 호출 — pause 해야 하면 True.

        규칙:
          - mode == STEP_ONCE: 항상 멈춤. 멈춘 뒤 mode 는 그대로 두면 다음
            resume() 호출 시 어떤 동작인지에 따라 AUTO/STEP 으로 다시 결정됨.
            여기서는 mode reset 안 함 — step_once() 가 다시 SET 함.
          - mode == RUN_TO 이고 step.id == target: 멈춤. mode 는 그대로.
          - step.id 가 breakpoint set 에 있음: 멈춤.
          - 그 외: 진행.
        """
        with self._state_lock:
            mode = self._mode
            target = self._run_to_target
            is_breakpoint = step.id in self._breakpoints

        if mode == DebugMode.STEP_ONCE:
            return True
        if mode == DebugMode.RUN_TO and target == step.id:
            return True
        if is_breakpoint:
            return True
        return False

    def _set_step_status(self, step_id: str, status: str) -> None:
        with self._state_lock:
            self._state.step_statuses[step_id] = status
        self._publish_state()

    def _update_state(self, **kwargs) -> None:
        with self._state_lock:
            for k, v in kwargs.items():
                setattr(self._state, k, v)
        self._publish_state()

    def _publish_state(self) -> None:
        with self._state_lock:
            snapshot = TaskState(
                status=self._state.status,
                task_name=self._state.task_name,
                current_step=self._state.current_step,
                total_steps=self._state.total_steps,
                current_label=self._state.current_label,
                current_step_id=self._state.current_step_id,
                error=self._state.error,
                step_statuses=dict(self._state.step_statuses),
                breakpoints=sorted(self._breakpoints),
            )
        self._on_state_change(snapshot)
