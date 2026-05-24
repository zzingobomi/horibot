from dataclasses import dataclass
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


@dataclass
class TaskState:
    status: TaskStatus = TaskStatus.IDLE
    task_name: str = ""
    current_step: int = 0
    total_steps: int = 0
    current_label: str = ""
    error: str | None = None

    def to_dict(self) -> dict:
        return {
            "status": self.status.value,
            "task_name": self.task_name,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "current_label": self.current_label,
            "error": self.error,
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
                error=self._state.error,
            )

    def run(self, task: Task) -> bool:
        with self._state_lock:
            if self._state.status == TaskStatus.RUNNING:
                return False

        self._stop_event.clear()
        self._pause_event.set()

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
        with self._state_lock:
            if self._state.status != TaskStatus.PAUSED:
                return False
        self._update_state(status=TaskStatus.RUNNING)
        self._pause_event.set()
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
            error=None,
        )

        for i, step in enumerate(task.steps):
            # stop 체크
            if self._stop_event.is_set():
                self._update_state(status=TaskStatus.STOPPED)
                return

            # pause 대기
            self._pause_event.wait()

            # pause 해제와 동시에 stop이 들어올 수 있으므로 재확인
            if self._stop_event.is_set():
                self._update_state(status=TaskStatus.STOPPED)
                return

            label = getattr(step, "label", "") or step.type
            self._update_state(
                status=TaskStatus.RUNNING,
                current_step=i + 1,
                current_label=label,
            )

            try:
                ok = self._executor.execute(step, context, self._stop_event)
            except Exception as exc:
                self._update_state(
                    status=TaskStatus.FAILED,
                    error=f"[{label}] Exception: {exc}",
                )
                return

            if not ok:
                self._update_state(
                    status=TaskStatus.FAILED,
                    error=f"[{label}] step 실패 ({i + 1}/{len(task.steps)})",
                )
                return

        self._update_state(
            status=TaskStatus.SUCCESS,
            current_step=len(task.steps),
            current_label="",
        )

    def _update_state(self, **kwargs) -> None:
        with self._state_lock:
            for k, v in kwargs.items():
                setattr(self._state, k, v)
            snapshot = TaskState(
                status=self._state.status,
                task_name=self._state.task_name,
                current_step=self._state.current_step,
                total_steps=self._state.total_steps,
                current_label=self._state.current_label,
                error=self._state.error,
            )
        self._on_state_change(snapshot)
