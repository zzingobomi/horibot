"""TaskRunner — Step list 를 순차 실행 + pause/resume/breakpoint/run_to 디버거.

이전 [step_executor.py](step_executor.py) 의 실행 책임 흡수 — 별도 executor
객체 없음. TaskRunner 가 StepContext 직접 보유하고, 각 step 의 polymorphic
`execute(ctx)` 호출 → 반환값을 ctx.results 에 저장.

설계 결정:
- match/case dispatch 추방 — `step.execute(ctx)` 만 호출 (ideas.md lego test #3)
- TaskContext.data dict 추방 — `StepContext.results: dict[step_id, Any]` 로 대체
- step.id 영구 UUID — 옛 enumerate `step-N` 재할당 패턴 제거 (Slot.step_id 무결성)
"""

from dataclasses import dataclass, field
from enum import Enum
import logging
import threading
from typing import TYPE_CHECKING, Callable

from core.transport.messages.motion import MotionTrajState
from core.transport.topic_map import Topic
from modules.task.schema import StepResult
from modules.task.step import Step, StepContext, Task, collect_step_ids

if TYPE_CHECKING:
    from core.transport.base_node import BaseNode
    from core.cache.joint_state_cache import JointStateCache
    from modules.calibration.loader import CalibrationData
    from modules.motor.motor_config import MotorConfig


logger = logging.getLogger(__name__)


class TaskStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCESS = "success"
    FAILED = "failed"
    STOPPED = "stopped"


# ─── Runner internal exceptions — _run_task 의 try 절 control 용 ─────


class _StopRequested(Exception):
    """외부 stop() 호출 — _execute_one_step 이 raise → _run_task 가 STOPPED 로."""


class _StepFailed(Exception):
    """step 실행 중 일반 예외 wrap — _execute_one_step 이 raise → FAILED 로."""


# Step 별 실행 상태 — TaskState.step_statuses 에 step_id → 이 값으로 보관.
STEP_PENDING = "pending"
STEP_RUNNING = "running"
STEP_COMPLETED = "completed"
STEP_FAILED = "failed"


# 디버거 실행 모드 — PAUSED 해제 시 다음 동작 결정. 외부 publish 안 함.
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
        node: "BaseNode",
        joint_cache: "JointStateCache",
        arm_cfgs: list["MotorConfig"],
        calibration: "CalibrationData | None",
        on_state_change: OnStateChange | None = None,
    ) -> None:
        self._on_state_change = on_state_change or (lambda _: None)

        self._state = TaskState()
        self._state_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # 초기: 일시정지 아님

        # StepContext 는 1회 생성 후 재사용. run() 마다 results clear.
        # MOTION_STATE_TRAJ subscriber 도 같이 1회만 등록 (node.stop() 시 해제).
        self._ctx = StepContext(
            node=node,
            joint_cache=joint_cache,
            arm_cfgs=arm_cfgs,
            calibration=calibration,
            stop_event=self._stop_event,
        )
        node.create_subscriber(
            Topic.MOTION_STATE_TRAJ, MotionTrajState, self._ctx.on_traj_state
        )

        # ForEach / Try 같은 control flow step 이 자식 unroll 시 사용.
        # 자기 _execute_one_step 을 ctx 에 주입 → ctx.run_child(child) 가 디버거
        # 게이트 / status 갱신 / publish 를 nested step 에도 일관 적용.
        self._ctx.set_run_child(self._execute_one_step)

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
            # 새 task — 모든 step (nested 포함) 을 pending 으로 초기화.
            # breakpoint set 은 보존 (사용자가 task 시작 전 미리 박아둘 수 있음).
            all_ids = collect_step_ids(task.steps)
            self._state.step_statuses = {sid: STEP_PENDING for sid in all_ids}

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
        """target step *직전* 까지 진행 후 pause. VSCode 'Run to cursor' 와 동일."""
        with self._state_lock:
            if self._state.status != TaskStatus.PAUSED:
                return False
            self._mode = DebugMode.RUN_TO
            self._run_to_target = target_step_id
        self._update_state(status=TaskStatus.RUNNING)
        self._pause_event.set()
        return True

    def toggle_breakpoint(self, step_id: str) -> bool:
        """breakpoint 토글 — set 에 있으면 제거, 없으면 추가."""
        with self._state_lock:
            if step_id in self._breakpoints:
                self._breakpoints.discard(step_id)
            else:
                self._breakpoints.add(step_id)
        self._update_state()
        return True

    def is_running(self) -> bool:
        with self._state_lock:
            return self._state.status in (TaskStatus.RUNNING, TaskStatus.PAUSED)

    # ─── Internal ─────────────────────────────────────────────────

    def _run_task(self, task: Task) -> None:
        # 매 task 마다 results 재시작 — 이전 task 의 Slot 결과가 새 task 에
        # 누출되지 않게.
        self._ctx.results.clear()

        self._update_state(
            status=TaskStatus.RUNNING,
            task_name=task.name,
            current_step=0,
            total_steps=len(task.steps),
            current_label="",
            current_step_id="",
            error=None,
        )

        # _BreakLoop 가 ForEach 밖까지 올라오면 잘못된 사용 — 잡아서 FAILED.
        from modules.task.steps import _BreakLoop

        try:
            for i, step in enumerate(task.steps):
                self._update_state(current_step=i + 1)
                self._execute_one_step(step)
        except _StopRequested:
            self._update_state(status=TaskStatus.STOPPED)
            return
        except _BreakLoop:
            self._update_state(
                status=TaskStatus.FAILED,
                error="BreakIf 가 ForEach 밖에서 발생 — 잘못된 사용",
            )
            return
        except _StepFailed as exc:
            self._update_state(status=TaskStatus.FAILED, error=str(exc))
            return

        self._update_state(
            status=TaskStatus.SUCCESS,
            current_step=len(task.steps),
            current_label="",
            current_step_id="",
        )

    def _execute_one_step(self, step: Step) -> object:
        """단일 step 실행 — 디버거 게이트 + status + result publish.

        ForEach / Try 같은 control flow step 의 execute() 가 `ctx.run_child(child)`
        호출 시 이 함수가 다시 들어옴 → nested step 도 동일 인프라 자동 적용.

        예외 흐름:
            - StopRequested  → 외부 `stop()` 호출. 호출 stack 위로 전파해서
              _run_task 가 잡고 STOPPED 처리
            - _BreakLoop     → BreakIf step 이 raise. ForEach 가 catch.
              ForEach 밖이면 _run_task 가 _UnhandledBreak 로 변환
            - 일반 예외      → step FAILED 표시 후 _StepFailed 로 wrap
        """
        from modules.task.steps import _BreakLoop  # circular import 방지 lazy

        if self._stop_event.is_set():
            raise _StopRequested()

        if self._should_pause_before(step):
            label = step.label or step.type_name
            self._update_state(
                status=TaskStatus.PAUSED,
                current_label=label,
                current_step_id=step.id,
            )
            self._pause_event.clear()
        self._pause_event.wait()

        if self._stop_event.is_set():
            raise _StopRequested()

        label = step.label or step.type_name
        self._update_state(
            status=TaskStatus.RUNNING,
            current_label=label,
            current_step_id=step.id,
        )
        self._set_step_status(step.id, STEP_RUNNING)

        try:
            result = step.execute(self._ctx)
        except _BreakLoop:
            # ForEach 가 잡으라고 위로. 잡히면 ForEach 의 execute 가 정상 return —
            # 그 경우 status COMPLETED 는 ForEach 본인이 처리 (아래 finally).
            # 여기 step 자체는 BreakIf — 그것도 정상 종료로 본다.
            self._set_step_status(step.id, STEP_COMPLETED)
            self._publish_step_result(step, None)
            raise
        except _StopRequested:
            raise
        except _StepFailed:
            # 이미 _StepFailed 로 wrap 된 상태 — 그대로 위로 전파
            raise
        except Exception as exc:
            self._set_step_status(step.id, STEP_FAILED)
            logger.exception("step 실행 중 예외 [%s]", label)
            raise _StepFailed(f"[{label}] {type(exc).__name__}: {exc}") from exc

        self._ctx.store(step.id, result)
        self._publish_step_result(step, result)
        self._set_step_status(step.id, STEP_COMPLETED)
        return result

    def _should_pause_before(self, step: Step) -> bool:
        """다음 step 실행 *직전* 에 호출 — pause 해야 하면 True.

        - mode == STEP_ONCE: 항상 멈춤
        - mode == RUN_TO 이고 step.id == target: 멈춤
        - step.id 가 breakpoint set 에 있음: 멈춤
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

    def _publish_step_result(self, step: Step, value: object | None) -> None:
        """완료된 step 의 출력을 토픽으로 publish — frontend 자동 렌더 hook.

        None 출력 (MoveTCP/Gripper/...) 도 발행 — "여기까지 도달했음" 시각 마커.
        type 문자열 = step.out 의 dataclass 클래스 이름 (Detection/Position3/...)
        또는 출력 없으면 "None".
        """
        type_name = type(value).__name__ if value is not None else "None"
        payload = StepResult(
            step_id=step.id, type_name=type_name, value=value
        ).to_dict()
        try:
            self._ctx.node.publish(Topic.TASK_STEP_RESULT, payload)
        except Exception as exc:
            logger.warning("step_result publish 실패 (%s): %s", step.id, exc)

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
