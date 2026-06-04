"""Step base class + StepContext — lego 화 핵심 인프라.

설계:
- 각 step 은 `Step[T_out]` 상속. `execute(ctx)` 메서드가 출력 `T_out` 반환.
- `step.out` 은 `Slot[T_out]` 반환 — 다음 step 인자로 직접 넘김.
- TaskRunner 는 step 순회하며 `step.execute(ctx)` 호출 → 반환값을
  `ctx.results[step.id]` 에 저장. 다음 step 의 Slot resolve 시 lookup.

이전 [step_executor.py](step_executor.py) 의 `match step.type` dispatch 추방 —
새 step 추가가 executor 코드 수정 0줄 (ideas.md lego test #3).

stage 2 의 ForEach 같은 control flow step 은 `Step` 을 상속하되 자식 step
list 를 필드로 보유 — runner 가 그 children 을 별도로 unroll. v1 에서는
leaf step 만 다룸.
"""

from __future__ import annotations

import logging
import threading
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Generic, TypeVar, overload

from pydantic import BaseModel

from core.transport.messages.base import EmptyData, ServiceResponse
from core.transport.messages.motion import TrajStatus
from modules.task.schema import Slot

if TYPE_CHECKING:
    from core.transport.base_node import BaseNode
    from core.transport.messages.motion import MotionTrajState
    from core.cache.joint_state_cache import JointStateCache
    from modules.calibration.loader import CalibrationData
    from modules.motor.motor_config import MotorConfig


ReqT = TypeVar("ReqT", bound=BaseModel)
ResT = TypeVar("ResT", bound=BaseModel)


logger = logging.getLogger(__name__)


T_out = TypeVar("T_out")


# ─── Errors ──────────────────────────────────────────────────────────


class StepResolveError(RuntimeError):
    """Slot resolve 실패 — 의존 step 출력이 results 에 없음."""


# ─── Step base ───────────────────────────────────────────────────────


def _new_step_id() -> str:
    """UUID 기반 영구 id. 생성 즉시 부여, Slot reference 후 절대 안 바뀜.

    이전 코드의 `step-N` enumerate 패턴은 `Task.__post_init__` 에서 id 를 사후
    재할당 → Slot.step_id stale 위험. UUID 로 plan-time 안정성 확보.
    """
    return f"step-{uuid.uuid4().hex[:8]}"


@dataclass(kw_only=True)
class Step(Generic[T_out], ABC):
    """모든 step 의 base.

    Subclass 규약:
        @dataclass(kw_only=True)
        class MyStep(Step[Detection]):
            prompt: SlotOr[str] = ""

            def execute(self, ctx: StepContext) -> Detection:
                ...

    필드:
        label — 사용자 친화적 표시명 (frontend 트리)
        id    — 영구 unique id, Slot reference 가 들고 다님
    """

    label: str = ""
    id: str = field(default_factory=_new_step_id)

    @property
    def out(self) -> Slot[T_out]:
        """다음 step 인자에 넘기는 typed reference."""
        return Slot(self.id)

    @property
    def type_name(self) -> str:
        """frontend tree publish 용 type 식별자."""
        return type(self).__name__

    @abstractmethod
    def execute(self, ctx: StepContext) -> T_out | None:
        """step 본체. 출력값 반환 (사이드이펙트만 있는 step 은 None).

        반환값은 runner 가 `ctx.results[self.id]` 에 저장.
        실패는 예외로 raise — runner 가 잡아서 TaskStatus.FAILED 로 표시.
        """
        ...


# ─── Task — Step list wrapper ────────────────────────────────────────


@dataclass
class Task:
    """Step list + 메타. 사용자 코드 (task factory) 가 반환하는 단위.

    step.id 는 dataclass 생성 시점에 UUID 로 자동 부여. enumerate 기반
    재할당 X — Slot reference 무결성 보존.
    """

    name: str
    steps: list[Step]
    description: str = ""


def step_to_dict(step: Step) -> dict:
    """Step → JSON 호환 dict. frontend tree publish 용.

    재귀:
    - `list[Step]` 필드 (ForEach.children 등) → 각 Step 도 step_to_dict 거침
    - 중첩 Step 도 type 필드 부여
    - Slot 은 frozen dataclass → asdict 로 `{step_id: "..."}` 자동
    - 다른 dataclass (Position3, Detection) 도 asdict
    """
    from dataclasses import fields

    d: dict = {"id": step.id, "label": step.label, "type": step.type_name}
    for f in fields(step):
        if f.name in ("id", "label"):
            continue
        d[f.name] = _convert_value(getattr(step, f.name))
    return d


def _convert_value(value: Any) -> Any:
    from dataclasses import asdict, is_dataclass

    if isinstance(value, Step):
        return step_to_dict(value)
    if isinstance(value, list):
        return [_convert_value(v) for v in value]
    if isinstance(value, tuple):
        return [_convert_value(v) for v in value]
    if isinstance(value, dict):
        return {k: _convert_value(v) for k, v in value.items()}
    if isinstance(value, BaseModel):
        return value.model_dump()
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return value


def collect_step_ids(steps: list[Step]) -> list[str]:
    """task.steps 전체에서 모든 step.id 수집 (nested 포함).

    TaskRunner 가 task 시작 시 step_statuses 를 pending 으로 초기화할 때 사용.
    필드 introspection 으로 `list[Step]` / `Step` 필드 자동 traverse → ForEach
    같은 새 control flow step 추가해도 여기 수정 불필요.
    """
    from dataclasses import fields

    out: list[str] = []

    def visit(step: Step) -> None:
        out.append(step.id)
        for f in fields(step):
            value = getattr(step, f.name)
            _visit_value(value)

    def _visit_value(value: Any) -> None:
        if isinstance(value, Step):
            visit(value)
        elif isinstance(value, (list, tuple)):
            for v in value:
                _visit_value(v)

    for s in steps:
        visit(s)
    return out


def task_tree(task: Task) -> dict:
    """Task → frontend 가 받는 tree payload.

    v1 은 평면 list. stage 2 의 ForEach 같은 control flow step 이 도입되면
    여기서 자식 step 들을 children 필드로 재귀 처리.
    """
    return {
        "task_name": task.name,
        "description": task.description,
        "steps": [step_to_dict(s) for s in task.steps],
    }


# ─── StepContext — execute() 가 받는 실행 자원 + 결과 lookup ──────────


class StepContext:
    """Step 실행 시 필요한 자원 모음 + Slot resolver.

    이전 `TaskContext.data: dict` (string key 자유분방) 의 대체.
    - 사용자 코드 (task factory) 는 StepContext 직접 접근 X — Slot 만 다룸
    - StepContext 는 runner 가 만들고 step.execute 에 인자로 넘김
    - results 는 step.id → 출력값. Slot.step_id 로 lookup
    """

    def __init__(
        self,
        node: "BaseNode",
        joint_cache: "JointStateCache",
        arm_cfgs: list["MotorConfig"],
        calibration: "CalibrationData | None",
        stop_event: threading.Event,
    ) -> None:
        self.node = node
        self.joint_cache = joint_cache
        self.arm_cfgs = arm_cfgs
        self.calibration = calibration
        self.stop_event = stop_event

        # step.id → 출력값 (typed value class instance)
        self.results: dict[str, Any] = {}

        # motion trajectory 완료 wait 용 — 모든 motion step 이 공유
        self._traj_event = threading.Event()
        self._traj_status: str = TrajStatus.IDLE

        # runner callback — ForEach/Try 같은 step 이 자식 unroll 시 사용.
        # runner 가 자기 _execute_one_step 을 set_run_child 로 주입.
        # 이 indirection 으로 ControlFlowStep 같은 별도 base 클래스 없이도
        # 디버거 게이트 / status / publish 가 nested step 까지 일관 작동.
        self._run_child: Callable[["Step"], Any] | None = None

    # ─── Slot resolve ────────────────────────────────────────

    def resolve(self, value_or_slot: Any) -> Any:
        """Slot 이면 results 에서 lookup, 아니면 그대로 반환.

        모든 step 의 execute() 첫 줄에서 입력 필드를 이걸로 풀어서 씀:
            target_pose = ctx.resolve(self.target)   # Pose6 | Slot[Pose6] → Pose6
        """
        if isinstance(value_or_slot, Slot):
            try:
                return self.results[value_or_slot.step_id]
            except KeyError as exc:
                raise StepResolveError(
                    f"Slot resolve 실패: step_id='{value_or_slot.step_id}' "
                    f"결과 없음 — 의존 step 이 먼저 실행됐는지 확인."
                ) from exc
        return value_or_slot

    def store(self, step_id: str, value: Any) -> None:
        """runner 가 step.execute 반환값을 results 에 저장할 때 호출."""
        if value is None:
            return
        self.results[step_id] = value

    # ─── Child step 위임 — control flow step 용 ─────────────────

    def set_run_child(self, run_child: Callable[["Step"], Any]) -> None:
        """runner 가 자기 _execute_one_step 을 주입."""
        self._run_child = run_child

    def run_child(self, step: "Step") -> Any:
        """ForEach / Try 같은 control flow step 의 execute() 가 호출.

        runner 의 _execute_one_step 을 거쳐 디버거 게이트 / status 갱신 /
        step result publish 가 nested step 에도 일관 작동.
        """
        if self._run_child is None:
            raise RuntimeError(
                "StepContext.run_child: runner callback 미주입 "
                "— TaskRunner 가 set_run_child 호출했는지 확인"
            )
        return self._run_child(step)

    # ─── Motion trajectory wait — step_executor 의 _on_traj_state 패턴 ─

    def on_traj_state(self, state: "MotionTrajState") -> None:
        """MOTION_STATE_TRAJ subscriber callback. runner 가 셋업."""
        self._traj_status = state.status
        if state.status in (TrajStatus.DONE, TrajStatus.FAILED, TrajStatus.STOPPED):
            self._traj_event.set()

    def start_traj(self) -> None:
        """motion 호출 직전 — 이전 event clear."""
        self._traj_event.clear()

    def wait_for_traj(self, timeout: float = 30.0) -> bool:
        """trajectory DONE 까지 대기. 성공 시 True."""
        triggered = self._traj_event.wait(timeout=timeout)
        if not triggered:
            logger.warning("궤적 대기 timeout (%.0fs)", timeout)
            return False
        return self._traj_status == TrajStatus.DONE

    # ─── Convenience: service call / publish — base_node passthrough ──

    @overload
    def call_service(
        self, key: str, data: dict, timeout: float = 5.0
    ) -> dict: ...

    @overload
    def call_service(
        self,
        key: str,
        data: BaseModel,
        res_cls: type[ResT],
        timeout: float = 5.0,
    ) -> "ServiceResponse[ResT]": ...

    def call_service(self, key, data, *args, **kwargs):  # type: ignore[no-untyped-def]
        """BaseNode.call_service 의 passthrough — dict / typed 두 형태 모두 지원.

        key 가 robot-scoped template (`horibot/{robot_id}/...`) 이면 node.r() 로
        expand. step 코드는 그대로 `Service.MOTION_MOVE_L` 같은 raw template
        넘김 — multi-robot 진입 시 step 이 robot 명시 가능하도록 reversible.
        """
        return self.node.call_service(self.node.r(key), data, *args, **kwargs)

    def call_motion(
        self, key: str, data: BaseModel, timeout: float = 5.0
    ) -> bool:
        """motion 서비스 호출 + trajectory 완료 대기 — 거의 모든 move 의 공통 패턴.

        성공 (서비스 OK + 궤적 DONE) 이면 True. 사용자 step 의 execute 가
        흔히 쓸 헬퍼. trajectory 안 따르는 service (Gripper 등) 는 그냥
        call_service.

        motion 서비스는 모두 응답 data 가 EmptyData 라 res_cls 고정.
        """
        self.start_traj()
        res = self.call_service(key, data, EmptyData, timeout)
        if not res.success:
            logger.error("motion 서비스 실패 (%s): %s", key, res.message)
            return False
        return self.wait_for_traj()
