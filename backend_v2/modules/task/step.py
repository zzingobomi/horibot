"""Step base + StepContext — lego DSL 핵심 인프라 (v2 async 포팅).

옛 backend/modules/task/step.py 를 v2 로 재구성 (§17.4):
  - `execute(ctx)` → **`async def execute(ctx)`** (v2 async runner, `await runtime.call`).
  - StepContext 가 node 대신 **runtime + robot_id** 보유. threading/traj-event 제거 —
    motion 완료는 §17.3 계약 (await = DONE, 실패 = exception) 으로 자연 흡수.

Step 추가 = 클래스 하나 + execute 구현. runner 는 match/case 없이 polymorphic
`await step.execute(ctx)` 만 호출. (control flow [ForEach/Try] = §17.1 Orchestration
→ rule of three 로 defer. run_child 재진입 plumbing 도 그때 부활.)
"""

from __future__ import annotations

import logging
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, fields
from typing import TYPE_CHECKING, Any, Callable, Generic, TypeVar

from pydantic import BaseModel

from .schema import Slot

if TYPE_CHECKING:
    from framework.runtime.api import ModuleRuntime

    from .spec import TaskRobotSpec

logger = logging.getLogger(__name__)

T_out = TypeVar("T_out")
TRes = TypeVar("TRes", bound=BaseModel)


class StepResolveError(RuntimeError):
    """Slot resolve 실패 — 의존 step 출력이 results 에 없음 (선행 step 미실행)."""


def _new_step_id() -> str:
    """UUID 기반 영구 id — 생성 즉시 부여, Slot reference 후 불변.

    옛 `step-N` enumerate 재할당 패턴 금지 (Slot.step_id stale 위험)."""
    return f"step-{uuid.uuid4().hex[:8]}"


@dataclass(kw_only=True)
class Step(Generic[T_out], ABC):
    """모든 step 의 base.

    Subclass 규약:
        @dataclass(kw_only=True)
        class MyStep(Step[Detection]):
            prompt: SlotOr[str] = ""
            async def execute(self, ctx: StepContext) -> Detection: ...
    """

    label: str = ""
    id: str = field(default_factory=_new_step_id)

    @property
    def out(self) -> Slot[T_out]:
        """다음 step 인자에 넘기는 typed reference."""
        return Slot(self.id)

    @property
    def type_name(self) -> str:
        """frontend tree/step_result publish 용 type 식별자."""
        return type(self).__name__

    @abstractmethod
    async def execute(self, ctx: "StepContext") -> T_out | None:
        """step 본체. 출력 반환 (사이드이펙트만 있는 step 은 None).

        반환값은 runner 가 ctx.results[self.id] 에 저장. 실패는 예외 raise —
        runner 가 잡아 TaskStatus.FAILED."""
        ...


@dataclass
class TaskSpec:
    """Step list + 메타. task factory 가 반환하는 단위 (contract 의 outer class
    `Task` 와 이름 충돌 피해 Spec). step.id 는 생성 시 UUID 자동 부여 —
    enumerate 재할당 X (Slot 무결성)."""

    name: str
    steps: list[Step]
    description: str = ""


# ─── tree / id 직렬화 (frontend TASK_TREE payload) ───────────────────


def step_to_dict(step: Step) -> dict:
    """Step → JSON 호환 dict. children(list[Step]) 재귀 (ForEach 등)."""
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
    if isinstance(value, (list, tuple)):
        return [_convert_value(v) for v in value]
    if isinstance(value, dict):
        return {k: _convert_value(v) for k, v in value.items()}
    if isinstance(value, BaseModel):
        return value.model_dump()
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return value


def collect_step_ids(steps: list[Step]) -> list[str]:
    """nested 포함 모든 step.id 수집 (runner 가 step_statuses pending 초기화용).

    필드 introspection 으로 list[Step]/Step 자동 traverse → 새 control flow step
    추가해도 수정 불필요."""
    out: list[str] = []

    def visit(step: Step) -> None:
        out.append(step.id)
        for f in fields(step):
            _visit_value(getattr(step, f.name))

    def _visit_value(value: Any) -> None:
        if isinstance(value, Step):
            visit(value)
        elif isinstance(value, (list, tuple)):
            for v in value:
                _visit_value(v)

    for s in steps:
        visit(s)
    return out


def task_tree(task: TaskSpec) -> dict:
    """TaskSpec → frontend TASK_TREE payload (children 재귀 포함)."""
    return {
        "task_name": task.name,
        "description": task.description,
        "steps": [step_to_dict(s) for s in task.steps],
    }


# ─── StepContext — execute() 가 받는 실행 자원 + Slot resolver ────────


class StepContext:
    """Step 실행 자원 + Slot resolver (v2 async).

    사용자 코드(task factory)는 StepContext 직접 접근 X — Slot 만 다룸. runner 가
    만들고 step.execute 에 넘김. results 는 step.id → 출력값 (Slot.step_id lookup).
    """

    def __init__(
        self,
        runtime: "ModuleRuntime",
        robot_id: str,
        robot_spec: "TaskRobotSpec | None" = None,
        gripper_raw: Callable[[], int | None] | None = None,
    ) -> None:
        self.runtime = runtime
        self.robot_id = robot_id
        self.robot_spec = robot_spec  # per-robot 물리 config (gripper 등)
        self._gripper_raw = gripper_raw  # Motor.RAW_STATE 캐시 accessor
        self.results: dict[str, Any] = {}

    def require_spec(self) -> "TaskRobotSpec":
        """gripper 등 물리값 필요 step 이 호출 — spec 미주입이면 fail-fast."""
        if self.robot_spec is None:
            raise RuntimeError(
                f"TaskRobotSpec 미주입 (robot={self.robot_id}) — resolve 가 robots "
                "dict 를 주입했는지 확인 (gripper open/close raw 등 필요)."
            )
        return self.robot_spec

    def gripper_raw(self) -> int | None:
        """현재 gripper raw position (Motor.RAW_STATE 캐시). 미수신이면 None."""
        return self._gripper_raw() if self._gripper_raw is not None else None

    # ─── Slot resolve ───
    def resolve(self, value_or_slot: Any) -> Any:
        """Slot 이면 results lookup, 아니면 그대로. 모든 execute 첫 줄에서 입력 해소."""
        if isinstance(value_or_slot, Slot):
            try:
                return self.results[value_or_slot.step_id]
            except KeyError as exc:
                raise StepResolveError(
                    f"Slot resolve 실패: step_id='{value_or_slot.step_id}' 결과 없음 "
                    "— 선행 step 이 먼저 실행됐는지 확인."
                ) from exc
        return value_or_slot

    def store(self, step_id: str, value: Any) -> None:
        if value is not None:
            self.results[step_id] = value

    # ─── service call — runtime passthrough ───
    async def call(
        self, key: str, req: BaseModel, res_cls: type[TRes], *, timeout: float = 5.0
    ) -> TRes:
        """robot-scoped 서비스 호출 — robot_id = task 대상 robot 자동 주입 (키 expand).

        robot-agnostic 서비스 (robot_id=req 필드, §2.7) 는 `ctx.runtime.call(...)`
        직접 호출 + req.robot_id 세팅.
        """
        return await self.runtime.call(
            key, req, res_cls, robot_id=self.robot_id, timeout=timeout
        )
