"""Task 정본 registry — name → (factory(params) → TaskSpec, robot_ids).

첫 task = 단팔 pick-and-place (§17.1). noop = runner/디버거 e2e 용 trivial task (§17.4
"도메인 step 0개 (Wait + no-op)"). 새 task = factory 하나 + registry 등록.

**robot 바인딩**: 각 task 는 자기가 어느 robot"들"에서 도는지 여기서 선언한다
(frontend default 로봇 추측 제거 — task 가 대상 robot 의 SSOT). 참여 robot 은
**리스트** — 단팔 task 는 1개, 협동 task 는 여러 개. (누가 어떤 역할이냐 =
역할/동시성 실행 모델은 실제 협동 task 도입 시 별도 설계. 지금은 "참여 robot
목록"까지.) 빈 리스트 = robot 무관 task (noop 등 디버거 검증).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from ..step import TaskSpec
from ..steps import NoOp, Wait
from .pick_and_place import create_pick_and_place_task


@dataclass(frozen=True)
class TaskEntry:
    """registry 원소 — factory + 이 task 에 참여하는 robot 목록.

    robot_ids 는 task 정의의 일부 (task 코드가 SSOT) — frontend 는 이 목록으로
    통신 대상 robot 을 정한다 (ambient default 로봇 없음). 단팔=1개, 협동=여러 개,
    빈 리스트=robot 무관.
    """

    factory: Callable[[dict[str, str]], TaskSpec]
    robot_ids: list[str] = field(default_factory=list)


def _noop_task(params: dict[str, str]) -> TaskSpec:
    """runner/디버거 검증용 — 도메인 step 0개 (Wait + no-op)."""
    return TaskSpec(
        name="noop",
        description="runner/디버거 검증용 trivial task",
        steps=[
            Wait(duration_sec=0.0, label="wait_a"),
            NoOp(label="noop_b"),
            Wait(duration_sec=0.0, label="wait_c"),
        ],
    )


def _pick_and_place_task(params: dict[str, str]) -> TaskSpec:
    """params: pick_object(필수) / place_object(선택) / search_group(기본 "search").

    LLM(modules/llm)이 한국어 명령을 파싱해 이 params 로 Task.RUN 호출."""
    pick = params.get("pick_object", "").strip()
    if not pick:
        raise ValueError("pick_and_place: 'pick_object' param 필요")
    place = params.get("place_object", "").strip() or None
    group = params.get("search_group", "search").strip() or "search"
    return create_pick_and_place_task(pick, place, search_group=group)


# robot 바인딩은 task 정의의 일부 — pick_and_place 는 rgbd+gripper 인 so101 1대.
# (인스턴스 id 직접 지정이 이 규모엔 가장 단순·정직. 협동 task 는 robot_ids 에
#  여러 대를 넣는다 — 역할/동시성은 그때 별도 설계.)
TASK_REGISTRY: dict[str, TaskEntry] = {
    "noop": TaskEntry(_noop_task, robot_ids=[]),
    "pick_and_place": TaskEntry(_pick_and_place_task, robot_ids=["so101_6dof_0"]),
}


def build_task(name: str, params: dict[str, str]) -> TaskSpec:
    """name → TaskSpec (factory 호출). 미등록이면 KeyError."""
    entry = TASK_REGISTRY.get(name)
    if entry is None:
        raise KeyError(name)
    return entry.factory(params)


def task_names() -> list[str]:
    return sorted(TASK_REGISTRY)


def task_infos() -> list[tuple[str, list[str]]]:
    """(name, robot_ids) 목록 — bridge GET /tasks 노출용 (name 정렬)."""
    return [
        (name, list(TASK_REGISTRY[name].robot_ids)) for name in sorted(TASK_REGISTRY)
    ]
