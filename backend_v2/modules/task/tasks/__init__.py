"""Task 정본 registry — name → factory(params) → TaskSpec.

첫 task = 단팔 pick-and-place (§17.1). noop = runner/디버거 e2e 용 trivial task (§17.4
"도메인 step 0개 (Wait + no-op)"). 새 task = factory 하나 + registry 등록.
"""

from __future__ import annotations

from collections.abc import Callable

from ..step import TaskSpec
from ..steps import NoOp, Wait
from .pick_and_place import create_pick_and_place_task


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


TASK_REGISTRY: dict[str, Callable[[dict[str, str]], TaskSpec]] = {
    "noop": _noop_task,
    "pick_and_place": _pick_and_place_task,
}


def build_task(name: str, params: dict[str, str]) -> TaskSpec:
    """name → TaskSpec (factory 호출). 미등록이면 KeyError."""
    factory = TASK_REGISTRY.get(name)
    if factory is None:
        raise KeyError(name)
    return factory(params)


def task_names() -> list[str]:
    return sorted(TASK_REGISTRY)
